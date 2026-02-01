# src/mppi/mppi/utils/mppi_controller.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, List

import numpy as np


@dataclass
class MPPIConfig:
    # --- time ---
    dt: float = 0.05
    horizon_sec: float = 1.5

    # --- sampling ---
    num_samples: int = 600
    seed: Optional[int] = 1

    # --- MPPI temperature (lambda) ---
    lambda_: float = 1.0  # smaller -> greedier, larger -> smoother

    # --- noise std (control perturbation) ---
    sigma_vx: float = 0.8
    sigma_vy: float = 0.8
    sigma_yaw_rate: float = 0.8

    # --- bounds ---
    max_v_xy: float = 2.0          # m/s (speed cap)
    max_yaw_rate: float = 1.2      # rad/s

    # --- cost weights ---
    w_goal: float = 10.0
    w_ctrl: float = 0.05
    w_heading: float = 0.2         # optional: yaw alignment term at terminal

    # --- optional behavior ---
    use_body_frame_cmd: bool = False
    # False: u=(vx,vy) is in local/world frame (ENU)
    # True : u=(vx,vy) is in body frame (forward/right), will be rotated to world in rollout

    # --- smoothing / inertia on nominal sequence ---
    u_smooth_alpha: float = 0.7
    # 0.0 -> instantly replace with new u
    # 0.7 -> keep 70% of previous nominal (usually 안정적)

    # --- goal tolerance (for external state machine use) ---
    goal_radius: float = 0.5

    # --- obstacles ---
    # list of (cx, cy, r)
    obstacles: List[Tuple[float, float, float]] = field(default_factory=list)

    safe_margin: float = 1.0      # inflate obstacles by margin
    w_obs: float = 50.0           # obstacle proximity weight (tune)
    obs_beta: float = 6.0         # barrier sharpness
    w_collision: float = 1e6      # hard penalty if inside inflated obstacle


class MPPIController:
    """
    Minimal MPPI core for 2D position + yaw with controls (vx, vy, yaw_rate).

    State input:
      x, y, yaw  (yaw in rad)

    Goal input:
      x_goal, y_goal, yaw_goal(optional; if None, it will use bearing-to-goal)

    Output:
      vx_cmd, vy_cmd, yaw_rate_cmd
    """

    def __init__(self, cfg: MPPIConfig):
        self.cfg = cfg
        self.T = max(1, int(round(cfg.horizon_sec / cfg.dt)))

        self._rng = np.random.default_rng(cfg.seed)
        # nominal control sequence (T, 3): [vx, vy, yaw_rate]
        self._u_nom = np.zeros((self.T, 3), dtype=np.float32)

        # precompute noise std vector
        self._sigma = np.array([cfg.sigma_vx, cfg.sigma_vy, cfg.sigma_yaw_rate], dtype=np.float32)

    @staticmethod
    def _wrap_pi(a: np.ndarray) -> np.ndarray:
        return (a + np.pi) % (2.0 * np.pi) - np.pi

    @staticmethod
    def _softplus(x: np.ndarray) -> np.ndarray:
        """
        Numerically-stable softplus: log(1+exp(x))
        """
        # clamp to avoid overflow, but keep smoothness
        xc = np.clip(x, -50.0, 50.0)
        return np.log1p(np.exp(xc))

    def reset(self):
        self._u_nom[:] = 0.0

    def step(
        self,
        state: Tuple[float, float, float],
        goal_xy: Tuple[float, float],
        goal_yaw: Optional[float] = None,
    ) -> Tuple[float, float, float, Dict[str, float]]:
        """
        Returns (vx, vy, yaw_rate, debug_dict)
        """
        x0, y0, yaw0 = state
        gx, gy = goal_xy

        # If goal_yaw not provided, set desired yaw to bearing-to-goal (terminal alignment)
        if goal_yaw is None:
            goal_yaw = float(np.arctan2(gy - y0, gx - x0))

        # Sample perturbations: eps shape (N, T, 3)
        N, T = self.cfg.num_samples, self.T
        eps = self._rng.normal(loc=0.0, scale=1.0, size=(N, T, 3)).astype(np.float32)
        eps *= self._sigma  # per-dim std

        # Candidate controls: u_k = u_nom + eps_k
        u = self._u_nom[None, :, :] + eps  # (N, T, 3)

        # Apply bounds to candidates
        u = self._clip_controls(u)

        # Rollout all candidates (vectorized)
        J = self._rollout_cost(x0, y0, yaw0, gx, gy, float(goal_yaw), u)  # (N,)

        # MPPI weights
        J_min = float(np.min(J))
        # numerically stable softmin
        w = np.exp(-(J - J_min) / max(1e-6, self.cfg.lambda_)).astype(np.float64)  # (N,)
        w_sum = float(np.sum(w)) + 1e-12
        w /= w_sum

        # Weighted update: u_nom <- u_nom + Σ w_k * eps_k
        # 안정적으로: delta = Σ w_k * (u_k - u_nom)
        delta = np.tensordot(w, (u - self._u_nom[None, :, :]), axes=(0, 0))  # (T,3)

        u_new = self._u_nom + delta

        # Smooth update
        a = float(self.cfg.u_smooth_alpha)
        self._u_nom = (a * self._u_nom + (1.0 - a) * u_new).astype(np.float32)

        # Shift horizon (receding)
        u0 = self._u_nom[0].copy()
        self._u_nom[:-1] = self._u_nom[1:]
        self._u_nom[-1] = 0.0

        # Final command clamp
        u0 = self._clip_controls(u0[None, None, :])[0, 0, :]

        dbg = {
            "J_min": J_min,
            "J_mean": float(np.mean(J)),
            "w_max": float(np.max(w)),
            "dist_to_goal": float(np.hypot(gx - x0, gy - y0)),
            "yaw_err": float(self._wrap_pi(np.array([goal_yaw - yaw0], dtype=np.float32))[0]),
            "num_obstacles": float(len(self.cfg.obstacles)),
        }
        return float(u0[0]), float(u0[1]), float(u0[2]), dbg

    def _clip_controls(self, u: np.ndarray) -> np.ndarray:
        """
        u shape: (..., 3)
        clip speed magnitude and yaw_rate
        """
        # clip yaw rate
        u[..., 2] = np.clip(u[..., 2], -self.cfg.max_yaw_rate, self.cfg.max_yaw_rate)

        # clip xy speed magnitude
        vx = u[..., 0]
        vy = u[..., 1]
        v = np.sqrt(vx * vx + vy * vy) + 1e-12
        v_cap = self.cfg.max_v_xy
        scale = np.minimum(1.0, v_cap / v)
        u[..., 0] = vx * scale
        u[..., 1] = vy * scale
        return u

    def _rollout_cost(
        self,
        x0: float,
        y0: float,
        yaw0: float,
        gx: float,
        gy: float,
        gyaw: float,
        u: np.ndarray,
    ) -> np.ndarray:
        """
        Vectorized rollout for all samples.
        u: (N, T, 3)
        Returns J: (N,)
        """
        cfg = self.cfg
        N, T = u.shape[0], u.shape[1]

        x = np.full((N,), x0, dtype=np.float32)
        y = np.full((N,), y0, dtype=np.float32)
        yaw = np.full((N,), yaw0, dtype=np.float32)

        J = np.zeros((N,), dtype=np.float32)

        # Control effort cost (running)
        # Σ ||u||^2
        J += cfg.w_ctrl * np.sum(
            u[:, :, 0] ** 2 + u[:, :, 1] ** 2 + 0.3 * (u[:, :, 2] ** 2),
            axis=1,
        )

        # Obstacles pre-pack for vectorization (if any)
        has_obs = (cfg.obstacles is not None) and (len(cfg.obstacles) > 0)
        if has_obs:
            obs = np.array(cfg.obstacles, dtype=np.float32)  # (M,3)
            ocx = obs[:, 0][None, :]  # (1,M)
            ocy = obs[:, 1][None, :]  # (1,M)
            orad = (obs[:, 2] + float(cfg.safe_margin))[None, :]  # (1,M) inflated radius

            beta = float(cfg.obs_beta)
            w_obs = float(cfg.w_obs)
            w_col = float(cfg.w_collision)

            # track whether each sample has collided at least once
            collided = np.zeros((N,), dtype=bool)

        # Forward simulate
        dt = float(cfg.dt)
        for t in range(T):
            vx = u[:, t, 0]
            vy = u[:, t, 1]
            r = u[:, t, 2]

            if cfg.use_body_frame_cmd:
                # body -> world rotation
                cy = np.cos(yaw)
                sy = np.sin(yaw)
                vx_w = cy * vx - sy * vy
                vy_w = sy * vx + cy * vy
            else:
                vx_w = vx
                vy_w = vy

            x = x + vx_w * dt
            y = y + vy_w * dt
            yaw = self._wrap_pi(yaw + r * dt)

            # Obstacle running cost
            if has_obs:
                dx = x[:, None] - ocx  # (N,M)
                dy = y[:, None] - ocy
                dist = np.sqrt(dx * dx + dy * dy + 1e-12)  # (N,M)
                clearance = dist - orad  # (N,M), >0 safe, <0 inside inflated obstacle

                # soft barrier: softplus(-beta * clearance)
                z = -beta * clearance
                soft = self._softplus(z).astype(np.float32)  # (N,M)
                J += (w_obs * np.sum(soft, axis=1) * dt).astype(np.float32)

                # hard collision: add once per rollout if ever inside
                hit_now = np.any(clearance < 0.0, axis=1)
                new_hit = hit_now & (~collided)
                if np.any(new_hit):
                    J += new_hit.astype(np.float32) * np.float32(w_col)
                    collided |= hit_now

        # Terminal goal cost
        dxg = x - gx
        dyg = y - gy
        J += cfg.w_goal * (dxg * dxg + dyg * dyg)

        # Terminal heading alignment (optional)
        yaw_err = self._wrap_pi(yaw - gyaw)
        J += cfg.w_heading * (yaw_err * yaw_err)

        return J.astype(np.float32)
