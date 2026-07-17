#!/usr/bin/env python3

"""Pure MPPI flight controller using exact SDF collision geometry."""

import math

import numpy as np

from . import mppi_node as core
from .world_geometry import load_world_geometry


_geometry = None


def _load_exact_geometry(loader):
    global _geometry
    _geometry = load_world_geometry(str(loader.world_path))
    # The enhanced controller consumes exact geometry instead of circle samples.
    return []


class PureMPPIController(core.MPPIController):
    """Warm-started MPPI with progress and decision-consistency costs."""

    def __init__(self, cfg, _legacy_obstacles):
        super().__init__(cfg, [])
        self.geometry = _geometry
        self.initialized = False
        self.committed_lateral_sign = 0
        self.last_ess = 0.0
        self.last_temperature = cfg.lam

    def reset(self):
        super().reset()
        self.initialized = False
        self.committed_lateral_sign = 0

    def set_nominal_towards_goal(self, x, y, goal_x, goal_y, alpha=1.0):
        # Initialize once. Afterwards step()'s shifted solution is the warm start.
        if self.initialized:
            return
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy) + 1e-6
        self.u_nom[:, 0] = self.cfg.v_nom * dx / distance
        self.u_nom[:, 1] = self.cfg.v_nom * dy / distance
        self.u_nom[:, 2] = 0.0
        self.initialized = True

    @staticmethod
    def _rectangle_signed_distance(xs, ys, obstacle):
        cos_yaw, sin_yaw = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
        dx, dy = xs - obstacle.x, ys - obstacle.y
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        qx = np.abs(local_x) - obstacle.half_x
        qy = np.abs(local_y) - obstacle.half_y
        outside = np.sqrt(np.maximum(qx, 0.0) ** 2 + np.maximum(qy, 0.0) ** 2)
        inside = np.minimum(np.maximum(qx, qy), 0.0)
        return outside + inside

    def step(self, state, goal):
        cfg = self.cfg
        x0, y0, yaw0 = state
        gx, gy, gyaw = goal
        horizon, samples, dt = cfg.horizon, cfg.num_samples, cfg.dt
        previous_nominal = self.u_nom.copy()

        noise = np.zeros((samples, horizon, 3), dtype=np.float32)
        noise[:, :, 0] = self.rng.normal(0.0, cfg.sigma_v, (samples, horizon))
        noise[:, :, 1] = self.rng.normal(0.0, cfg.sigma_v, (samples, horizon))
        noise[:, :, 2] = self.rng.normal(0.0, cfg.sigma_yaw_rate, (samples, horizon))
        # Correlated noise produces coherent turns. Independent noise mostly
        # cancels over the horizon and cannot reach a distant lateral gap.
        correlation = 0.98
        innovation_scale = math.sqrt(1.0 - correlation ** 2)
        for index in range(1, horizon):
            noise[:, index, :] = (
                correlation * noise[:, index - 1, :]
                + innovation_scale * noise[:, index, :]
            )
        # Antithetic pairs reduce random left/right sampling imbalance.
        half = samples // 2
        noise[half:2 * half] = -noise[:half]

        controls = previous_nominal[None, :, :] + noise
        controls[:, :, 0:2] = np.clip(controls[:, :, 0:2], -cfg.v_max, cfg.v_max)
        planar_speed = np.linalg.norm(controls[:, :, 0:2], axis=2)
        speed_scale = np.minimum(1.0, cfg.v_max / np.maximum(planar_speed, 1e-6))
        controls[:, :, 0] *= speed_scale
        controls[:, :, 1] *= speed_scale
        controls[:, :, 2] = np.clip(controls[:, :, 2], -cfg.yaw_rate_max, cfg.yaw_rate_max)

        xs = np.empty((samples, horizon + 1), dtype=np.float32)
        ys = np.empty((samples, horizon + 1), dtype=np.float32)
        yaws = np.empty((samples, horizon + 1), dtype=np.float32)
        xs[:, 0], ys[:, 0], yaws[:, 0] = x0, y0, yaw0
        for index in range(horizon):
            xs[:, index + 1] = xs[:, index] + controls[:, index, 0] * dt
            ys[:, index + 1] = ys[:, index] + controls[:, index, 1] * dt
            yaws[:, index + 1] = yaws[:, index] + controls[:, index, 2] * dt

        goal_dx, goal_dy = gx - x0, gy - y0
        goal_norm = math.hypot(goal_dx, goal_dy) + 1e-6
        direction_x, direction_y = goal_dx / goal_norm, goal_dy / goal_norm
        dx, dy = xs[:, 1:] - gx, ys[:, 1:] - gy
        costs = cfg.w_goal * np.mean(dx * dx + dy * dy, axis=1)
        costs += cfg.w_goal_final * ((xs[:, -1] - gx) ** 2 + (ys[:, -1] - gy) ** 2)

        # Reward actual progress along the goal direction, not just proximity.
        progress = direction_x * (xs[:, -1] - x0) + direction_y * (ys[:, -1] - y0)
        costs -= 80.0 * progress

        dyaw = (yaws[:, -1] - gyaw + np.pi) % (2.0 * np.pi) - np.pi
        costs += 0.5 * dyaw * dyaw

        obstacle_cost = np.zeros(samples, dtype=np.float32)
        if self.geometry is not None:
            for obstacle in self.geometry.rectangles:
                signed = self._rectangle_signed_distance(xs[:, 1:], ys[:, 1:], obstacle)
                clearance = np.min(signed, axis=1) - cfg.safety_margin
                penetration = np.clip(-clearance, 0.0, None)
                near = np.clip(cfg.near_buffer - clearance, 0.0, cfg.near_buffer)
                obstacle_cost += (
                    cfg.penetrate_bias * (penetration > 0.0)
                    + cfg.penetrate_k * penetration ** 2
                    + cfg.near_k * near ** 2
                )
            for obstacle in self.geometry.circles:
                distance = np.sqrt(
                    (xs[:, 1:] - obstacle.x) ** 2
                    + (ys[:, 1:] - obstacle.y) ** 2
                )
                clearance = np.min(distance, axis=1) - obstacle.radius - cfg.safety_margin
                penetration = np.clip(-clearance, 0.0, None)
                near = np.clip(cfg.near_buffer - clearance, 0.0, cfg.near_buffer)
                obstacle_cost += (
                    cfg.penetrate_bias * (penetration > 0.0)
                    + cfg.penetrate_k * penetration ** 2
                    + cfg.near_k * near ** 2
                )
        costs += cfg.w_obst * obstacle_cost

        # Preserve the prior solution and penalize switching the chosen side.
        deviation = controls - previous_nominal[None, :, :]
        costs += 1.0 * np.mean(deviation[:, :, 0] ** 2 + deviation[:, :, 1] ** 2, axis=1)
        mean_lateral = np.mean(controls[:, :max(5, horizon // 4), 1], axis=1)
        # Deterministic, weak tie-break prevents symmetric left/right indecision.
        # The lower and upper routes can be exactly symmetric. Prefer the
        # lower side only as a weak deterministic tie-break for this corridor.
        costs += 2.0 * mean_lateral
        if self.committed_lateral_sign:
            costs += 40.0 * (mean_lateral * self.committed_lateral_sign < -0.05)

        costs += cfg.w_ctrl * np.mean(
            controls[:, :, 0] ** 2 + controls[:, :, 1] ** 2
            + 0.4 * controls[:, :, 2] ** 2, axis=1
        )
        delta = controls[:, 1:, :] - controls[:, :-1, :]
        costs += cfg.w_smooth * np.mean(
            delta[:, :, 0] ** 2 + delta[:, :, 1] ** 2
            + 0.4 * delta[:, :, 2] ** 2, axis=1
        )

        # Scale temperature to the observed cost spread to avoid weight collapse.
        minimum = float(np.min(costs))
        shifted = costs - minimum
        target_ess = max(8.0, 0.10 * samples)
        low = max(cfg.lam, 1e-3)
        high = max(low, float(np.max(shifted)), 1.0)
        for _ in range(18):
            temperature = 0.5 * (low + high)
            trial = np.exp(-shifted / temperature)
            trial /= float(np.sum(trial)) + 1e-12
            ess = float(1.0 / (np.sum(trial * trial) + 1e-12))
            if ess < target_ess:
                low = temperature
            else:
                high = temperature
        temperature = high
        weights = np.exp(-(costs - minimum) / temperature)
        weights /= float(np.sum(weights)) + 1e-12
        self.last_ess = float(1.0 / (np.sum(weights * weights) + 1e-12))
        self.last_temperature = temperature

        self.u_nom = np.tensordot(weights.astype(np.float32), controls, axes=(0, 0)).astype(np.float32)
        command = self.u_nom[0].copy()
        if abs(command[1]) > 0.15:
            self.committed_lateral_sign = 1 if command[1] > 0.0 else -1
        self.u_nom[:-1] = self.u_nom[1:]
        self.u_nom[-1] = self.u_nom[-2]
        return float(command[0]), float(command[1]), float(command[2])


def main(args=None):
    core.WorldObstacleLoader.load = _load_exact_geometry
    core.MPPIController = PureMPPIController
    core.main(args=args)


if __name__ == "__main__":
    main()
