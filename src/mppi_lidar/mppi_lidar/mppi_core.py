# src/mppi_lidar/mppi_lidar/mppi_core.py

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class MPPIParams:
    dt: float = 0.05
    horizon: int = 60
    num_samples: int = 800
    temperature: float = 0.7

    vxy_max: float = 2.0
    vz_max: float = 0.8
    yaw_rate_max: float = 1.2

    sigma_vxy: float = 0.6
    sigma_vz: float = 0.25
    sigma_yaw_rate: float = 0.5

    w_goal: float = 5.0
    w_goal_final: float = 25.0
    w_obstacle: float = 80.0
    w_control: float = 0.05
    w_smooth: float = 0.2

    safety_radius: float = 0.8
    lidar_max_range: float = 8.0


class MPPIPlannerCore:
    """
    State:
        x = [px, py, pz, yaw]

    Control:
        u = [vx, vy, vz, yaw_rate]

    이 core는 PX4에 직접 명령을 보내지 않는다.
    단순히 다음 cmd_vel 후보만 계산한다.
    """

    def __init__(self, params: MPPIParams):
        self.p = params
        self.u_dim = 4
        self.rng = np.random.default_rng()

        self.u_nom = np.zeros((self.p.horizon, self.u_dim), dtype=np.float32)

    def reset(self):
        self.u_nom[:] = 0.0

    def warm_start_to_goal(self, state: np.ndarray, goal: np.ndarray):
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        dz = goal[2] - state[2]

        dist_xy = np.hypot(dx, dy) + 1e-6

        vx = self.p.vxy_max * 0.6 * dx / dist_xy
        vy = self.p.vxy_max * 0.6 * dy / dist_xy
        vz = np.clip(0.8 * dz, -self.p.vz_max, self.p.vz_max)

        self.u_nom[:, 0] = np.clip(vx, -self.p.vxy_max, self.p.vxy_max)
        self.u_nom[:, 1] = np.clip(vy, -self.p.vxy_max, self.p.vxy_max)
        self.u_nom[:, 2] = vz
        self.u_nom[:, 3] = 0.0

    def step(
        self,
        state: np.ndarray,
        goal: np.ndarray,
        lidar_points_body: Optional[np.ndarray],
    ) -> Tuple[np.ndarray, dict]:
        """
        state: [x, y, z, yaw]
        goal:  [gx, gy, gz, gyaw]
        lidar_points_body: shape (M, 2), body frame 기준 LiDAR 점들
        """

        p = self.p
        N = p.num_samples
        H = p.horizon
        dt = p.dt

        # 1. control noise sampling
        noise = np.zeros((N, H, self.u_dim), dtype=np.float32)
        noise[:, :, 0] = self.rng.normal(0.0, p.sigma_vxy, size=(N, H))
        noise[:, :, 1] = self.rng.normal(0.0, p.sigma_vxy, size=(N, H))
        noise[:, :, 2] = self.rng.normal(0.0, p.sigma_vz, size=(N, H))
        noise[:, :, 3] = self.rng.normal(0.0, p.sigma_yaw_rate, size=(N, H))

        # 2. nominal control + noise
        u = self.u_nom[None, :, :] + noise

        # 3. control clipping
        u[:, :, 0] = np.clip(u[:, :, 0], -p.vxy_max, p.vxy_max)
        u[:, :, 1] = np.clip(u[:, :, 1], -p.vxy_max, p.vxy_max)
        u[:, :, 2] = np.clip(u[:, :, 2], -p.vz_max, p.vz_max)
        u[:, :, 3] = np.clip(u[:, :, 3], -p.yaw_rate_max, p.yaw_rate_max)

        # 4. rollout states
        xs = np.zeros((N, H + 1), dtype=np.float32)
        ys = np.zeros((N, H + 1), dtype=np.float32)
        zs = np.zeros((N, H + 1), dtype=np.float32)
        yaws = np.zeros((N, H + 1), dtype=np.float32)

        xs[:, 0] = state[0]
        ys[:, 0] = state[1]
        zs[:, 0] = state[2]
        yaws[:, 0] = state[3]

        for t in range(H):
            xs[:, t + 1] = xs[:, t] + u[:, t, 0] * dt
            ys[:, t + 1] = ys[:, t] + u[:, t, 1] * dt
            zs[:, t + 1] = zs[:, t] + u[:, t, 2] * dt
            yaws[:, t + 1] = yaws[:, t] + u[:, t, 3] * dt

        costs = np.zeros((N,), dtype=np.float32)

        # 5. goal tracking cost
        dx = xs[:, 1:] - goal[0]
        dy = ys[:, 1:] - goal[1]
        dz = zs[:, 1:] - goal[2]

        costs += p.w_goal * np.mean(
            dx * dx + dy * dy + 0.5 * dz * dz,
            axis=1,
        )

        # 6. final goal cost
        dxf = xs[:, -1] - goal[0]
        dyf = ys[:, -1] - goal[1]
        dzf = zs[:, -1] - goal[2]

        costs += p.w_goal_final * (
            dxf * dxf + dyf * dyf + 0.5 * dzf * dzf
        )

        # 7. yaw alignment cost
        dyaw = (yaws[:, -1] - goal[3] + np.pi) % (2.0 * np.pi) - np.pi
        costs += 0.5 * dyaw * dyaw

        # 8. LiDAR obstacle cost
        if lidar_points_body is not None and len(lidar_points_body) > 0:
            obs_cost = self._lidar_obstacle_cost(
                xs,
                ys,
                state,
                lidar_points_body,
            )
            costs += p.w_obstacle * obs_cost

        # 9. control effort cost
        costs += p.w_control * np.mean(
            u[:, :, 0] ** 2
            + u[:, :, 1] ** 2
            + 0.5 * u[:, :, 2] ** 2
            + 0.3 * u[:, :, 3] ** 2,
            axis=1,
        )

        # 10. smoothness cost
        du = u[:, 1:, :] - u[:, :-1, :]
        costs += p.w_smooth * np.mean(
            np.sum(du * du, axis=2),
            axis=1,
        )

        # 11. softmax weight update
        cmin = float(np.min(costs))
        weights = np.exp(-(costs - cmin) / max(p.temperature, 1e-6))
        weights = weights / (np.sum(weights) + 1e-9)

        # 12. update nominal control sequence
        self.u_nom = np.tensordot(
            weights.astype(np.float32),
            u,
            axes=(0, 0),
        )

        # 13. first command
        u0 = self.u_nom[0].copy()

        # 14. receding horizon shift
        self.u_nom[:-1] = self.u_nom[1:]
        self.u_nom[-1] = self.u_nom[-2]

        info = {
            "cost_min": float(np.min(costs)),
            "cost_mean": float(np.mean(costs)),
            "best_index": int(np.argmin(costs)),
            "u0": u0.tolist(),
        }

        return u0, info

    def _lidar_obstacle_cost(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        state: np.ndarray,
        lidar_points_body: np.ndarray,
    ) -> np.ndarray:
        """
        현재 body frame LiDAR 점을 world frame으로 변환한 뒤,
        rollout trajectory가 해당 점들에 가까워지면 cost 증가.
        """

        yaw = state[3]
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        bx = lidar_points_body[:, 0]
        by = lidar_points_body[:, 1]

        wx = state[0] + cy * bx - sy * by
        wy = state[1] + sy * bx + cy * by

        # LiDAR 점이 너무 많으면 계산량 감소를 위해 일부만 사용
        if len(wx) > 80:
            idx = np.linspace(0, len(wx) - 1, 80).astype(np.int32)
            wx = wx[idx]
            wy = wy[idx]

        N = xs.shape[0]
        obs_cost = np.zeros((N,), dtype=np.float32)

        for ox, oy in zip(wx, wy):
            dx = xs[:, 1:] - ox
            dy = ys[:, 1:] - oy

            d = np.sqrt(dx * dx + dy * dy)
            dmin = np.min(d, axis=1)

            violation = np.clip(self.p.safety_radius - dmin, 0.0, None)
            obs_cost += violation * violation

        return obs_cost