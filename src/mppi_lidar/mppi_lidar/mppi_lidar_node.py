#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class RectangleObstacle2D:
    """LiDAR points fitted to the rectangle contract used by pure MPPI."""

    x: float
    y: float
    half_x: float
    half_y: float
    yaw: float


@dataclass
class MPPIConfig:
    dt: float = 0.05
    horizon: int = 100
    num_samples: int = 400
    lam: float = 1.0

    v_max: float = 2.0
    yaw_rate_max: float = 1.2

    sigma_v: float = 0.7
    sigma_yaw_rate: float = 0.6

    w_goal: float = 8.0
    w_goal_final: float = 30.0

    w_obst: float = 200.0
    w_ctrl: float = 0.2
    w_smooth: float = 0.4

    safety_margin: float = 0.9
    near_buffer: float = 1.0

    near_k: float = 18.0
    penetrate_k: float = 80.0
    penetrate_bias: float = 200.0

    v_nom: float = 1.2
class MPPIController:
    def __init__(self, cfg: MPPIConfig, obstacles: List[RectangleObstacle2D]):
        self.cfg = cfg
        self.obstacles = obstacles
        self.u_nom = np.zeros((cfg.horizon, 3), dtype=np.float32)
        self.rng = np.random.default_rng()
        self.initialized = False
        self.last_ess = 0.0
        self.last_temperature = cfg.lam

    def reset(self):
        self.u_nom[:] = 0.0
        self.initialized = False

    def set_nominal_towards_goal(self, x: float, y: float, goal_x: float, goal_y: float):
        # 이후에는 직전 최적해를 한 스텝 이동한 값이 warm start가 된다.
        if self.initialized:
            return
        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy) + 1e-6

        vx = self.cfg.v_nom * dx / dist
        vy = self.cfg.v_nom * dy / dist
        vx = clamp(vx, -self.cfg.v_max, self.cfg.v_max)
        vy = clamp(vy, -self.cfg.v_max, self.cfg.v_max)

        self.u_nom[:, 0] = vx
        self.u_nom[:, 1] = vy
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

    def step(
        self,
        state: Tuple[float, float, float],
        goal: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        cfg = self.cfg
        x0, y0, yaw0 = state
        gx, gy, gyaw = goal

        H = cfg.horizon
        N = cfg.num_samples
        dt = cfg.dt
        previous_nominal = self.u_nom.copy()

        noise = np.zeros((N, H, 3), dtype=np.float32)
        noise[:, :, 0] = self.rng.normal(0.0, cfg.sigma_v, size=(N, H))
        noise[:, :, 1] = self.rng.normal(0.0, cfg.sigma_v, size=(N, H))
        noise[:, :, 2] = self.rng.normal(0.0, cfg.sigma_yaw_rate, size=(N, H))

        # 시간축 상관 노이즈로 회피 방향이 유지되는 궤적을 생성한다.
        correlation = 0.98
        innovation_scale = math.sqrt(1.0 - correlation ** 2)
        for k in range(1, H):
            noise[:, k, :] = (
                correlation * noise[:, k - 1, :]
                + innovation_scale * noise[:, k, :]
            )

        # 좌/우 샘플 불균형을 줄인다.
        half = N // 2
        noise[half:2 * half] = -noise[:half]

        u = previous_nominal[None, :, :] + noise

        u[:, :, 0:2] = np.clip(u[:, :, 0:2], -cfg.v_max, cfg.v_max)
        planar_speed = np.linalg.norm(u[:, :, 0:2], axis=2)
        speed_scale = np.minimum(1.0, cfg.v_max / np.maximum(planar_speed, 1e-6))
        u[:, :, 0] *= speed_scale
        u[:, :, 1] *= speed_scale
        u[:, :, 2] = np.clip(u[:, :, 2], -cfg.yaw_rate_max, cfg.yaw_rate_max)

        xs = np.zeros((N, H + 1), dtype=np.float32)
        ys = np.zeros((N, H + 1), dtype=np.float32)
        yaws = np.zeros((N, H + 1), dtype=np.float32)
        xs[:, 0] = x0
        ys[:, 0] = y0
        yaws[:, 0] = yaw0

        for k in range(H):
            xs[:, k + 1] = xs[:, k] + u[:, k, 0] * dt
            ys[:, k + 1] = ys[:, k] + u[:, k, 1] * dt
            yaws[:, k + 1] = yaws[:, k] + u[:, k, 2] * dt

        costs = np.zeros((N,), dtype=np.float32)

        dx = xs[:, 1:] - gx
        dy = ys[:, 1:] - gy
        dist2 = dx * dx + dy * dy
        costs += cfg.w_goal * np.mean(dist2, axis=1)

        dxf = xs[:, -1] - gx
        dyf = ys[:, -1] - gy
        costs += cfg.w_goal_final * (dxf * dxf + dyf * dyf)

        goal_dx = gx - x0
        goal_dy = gy - y0
        goal_norm = math.hypot(goal_dx, goal_dy) + 1e-6
        direction_x = goal_dx / goal_norm
        direction_y = goal_dy / goal_norm
        progress = direction_x * (xs[:, -1] - x0) + direction_y * (ys[:, -1] - y0)
        costs -= 80.0 * progress

        dyaw = wrap_pi(yaws[:, -1] - gyaw)
        costs += 0.5 * (dyaw * dyaw)

        if self.obstacles:
            obst_cost = np.zeros((N,), dtype=np.float32)

            for obstacle in self.obstacles:
                signed = self._rectangle_signed_distance(xs[:, 1:], ys[:, 1:], obstacle)
                clearance = np.min(signed, axis=1) - cfg.safety_margin
                penetration = np.clip(-clearance, 0.0, None)
                near = np.clip(cfg.near_buffer - clearance, 0.0, cfg.near_buffer)
                obst_cost += (
                    cfg.penetrate_bias * (penetration > 0.0)
                    + cfg.penetrate_k * penetration ** 2
                    + cfg.near_k * near ** 2
                )

            costs += cfg.w_obst * obst_cost

        deviation = u - previous_nominal[None, :, :]
        costs += 1.0 * np.mean(
            deviation[:, :, 0] ** 2 + deviation[:, :, 1] ** 2,
            axis=1,
        )

        costs += cfg.w_ctrl * np.mean(
            u[:, :, 0] ** 2 + u[:, :, 1] ** 2 + 0.4 * u[:, :, 2] ** 2,
            axis=1,
        )

        du = u[:, 1:, :] - u[:, :-1, :]
        costs += cfg.w_smooth * np.mean(
            du[:, :, 0] ** 2 + du[:, :, 1] ** 2 + 0.4 * du[:, :, 2] ** 2,
            axis=1,
        )

        # 비용 스케일이 LiDAR 군집 수에 따라 바뀌어도 weight collapse가
        # 발생하지 않도록 목표 ESS에 맞춰 temperature를 조정한다.
        cmin = float(np.min(costs))
        shifted = costs - cmin
        target_ess = max(8.0, 0.10 * N)
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

        self.last_temperature = high
        weights = np.exp(-shifted / high)
        weights /= float(np.sum(weights)) + 1e-12
        self.last_ess = float(1.0 / (np.sum(weights * weights) + 1e-12))
        weights = weights.astype(np.float32)

        self.u_nom = np.tensordot(weights, u, axes=(0, 0)).astype(np.float32)

        u0 = self.u_nom[0].copy()
        self.u_nom[:-1] = self.u_nom[1:]
        self.u_nom[-1] = self.u_nom[-2]

        return float(u0[0]), float(u0[1]), float(u0[2])


class MPPIOffboardNode(Node):
    def __init__(self):
        super().__init__("mppi_lidar")

        self.declare_parameter("takeoff_z", 3.0)
        self.declare_parameter("goal_x", 140.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 3.0)
        self.declare_parameter("goal_yaw", 0.0)

        self.declare_parameter("hover_sec_after_takeoff", 2.0)
        self.declare_parameter("hover_sec_at_goal", 3.0)
        self.declare_parameter("goal_tol_xy", 0.6)

        self.declare_parameter("dt", 0.05)
        self.declare_parameter("horizon", 100)
        self.declare_parameter("num_samples", 400)
        self.declare_parameter("lam", 1.0)

        self.declare_parameter("v_max", 2.0)
        self.declare_parameter("yaw_rate_max", 1.2)
        self.declare_parameter("sigma_v", 0.7)
        self.declare_parameter("sigma_yaw_rate", 0.6)
        self.declare_parameter("v_nom", 1.2)

        self.declare_parameter("safety_margin", 0.9)
        self.declare_parameter("near_buffer", 1.0)
        self.declare_parameter("w_obst", 200.0)
        self.declare_parameter("w_goal", 8.0)
        self.declare_parameter("w_goal_final", 30.0)
        self.declare_parameter("w_ctrl", 0.2)
        self.declare_parameter("w_smooth", 0.4)
        self.declare_parameter("near_k", 18.0)
        self.declare_parameter("penetrate_k", 80.0)
        self.declare_parameter("penetrate_bias", 200.0)

        self.declare_parameter("kp_z", 1.2)
        self.declare_parameter("vz_max", 1.2)
        self.declare_parameter("land_descent_speed", 0.45)
        self.declare_parameter("land_final_speed", 0.15)
        self.declare_parameter("disarm_height", 0.18)

        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("startup_wait_sec", 3.0)

        self.declare_parameter("use_lidar_obstacles", True)
        self.declare_parameter("scan_topic", "/drone1/scan")
        self.declare_parameter("scan_valid_min_range", 0.5)
        self.declare_parameter("scan_valid_max_range", 8.0)
        self.declare_parameter("scan_downsample", 2)
        self.declare_parameter("cluster_dist_thresh", 0.8)
        self.declare_parameter("cluster_min_points", 3)
        self.declare_parameter("dynamic_obs_max", 8)
        self.declare_parameter("dynamic_obs_radius_pad", 0.25)

        self.declare_parameter("lidar_forward_only", False)
        self.declare_parameter("lidar_fov_deg", 220.0)
        self.declare_parameter("scan_timeout_sec", 0.5)

        self.declare_parameter("lidar_offset_x", 0.0)
        self.declare_parameter("lidar_offset_y", 0.0)
        self.declare_parameter("lidar_yaw_offset", 0.0)

        self.declare_parameter("emergency_stop_dist", 0.6)
        self.declare_parameter("obstacle_slowdown_dist", 2.5)
        self.declare_parameter("max_planar_accel", 1.5)

        self.current_state = State()
        self.pose: Optional[PoseStamped] = None
        self.last_pose_t = 0.0

        self.phase = "WAIT_STREAM"
        self.phase_t0 = time.time()
        self.pre_stream_count = 0
        self.boot_t0 = time.time()

        self.dynamic_obstacles: List[RectangleObstacle2D] = []
        self.last_scan_t = 0.0
        self.last_scan_min = math.inf
        self.last_planar_cmd = np.zeros(2, dtype=np.float64)

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )

        scan_topic = str(self.get_parameter("scan_topic").value)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            10,
        )
        self.phase_pub = self.create_publisher(String, "/drone1/mission/phase", 10)
        self.goal_pub = self.create_publisher(Bool, "/drone1/mission/goal_reached", 10)

        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.mode_future = None
        self.arm_future = None
        self.last_mode_req_t = 0.0
        self.last_arm_req_t = 0.0

        cfg = MPPIConfig(
            dt=float(self.get_parameter("dt").value),
            horizon=int(self.get_parameter("horizon").value),
            num_samples=int(self.get_parameter("num_samples").value),
            lam=float(self.get_parameter("lam").value),
            v_max=float(self.get_parameter("v_max").value),
            yaw_rate_max=float(self.get_parameter("yaw_rate_max").value),
            sigma_v=float(self.get_parameter("sigma_v").value),
            sigma_yaw_rate=float(self.get_parameter("sigma_yaw_rate").value),
            v_nom=float(self.get_parameter("v_nom").value),
            safety_margin=float(self.get_parameter("safety_margin").value),
            near_buffer=float(self.get_parameter("near_buffer").value),
            w_obst=float(self.get_parameter("w_obst").value),
            w_goal=float(self.get_parameter("w_goal").value),
            w_goal_final=float(self.get_parameter("w_goal_final").value),
            w_ctrl=float(self.get_parameter("w_ctrl").value),
            w_smooth=float(self.get_parameter("w_smooth").value),
            near_k=float(self.get_parameter("near_k").value),
            penetrate_k=float(self.get_parameter("penetrate_k").value),
            penetrate_bias=float(self.get_parameter("penetrate_bias").value),
        )
        self.mppi = MPPIController(cfg, [])

        rate = float(self.get_parameter("cmd_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self.get_logger().info("static obstacles loaded: [] (LiDAR-only mode)")
        self.get_logger().info(
            f"mppi cfg: dt={cfg.dt}, horizon={cfg.horizon}, T={cfg.dt * cfg.horizon:.2f}s, "
            f"w_obst={cfg.w_obst}, safety_margin={cfg.safety_margin}, near_buffer={cfg.near_buffer}"
        )
        self.get_logger().info(
            f"lidar obstacle mode: {bool(self.get_parameter('use_lidar_obstacles').value)}, "
            f"scan_topic={scan_topic}"
        )

    def _on_state(self, msg: State):
        self.current_state = msg

    def _on_pose(self, msg: PoseStamped):
        self.pose = msg
        self.last_pose_t = time.time()

    def _pose_age(self) -> float:
        if self.pose is None:
            return 1e9
        return time.time() - self.last_pose_t

    def _scan_age(self) -> float:
        if self.last_scan_t <= 0.0:
            return 1e9
        return time.time() - self.last_scan_t

    def _get_xyz_yaw(self) -> Tuple[float, float, float, float]:
        p = self.pose.pose.position
        q = self.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return float(p.x), float(p.y), float(p.z), float(yaw)

    def _publish_cmd(self, vx: float, vy: float, vz: float, yaw_rate: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(yaw_rate)
        self.cmd_pub.publish(msg)

    def _enter_phase(self, name: str):
        if self.phase != name:
            self.phase = name
            self.phase_t0 = time.time()
            self.get_logger().info(f"PHASE => {name}")

    def _publish_mission_state(self):
        phase = String()
        phase.data = self.phase
        self.phase_pub.publish(phase)
        reached = Bool()
        reached.data = self.phase in {"HOVER_AT_GOAL", "LAND", "DONE"}
        self.goal_pub.publish(reached)

    def _phase_elapsed(self) -> float:
        return time.time() - self.phase_t0

    def _request_set_mode(self, mode: str):
        if not self.mode_cli.service_is_ready():
            return
        if self.mode_future is not None and not self.mode_future.done():
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_future = self.mode_cli.call_async(req)

    def _request_arm(self, arm: bool):
        if not self.arm_cli.service_is_ready():
            return
        if self.arm_future is not None and not self.arm_future.done():
            return
        req = CommandBool.Request()
        req.value = bool(arm)
        self.arm_future = self.arm_cli.call_async(req)

    def _services_ready(self) -> bool:
        return self.mode_cli.service_is_ready() and self.arm_cli.service_is_ready()

    def _body_to_world_2d(
        self,
        xb: float,
        yb: float,
        x: float,
        y: float,
        yaw: float,
    ) -> Tuple[float, float]:
        c = math.cos(yaw)
        s = math.sin(yaw)
        xw = x + c * xb - s * yb
        yw = y + s * xb + c * yb
        return xw, yw

    def _cluster_points(self, pts: List[Tuple[float, float]]) -> List[List[Tuple[float, float]]]:
        if not pts:
            return []

        dist_thresh = float(self.get_parameter("cluster_dist_thresh").value)
        min_points = int(self.get_parameter("cluster_min_points").value)

        clusters: List[List[Tuple[float, float]]] = []
        cur = [pts[0]]

        for i in range(1, len(pts)):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            if math.hypot(x1 - x0, y1 - y0) <= dist_thresh:
                cur.append(pts[i])
            else:
                if len(cur) >= min_points:
                    clusters.append(cur)
                cur = [pts[i]]

        if len(cur) >= min_points:
            clusters.append(cur)

        return clusters

    def _clusters_to_obstacles(
        self,
        clusters: List[List[Tuple[float, float]]],
        drone_x: float,
        drone_y: float,
    ) -> List[RectangleObstacle2D]:
        obs_list: List[RectangleObstacle2D] = []
        geometry_pad = float(self.get_parameter("dynamic_obs_radius_pad").value)

        for cl in clusters:
            points = np.asarray(cl, dtype=np.float64)
            mean = np.mean(points, axis=0)
            centered = points - mean

            # PCA의 주축으로 LiDAR wall segment의 방향을 추정한다.
            covariance = centered.T @ centered / max(len(cl), 1)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            major = eigenvectors[:, int(np.argmax(eigenvalues))]
            minor = np.array([-major[1], major[0]], dtype=np.float64)

            major_projection = centered @ major
            minor_projection = centered @ minor
            major_min, major_max = np.min(major_projection), np.max(major_projection)
            minor_min, minor_max = np.min(minor_projection), np.max(minor_projection)

            local_center_major = 0.5 * (major_min + major_max)
            local_center_minor = 0.5 * (minor_min + minor_max)
            center = mean + local_center_major * major + local_center_minor * minor

            obs_list.append(RectangleObstacle2D(
                x=float(center[0]),
                y=float(center[1]),
                half_x=float(0.5 * (major_max - major_min) + geometry_pad),
                half_y=float(0.5 * (minor_max - minor_min) + geometry_pad),
                yaw=float(math.atan2(major[1], major[0])),
            ))

        max_obs = int(self.get_parameter("dynamic_obs_max").value)
        obs_list.sort(key=lambda o: math.hypot(o.x - drone_x, o.y - drone_y))
        return obs_list[:max_obs]

    def _update_mppi_obstacles(self):
        use_lidar = bool(self.get_parameter("use_lidar_obstacles").value)
        if use_lidar:
            self.mppi.obstacles = list(self.dynamic_obstacles)
        else:
            self.mppi.obstacles = []

    def _on_scan(self, msg: LaserScan):
        use_lidar = bool(self.get_parameter("use_lidar_obstacles").value)
        if not use_lidar:
            return
        if self.pose is None:
            return

        x, y, z, yaw = self._get_xyz_yaw()

        valid_min = float(self.get_parameter("scan_valid_min_range").value)
        valid_max = float(self.get_parameter("scan_valid_max_range").value)
        downsample = max(1, int(self.get_parameter("scan_downsample").value))
        lidar_forward_only = bool(self.get_parameter("lidar_forward_only").value)
        lidar_fov_deg = float(self.get_parameter("lidar_fov_deg").value)

        off_x = float(self.get_parameter("lidar_offset_x").value)
        off_y = float(self.get_parameter("lidar_offset_y").value)
        off_yaw = float(self.get_parameter("lidar_yaw_offset").value)

        half_fov = math.radians(lidar_fov_deg * 0.5)

        pts_world: List[Tuple[float, float]] = []
        valid_ranges: List[float] = []

        for i in range(0, len(msg.ranges), downsample):
            r = float(msg.ranges[i])

            if not math.isfinite(r):
                continue
            if r < max(msg.range_min, valid_min) or r > min(msg.range_max, valid_max):
                continue

            ang = msg.angle_min + i * msg.angle_increment

            if lidar_forward_only and abs(ang) > half_fov:
                continue

            valid_ranges.append(r)

            ang_b = ang + off_yaw
            xb = off_x + r * math.cos(ang_b)
            yb = off_y + r * math.sin(ang_b)

            xw, yw = self._body_to_world_2d(xb, yb, x, y, yaw)
            pts_world.append((xw, yw))

        if not pts_world:
            self.dynamic_obstacles = []
            self.last_scan_min = math.inf
            self.last_scan_t = time.time()
            return

        clusters = self._cluster_points(pts_world)
        self.dynamic_obstacles = self._clusters_to_obstacles(clusters, x, y)
        self.last_scan_min = min(valid_ranges) if valid_ranges else math.inf
        self.last_scan_t = time.time()

    def _tick(self):
        self._publish_mission_state()
        if not self.current_state.connected:
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            return
        if self.pose is None or self._pose_age() > 0.5:
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            return

        startup_wait_sec = float(self.get_parameter("startup_wait_sec").value)
        if (time.time() - self.boot_t0) < startup_wait_sec:
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            return
        if not self._services_ready():
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            return

        takeoff_z = float(self.get_parameter("takeoff_z").value)
        gx = float(self.get_parameter("goal_x").value)
        gy = float(self.get_parameter("goal_y").value)
        gz = float(self.get_parameter("goal_z").value)
        gyaw = float(self.get_parameter("goal_yaw").value)

        hover_after_takeoff = float(self.get_parameter("hover_sec_after_takeoff").value)
        hover_at_goal = float(self.get_parameter("hover_sec_at_goal").value)
        goal_tol = float(self.get_parameter("goal_tol_xy").value)

        kp_z = float(self.get_parameter("kp_z").value)
        vz_max = float(self.get_parameter("vz_max").value)

        x, y, z, yaw = self._get_xyz_yaw()

        if self.phase == "WAIT_STREAM":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.pre_stream_count += 1
            if self.pre_stream_count >= 40:
                self._enter_phase("OFFBOARD_ARM")

        elif self.phase == "OFFBOARD_ARM":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            now = time.time()

            if self.current_state.mode != "OFFBOARD":
                if (now - self.last_mode_req_t) > 1.0:
                    self._request_set_mode("OFFBOARD")
                    self.last_mode_req_t = now
                return

            if not self.current_state.armed:
                if (now - self.last_arm_req_t) > 1.0:
                    self._request_arm(True)
                    self.last_arm_req_t = now

                if self.arm_future is not None and self.arm_future.done():
                    res = self.arm_future.result()
                    if res is not None and bool(res.success):
                        self.mppi.reset()
                        self._enter_phase("TAKEOFF")
                return

            self.mppi.reset()
            self._enter_phase("TAKEOFF")

        elif self.phase == "TAKEOFF":
            err_z = takeoff_z - z
            vz_cmd = clamp(kp_z * err_z, -vz_max, vz_max)
            vz_cmd = clamp(vz_cmd, 0.2, vz_max) if err_z > 0.2 else vz_cmd
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if z >= takeoff_z - 0.15:
                self._enter_phase("HOVER_AFTER_TAKEOFF")

        elif self.phase == "HOVER_AFTER_TAKEOFF":
            err_z = takeoff_z - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if self._phase_elapsed() >= hover_after_takeoff:
                use_lidar = bool(self.get_parameter("use_lidar_obstacles").value)
                scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)

                if use_lidar and self._scan_age() > scan_timeout_sec:
                    return

                self.mppi.set_nominal_towards_goal(x, y, gx, gy)
                self.last_planar_cmd[:] = 0.0
                self._enter_phase("MPPI_GO")

        elif self.phase == "MPPI_GO":
            err_z = gz - z
            vz_hold = clamp(kp_z * err_z, -vz_max, vz_max)

            self._update_mppi_obstacles()

            scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)
            use_lidar = bool(self.get_parameter("use_lidar_obstacles").value)
            scan_timed_out = use_lidar and (self._scan_age() > scan_timeout_sec)

            if scan_timed_out:
                self.dynamic_obstacles = []
                self._update_mppi_obstacles()
                self._publish_cmd(0.0, 0.0, vz_hold, 0.0)
                return

            emergency_stop_dist = float(self.get_parameter("emergency_stop_dist").value)
            slowdown_dist = float(self.get_parameter("obstacle_slowdown_dist").value)

            vx, vy, yr = self.mppi.step(state=(x, y, yaw), goal=(gx, gy, gyaw))

            # 가까운 벽에서 모든 수평 자유도를 제거하면 gap 쪽으로 미세 조정할
            # 수 없다. 실제 scan 거리에 따라 연속 감속하되 MPPI의 측방 회피는
            # 유지하고 최소 이동 비율을 남겨 gap 중심으로 수렴하게 한다.
            if self.last_scan_min < slowdown_dist:
                obstacle_scale = clamp(
                    (self.last_scan_min - emergency_stop_dist)
                    / max(slowdown_dist - emergency_stop_dist, 1e-6),
                    0.15,
                    1.0,
                )
                vx *= obstacle_scale
                vy *= obstacle_scale
            yr = 0.0

            d_goal = math.hypot(gx - x, gy - y)
            if d_goal < 2.0:
                scale = clamp(d_goal / 2.0, 0.25, 1.0)
                vx *= scale
                vy *= scale

            max_accel = float(self.get_parameter("max_planar_accel").value)
            cmd = np.array([vx, vy], dtype=np.float64)
            delta = cmd - self.last_planar_cmd
            max_delta = max_accel / max(float(self.get_parameter("cmd_rate_hz").value), 1.0)
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > max_delta > 0.0:
                delta *= max_delta / delta_norm
            self.last_planar_cmd += delta
            vx, vy = float(self.last_planar_cmd[0]), float(self.last_planar_cmd[1])

            self._publish_cmd(vx, vy, vz_hold, yr)

            if d_goal <= goal_tol:
                self.last_planar_cmd[:] = 0.0
                self._enter_phase("HOVER_AT_GOAL")

        elif self.phase == "HOVER_AT_GOAL":
            err_z = gz - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if self._phase_elapsed() >= hover_at_goal:
                self._enter_phase("LAND")

        elif self.phase == "LAND":
            descent_speed = abs(float(self.get_parameter("land_descent_speed").value))
            disarm_height = float(self.get_parameter("disarm_height").value)
            self._publish_cmd(0.0, 0.0, -descent_speed, 0.0)
            if z <= max(0.5, disarm_height + 0.2):
                self._enter_phase("WAIT_LANDED")

        elif self.phase == "WAIT_LANDED":
            final_speed = abs(float(self.get_parameter("land_final_speed").value))
            disarm_height = float(self.get_parameter("disarm_height").value)
            if z >= disarm_height:
                self._publish_cmd(0.0, 0.0, -final_speed, 0.0)
            else:
                self._publish_cmd(0.0, 0.0, 0.0, 0.0)
                now = time.time()

                # AUTO.LAND는 고도에서 전환하면 수평 드리프트가 발생할 수 있다.
                # 지면 접촉 후에만 전환해 PX4 land detector의 자동 disarm을
                # 활성화한다.
                if self.current_state.mode != "AUTO.LAND":
                    if (now - self.last_mode_req_t) > 1.0:
                        self._request_set_mode("AUTO.LAND")
                        self.last_mode_req_t = now
                    return

                if not self.current_state.armed:
                    self._enter_phase("DONE")
                    return

                if (now - self.last_arm_req_t) > 1.0:
                    self._request_arm(False)
                    self.last_arm_req_t = now

                if self.arm_future is not None and self.arm_future.done():
                    res = self.arm_future.result()
                    if res is not None and bool(res.success):
                        self._enter_phase("DONE")

        elif self.phase == "DONE":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MPPIOffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
