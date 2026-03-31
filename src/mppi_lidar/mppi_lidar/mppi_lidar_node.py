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
class Obstacle2D:
    cx: float
    cy: float
    r: float


@dataclass
class MPPIConfig:
    dt: float = 0.05
    horizon: int = 100
    num_samples: int = 400
    lam: float = 1.0

    v_max: float = 2.0
    yaw_rate_max: float = 1.2

    sigma_v: float = 0.6
    sigma_yaw_rate: float = 0.6

    w_goal: float = 8.0
    w_goal_final: float = 30.0

    w_obst: float = 120.0
    w_ctrl: float = 0.2
    w_smooth: float = 0.4

    safety_margin: float = 0.9
    near_buffer: float = 0.6

    near_k: float = 18.0
    penetrate_k: float = 80.0
    penetrate_bias: float = 200.0

    v_nom: float = 1.2


class MPPIController:
    def __init__(self, cfg: MPPIConfig, obstacles: List[Obstacle2D]):
        self.cfg = cfg
        self.obstacles = obstacles
        self.u_nom = np.zeros((cfg.horizon, 3), dtype=np.float32)
        self.rng = np.random.default_rng()

    def reset(self):
        self.u_nom[:] = 0.0

    def set_nominal_towards_goal(self, x: float, y: float, goal_x: float, goal_y: float):
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

        noise = np.zeros((N, H, 3), dtype=np.float32)
        noise[:, :, 0] = self.rng.normal(0.0, cfg.sigma_v, size=(N, H))
        noise[:, :, 1] = self.rng.normal(0.0, cfg.sigma_v, size=(N, H))
        noise[:, :, 2] = self.rng.normal(0.0, cfg.sigma_yaw_rate, size=(N, H))

        u = self.u_nom[None, :, :] + noise

        u[:, :, 0] = np.clip(u[:, :, 0], -cfg.v_max, cfg.v_max)
        u[:, :, 1] = np.clip(u[:, :, 1], -cfg.v_max, cfg.v_max)
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

        dyaw = wrap_pi(yaws[:, -1] - gyaw)
        costs += 0.5 * (dyaw * dyaw)

        if self.obstacles:
            obst_cost = np.zeros((N,), dtype=np.float32)

            for obs in self.obstacles:
                ox = xs[:, 1:] - obs.cx
                oy = ys[:, 1:] - obs.cy
                d = np.sqrt(ox * ox + oy * oy)

                dmin = np.min(d, axis=1)
                sdmin = dmin - (obs.r + cfg.safety_margin)

                pen_depth = np.clip(-sdmin, 0.0, None)
                pen_cost = (
                    cfg.penetrate_bias * (pen_depth > 0.0).astype(np.float32)
                    + cfg.penetrate_k * (pen_depth ** 2)
                )

                near_depth = np.clip(cfg.near_buffer - sdmin, 0.0, cfg.near_buffer)
                near_cost = cfg.near_k * (near_depth ** 2)

                obst_cost += (pen_cost + near_cost)

            costs += cfg.w_obst * obst_cost

        costs += cfg.w_ctrl * np.mean(
            u[:, :, 0] ** 2 + u[:, :, 1] ** 2 + 0.4 * u[:, :, 2] ** 2,
            axis=1,
        )

        du = u[:, 1:, :] - u[:, :-1, :]
        costs += cfg.w_smooth * np.mean(
            du[:, :, 0] ** 2 + du[:, :, 1] ** 2 + 0.4 * du[:, :, 2] ** 2,
            axis=1,
        )

        cmin = float(np.min(costs))
        weights = np.exp(-(costs - cmin) / max(cfg.lam, 1e-6))
        wsum = float(np.sum(weights)) + 1e-9
        weights = (weights / wsum).astype(np.float32)

        self.u_nom = np.tensordot(weights, u, axes=(0, 0)).astype(np.float32)

        u0 = self.u_nom[0].copy()
        self.u_nom[:-1] = self.u_nom[1:]
        self.u_nom[-1] = self.u_nom[-2]

        return float(u0[0]), float(u0[1]), float(u0[2])


class MPPIOffboardNode(Node):
    def __init__(self):
        super().__init__("mppi_lidar")

        self.declare_parameter("takeoff_z", 3.0)
        self.declare_parameter("goal_x", 24.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 3.0)
        self.declare_parameter("goal_yaw", 0.0)

        self.declare_parameter("hover_sec_after_takeoff", 2.0)
        self.declare_parameter("hover_sec_at_goal", 3.0)
        self.declare_parameter("goal_tol_xy", 0.6)

        self.declare_parameter("dt", 0.05)
        self.declare_parameter("horizon", 100)
        self.declare_parameter("num_samples", 400)

        self.declare_parameter("v_max", 2.0)
        self.declare_parameter("yaw_rate_max", 1.2)

        self.declare_parameter("safety_margin", 0.9)
        self.declare_parameter("near_buffer", 0.6)
        self.declare_parameter("w_obst", 120.0)

        self.declare_parameter("kp_z", 1.2)
        self.declare_parameter("vz_max", 1.2)

        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("startup_wait_sec", 3.0)

        self.declare_parameter("use_lidar_obstacles", True)
        self.declare_parameter("scan_topic", "/drone1/scan")
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

        self.declare_parameter("emergency_stop_dist", 1.0)

        self.current_state = State()
        self.pose: Optional[PoseStamped] = None
        self.last_pose_t = 0.0

        self.phase = "WAIT_STREAM"
        self.phase_t0 = time.time()
        self.pre_stream_count = 0
        self.boot_t0 = time.time()

        self.static_obstacles: List[Obstacle2D] = []
        self.dynamic_obstacles: List[Obstacle2D] = []
        self.last_scan_t = 0.0

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
            v_max=float(self.get_parameter("v_max").value),
            yaw_rate_max=float(self.get_parameter("yaw_rate_max").value),
            safety_margin=float(self.get_parameter("safety_margin").value),
            near_buffer=float(self.get_parameter("near_buffer").value),
            w_obst=float(self.get_parameter("w_obst").value),
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
    ) -> List[Obstacle2D]:
        obs_list: List[Obstacle2D] = []
        radius_pad = float(self.get_parameter("dynamic_obs_radius_pad").value)

        for cl in clusters:
            xs = [p[0] for p in cl]
            ys = [p[1] for p in cl]

            cx = float(sum(xs) / len(xs))
            cy = float(sum(ys) / len(ys))

            r = 0.0
            for px, py in cl:
                r = max(r, math.hypot(px - cx, py - cy))
            r += radius_pad

            obs_list.append(Obstacle2D(cx, cy, r))

        max_obs = int(self.get_parameter("dynamic_obs_max").value)
        obs_list.sort(key=lambda o: math.hypot(o.cx - drone_x, o.cy - drone_y))
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

        valid_max = float(self.get_parameter("scan_valid_max_range").value)
        downsample = max(1, int(self.get_parameter("scan_downsample").value))
        lidar_forward_only = bool(self.get_parameter("lidar_forward_only").value)
        lidar_fov_deg = float(self.get_parameter("lidar_fov_deg").value)

        off_x = float(self.get_parameter("lidar_offset_x").value)
        off_y = float(self.get_parameter("lidar_offset_y").value)
        off_yaw = float(self.get_parameter("lidar_yaw_offset").value)

        half_fov = math.radians(lidar_fov_deg * 0.5)

        pts_world: List[Tuple[float, float]] = []

        for i in range(0, len(msg.ranges), downsample):
            r = float(msg.ranges[i])

            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > min(msg.range_max, valid_max):
                continue

            ang = msg.angle_min + i * msg.angle_increment

            if lidar_forward_only and abs(ang) > half_fov:
                continue

            ang_b = ang + off_yaw
            xb = off_x + r * math.cos(ang_b)
            yb = off_y + r * math.sin(ang_b)

            xw, yw = self._body_to_world_2d(xb, yb, x, y, yaw)
            pts_world.append((xw, yw))

        if not pts_world:
            self.dynamic_obstacles = []
            self.last_scan_t = time.time()
            return

        clusters = self._cluster_points(pts_world)
        self.dynamic_obstacles = self._clusters_to_obstacles(clusters, x, y)
        self.last_scan_t = time.time()

    def _tick(self):
        self._publish_cmd(0.0, 0.0, 0.0, 0.0)

        if not self.current_state.connected:
            return
        if self.pose is None or self._pose_age() > 0.5:
            return

        startup_wait_sec = float(self.get_parameter("startup_wait_sec").value)
        if (time.time() - self.boot_t0) < startup_wait_sec:
            return
        if not self._services_ready():
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
            self.pre_stream_count += 1
            if self.pre_stream_count >= 40:
                self._enter_phase("OFFBOARD_ARM")

        elif self.phase == "OFFBOARD_ARM":
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
            too_close = False
            for obs in self.dynamic_obstacles:
                d = math.hypot(obs.cx - x, obs.cy - y) - obs.r
                if d < emergency_stop_dist:
                    too_close = True
                    break

            if too_close:
                self._publish_cmd(0.0, 0.0, vz_hold, 0.0)
                return

            vx, vy, yr = self.mppi.step(state=(x, y, yaw), goal=(gx, gy, gyaw))

            d_goal = math.hypot(gx - x, gy - y)
            if d_goal < 2.0:
                scale = clamp(d_goal / 2.0, 0.25, 1.0)
                vx *= scale
                vy *= scale

            self._publish_cmd(vx, vy, vz_hold, yr)

            if d_goal <= goal_tol:
                self._enter_phase("HOVER_AT_GOAL")

        elif self.phase == "HOVER_AT_GOAL":
            err_z = gz - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if self._phase_elapsed() >= hover_at_goal:
                self._enter_phase("LAND")

        elif self.phase == "LAND":
            now = time.time()
            if (now - self.last_mode_req_t) > 1.0:
                self._request_set_mode("AUTO.LAND")
                self.last_mode_req_t = now

            if self.mode_future is not None and self.mode_future.done():
                res = self.mode_future.result()
                if res is not None and bool(res.mode_sent):
                    self._enter_phase("WAIT_LANDED")

        elif self.phase == "WAIT_LANDED":
            if z < 0.2:
                now = time.time()
                if (now - self.last_arm_req_t) > 1.0:
                    self._request_arm(False)
                    self.last_arm_req_t = now

                if self.arm_future is not None and self.arm_future.done():
                    res = self.arm_future.result()
                    if res is not None and bool(res.success):
                        self._enter_phase("DONE")

        elif self.phase == "DONE":
            pass


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