#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def xml_tag_name(elem: ET.Element) -> str:
    return elem.tag.split('}', 1)[-1] if '}' in elem.tag else elem.tag


def child_text(elem: ET.Element, child_name: str, default: str = '') -> str:
    for child in list(elem):
        if xml_tag_name(child) == child_name:
            return (child.text or '').strip()
    return default


def first_child(elem: ET.Element, child_name: str) -> Optional[ET.Element]:
    for child in list(elem):
        if xml_tag_name(child) == child_name:
            return child
    return None


def parse_pose_xy(text: str) -> Tuple[float, float]:
    vals = [float(v) for v in text.split()]
    if len(vals) < 2:
        return 0.0, 0.0
    return vals[0], vals[1]


@dataclass
class Obstacle2D:
    cx: float
    cy: float
    r: float


class WorldObstacleLoader:
    def __init__(
        self,
        world_path: str,
        cylinder_model_dir: str,
        cylinder_model_name: str = 'cylinder_r05_h5',
        default_radius: float = 0.5,
        include_name_keywords: Optional[List[str]] = None,
        ignore_within_radius: float = 0.0,
        ignore_center_x: float = 0.0,
        ignore_center_y: float = 0.0,
    ):
        self.world_path = Path(world_path)
        self.cylinder_model_dir = Path(cylinder_model_dir)
        self.cylinder_model_name = cylinder_model_name
        self.default_radius = float(default_radius)
        self.include_name_keywords = include_name_keywords or ['obstacle', 'cylinder', cylinder_model_name]
        self.ignore_within_radius = float(ignore_within_radius)
        self.ignore_center_x = float(ignore_center_x)
        self.ignore_center_y = float(ignore_center_y)

    def load(self) -> List[Obstacle2D]:
        if not self.world_path.exists():
            raise FileNotFoundError(f'world file not found: {self.world_path}')

        root = ET.parse(str(self.world_path)).getroot()
        model_radius = self._load_cylinder_radius_from_model_sdf()
        obstacles: List[Obstacle2D] = []
        seen = set()

        for include in root.iter():
            if xml_tag_name(include) != 'include':
                continue

            uri = child_text(include, 'uri')
            name = child_text(include, 'name')
            pose_text = child_text(include, 'pose', '0 0 0 0 0 0')

            if not self._is_obstacle_include(uri, name):
                continue

            cx, cy = parse_pose_xy(pose_text)
            r = model_radius
            key = (round(cx, 6), round(cy, 6), round(r, 6), name, uri)
            if key in seen:
                continue
            seen.add(key)

            if not self._is_ignored_near_start(cx, cy):
                obstacles.append(Obstacle2D(cx=cx, cy=cy, r=r))

        for model in root.iter():
            if xml_tag_name(model) != 'model':
                continue

            name = model.attrib.get('name', '')
            if name in {'ground_plane', 'sun'}:
                continue

            pose_text = child_text(model, 'pose', '0 0 0 0 0 0')
            cx, cy = parse_pose_xy(pose_text)
            radius = self._radius_from_inline_model(model)
            if radius is None:
                continue

            if not self._is_obstacle_model_name(name) and self.cylinder_model_name not in name:
                continue

            key = (round(cx, 6), round(cy, 6), round(radius, 6), name, 'inline')
            if key in seen:
                continue
            seen.add(key)

            if not self._is_ignored_near_start(cx, cy):
                obstacles.append(Obstacle2D(cx=cx, cy=cy, r=radius))

        return obstacles

    def _load_cylinder_radius_from_model_sdf(self) -> float:
        model_sdf = self.cylinder_model_dir / 'model.sdf'
        if not model_sdf.exists():
            return self.default_radius

        try:
            root = ET.parse(str(model_sdf)).getroot()
            for elem in root.iter():
                if xml_tag_name(elem) == 'radius' and elem.text is not None:
                    return float(elem.text.strip())
        except Exception:
            return self.default_radius

        return self.default_radius

    def _is_obstacle_include(self, uri: str, name: str) -> bool:
        text = f'{uri} {name}'.lower()
        if self.cylinder_model_name.lower() in text:
            return True
        return any(k.lower() in text for k in self.include_name_keywords)

    def _is_obstacle_model_name(self, name: str) -> bool:
        n = name.lower()
        return any(k.lower() in n for k in self.include_name_keywords)

    def _radius_from_inline_model(self, model: ET.Element) -> Optional[float]:
        max_radius: Optional[float] = None

        for geom in model.iter():
            if xml_tag_name(geom) != 'geometry':
                continue

            cylinder = first_child(geom, 'cylinder')
            if cylinder is not None:
                radius_text = child_text(cylinder, 'radius')
                if radius_text:
                    r = float(radius_text)
                    max_radius = r if max_radius is None else max(max_radius, r)

            box = first_child(geom, 'box')
            if box is not None:
                size_text = child_text(box, 'size')
                if size_text:
                    vals = [float(v) for v in size_text.split()]
                    if len(vals) >= 2:
                        r = math.hypot(vals[0] * 0.5, vals[1] * 0.5)
                        max_radius = r if max_radius is None else max(max_radius, r)

        return max_radius

    def _is_ignored_near_start(self, cx: float, cy: float) -> bool:
        if self.ignore_within_radius <= 0.0:
            return False
        return math.hypot(cx - self.ignore_center_x, cy - self.ignore_center_y) <= self.ignore_within_radius


@dataclass
class MPPIConfig:
    dt: float = 0.05
    horizon: int = 50
    num_samples: int = 120
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
    nominal_update_alpha: float = 0.35
    repulsion_range: float = 5.0
    repulsion_gain: float = 1.0
    repulsion_max: float = 0.8


class MPPIController:
    def __init__(self, cfg: MPPIConfig, obstacles: List[Obstacle2D]):
        self.cfg = cfg
        self.obstacles = obstacles
        self.u_nom = np.zeros((cfg.horizon, 3), dtype=np.float32)
        self.rng = np.random.default_rng()

    def reset(self) -> None:
        self.u_nom[:] = 0.0

    def _repulsion_velocity(self, x: float, y: float) -> Tuple[float, float]:
        if not self.obstacles or self.cfg.repulsion_range <= 0.0:
            return 0.0, 0.0

        rx = 0.0
        ry = 0.0
        for obs in self.obstacles:
            dx = x - obs.cx
            dy = y - obs.cy
            center_dist = math.hypot(dx, dy)
            if center_dist < 1e-6:
                continue

            surface_dist = center_dist - obs.r
            if surface_dist >= self.cfg.repulsion_range:
                continue

            clearance = max(surface_dist, 0.05)
            activation = (self.cfg.repulsion_range - clearance) / self.cfg.repulsion_range
            strength = self.cfg.repulsion_gain * activation * activation
            rx += strength * dx / center_dist
            ry += strength * dy / center_dist

        mag = math.hypot(rx, ry)
        if mag > self.cfg.repulsion_max > 0.0:
            scale = self.cfg.repulsion_max / mag
            rx *= scale
            ry *= scale
        return rx, ry

    def set_nominal_towards_goal(
        self,
        x: float,
        y: float,
        goal_x: float,
        goal_y: float,
        alpha: float = 1.0,
    ) -> None:
        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy) + 1e-6
        vx = self.cfg.v_nom * dx / dist
        vy = self.cfg.v_nom * dy / dist
        rx, ry = self._repulsion_velocity(x, y)
        vx += rx
        vy += ry

        speed = math.hypot(vx, vy)
        if speed > self.cfg.v_max:
            scale = self.cfg.v_max / speed
            vx *= scale
            vy *= scale

        desired = np.zeros_like(self.u_nom)
        desired[:, 0] = float(vx)
        desired[:, 1] = float(vy)
        alpha = clamp(float(alpha), 0.0, 1.0)
        self.u_nom = ((1.0 - alpha) * self.u_nom + alpha * desired).astype(np.float32)

    def step(self, state: Tuple[float, float, float], goal: Tuple[float, float, float]) -> Tuple[float, float, float]:
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
        costs += cfg.w_goal * np.mean(dx * dx + dy * dy, axis=1)
        costs += cfg.w_goal_final * ((xs[:, -1] - gx) ** 2 + (ys[:, -1] - gy) ** 2)
        dyaw = (yaws[:, -1] - gyaw + np.pi) % (2.0 * np.pi) - np.pi
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
                pen_cost = cfg.penetrate_bias * (pen_depth > 0.0).astype(np.float32) + cfg.penetrate_k * (pen_depth ** 2)
                near_depth = np.clip(cfg.near_buffer - sdmin, 0.0, cfg.near_buffer)
                near_cost = cfg.near_k * (near_depth ** 2)
                obst_cost += pen_cost + near_cost
            costs += cfg.w_obst * obst_cost

        costs += cfg.w_ctrl * np.mean(u[:, :, 0] ** 2 + u[:, :, 1] ** 2 + 0.4 * u[:, :, 2] ** 2, axis=1)
        du = u[:, 1:, :] - u[:, :-1, :]
        costs += cfg.w_smooth * np.mean(du[:, :, 0] ** 2 + du[:, :, 1] ** 2 + 0.4 * du[:, :, 2] ** 2, axis=1)

        cmin = float(np.min(costs))
        weights = np.exp(-(costs - cmin) / max(cfg.lam, 1e-6))
        weights = (weights / (float(np.sum(weights)) + 1e-9)).astype(np.float32)
        self.u_nom = np.tensordot(weights, u, axes=(0, 0)).astype(np.float32)

        u0 = self.u_nom[0].copy()
        self.u_nom[:-1] = self.u_nom[1:]
        self.u_nom[-1] = self.u_nom[-2]
        return float(u0[0]), float(u0[1]), float(u0[2])


class MPPIPlannerNode(Node):
    def __init__(self):
        super().__init__('mppi')
        self.declare_parameter('pose_topic', '/local_position/pose')
        self.declare_parameter('cmd_topic', '/drone1/autonomy/cmd_vel')
        self.declare_parameter('goal_reached_topic', '/drone1/mission/goal_reached')
        self.declare_parameter('goal_x', 50.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 3.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('goal_tol_xy', 0.8)
        self.declare_parameter('world_path', '/workspace/AV_Drone/sim_assets/worlds/obstacle_demo.world')
        self.declare_parameter('cylinder_model_dir', '/workspace/AV_Drone/sim_assets/models/cylinder_r05_h5')
        self.declare_parameter('cylinder_model_name', 'cylinder_r05_h5')
        self.declare_parameter('obstacle_default_radius', 0.5)
        self.declare_parameter('ignore_obstacles_within_start_radius', 0.0)
        self.declare_parameter('ignore_obstacles_center_x', 0.0)
        self.declare_parameter('ignore_obstacles_center_y', 0.0)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('horizon', 50)
        self.declare_parameter('num_samples', 120)
        self.declare_parameter('lam', 1.0)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('yaw_rate_max', 1.2)
        self.declare_parameter('sigma_v', 0.6)
        self.declare_parameter('sigma_yaw_rate', 0.6)
        self.declare_parameter('v_nom', 1.2)
        self.declare_parameter('w_goal', 8.0)
        self.declare_parameter('w_goal_final', 30.0)
        self.declare_parameter('w_obst', 120.0)
        self.declare_parameter('w_ctrl', 0.2)
        self.declare_parameter('w_smooth', 0.4)
        self.declare_parameter('safety_margin', 0.9)
        self.declare_parameter('near_buffer', 0.6)
        self.declare_parameter('near_k', 18.0)
        self.declare_parameter('penetrate_k', 80.0)
        self.declare_parameter('penetrate_bias', 200.0)
        self.declare_parameter('nominal_update_alpha', 0.35)
        self.declare_parameter('repulsion_range', 5.0)
        self.declare_parameter('repulsion_gain', 1.0)
        self.declare_parameter('repulsion_max', 0.8)
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('slowdown_dist', 3.0)
        self.declare_parameter('min_goal_scale', 0.25)
        self.declare_parameter('log_cmd_period_sec', 1.0)

        self.pose: Optional[PoseStamped] = None
        self.last_pose_t = 0.0
        self.goal_reached = False
        self.nominal_initialized = False
        self.last_cmd_log_t = 0.0

        pose_topic = str(self.get_parameter('pose_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)
        goal_reached_topic = str(self.get_parameter('goal_reached_topic').value)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_topic, 10)
        self.goal_pub = self.create_publisher(Bool, goal_reached_topic, 10)

        obstacles = self._load_obstacles()
        cfg = self._load_mppi_config()
        self.mppi = MPPIController(cfg, obstacles)
        rate = float(self.get_parameter('cmd_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self._log_obstacles(obstacles, cfg)
        self.get_logger().info(f'Known-map MPPI planner ready: pose={pose_topic}, cmd={cmd_topic}, goal_reached={goal_reached_topic}')
        self.get_logger().info(
            f'mppi cfg: dt={cfg.dt}, horizon={cfg.horizon}, '
            f'T={cfg.dt * cfg.horizon:.2f}s, samples={cfg.num_samples}, '
            f'v_max={cfg.v_max}, w_obst={cfg.w_obst}, '
            f'safety_margin={cfg.safety_margin}, near_buffer={cfg.near_buffer}, '
            f'repulsion_range={cfg.repulsion_range}, repulsion_gain={cfg.repulsion_gain}'
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg
        self.last_pose_t = time.time()

    def _pose_age(self) -> float:
        if self.pose is None:
            return 1e9
        return time.time() - self.last_pose_t

    def _get_xyz_yaw(self) -> Tuple[float, float, float, float]:
        p = self.pose.pose.position
        q = self.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return float(p.x), float(p.y), float(p.z), float(yaw)

    def _publish_cmd(self, vx: float, vy: float, yaw_rate: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = float(yaw_rate)
        self.cmd_pub.publish(msg)

    def _publish_goal_reached(self, reached: bool) -> None:
        msg = Bool()
        msg.data = bool(reached)
        self.goal_pub.publish(msg)

    def _load_obstacles(self) -> List[Obstacle2D]:
        try:
            loader = WorldObstacleLoader(
                world_path=str(self.get_parameter('world_path').value),
                cylinder_model_dir=str(self.get_parameter('cylinder_model_dir').value),
                cylinder_model_name=str(self.get_parameter('cylinder_model_name').value),
                default_radius=float(self.get_parameter('obstacle_default_radius').value),
                ignore_within_radius=float(self.get_parameter('ignore_obstacles_within_start_radius').value),
                ignore_center_x=float(self.get_parameter('ignore_obstacles_center_x').value),
                ignore_center_y=float(self.get_parameter('ignore_obstacles_center_y').value),
            )
            obstacles = loader.load()
            self.get_logger().info(f"world obstacle source: {self.get_parameter('world_path').value}")
            self.get_logger().info(f"cylinder model source: {self.get_parameter('cylinder_model_dir').value}")
            if not obstacles:
                self.get_logger().warn('world obstacle loader returned 0 obstacles')
            return obstacles
        except Exception as exc:
            self.get_logger().error(f'failed to load world obstacles: {exc}')
            return []

    def _load_mppi_config(self) -> MPPIConfig:
        return MPPIConfig(
            dt=float(self.get_parameter('dt').value),
            horizon=int(self.get_parameter('horizon').value),
            num_samples=int(self.get_parameter('num_samples').value),
            lam=float(self.get_parameter('lam').value),
            v_max=float(self.get_parameter('v_max').value),
            yaw_rate_max=float(self.get_parameter('yaw_rate_max').value),
            sigma_v=float(self.get_parameter('sigma_v').value),
            sigma_yaw_rate=float(self.get_parameter('sigma_yaw_rate').value),
            w_goal=float(self.get_parameter('w_goal').value),
            w_goal_final=float(self.get_parameter('w_goal_final').value),
            w_obst=float(self.get_parameter('w_obst').value),
            w_ctrl=float(self.get_parameter('w_ctrl').value),
            w_smooth=float(self.get_parameter('w_smooth').value),
            safety_margin=float(self.get_parameter('safety_margin').value),
            near_buffer=float(self.get_parameter('near_buffer').value),
            near_k=float(self.get_parameter('near_k').value),
            penetrate_k=float(self.get_parameter('penetrate_k').value),
            penetrate_bias=float(self.get_parameter('penetrate_bias').value),
            v_nom=float(self.get_parameter('v_nom').value),
            nominal_update_alpha=float(self.get_parameter('nominal_update_alpha').value),
            repulsion_range=float(self.get_parameter('repulsion_range').value),
            repulsion_gain=float(self.get_parameter('repulsion_gain').value),
            repulsion_max=float(self.get_parameter('repulsion_max').value),
        )

    def _log_obstacles(self, obstacles: List[Obstacle2D], cfg: MPPIConfig) -> None:
        self.get_logger().info(f'obstacles loaded: {len(obstacles)}')
        preview = obstacles[:10]
        for i, obs in enumerate(preview):
            self.get_logger().info(f'obstacle[{i:03d}]: cx={obs.cx:.3f}, cy={obs.cy:.3f}, r={obs.r:.3f}, effective_r={obs.r + cfg.safety_margin:.3f}')
        if len(obstacles) > len(preview):
            self.get_logger().info(f'... {len(obstacles) - len(preview)} more obstacles')

    def _tick(self) -> None:
        self._publish_goal_reached(self.goal_reached)
        pose_timeout = float(self.get_parameter('pose_timeout_sec').value)
        if self.pose is None or self._pose_age() > pose_timeout:
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        gx = float(self.get_parameter('goal_x').value)
        gy = float(self.get_parameter('goal_y').value)
        gyaw = float(self.get_parameter('goal_yaw').value)
        goal_tol = float(self.get_parameter('goal_tol_xy').value)
        x, y, _z, yaw = self._get_xyz_yaw()
        d_goal = math.hypot(gx - x, gy - y)

        if d_goal <= goal_tol:
            self.goal_reached = True
            self._publish_goal_reached(True)
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if not self.nominal_initialized:
            self.mppi.reset()
            self.mppi.set_nominal_towards_goal(x, y, gx, gy)
            self.nominal_initialized = True
        else:
            self.mppi.set_nominal_towards_goal(
                x,
                y,
                gx,
                gy,
                alpha=float(self.get_parameter('nominal_update_alpha').value),
            )

        vx, vy, yr = self.mppi.step(state=(x, y, yaw), goal=(gx, gy, gyaw))
        slowdown_dist = float(self.get_parameter('slowdown_dist').value)
        min_goal_scale = float(self.get_parameter('min_goal_scale').value)
        if d_goal < slowdown_dist:
            scale = clamp(d_goal / max(slowdown_dist, 1e-6), min_goal_scale, 1.0)
            vx *= scale
            vy *= scale

        self._publish_goal_reached(False)
        self._publish_cmd(vx, vy, yr)
        now = time.time()
        log_period = float(self.get_parameter('log_cmd_period_sec').value)
        if log_period > 0.0 and now - self.last_cmd_log_t >= log_period:
            self.last_cmd_log_t = now
            self.get_logger().info(f'cmd: x={x:.2f}, y={y:.2f}, goal=({gx:.2f},{gy:.2f}), d={d_goal:.2f}, vx={vx:.2f}, vy={vy:.2f}, yr={yr:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MPPIPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
