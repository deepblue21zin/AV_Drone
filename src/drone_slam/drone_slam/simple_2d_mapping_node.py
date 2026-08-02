#!/usr/bin/env python3

import math
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class Simple2DMappingNode(Node):
    """Build a simple 2D occupancy grid from LaserScan + MAVROS pose.

    This is a mapping baseline, not a full SLAM back-end. It uses the simulator
    pose as the localization input and accumulates LiDAR observations into a
    namespaced known-pose map.
    """

    def __init__(self) -> None:
        super().__init__("simple_2d_mapping")

        self.declare_parameter("scan_topic", "/drone1/scan")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("map_topic", "mapping/known_pose_map")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("map_resolution", 0.12)
        self.declare_parameter("map_min_x", -3.0)
        self.declare_parameter("map_max_x", 37.0)
        self.declare_parameter("map_min_y", -7.0)
        self.declare_parameter("map_max_y", 7.0)
        self.declare_parameter("max_lidar_range", 10.0)
        self.declare_parameter("ray_step_m", 0.08)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("input_timeout_sec", 0.8)
        self.declare_parameter("slam_status_topic", "/drone1/slam/status")
        self.declare_parameter("slam_input_ready_topic", "/drone1/slam/input_ready")
        self.declare_parameter("slam_map_ready_topic", "/drone1/slam/map_ready")
        self.declare_parameter("slam_localization_ok_topic", "/drone1/slam/localization_ok")
        self.declare_parameter("slam_coverage_topic", "/drone1/slam/coverage")
        self.declare_parameter("publish_health", True)

        self._resolution = float(self.get_parameter("map_resolution").value)
        self._min_x = float(self.get_parameter("map_min_x").value)
        self._max_x = float(self.get_parameter("map_max_x").value)
        self._min_y = float(self.get_parameter("map_min_y").value)
        self._max_y = float(self.get_parameter("map_max_y").value)
        self._width = int(math.ceil((self._max_x - self._min_x) / self._resolution))
        self._height = int(math.ceil((self._max_y - self._min_y) / self._resolution))
        self._log_odds: List[float] = [0.0] * (self._width * self._height)

        self._latest_pose: Optional[Tuple[float, float, float]] = None
        self._last_scan_time: Optional[float] = None
        self._last_pose_time: Optional[float] = None
        self._last_status: Optional[str] = None
        self._scan_count = 0
        self._map_update_count = 0

        scan_topic = str(self.get_parameter("scan_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._map_pub = self.create_publisher(OccupancyGrid, map_topic, map_qos)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("slam_status_topic").value), 10
        )
        self._input_ready_pub = self.create_publisher(
            Bool, str(self.get_parameter("slam_input_ready_topic").value), 10
        )
        self._map_ready_pub = self.create_publisher(
            Bool, str(self.get_parameter("slam_map_ready_topic").value), 10
        )
        self._localization_ok_pub = self.create_publisher(
            Bool, str(self.get_parameter("slam_localization_ok_topic").value), 10
        )
        self._coverage_pub = self.create_publisher(
            Float32, str(self.get_parameter("slam_coverage_topic").value), 10
        )

        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)

        publish_hz = max(0.2, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / publish_hz, self._publish_map_and_status)

        self.get_logger().info(
            "Simple 2D mapping ready: "
            f"scan={scan_topic}, pose={pose_topic}, map={map_topic}, "
            f"size={self._width}x{self._height}, resolution={self._resolution:.2f}m"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self._latest_pose = (float(msg.pose.position.x), float(msg.pose.position.y), yaw)
        self._last_pose_time = time.time()

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_time = time.time()
        self._scan_count += 1
        if self._latest_pose is None:
            return

        robot_x, robot_y, robot_yaw = self._latest_pose
        max_lidar_range = min(
            float(self.get_parameter("max_lidar_range").value),
            float(msg.range_max) if math.isfinite(msg.range_max) else float("inf"),
        )
        if not math.isfinite(max_lidar_range) or max_lidar_range <= 0.0:
            max_lidar_range = float(self.get_parameter("max_lidar_range").value)

        angle = float(msg.angle_min)
        for measured_range in msg.ranges:
            if math.isfinite(measured_range):
                clamped_range = min(float(measured_range), max_lidar_range)
                hit_obstacle = msg.range_min <= measured_range <= max_lidar_range
            else:
                clamped_range = max_lidar_range
                hit_obstacle = False

            world_angle = robot_yaw + angle
            end_x = robot_x + math.cos(world_angle) * clamped_range
            end_y = robot_y + math.sin(world_angle) * clamped_range
            self._mark_free_ray(robot_x, robot_y, end_x, end_y)
            if hit_obstacle:
                self._add_log_odds(end_x, end_y, 1.1)

            angle += float(msg.angle_increment)

        self._map_update_count += 1

    def _world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        gx = int(math.floor((x - self._min_x) / self._resolution))
        gy = int(math.floor((y - self._min_y) / self._resolution))
        if gx < 0 or gx >= self._width or gy < 0 or gy >= self._height:
            return None
        return gx, gy

    def _grid_index(self, gx: int, gy: int) -> int:
        return gy * self._width + gx

    def _add_log_odds(self, x: float, y: float, delta: float) -> None:
        cell = self._world_to_grid(x, y)
        if cell is None:
            return
        index = self._grid_index(cell[0], cell[1])
        self._log_odds[index] = max(-3.5, min(4.5, self._log_odds[index] + delta))

    def _mark_free_ray(self, start_x: float, start_y: float, end_x: float, end_y: float) -> None:
        ray_step = max(0.02, float(self.get_parameter("ray_step_m").value))
        distance = math.hypot(end_x - start_x, end_y - start_y)
        if distance <= 0.0:
            return

        steps = max(1, int(distance / ray_step))
        # Leave the final point to the occupied update if the scan hit an obstacle.
        for step in range(steps):
            t = step / steps
            x = start_x + (end_x - start_x) * t
            y = start_y + (end_y - start_y) * t
            self._add_log_odds(x, y, -0.35)

    def _to_occupancy_data(self) -> List[int]:
        data: List[int] = []
        for value in self._log_odds:
            if abs(value) < 0.25:
                data.append(-1)
            elif value < 0.0:
                data.append(0)
            else:
                probability = 1.0 - 1.0 / (1.0 + math.exp(value))
                data.append(max(1, min(100, int(round(probability * 100.0)))))
        return data

    def _inputs_ready(self) -> bool:
        now = time.time()
        timeout = max(0.05, float(self.get_parameter("input_timeout_sec").value))
        scan_ready = self._last_scan_time is not None and now - self._last_scan_time <= timeout
        pose_ready = self._last_pose_time is not None and now - self._last_pose_time <= timeout
        return scan_ready and pose_ready

    def _publish_status(self, status: str) -> None:
        if status != self._last_status:
            self.get_logger().info(status)
            self._last_status = status
        self._status_pub.publish(String(data=status))

    def _publish_map_and_status(self) -> None:
        input_ready = self._inputs_ready()
        map_ready = self._map_update_count > 0
        publish_health = bool(self.get_parameter("publish_health").value)

        if publish_health:
            self._input_ready_pub.publish(Bool(data=input_ready))
            self._map_ready_pub.publish(Bool(data=map_ready))
            self._localization_ok_pub.publish(Bool(data=self._last_pose_time is not None))

        if not input_ready:
            if publish_health:
                self._publish_status("mapping_waiting_for_scan_or_pose")
            return

        if publish_health:
            self._publish_status(
                f"mapping_active scans={self._scan_count} updates={self._map_update_count}"
            )

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame_id").value)
        msg.info.resolution = self._resolution
        msg.info.width = self._width
        msg.info.height = self._height
        msg.info.origin.position.x = self._min_x
        msg.info.origin.position.y = self._min_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        occupancy_data = self._to_occupancy_data()
        msg.data = occupancy_data
        self._map_pub.publish(msg)
        if publish_health:
            observed = sum(1 for value in occupancy_data if value >= 0)
            coverage = observed / max(1, len(occupancy_data))
            self._coverage_pub.publish(Float32(data=float(coverage)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Simple2DMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()
