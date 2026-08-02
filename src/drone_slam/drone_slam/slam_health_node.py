#!/usr/bin/env python3

import time
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


class SlamHealthNode(Node):
    """Normalizes mapper and slam_toolbox health into one per-UAV interface."""

    def __init__(self) -> None:
        super().__init__("slam_health")
        self.declare_parameter("map_topic", "slam/map")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("status_topic", "slam/status")
        self.declare_parameter("input_ready_topic", "slam/input_ready")
        self.declare_parameter("map_ready_topic", "slam/map_ready")
        self.declare_parameter("localization_ok_topic", "slam/localization_ok")
        self.declare_parameter("coverage_topic", "slam/coverage")
        self.declare_parameter("input_timeout_sec", 2.0)
        self.declare_parameter("heartbeat_hz", 2.0)

        self._last_map_time: Optional[float] = None
        self._last_odom_time: Optional[float] = None
        self._coverage = 0.0
        self._last_status: Optional[str] = None

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )

        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._input_pub = self.create_publisher(
            Bool, str(self.get_parameter("input_ready_topic").value), 10
        )
        self._map_pub = self.create_publisher(
            Bool, str(self.get_parameter("map_ready_topic").value), 10
        )
        self._localization_pub = self.create_publisher(
            Bool, str(self.get_parameter("localization_ok_topic").value), 10
        )
        self._coverage_pub = self.create_publisher(
            Float32, str(self.get_parameter("coverage_topic").value), 10
        )
        hz = max(0.2, float(self.get_parameter("heartbeat_hz").value))
        self.create_timer(1.0 / hz, self._tick)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last_map_time = time.monotonic()
        observed = sum(1 for value in msg.data if value >= 0)
        self._coverage = observed / max(1, len(msg.data))

    def _on_odom(self, _msg: Odometry) -> None:
        self._last_odom_time = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        timeout = max(0.1, float(self.get_parameter("input_timeout_sec").value))
        map_ok = self._last_map_time is not None and now - self._last_map_time <= timeout
        odom_ok = self._last_odom_time is not None and now - self._last_odom_time <= timeout
        status = "healthy" if map_ok and odom_ok else "waiting_map_or_odom"
        if status != self._last_status:
            self._last_status = status
            self.get_logger().info(f"SLAM health => {status}")
        self._status_pub.publish(String(data=status))
        self._input_pub.publish(Bool(data=odom_ok))
        self._map_pub.publish(Bool(data=map_ok))
        self._localization_pub.publish(Bool(data=map_ok and odom_ok))
        self._coverage_pub.publish(Float32(data=float(self._coverage)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
