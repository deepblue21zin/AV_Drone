#!/usr/bin/env python3

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SlamScaffoldNode(Node):
    """Reserved SLAM interface for separated team development.

    This node does not perform SLAM. It exists so the team can:
    - run the scan + pose input path without the avoidance stack
    - reserve stable SLAM health topics early
    - replace this scaffold with the real SLAM implementation later
    """

    def __init__(self) -> None:
        super().__init__("slam_scaffold")

        self.declare_parameter("scan_topic", "/drone1/scan")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("slam_status_topic", "/drone1/slam/status")
        self.declare_parameter("slam_input_ready_topic", "/drone1/slam/input_ready")
        self.declare_parameter("slam_map_ready_topic", "/drone1/slam/map_ready")
        self.declare_parameter("slam_localization_ok_topic", "/drone1/slam/localization_ok")
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("heartbeat_hz", 2.0)

        scan_topic = str(self.get_parameter("scan_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        status_topic = str(self.get_parameter("slam_status_topic").value)
        input_ready_topic = str(self.get_parameter("slam_input_ready_topic").value)
        map_ready_topic = str(self.get_parameter("slam_map_ready_topic").value)
        localization_ok_topic = str(self.get_parameter("slam_localization_ok_topic").value)

        self._last_scan_time: Optional[float] = None
        self._last_pose_time: Optional[float] = None
        self._last_status: Optional[str] = None

        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._input_ready_pub = self.create_publisher(Bool, input_ready_topic, 10)
        self._map_ready_pub = self.create_publisher(Bool, map_ready_topic, 10)
        self._localization_ok_pub = self.create_publisher(Bool, localization_ok_topic, 10)

        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)

        heartbeat_hz = max(0.5, float(self.get_parameter("heartbeat_hz").value))
        self.create_timer(1.0 / heartbeat_hz, self._tick)

        self.get_logger().info(
            f"SLAM scaffold ready: scan={scan_topic}, pose={pose_topic}, "
            f"status={status_topic}, input_ready={input_ready_topic}, "
            f"map_ready={map_ready_topic}, localization_ok={localization_ok_topic}"
        )

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan_time = time.time()

    def _on_pose(self, _msg: PoseStamped) -> None:
        self._last_pose_time = time.time()

    def _publish_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f"SLAM scaffold status => {status}")

    def _tick(self) -> None:
        now = time.time()
        input_timeout = max(0.05, float(self.get_parameter("input_timeout_sec").value))

        scan_fresh = self._last_scan_time is not None and (now - self._last_scan_time) <= input_timeout
        pose_fresh = self._last_pose_time is not None and (now - self._last_pose_time) <= input_timeout
        input_ready = scan_fresh and pose_fresh

        if input_ready:
            status = "scaffold_inputs_ready"
        else:
            status = "scaffold_waiting_inputs"

        # The scaffold reserves the interface only.
        self._input_ready_pub.publish(Bool(data=input_ready))
        self._map_ready_pub.publish(Bool(data=False))
        self._localization_ok_pub.publish(Bool(data=False))
        self._publish_status(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamScaffoldNode()
    rclpy.spin(node)
    rclpy.shutdown()
