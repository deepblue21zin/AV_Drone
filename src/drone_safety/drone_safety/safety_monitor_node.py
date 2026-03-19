#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__("safety_monitor")

        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("scan_topic", "/drone1/scan")
        self.declare_parameter("planner_cmd_topic", "/drone1/autonomy/cmd_vel")
        self.declare_parameter("safe_cmd_topic", "/drone1/safety/cmd_vel")
        self.declare_parameter("safety_event_topic", "/drone1/safety/event")
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("planner_cmd_timeout_sec", 0.5)
        self.declare_parameter("emergency_stop_distance", 1.0)
        self.declare_parameter("startup_grace_sec", 3.0)

        self.last_pose_t = None
        self.last_scan_t = None
        self.last_cmd_t = None
        self.last_scan_min = float("inf")
        self.latest_cmd = TwistStamped()
        self.last_reason = "startup_hold"
        self._node_start_time = time.time()

        pose_topic = str(self.get_parameter("pose_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        planner_cmd_topic = str(self.get_parameter("planner_cmd_topic").value)
        safe_cmd_topic = str(self.get_parameter("safe_cmd_topic").value)
        safety_event_topic = str(self.get_parameter("safety_event_topic").value)

        self.safe_cmd_pub = self.create_publisher(TwistStamped, safe_cmd_topic, 10)
        self.event_pub = self.create_publisher(String, safety_event_topic, 10)

        self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(TwistStamped, planner_cmd_topic, self._on_cmd, 10)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Safety monitor ready: pose={pose_topic}, scan={scan_topic}, planner_cmd={planner_cmd_topic}, safe_cmd={safe_cmd_topic}"
        )

    def _on_pose(self, _msg: PoseStamped):
        self.last_pose_t = time.time()

    def _on_scan(self, msg: LaserScan):
        self.last_scan_t = time.time()
        valid = [r for r in msg.ranges if r >= msg.range_min and r <= msg.range_max]
        self.last_scan_min = min(valid) if valid else float("inf")

    def _on_cmd(self, msg: TwistStamped):
        self.last_cmd_t = time.time()
        self.latest_cmd = msg

    def _emit_event(self, reason: str):
        if reason == self.last_reason:
            return
        self.last_reason = reason
        self.event_pub.publish(String(data=reason))
        self.get_logger().warn(f"Safety state changed: {reason}")

    def _zero_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        return msg

    def _tick(self):
        now = time.time()
        pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        cmd_timeout = float(self.get_parameter("planner_cmd_timeout_sec").value)
        emergency_stop_distance = float(self.get_parameter("emergency_stop_distance").value)
        startup_grace = float(self.get_parameter("startup_grace_sec").value)

        if (now - self._node_start_time) < startup_grace:
            self._emit_event("startup_grace")
            self.safe_cmd_pub.publish(self._zero_cmd())
            return

        if self.last_pose_t is None or (now - self.last_pose_t) > pose_timeout:
            self._emit_event("pose_timeout")
            self.safe_cmd_pub.publish(self._zero_cmd())
            return

        if self.last_scan_t is None or (now - self.last_scan_t) > scan_timeout:
            self._emit_event("scan_timeout")
            self.safe_cmd_pub.publish(self._zero_cmd())
            return

        if self.last_cmd_t is None or (now - self.last_cmd_t) > cmd_timeout:
            self._emit_event("planner_cmd_timeout")
            self.safe_cmd_pub.publish(self._zero_cmd())
            return

        if self.last_scan_min <= emergency_stop_distance:
            self._emit_event("emergency_stop_obstacle")
            self.safe_cmd_pub.publish(self._zero_cmd())
            return

        self._emit_event("normal")
        safe_cmd = self.latest_cmd
        safe_cmd.header.stamp = self.get_clock().now().to_msg()
        self.safe_cmd_pub.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()
