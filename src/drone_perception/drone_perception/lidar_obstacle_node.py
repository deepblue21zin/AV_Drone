#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__("lidar_obstacle")

        self.declare_parameter("scan_topic", "/drone1/scan")
        self.declare_parameter(
            "nearest_obstacle_topic", "/drone1/perception/nearest_obstacle_distance"
        )

        scan_topic = str(self.get_parameter("scan_topic").value)
        obstacle_topic = str(self.get_parameter("nearest_obstacle_topic").value)

        self.pub = self.create_publisher(Float32, obstacle_topic, 10)
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Perception scaffold ready: scan={scan_topic}, nearest_obstacle={obstacle_topic}"
        )

    def _on_scan(self, msg: LaserScan):
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and r >= msg.range_min]
        nearest = min(valid_ranges) if valid_ranges else float("inf")
        self.pub.publish(Float32(data=float(nearest)))


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    rclpy.spin(node)
    rclpy.shutdown()
