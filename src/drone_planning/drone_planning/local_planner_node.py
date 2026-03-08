#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float32


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner")

        self.declare_parameter(
            "nearest_obstacle_topic", "/drone1/perception/nearest_obstacle_distance"
        )
        self.declare_parameter("autonomy_cmd_topic", "/drone1/autonomy/cmd_vel")
        self.declare_parameter("cruise_speed", 1.0)
        self.declare_parameter("obstacle_stop_distance", 2.0)

        self.nearest_obstacle = float("inf")
        self.have_obstacle_update = False
        self.autonomy_cmd_topic = str(self.get_parameter("autonomy_cmd_topic").value)
        nearest_topic = str(self.get_parameter("nearest_obstacle_topic").value)

        self.cmd_pub = self.create_publisher(TwistStamped, self.autonomy_cmd_topic, 10)
        self.create_subscription(Float32, nearest_topic, self._on_obstacle, 10)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Planner scaffold ready: nearest_obstacle={nearest_topic}, cmd_out={self.autonomy_cmd_topic}"
        )

    def _on_obstacle(self, msg: Float32):
        self.nearest_obstacle = float(msg.data)
        self.have_obstacle_update = True

    def _tick(self):
        cruise_speed = float(self.get_parameter("cruise_speed").value)
        stop_distance = float(self.get_parameter("obstacle_stop_distance").value)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        if not self.have_obstacle_update or self.nearest_obstacle <= stop_distance:
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
        else:
            msg.twist.linear.x = cruise_speed
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0

        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
