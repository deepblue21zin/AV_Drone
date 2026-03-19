#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner")

        self.declare_parameter(
            "nearest_obstacle_topic", "/drone1/perception/nearest_obstacle_distance"
        )
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("autonomy_cmd_topic", "/drone1/autonomy/cmd_vel")
        self.declare_parameter("goal_reached_topic", "/drone1/mission/goal_reached")
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tol_xy", 0.6)
        self.declare_parameter("cruise_speed", 1.0)
        self.declare_parameter("obstacle_stop_distance", 2.0)
        self.declare_parameter("goal_latch_enabled", True)

        self.nearest_obstacle = float("inf")
        self.have_obstacle_update = False
        self.pose = None
        self.goal_latched = False
        self.autonomy_cmd_topic = str(self.get_parameter("autonomy_cmd_topic").value)
        nearest_topic = str(self.get_parameter("nearest_obstacle_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        goal_reached_topic = str(self.get_parameter("goal_reached_topic").value)

        self.cmd_pub = self.create_publisher(TwistStamped, self.autonomy_cmd_topic, 10)
        self.goal_pub = self.create_publisher(Bool, goal_reached_topic, 10)
        self.create_subscription(Float32, nearest_topic, self._on_obstacle, 10)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Planner scaffold ready: nearest_obstacle={nearest_topic}, pose={pose_topic}, cmd_out={self.autonomy_cmd_topic}, goal_reached={goal_reached_topic}"
        )

    def _on_obstacle(self, msg: Float32):
        self.nearest_obstacle = float(msg.data)
        self.have_obstacle_update = True

    def _on_pose(self, msg: PoseStamped):
        self.pose = msg

    def _tick(self):
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        goal_tol_xy = float(self.get_parameter("goal_tol_xy").value)
        cruise_speed = float(self.get_parameter("cruise_speed").value)
        stop_distance = float(self.get_parameter("obstacle_stop_distance").value)
        goal_latch_enabled = bool(self.get_parameter("goal_latch_enabled").value)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        goal_reached = self.goal_latched if goal_latch_enabled else False

        if self.pose is None or not self.have_obstacle_update:
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
        else:
            px = float(self.pose.pose.position.x)
            py = float(self.pose.pose.position.y)
            dx = goal_x - px
            dy = goal_y - py
            dist = math.hypot(dx, dy)

            if dist <= goal_tol_xy:
                goal_reached = True
                if goal_latch_enabled and not self.goal_latched:
                    self.goal_latched = True
                    self.get_logger().info(
                        f"Goal latched at xy distance {dist:.2f} m"
                    )
            elif self.nearest_obstacle > stop_distance:
                msg.twist.linear.x = cruise_speed * dx / max(dist, 1e-6)
                msg.twist.linear.y = cruise_speed * dy / max(dist, 1e-6)
                msg.twist.linear.z = 0.0

        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.goal_pub.publish(Bool(data=goal_reached))


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
