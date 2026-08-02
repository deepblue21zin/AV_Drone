#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class PoseOdomTfNode(Node):
    """Expose a MAVROS local pose as odometry and an odom-to-base TF.

    The PX4 estimator remains the odometry prior. slam_toolbox owns map-to-odom
    and can therefore correct this prior with scan matching without competing
    for TF authority.
    """

    def __init__(self) -> None:
        super().__init__("pose_odom_tf")
        self.declare_parameter("pose_topic", "mavros/local_position/pose")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self._broadcaster = TransformBroadcaster(self)
        self._last_stamp_ns: Optional[int] = None
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            qos_profile_sensor_data,
        )

    def _on_pose(self, pose: PoseStamped) -> None:
        stamp_ns = int(pose.header.stamp.sec) * 1_000_000_000 + int(
            pose.header.stamp.nanosec
        )
        if stamp_ns <= 0:
            stamp = self.get_clock().now().to_msg()
        else:
            if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
                self.get_logger().warn("Ignoring out-of-order pose timestamp")
                return
            self._last_stamp_ns = stamp_ns
            stamp = pose.header.stamp

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose = pose.pose
        self._publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseOdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
