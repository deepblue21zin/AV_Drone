#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
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
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("tf_stamp_source", "pose")
        self.declare_parameter("zero_initial_yaw", False)
        self.declare_parameter("normalized_pose_topic", "")

        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._tf_stamp_source = str(
            self.get_parameter("tf_stamp_source").value
        ).strip().lower()
        if self._tf_stamp_source not in {"pose", "scan"}:
            raise ValueError("tf_stamp_source must be 'pose' or 'scan'")
        self._zero_initial_yaw = bool(
            self.get_parameter("zero_initial_yaw").value
        )
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        normalized_pose_topic = str(
            self.get_parameter("normalized_pose_topic").value
        ).strip()
        self._normalized_pose_publisher = (
            self.create_publisher(
                PoseStamped, normalized_pose_topic, qos_profile_sensor_data
            )
            if normalized_pose_topic
            else None
        )
        self._broadcaster = TransformBroadcaster(self)
        self._last_stamp_ns: Optional[int] = None
        self._last_tf_stamp_ns: Optional[int] = None
        self._latest_pose: Optional[PoseStamped] = None
        self._initial_yaw: Optional[float] = None
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            qos_profile_sensor_data,
        )
        if self._tf_stamp_source == "scan":
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self._on_scan,
                qos_profile_sensor_data,
            )

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _yaw_from_pose(pose: PoseStamped) -> float:
        q = pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _remove_yaw_offset(pose: PoseStamped, yaw_offset: float) -> PoseStamped:
        """Rotate only the pose orientation into the launch-aligned yaw frame."""
        corrected = PoseStamped()
        corrected.header = pose.header
        corrected.pose.position = pose.pose.position

        q = pose.pose.orientation
        half = -0.5 * yaw_offset
        sin_half = math.sin(half)
        cos_half = math.cos(half)
        corrected.pose.orientation.x = cos_half * q.x - sin_half * q.y
        corrected.pose.orientation.y = cos_half * q.y + sin_half * q.x
        corrected.pose.orientation.z = cos_half * q.z + sin_half * q.w
        corrected.pose.orientation.w = cos_half * q.w - sin_half * q.z
        return corrected

    def _broadcast_pose(self, pose: PoseStamped, stamp) -> None:
        stamp_ns = self._stamp_ns(stamp)
        if stamp_ns <= 0:
            self.get_logger().warning(
                "Skipping TF with an empty timestamp", throttle_duration_sec=5.0
            )
            return
        if self._last_tf_stamp_ns is not None and stamp_ns <= self._last_tf_stamp_ns:
            self.get_logger().warning(
                "Skipping non-monotonic odom-to-base TF timestamp",
                throttle_duration_sec=5.0,
            )
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._broadcaster.sendTransform(transform)
        self._last_tf_stamp_ns = stamp_ns

    def _on_pose(self, pose: PoseStamped) -> None:
        stamp_ns = self._stamp_ns(pose.header.stamp)
        if stamp_ns > 0:
            if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
                self.get_logger().warn("Ignoring out-of-order pose timestamp")
                return
            self._last_stamp_ns = stamp_ns

        if self._initial_yaw is None:
            position = pose.pose.position
            raw_yaw = self._yaw_from_pose(pose)
            self._initial_yaw = raw_yaw if self._zero_initial_yaw else 0.0
            self.get_logger().info(
                "Initial SLAM odometry pose: "
                f"x={position.x:.3f}, y={position.y:.3f}, "
                f"yaw={raw_yaw:.5f} rad, "
                f"zero_initial_yaw={self._zero_initial_yaw}"
            )
        corrected_pose = self._remove_yaw_offset(pose, self._initial_yaw)
        self._latest_pose = corrected_pose
        if self._normalized_pose_publisher is not None:
            self._normalized_pose_publisher.publish(corrected_pose)

        # MAVROS stamps PX4 data in synchronized epoch time, while Gazebo laser
        # scans use simulation time.  TF must use the same clock as the scan or
        # slam_toolbox rejects every scan as older than its transform cache.
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose = corrected_pose.pose
        self._publisher.publish(odom)

        if self._tf_stamp_source == "pose":
            self._broadcast_pose(corrected_pose, stamp)

    def _on_scan(self, scan: LaserScan) -> None:
        if self._latest_pose is None:
            self.get_logger().warning(
                "Skipping scan-stamped TF before the first pose",
                throttle_duration_sec=5.0,
            )
            return
        self._broadcast_pose(self._latest_pose, scan.header.stamp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseOdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
