from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import math

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class PoseSnapshot:
    msg: PoseStamped
    stamp_sec: float


class MavrosIO:
    """
    MAVROS IO (Step 2 최소 확장판)
      - pose 구독 (/mavros/local_position/pose)
        * QoS: qos_profile_sensor_data (BEST_EFFORT + 센서 스트림 호환 프리셋)
      - yaw 추출
      - velocity + yaw_rate publish
        기본 토픽: /mavros/setpoint_velocity/cmd_vel_unstamped (Twist)
        필요 시:  /mavros/setpoint_velocity/cmd_vel          (TwistStamped)
    """

    def __init__(
        self,
        node: Node,
        pose_topic: str = "/mavros/local_position/pose",
        vel_topic_unstamped: str = "/mavros/setpoint_velocity/cmd_vel_unstamped",
        vel_topic_stamped: str = "/mavros/setpoint_velocity/cmd_vel",
        use_stamped_vel: bool = False,
        debug_pose: bool = True,
    ):
        self._node = node
        self._pose: Optional[PoseSnapshot] = None

        # 디버그: pose rx 로그 (1 Hz)
        self._debug_pose = bool(debug_pose)
        self._last_pose_log_sec: float = 0.0

        # --- pose sub (센서 QoS 프리셋으로 고정) ---
        self._sub = node.create_subscription(
            PoseStamped,
            pose_topic,
            self._on_pose,
            qos_profile_sensor_data,
        )

        # --- vel pub ---
        self._use_stamped_vel = bool(use_stamped_vel)

        if self._use_stamped_vel:
            self._vel_pub_stamped = node.create_publisher(
                TwistStamped, vel_topic_stamped, 10
            )
            self._vel_pub_unstamped = None
        else:
            self._vel_pub_unstamped = node.create_publisher(
                Twist, vel_topic_unstamped, 10
            )
            self._vel_pub_stamped = None

    def _now_sec(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg: PoseStamped):
        now = self._now_sec()
        self._pose = PoseSnapshot(msg=msg, stamp_sec=now)

        # 1초에 한 번만 수신 로그
        if self._debug_pose and (now - self._last_pose_log_sec) >= 1.0:
            self._last_pose_log_sec = now
            self._node.get_logger().info("[MavrosIO] pose rx OK")

    # ---------------------------
    # Pose getters
    # ---------------------------
    def pose(self) -> Optional[PoseStamped]:
        return None if self._pose is None else self._pose.msg

    def pose_age_sec(self) -> float:
        if self._pose is None:
            return 1e9
        return self._now_sec() - self._pose.stamp_sec

    def x(self) -> Optional[float]:
        p = self.pose()
        return None if p is None else float(p.pose.position.x)

    def y(self) -> Optional[float]:
        p = self.pose()
        return None if p is None else float(p.pose.position.y)

    def z(self) -> Optional[float]:
        p = self.pose()
        return None if p is None else float(p.pose.position.z)

    def yaw(self) -> Optional[float]:
        """
        Quaternion -> yaw (rad), ENU 기준.
        yaw ∈ (-pi, pi]
        """
        p = self.pose()
        if p is None:
            return None

        q = p.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(math.atan2(siny_cosp, cosy_cosp))

    # ---------------------------
    # Velocity publisher
    # ---------------------------
    def publish_velocity(
        self,
        vx: float,
        vy: float,
        yaw_rate: float,
        vz: float = 0.0,
    ) -> None:
        """
        Publish velocity setpoint.
          - vx, vy, vz: m/s (ENU local frame)
          - yaw_rate: rad/s (positive CCW)
        """
        if self._use_stamped_vel:
            msg = TwistStamped()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.twist.linear.x = float(vx)
            msg.twist.linear.y = float(vy)
            msg.twist.linear.z = float(vz)
            msg.twist.angular.z = float(yaw_rate)
            self._vel_pub_stamped.publish(msg)  # type: ignore[union-attr]
        else:
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.linear.z = float(vz)
            msg.angular.z = float(yaw_rate)
            self._vel_pub_unstamped.publish(msg)  # type: ignore[union-attr]
