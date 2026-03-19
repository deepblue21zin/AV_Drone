#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from std_msgs.msg import Bool, String

from drone_control.vehicle_interface import VehicleInterface


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class AutonomyManagerNode(Node):
    def __init__(self):
        super().__init__("autonomy_manager")

        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("safe_cmd_topic", "/drone1/safety/cmd_vel")
        self.declare_parameter("goal_reached_topic", "/drone1/mission/goal_reached")
        self.declare_parameter("mission_phase_topic", "/drone1/mission/phase")
        self.declare_parameter("takeoff_z", 3.0)
        self.declare_parameter("goal_z", 3.0)
        self.declare_parameter("hover_sec_after_takeoff", 2.0)
        self.declare_parameter("kp_z", 1.2)
        self.declare_parameter("vz_max", 1.0)
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("prestream_setpoints", 40)

        pose_topic = str(self.get_parameter("pose_topic").value)
        self.vehicle = VehicleInterface(
            self,
            str(self.get_parameter("mavros_namespace").value),
            pose_topic=pose_topic,
        )
        self.safe_cmd_topic = str(self.get_parameter("safe_cmd_topic").value)
        self.goal_reached_topic = str(self.get_parameter("goal_reached_topic").value)
        self.phase_pub = self.create_publisher(
            String, str(self.get_parameter("mission_phase_topic").value), 10
        )

        self._latest_cmd = TwistStamped()
        self._have_cmd = False
        self._goal_reached = False
        self._last_mode_req_t = 0.0
        self._last_arm_req_t = 0.0
        self._phase = "WAIT_STREAM"
        self._phase_t0 = time.time()
        self._prestream_count = 0

        self.create_subscription(TwistStamped, self.safe_cmd_topic, self._on_cmd, 10)
        self.create_subscription(Bool, self.goal_reached_topic, self._on_goal_reached, 10)

        rate_hz = max(float(self.get_parameter("cmd_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(1.0, self._publish_phase_heartbeat)
        self.phase_pub.publish(String(data=self._phase))

        self.get_logger().info(
            f"Autonomy manager ready: pose={pose_topic}, safe_cmd={self.safe_cmd_topic}, goal_reached={self.goal_reached_topic}"
        )

    def _on_cmd(self, msg: TwistStamped):
        self._latest_cmd = msg
        self._have_cmd = True

    def _on_goal_reached(self, msg: Bool):
        self._goal_reached = bool(msg.data)

    def _request_mode(self, mode: str):
        if not self.vehicle.mode_cli.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.vehicle.mode_cli.call_async(req)

    def _request_arm(self, arm: bool):
        if not self.vehicle.arm_cli.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = arm
        self.vehicle.arm_cli.call_async(req)

    def _publish_cmd(self, vx: float, vy: float, vz: float, yaw_rate: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(yaw_rate)
        self.vehicle.publish_velocity(msg)

    def _enter_phase(self, name: str):
        if self._phase == name:
            return
        self._phase = name
        self._phase_t0 = time.time()
        self.phase_pub.publish(String(data=name))
        self.get_logger().info(f"PHASE => {name}")

    def _phase_elapsed(self) -> float:
        return time.time() - self._phase_t0

    def _publish_phase_heartbeat(self):
        self.phase_pub.publish(String(data=self._phase))

    def _get_xyz_yaw(self):
        pose = self.vehicle.pose
        p = pose.pose.position
        q = pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return float(p.x), float(p.y), float(p.z), float(yaw)

    def _tick(self):
        now = time.time()
        pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        takeoff_z = float(self.get_parameter("takeoff_z").value)
        goal_z = float(self.get_parameter("goal_z").value)
        hover_after_takeoff = float(self.get_parameter("hover_sec_after_takeoff").value)
        kp_z = float(self.get_parameter("kp_z").value)
        vz_max = float(self.get_parameter("vz_max").value)
        prestream_setpoints = int(self.get_parameter("prestream_setpoints").value)

        # Always stream a setpoint for PX4 offboard admission.
        self._publish_cmd(0.0, 0.0, 0.0, 0.0)

        if not self.vehicle.state.connected:
            return
        if self.vehicle.pose is None or self.vehicle.pose_age() > pose_timeout:
            return

        _, _, z, _ = self._get_xyz_yaw()

        if self._phase == "WAIT_STREAM":
            self._prestream_count += 1
            if self._prestream_count >= prestream_setpoints:
                self._enter_phase("OFFBOARD_ARM")
            return

        if self._phase not in {"WAIT_STREAM", "OFFBOARD_ARM"}:
            if self.vehicle.state.mode != "OFFBOARD" or not self.vehicle.state.armed:
                self._enter_phase("OFFBOARD_ARM")
                return

        if self._phase == "OFFBOARD_ARM":
            if self.vehicle.state.mode != "OFFBOARD":
                if (now - self._last_mode_req_t) > 1.0:
                    self._request_mode("OFFBOARD")
                    self._last_mode_req_t = now
                    self.get_logger().info("Requesting OFFBOARD mode")
                return

            if not self.vehicle.state.armed:
                if (now - self._last_arm_req_t) > 1.0:
                    self._request_arm(True)
                    self._last_arm_req_t = now
                    self.get_logger().info("Requesting arm")
                return

            self._enter_phase("TAKEOFF")
            return

        if self._phase == "TAKEOFF":
            err_z = takeoff_z - z
            vz_cmd = clamp(kp_z * err_z, -vz_max, vz_max)
            if err_z > 0.2:
                vz_cmd = clamp(vz_cmd, 0.2, vz_max)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if z >= takeoff_z - 0.15:
                self._enter_phase("HOVER_AFTER_TAKEOFF")
            return

        if self._phase == "HOVER_AFTER_TAKEOFF":
            err_z = takeoff_z - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if self._phase_elapsed() >= hover_after_takeoff:
                self._enter_phase("FOLLOW_PLAN")
            return

        if self._phase == "FOLLOW_PLAN":
            if self._goal_reached:
                self._enter_phase("HOVER_AT_GOAL")
                return

            err_z = goal_z - z
            vz_hold = clamp(kp_z * err_z, -vz_max, vz_max)
            cmd = self._latest_cmd if self._have_cmd else TwistStamped()
            self._publish_cmd(
                cmd.twist.linear.x,
                cmd.twist.linear.y,
                vz_hold,
                cmd.twist.angular.z,
            )
            return

        if self._phase == "HOVER_AT_GOAL":
            err_z = goal_z - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()
