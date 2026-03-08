#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node

from drone_control.vehicle_interface import VehicleInterface


class AutonomyManagerNode(Node):
    def __init__(self):
        super().__init__("autonomy_manager")

        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("safe_cmd_topic", "/drone1/safety/cmd_vel")

        self.vehicle = VehicleInterface(
            self, str(self.get_parameter("mavros_namespace").value)
        )
        self.safe_cmd_topic = str(self.get_parameter("safe_cmd_topic").value)

        self._latest_cmd = TwistStamped()
        self._have_cmd = False
        self._last_mode_req_t = 0.0
        self._last_arm_req_t = 0.0

        self.create_subscription(TwistStamped, self.safe_cmd_topic, self._on_cmd, 10)
        self.create_timer(0.05, self._tick)

    def _on_cmd(self, msg: TwistStamped):
        self._latest_cmd = msg
        self._have_cmd = True

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

    def _tick(self):
        now = time.time()

        # Always stream a setpoint for PX4 offboard admission.
        msg = self._latest_cmd if self._have_cmd else TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.vehicle.publish_velocity(msg)

        if not self.vehicle.state.connected:
            return

        if self.vehicle.state.mode != "OFFBOARD" and (now - self._last_mode_req_t) > 1.0:
            self._request_mode("OFFBOARD")
            self._last_mode_req_t = now
            self.get_logger().info("Requesting OFFBOARD mode")
            return

        if not self.vehicle.state.armed and (now - self._last_arm_req_t) > 1.0:
            self._request_arm(True)
            self._last_arm_req_t = now
            self.get_logger().info("Requesting arm")


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()
