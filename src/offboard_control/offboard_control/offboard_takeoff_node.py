#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class OffboardTakeoffNode(Node):
    def __init__(self):
        super().__init__('offboard_takeoff')

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Service clientsS
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Setpoint 메시지 생성
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 3.0  # 이륙 고도 3m

        # 주기적으로 퍼블리시 시작
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # flag
        self.setpoint_sent = 0
        self.armed = False
        self.offboard_mode_set = False

    def timer_callback(self):
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.target_pose)

        if self.setpoint_sent < 40:  # 약 2초 동안 Setpoint 보내기
            self.setpoint_sent += 1
            return

        if not self.armed:
            self.arm_drone()
        elif not self.offboard_mode_set:
            self.set_offboard_mode()

    def arm_drone(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            self.armed = True
            self.get_logger().info("Arming 요청 보냈습니다!")

    def set_offboard_mode(self):
        if self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.mode_client.call_async(req)
            self.offboard_mode_set = True
            self.get_logger().info("Offboard 모드 전환 요청 보냈습니다!")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()