#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_body(vx_world: float, vy_world: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * vx_world + sin_yaw * vy_world,
        -sin_yaw * vx_world + cos_yaw * vy_world,
    )


def body_to_world(vx_body: float, vy_body: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * vx_body - sin_yaw * vy_body,
        sin_yaw * vx_body + cos_yaw * vy_body,
    )


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner")

        self.declare_parameter(
            "nearest_obstacle_topic", "/drone1/perception/nearest_obstacle_distance"
        )
        self.declare_parameter("scan_topic", "/drone1/scan")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("autonomy_cmd_topic", "/drone1/autonomy/cmd_vel")
        self.declare_parameter("goal_reached_topic", "/drone1/mission/goal_reached")
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tol_xy", 0.6)
        self.declare_parameter("cruise_speed", 1.0)
        self.declare_parameter("max_speed", 1.1)
        self.declare_parameter("max_reverse_speed", 0.35)
        self.declare_parameter("obstacle_stop_distance", 2.0)
        self.declare_parameter("obstacle_slow_distance", 3.2)
        self.declare_parameter("obstacle_influence_distance", 4.5)
        self.declare_parameter("goal_latch_enabled", True)
        self.declare_parameter("allow_motion_without_scan", False)
        self.declare_parameter("forward_gain", 1.0)
        self.declare_parameter("avoidance_gain", 1.4)
        self.declare_parameter("lateral_bias_gain", 0.85)
        self.declare_parameter("backoff_gain", 0.55)
        self.declare_parameter("front_sector_half_angle_deg", 32.0)
        self.declare_parameter("side_sector_width_deg", 85.0)

        self.nearest_obstacle = float("inf")
        self.have_obstacle_update = False
        self.pose = None
        self.scan = None
        self.goal_latched = False
        self.autonomy_cmd_topic = str(self.get_parameter("autonomy_cmd_topic").value)
        nearest_topic = str(self.get_parameter("nearest_obstacle_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        goal_reached_topic = str(self.get_parameter("goal_reached_topic").value)

        self.cmd_pub = self.create_publisher(TwistStamped, self.autonomy_cmd_topic, 10)
        self.goal_pub = self.create_publisher(Bool, goal_reached_topic, 10)
        self.create_subscription(Float32, nearest_topic, self._on_obstacle, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            "Reactive local planner ready: "
            f"scan={scan_topic}, nearest_obstacle={nearest_topic}, pose={pose_topic}, "
            f"cmd_out={self.autonomy_cmd_topic}, goal_reached={goal_reached_topic}"
        )

    def _on_obstacle(self, msg: Float32):
        self.nearest_obstacle = float(msg.data)
        self.have_obstacle_update = True

    def _on_pose(self, msg: PoseStamped):
        self.pose = msg

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _scan_avoidance(self, scan: LaserScan, influence_distance: float, front_half_angle: float, side_sector_width: float) -> tuple[float, float, float, float, float, int]:
        repulse_x = 0.0
        repulse_y = 0.0
        repulse_count = 0
        valid_count = 0
        front_clearance = float("inf")
        left_open_sum = 0.0
        left_open_count = 0
        right_open_sum = 0.0
        right_open_count = 0

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            valid_count += 1

            if abs(angle) <= front_half_angle:
                front_clearance = min(front_clearance, float(distance))

            clipped_distance = min(float(distance), influence_distance)
            if front_half_angle <= angle <= front_half_angle + side_sector_width:
                left_open_sum += clipped_distance
                left_open_count += 1
            elif -(front_half_angle + side_sector_width) <= angle <= -front_half_angle:
                right_open_sum += clipped_distance
                right_open_count += 1

            if distance >= influence_distance:
                continue

            proximity = (influence_distance - float(distance)) / max(influence_distance, 1e-6)
            strength = proximity * proximity
            repulse_x -= math.cos(angle) * strength
            repulse_y -= math.sin(angle) * strength
            repulse_count += 1

        if repulse_count > 0:
            repulse_x /= repulse_count
            repulse_y /= repulse_count
            magnitude = math.hypot(repulse_x, repulse_y)
            if magnitude > 1.0:
                repulse_x /= magnitude
                repulse_y /= magnitude

        if front_clearance == float("inf") and valid_count > 0:
            front_clearance = influence_distance

        left_open = left_open_sum / left_open_count if left_open_count > 0 else influence_distance
        right_open = right_open_sum / right_open_count if right_open_count > 0 else influence_distance
        return repulse_x, repulse_y, front_clearance, left_open, right_open, valid_count

    def _front_speed_scale(self, front_clearance: float, stop_distance: float, slow_distance: float) -> float:
        if front_clearance == float("inf") or front_clearance >= slow_distance:
            return 1.0
        if front_clearance <= stop_distance:
            return 0.0
        return clamp(
            (front_clearance - stop_distance) / max(slow_distance - stop_distance, 1e-6),
            0.0,
            1.0,
        )

    def _tick(self):
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        goal_tol_xy = float(self.get_parameter("goal_tol_xy").value)
        cruise_speed = float(self.get_parameter("cruise_speed").value)
        max_speed = float(self.get_parameter("max_speed").value)
        max_reverse_speed = float(self.get_parameter("max_reverse_speed").value)
        stop_distance = float(self.get_parameter("obstacle_stop_distance").value)
        slow_distance = float(self.get_parameter("obstacle_slow_distance").value)
        influence_distance = float(self.get_parameter("obstacle_influence_distance").value)
        goal_latch_enabled = bool(self.get_parameter("goal_latch_enabled").value)
        allow_motion_without_scan = bool(self.get_parameter("allow_motion_without_scan").value)
        forward_gain = float(self.get_parameter("forward_gain").value)
        avoidance_gain = float(self.get_parameter("avoidance_gain").value)
        lateral_bias_gain = float(self.get_parameter("lateral_bias_gain").value)
        backoff_gain = float(self.get_parameter("backoff_gain").value)
        front_half_angle = math.radians(float(self.get_parameter("front_sector_half_angle_deg").value))
        side_sector_width = math.radians(float(self.get_parameter("side_sector_width_deg").value))

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        goal_reached = self.goal_latched if goal_latch_enabled else False
        vx_world = 0.0
        vy_world = 0.0

        if self.pose is not None:
            px = float(self.pose.pose.position.x)
            py = float(self.pose.pose.position.y)
            dx = goal_x - px
            dy = goal_y - py
            dist = math.hypot(dx, dy)

            if dist <= goal_tol_xy:
                goal_reached = True
                if goal_latch_enabled and not self.goal_latched:
                    self.goal_latched = True
                    self.get_logger().info(f"Goal latched at xy distance {dist:.2f} m")
            else:
                goal_vx_world = cruise_speed * dx / max(dist, 1e-6)
                goal_vy_world = cruise_speed * dy / max(dist, 1e-6)

                if self.scan is None:
                    if allow_motion_without_scan:
                        vx_world = goal_vx_world
                        vy_world = goal_vy_world
                else:
                    yaw = yaw_from_quaternion(self.pose.pose.orientation)
                    goal_vx_body, goal_vy_body = world_to_body(goal_vx_world, goal_vy_world, yaw)
                    repulse_x, repulse_y, front_clearance, left_open, right_open, valid_count = self._scan_avoidance(
                        self.scan,
                        influence_distance,
                        front_half_angle,
                        side_sector_width,
                    )

                    if valid_count == 0 and allow_motion_without_scan:
                        vx_world = goal_vx_world
                        vy_world = goal_vy_world
                    elif valid_count > 0:
                        cmd_x_body = forward_gain * goal_vx_body + avoidance_gain * repulse_x * cruise_speed
                        cmd_y_body = forward_gain * goal_vy_body + avoidance_gain * repulse_y * cruise_speed

                        cmd_x_body *= self._front_speed_scale(front_clearance, stop_distance, slow_distance)

                        if front_clearance < slow_distance:
                            preferred_side = 1.0 if left_open >= right_open else -1.0
                            bias_ratio = clamp(
                                (slow_distance - front_clearance) / max(slow_distance - stop_distance, 1e-6),
                                0.0,
                                1.0,
                            )
                            cmd_y_body += preferred_side * lateral_bias_gain * bias_ratio * cruise_speed

                        if front_clearance < stop_distance:
                            backoff_ratio = clamp(
                                (stop_distance - front_clearance) / max(stop_distance, 1e-6),
                                0.0,
                                1.0,
                            )
                            cmd_x_body -= backoff_gain * backoff_ratio * cruise_speed

                        cmd_x_body = max(cmd_x_body, -max_reverse_speed)
                        vx_world, vy_world = body_to_world(cmd_x_body, cmd_y_body, yaw)

        speed = math.hypot(vx_world, vy_world)
        if speed > max_speed:
            scale = max_speed / max(speed, 1e-6)
            vx_world *= scale
            vy_world *= scale

        msg.twist.linear.x = vx_world
        msg.twist.linear.y = vy_world
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.goal_pub.publish(Bool(data=goal_reached))


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
