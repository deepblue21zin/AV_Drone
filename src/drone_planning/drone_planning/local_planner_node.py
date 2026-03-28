#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_body(vx_world: float, vy_world: float, yaw: float) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vx_body = cos_yaw * vx_world + sin_yaw * vy_world
    vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
    return vx_body, vy_body


def body_to_world(vx_body: float, vy_body: float, yaw: float) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vx_world = cos_yaw * vx_body - sin_yaw * vy_body
    vy_world = sin_yaw * vx_body + cos_yaw * vy_body
    return vx_world, vy_world


@dataclass
class GapCandidate:
    start_idx: int
    end_idx: int
    target_idx: int
    start_angle: float
    end_angle: float
    target_angle: float
    width_rad: float
    mean_clearance: float
    target_clearance: float
    score: float


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner")

        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("scan_topic", "/drone1/scan")
        self.declare_parameter("autonomy_cmd_topic", "/drone1/autonomy/cmd_vel")
        self.declare_parameter("goal_reached_topic", "/drone1/mission/goal_reached")
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tol_xy", 0.5)
        self.declare_parameter("goal_latch_enabled", True)
        self.declare_parameter("allow_motion_without_scan", False)
        self.declare_parameter("cruise_speed", 0.9)
        self.declare_parameter("max_speed", 1.1)
        self.declare_parameter("max_reverse_speed", 0.3)
        self.declare_parameter("obstacle_stop_distance", 1.5)
        self.declare_parameter("obstacle_slow_distance", 3.0)
        self.declare_parameter("obstacle_influence_distance", 4.5)
        self.declare_parameter("forward_gain", 1.0)
        self.declare_parameter("avoidance_gain", 1.0)
        self.declare_parameter("lateral_bias_gain", 0.9)
        self.declare_parameter("backoff_gain", 0.6)
        self.declare_parameter("front_sector_half_angle_deg", 26.0)
        self.declare_parameter("side_sector_width_deg", 85.0)
        self.declare_parameter("gap_search_half_angle_deg", 100.0)
        self.declare_parameter("gap_bubble_radius", 0.9)
        self.declare_parameter("gap_max_bubbles", 6)
        self.declare_parameter("gap_min_clearance", 1.35)
        self.declare_parameter("gap_min_points", 8)
        self.declare_parameter("gap_edge_margin_deg", 6.0)
        self.declare_parameter("gap_goal_weight", 2.2)
        self.declare_parameter("gap_clearance_weight", 0.85)
        self.declare_parameter("gap_width_weight", 0.45)
        self.declare_parameter("gap_clearance_relax_ratio", 0.82)
        self.declare_parameter("gap_center_bias", 0.65)
        self.declare_parameter("gap_target_commitment", 0.45)
        self.declare_parameter("gap_clearance_percentile", 0.25)
        self.declare_parameter("gap_escape_distance", 2.2)
        self.declare_parameter("target_sector_half_angle_deg", 8.0)
        self.declare_parameter("gap_turn_min_scale", 0.35)
        self.declare_parameter("side_clearance_sector_deg", 75.0)

        pose_topic = str(self.get_parameter("pose_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        cmd_topic = str(self.get_parameter("autonomy_cmd_topic").value)
        goal_topic = str(self.get_parameter("goal_reached_topic").value)

        self.cmd_pub = self.create_publisher(TwistStamped, cmd_topic, 10)
        self.goal_pub = self.create_publisher(Bool, goal_topic, 10)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)

        self.pose: Optional[PoseStamped] = None
        self.scan: Optional[LaserScan] = None
        self.last_scan_time: Optional[float] = None
        self.goal_latched = False
        self._last_target_angle: Optional[float] = None

        self.get_logger().info(
            f"Gap-based local planner ready: pose={pose_topic}, scan={scan_topic}, cmd={cmd_topic}"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg
        self.last_scan_time = time.time()

    def _publish_cmd(self, vx_world: float, vy_world: float, yaw_rate: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(vx_world)
        msg.twist.linear.y = float(vy_world)
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = float(yaw_rate)
        self.cmd_pub.publish(msg)

    def _publish_goal_reached(self, value: bool) -> None:
        self.goal_pub.publish(Bool(data=bool(value)))

    def _front_speed_scale(self, distance: float) -> float:
        stop_distance = float(self.get_parameter("obstacle_stop_distance").value)
        slow_distance = float(self.get_parameter("obstacle_slow_distance").value)

        if distance <= stop_distance:
            return 0.0
        if distance >= slow_distance:
            return 1.0
        return clamp((distance - stop_distance) / max(slow_distance - stop_distance, 1e-6), 0.0, 1.0)

    def _scan_samples(self) -> Optional[Tuple[List[float], List[float], List[bool], float]]:
        if self.scan is None:
            return None

        search_half_angle = math.radians(float(self.get_parameter("gap_search_half_angle_deg").value))
        angles: List[float] = []
        ranges: List[float] = []
        valid: List[bool] = []

        for i, raw_range in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(angle) > search_half_angle:
                continue

            if math.isinf(raw_range) and raw_range > 0.0:
                distance = float(self.scan.range_max)
                is_valid = True
            elif math.isnan(raw_range):
                distance = float(self.scan.range_min)
                is_valid = False
            elif raw_range >= self.scan.range_min:
                distance = clamp(float(raw_range), float(self.scan.range_min), float(self.scan.range_max))
                is_valid = True
            else:
                distance = float(self.scan.range_min)
                is_valid = False

            angles.append(float(angle))
            ranges.append(distance)
            valid.append(is_valid)

        if not angles:
            return None

        angle_increment = abs(float(self.scan.angle_increment)) if self.scan.angle_increment != 0.0 else 0.01
        return angles, ranges, valid, angle_increment

    def _build_gap_mask(
        self,
        angles: Sequence[float],
        ranges: Sequence[float],
        valid: Sequence[bool],
        angle_increment: float,
        min_clearance: float,
    ) -> List[bool]:
        free_mask = [bool(valid[i] and ranges[i] >= min_clearance) for i in range(len(ranges))]
        candidate_indices = [i for i, is_valid in enumerate(valid) if is_valid]
        if not candidate_indices:
            return free_mask

        influence_distance = float(self.get_parameter("obstacle_influence_distance").value)
        bubble_radius = float(self.get_parameter("gap_bubble_radius").value)
        max_bubbles = max(1, int(self.get_parameter("gap_max_bubbles").value))

        if bubble_radius <= 0.0:
            return free_mask

        applied = 0
        bubble_covered = [False] * len(free_mask)
        for obstacle_idx in sorted(candidate_indices, key=lambda idx: ranges[idx]):
            obstacle_range = ranges[obstacle_idx]
            if obstacle_range >= influence_distance:
                break
            if bubble_covered[obstacle_idx]:
                continue

            bubble_half_angle = math.atan2(bubble_radius, max(obstacle_range, 0.05))
            bubble_span = max(1, int(math.ceil(bubble_half_angle / max(angle_increment, 1e-6))))
            start = max(0, obstacle_idx - bubble_span)
            end = min(len(free_mask) - 1, obstacle_idx + bubble_span)
            for idx in range(start, end + 1):
                free_mask[idx] = False
                bubble_covered[idx] = True

            applied += 1
            if applied >= max_bubbles:
                break

        return free_mask

    def _find_best_gap(
        self,
        angles: Sequence[float],
        ranges: Sequence[float],
        free_mask: Sequence[bool],
        goal_angle: float,
        angle_increment: float,
    ) -> Optional[GapCandidate]:
        min_points = int(self.get_parameter("gap_min_points").value)
        edge_margin = math.radians(float(self.get_parameter("gap_edge_margin_deg").value))
        margin_points = max(0, int(math.ceil(edge_margin / max(angle_increment, 1e-6))))
        search_half_angle = math.radians(float(self.get_parameter("gap_search_half_angle_deg").value))
        clearance_norm = max(float(self.get_parameter("obstacle_influence_distance").value), 0.1)
        goal_weight = float(self.get_parameter("gap_goal_weight").value)
        clearance_weight = float(self.get_parameter("gap_clearance_weight").value)
        width_weight = float(self.get_parameter("gap_width_weight").value)

        best: Optional[GapCandidate] = None
        idx = 0
        while idx < len(free_mask):
            if not free_mask[idx]:
                idx += 1
                continue

            start_idx = idx
            while idx < len(free_mask) and free_mask[idx]:
                idx += 1
            end_idx = idx - 1

            usable_start = start_idx + margin_points
            usable_end = end_idx - margin_points
            if usable_end < usable_start:
                usable_start = start_idx
                usable_end = end_idx

            if (usable_end - usable_start + 1) < min_points:
                continue

            target_idx = min(
                range(usable_start, usable_end + 1),
                key=lambda candidate_idx: abs(normalize_angle(angles[candidate_idx] - goal_angle)),
            )
            start_angle = angles[usable_start]
            end_angle = angles[usable_end]
            width_rad = max(0.0, end_angle - start_angle)
            mean_clearance = sum(ranges[usable_start : usable_end + 1]) / float(usable_end - usable_start + 1)
            target_clearance = ranges[target_idx]

            goal_alignment = 1.0 - min(abs(normalize_angle(angles[target_idx] - goal_angle)) / max(search_half_angle, 1e-6), 1.0)
            clearance_score = clamp(mean_clearance / clearance_norm, 0.0, 1.5)
            width_score = clamp(width_rad / max(2.0 * search_half_angle, 1e-6), 0.0, 1.0)
            score = (
                goal_weight * goal_alignment
                + clearance_weight * clearance_score
                + width_weight * width_score
            )

            candidate = GapCandidate(
                start_idx=usable_start,
                end_idx=usable_end,
                target_idx=target_idx,
                start_angle=start_angle,
                end_angle=end_angle,
                target_angle=angles[target_idx],
                width_rad=width_rad,
                mean_clearance=mean_clearance,
                target_clearance=target_clearance,
                score=score,
            )
            if best is None or candidate.score > best.score:
                best = candidate

        return best

    def _clearance_near_angle(
        self,
        angles: Sequence[float],
        ranges: Sequence[float],
        valid: Sequence[bool],
        target_angle: float,
        half_angle: float,
    ) -> float:
        values = [
            ranges[idx]
            for idx, angle in enumerate(angles)
            if valid[idx] and abs(normalize_angle(angle - target_angle)) <= half_angle
        ]
        if not values:
            return float("inf")
        values.sort()
        percentile = clamp(float(self.get_parameter("gap_clearance_percentile").value), 0.0, 1.0)
        percentile_idx = int(math.floor(percentile * max(len(values) - 1, 0)))
        return values[percentile_idx]

    def _side_clearance(
        self,
        angles: Sequence[float],
        ranges: Sequence[float],
        valid: Sequence[bool],
        positive_side: bool,
    ) -> float:
        sector_half = math.radians(float(self.get_parameter("side_clearance_sector_deg").value))
        values = []
        for idx, angle in enumerate(angles):
            if not valid[idx]:
                continue
            if positive_side and 0.0 < angle <= sector_half:
                values.append(ranges[idx])
            if not positive_side and -sector_half <= angle < 0.0:
                values.append(ranges[idx])
        if not values:
            return 0.0
        return sum(values) / float(len(values))

    def _escape_command(
        self,
        angles: Sequence[float],
        ranges: Sequence[float],
        valid: Sequence[bool],
    ) -> Tuple[float, float]:
        left_clearance = self._side_clearance(angles, ranges, valid, positive_side=True)
        right_clearance = self._side_clearance(angles, ranges, valid, positive_side=False)
        lateral_sign = 1.0 if left_clearance >= right_clearance else -1.0

        backoff_gain = float(self.get_parameter("backoff_gain").value)
        lateral_gain = float(self.get_parameter("lateral_bias_gain").value)
        cruise_speed = float(self.get_parameter("cruise_speed").value)
        max_reverse_speed = float(self.get_parameter("max_reverse_speed").value)
        front_sector = math.radians(float(self.get_parameter("front_sector_half_angle_deg").value))
        front_clearance = self._clearance_near_angle(angles, ranges, valid, 0.0, front_sector)

        reverse_speed = min(max_reverse_speed, backoff_gain * cruise_speed)
        if front_clearance > float(self.get_parameter("obstacle_stop_distance").value):
            reverse_speed = 0.0

        lateral_speed = lateral_gain * min(cruise_speed * 0.8, max_reverse_speed + 0.25)
        return -reverse_speed, lateral_sign * lateral_speed

    def _direct_goal_body_cmd(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        goal_dx = float(self.get_parameter("goal_x").value) - x
        goal_dy = float(self.get_parameter("goal_y").value) - y
        goal_vx_body, goal_vy_body = world_to_body(goal_dx, goal_dy, yaw)
        goal_norm = math.hypot(goal_vx_body, goal_vy_body)
        if goal_norm < 1e-6:
            return 0.0, 0.0

        speed = min(float(self.get_parameter("cruise_speed").value), goal_norm)
        return speed * goal_vx_body / goal_norm, speed * goal_vy_body / goal_norm

    def _tick(self) -> None:
        if self.pose is None:
            self._publish_goal_reached(False)
            self._publish_cmd(0.0, 0.0)
            return

        position = self.pose.pose.position
        orientation = self.pose.pose.orientation
        yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        goal_tol_xy = float(self.get_parameter("goal_tol_xy").value)
        goal_latch_enabled = bool(self.get_parameter("goal_latch_enabled").value)

        goal_dx_world = goal_x - float(position.x)
        goal_dy_world = goal_y - float(position.y)
        goal_distance = math.hypot(goal_dx_world, goal_dy_world)
        current_goal_reached = goal_distance <= goal_tol_xy

        if current_goal_reached and goal_latch_enabled:
            self.goal_latched = True
        if not goal_latch_enabled:
            self.goal_latched = current_goal_reached

        goal_reached = self.goal_latched if goal_latch_enabled else current_goal_reached
        self._publish_goal_reached(goal_reached)

        if goal_reached:
            self._last_target_angle = None
            self._publish_cmd(0.0, 0.0)
            return

        if self.scan is None:
            if not bool(self.get_parameter("allow_motion_without_scan").value):
                self._last_target_angle = None
                self._publish_cmd(0.0, 0.0)
                return
            vx_body, vy_body = self._direct_goal_body_cmd(float(position.x), float(position.y), yaw)
            vx_world, vy_world = body_to_world(vx_body, vy_body, yaw)
            self._publish_cmd(vx_world, vy_world)
            return

        sample_bundle = self._scan_samples()
        if sample_bundle is None:
            self._last_target_angle = None
            self._publish_cmd(0.0, 0.0)
            return

        angles, ranges, valid, angle_increment = sample_bundle
        goal_vx_body, goal_vy_body = world_to_body(goal_dx_world, goal_dy_world, yaw)
        goal_angle = math.atan2(goal_vy_body, goal_vx_body)

        desired_clearance = float(self.get_parameter("gap_min_clearance").value)
        free_mask = self._build_gap_mask(angles, ranges, valid, angle_increment, desired_clearance)
        best_gap = self._find_best_gap(angles, ranges, free_mask, goal_angle, angle_increment)

        if best_gap is None:
            relax_ratio = float(self.get_parameter("gap_clearance_relax_ratio").value)
            if 0.0 < relax_ratio < 1.0:
                relaxed_clearance = desired_clearance * relax_ratio
                free_mask = self._build_gap_mask(angles, ranges, valid, angle_increment, relaxed_clearance)
                best_gap = self._find_best_gap(angles, ranges, free_mask, goal_angle, angle_increment)

        if best_gap is None:
            vx_body, vy_body = self._escape_command(angles, ranges, valid)
            self._last_target_angle = None
            vx_world, vy_world = body_to_world(vx_body, vy_body, yaw)
            vx_world = clamp(vx_world, -float(self.get_parameter("max_speed").value), float(self.get_parameter("max_speed").value))
            vy_world = clamp(vy_world, -float(self.get_parameter("max_speed").value), float(self.get_parameter("max_speed").value))
            self._publish_cmd(vx_world, vy_world)
            return

        target_sector_half_angle = math.radians(float(self.get_parameter("target_sector_half_angle_deg").value))
        gap_center_angle = 0.5 * (best_gap.start_angle + best_gap.end_angle)
        center_bias = clamp(float(self.get_parameter("gap_center_bias").value), 0.0, 1.0)
        target_angle = normalize_angle(
            center_bias * gap_center_angle + (1.0 - center_bias) * best_gap.target_angle
        )
        target_angle = clamp(target_angle, best_gap.start_angle, best_gap.end_angle)

        commitment = clamp(float(self.get_parameter("gap_target_commitment").value), 0.0, 1.0)
        if (
            self._last_target_angle is not None
            and best_gap.start_angle <= self._last_target_angle <= best_gap.end_angle
        ):
            target_angle = normalize_angle(
                commitment * self._last_target_angle + (1.0 - commitment) * target_angle
            )
            target_angle = clamp(target_angle, best_gap.start_angle, best_gap.end_angle)

        target_clearance = self._clearance_near_angle(
            angles,
            ranges,
            valid,
            target_angle,
            target_sector_half_angle,
        )
        if not math.isfinite(target_clearance):
            target_clearance = best_gap.target_clearance

        speed_scale = self._front_speed_scale(target_clearance)
        turn_scale = max(
            float(self.get_parameter("gap_turn_min_scale").value),
            math.cos(min(abs(target_angle), math.pi / 2.0)),
        )
        cruise_speed = float(self.get_parameter("cruise_speed").value)
        forward_gain = float(self.get_parameter("forward_gain").value)
        lateral_gain = float(self.get_parameter("lateral_bias_gain").value)
        max_speed = float(self.get_parameter("max_speed").value)
        escape_distance = float(self.get_parameter("gap_escape_distance").value)

        base_speed = min(max_speed, cruise_speed * speed_scale)
        speed = base_speed * turn_scale

        if target_clearance <= escape_distance:
            vx_body, vy_body = self._escape_command(angles, ranges, valid)
            self._last_target_angle = None
        else:
            forward_scale = max(0.0, math.cos(target_angle))
            vx_body = forward_gain * speed * forward_scale
            vy_body = lateral_gain * base_speed * math.sin(target_angle)
            self._last_target_angle = target_angle

        vx_world, vy_world = body_to_world(vx_body, vy_body, yaw)
        vx_world = clamp(vx_world, -max_speed, max_speed)
        vy_world = clamp(vy_world, -max_speed, max_speed)
        self._publish_cmd(vx_world, vy_world)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
