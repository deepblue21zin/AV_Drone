#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
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
        self.declare_parameter("home_pose_topic", "/drone1/mission/home_pose")
        self.declare_parameter("active_goal_topic", "/drone1/mission/active_goal")

        self.declare_parameter("takeoff_z", 3.0)
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 3.0)
        self.declare_parameter("home_goal_z", 3.0)

        self.declare_parameter("hover_sec_after_takeoff", 2.0)
        self.declare_parameter("hover_sec_at_goal", 2.0)
        self.declare_parameter("hover_sec_at_home", 3.0)

        self.declare_parameter("kp_z", 1.2)
        self.declare_parameter("vz_max", 1.0)
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("prestream_setpoints", 40)
        self.declare_parameter("takeoff_skip_margin", 0.25)

        self.declare_parameter("continuous_mode", False)
        self.declare_parameter("return_home_enabled", False)
        self.declare_parameter("return_mode", "avoid")

        pose_topic = str(self.get_parameter("pose_topic").value)

        self.vehicle = VehicleInterface(
            self,
            str(self.get_parameter("mavros_namespace").value),
            pose_topic=pose_topic,
        )

        self.safe_cmd_topic = str(self.get_parameter("safe_cmd_topic").value)
        self.goal_reached_topic = str(self.get_parameter("goal_reached_topic").value)

        self.phase_pub = self.create_publisher(
            String,
            str(self.get_parameter("mission_phase_topic").value),
            10,
        )
        self.home_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("home_pose_topic").value),
            10,
        )
        self.active_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("active_goal_topic").value),
            10,
        )

        self._latest_cmd = TwistStamped()
        self._have_cmd = False
        self._goal_reached = False

        self._last_mode_req_t = 0.0
        self._last_arm_req_t = 0.0

        self._phase = "WAIT_STREAM"
        self._phase_t0 = time.time()
        self._prestream_count = 0

        self._takeoff_start_z = None
        self._ground_reference_z = None
        self._home_pose = None

        self._active_goal = self._make_goal_pose(
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value),
        )
        self._return_goal_active = False

        self.create_subscription(TwistStamped, self.safe_cmd_topic, self._on_cmd, 10)
        self.create_subscription(Bool, self.goal_reached_topic, self._on_goal_reached, 10)

        rate_hz = max(float(self.get_parameter("cmd_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(1.0, self._publish_phase_heartbeat)

        self.phase_pub.publish(String(data=self._phase))
        self._publish_active_goal()

        self.get_logger().info(
            f"Autonomy manager ready: pose={pose_topic}, "
            f"safe_cmd={self.safe_cmd_topic}, goal_reached={self.goal_reached_topic}"
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
        req.value = bool(arm)
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

    def _make_goal_pose(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        return msg

    def _copy_current_pose(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = self.vehicle.pose.pose
        return msg

    def _publish_home_pose(self):
        if self._home_pose is None:
            return
        self._home_pose.header.stamp = self.get_clock().now().to_msg()
        self.home_pose_pub.publish(self._home_pose)

    def _publish_active_goal(self):
        if self._active_goal is None:
            return
        self._active_goal.header.stamp = self.get_clock().now().to_msg()
        self.active_goal_pub.publish(self._active_goal)

    def _capture_home_pose(self):
        if self.vehicle.pose is None:
            return
        self._home_pose = self._copy_current_pose()
        self._publish_home_pose()
        p = self._home_pose.pose.position
        self.get_logger().info(
            f"Home pose captured: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}"
        )

    def _set_outbound_goal(self):
        self._active_goal = self._make_goal_pose(
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value),
        )
        self._return_goal_active = False
        self._goal_reached = False
        self._publish_active_goal()

    def _set_home_goal(self) -> bool:
        if self._home_pose is None:
            self._capture_home_pose()

        if self._home_pose is None:
            return False

        p = self._home_pose.pose.position
        self._active_goal = self._make_goal_pose(
            float(p.x),
            float(p.y),
            float(self.get_parameter("home_goal_z").value),
        )
        self._return_goal_active = True
        self._goal_reached = False
        self._publish_active_goal()

        self.get_logger().info(
            f"Return goal activated: x={p.x:.2f}, y={p.y:.2f}"
        )
        return True

    def _enter_phase(self, name: str):
        if self._phase == name:
            return

        self._phase = name
        self._phase_t0 = time.time()

        if name == "OFFBOARD_ARM":
            self._takeoff_start_z = None

        self.phase_pub.publish(String(data=name))
        self.get_logger().info(f"PHASE => {name}")

    def _phase_elapsed(self) -> float:
        return time.time() - self._phase_t0

    def _publish_phase_heartbeat(self):
        self.phase_pub.publish(String(data=self._phase))
        self._publish_home_pose()
        self._publish_active_goal()

    def _get_xyz_yaw(self):
        pose = self.vehicle.pose
        p = pose.pose.position
        q = pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return float(p.x), float(p.y), float(p.z), float(yaw)

    def _forward_latest_plan_cmd(self, target_z: float, current_z: float, kp_z: float, vz_max: float):
        err_z = target_z - current_z
        vz_hold = clamp(kp_z * err_z, -vz_max, vz_max)
        cmd = self._latest_cmd if self._have_cmd else TwistStamped()

        self._publish_cmd(
            cmd.twist.linear.x,
            cmd.twist.linear.y,
            vz_hold,
            cmd.twist.angular.z,
        )

    def _tick(self):
        now = time.time()

        pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        takeoff_z = float(self.get_parameter("takeoff_z").value)
        goal_z = float(self.get_parameter("goal_z").value)
        home_goal_z = float(self.get_parameter("home_goal_z").value)

        hover_after_takeoff = float(self.get_parameter("hover_sec_after_takeoff").value)
        hover_at_goal = float(self.get_parameter("hover_sec_at_goal").value)
        hover_at_home = float(self.get_parameter("hover_sec_at_home").value)

        kp_z = float(self.get_parameter("kp_z").value)
        vz_max = float(self.get_parameter("vz_max").value)

        prestream_setpoints = int(self.get_parameter("prestream_setpoints").value)
        takeoff_skip_margin = float(self.get_parameter("takeoff_skip_margin").value)

        continuous_mode = bool(self.get_parameter("continuous_mode").value)
        return_home_enabled = bool(self.get_parameter("return_home_enabled").value)
        return_mode = str(self.get_parameter("return_mode").value).strip().lower()

        if not self.vehicle.state.connected:
            return

        if self.vehicle.pose is None or self.vehicle.pose_age() > pose_timeout:
            return

        _, _, z, _ = self._get_xyz_yaw()

        if self._phase in {"WAIT_STREAM", "OFFBOARD_ARM"} and not self.vehicle.state.armed:
            if self._ground_reference_z is None:
                self._ground_reference_z = z
            else:
                self._ground_reference_z = min(self._ground_reference_z, z)

        reference_z = self._takeoff_start_z if self._takeoff_start_z is not None else z
        takeoff_target_z = reference_z + takeoff_z
        goal_target_z = reference_z + goal_z
        home_target_z = reference_z + home_goal_z

        self._publish_home_pose()
        self._publish_active_goal()

        if self._phase == "WAIT_STREAM":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self._prestream_count += 1

            if self._prestream_count >= prestream_setpoints:
                self._enter_phase("OFFBOARD_ARM")

            return

        if self._phase not in {"WAIT_STREAM", "OFFBOARD_ARM", "LAND_AT_GOAL", "LANDED"}:
            if self.vehicle.state.mode != "OFFBOARD" or not self.vehicle.state.armed:
                self._enter_phase("OFFBOARD_ARM")
                return

        if self._phase == "OFFBOARD_ARM":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)

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

            if z >= (takeoff_z - takeoff_skip_margin):
                self._takeoff_start_z = z - takeoff_z
                self._capture_home_pose()
                self._set_outbound_goal()

                self.get_logger().info(
                    f"Already airborne at z={z:.2f}; skipping TAKEOFF. "
                    f"effective_start_z={self._takeoff_start_z:.2f}, "
                    f"takeoff_target_z={self._takeoff_start_z + takeoff_z:.2f}, "
                    f"goal_target_z={self._takeoff_start_z + goal_z:.2f}"
                )

                self._enter_phase("MAPPING_TO_GOAL")
                return

            if self._ground_reference_z is not None:
                self._takeoff_start_z = self._ground_reference_z
            else:
                self._takeoff_start_z = z

            takeoff_target_z = self._takeoff_start_z + takeoff_z
            goal_target_z = self._takeoff_start_z + goal_z
            home_target_z = self._takeoff_start_z + home_goal_z

            self._set_outbound_goal()

            self.get_logger().info(
                f"Takeoff reference locked: start_z={self._takeoff_start_z:.2f}, "
                f"current_z={z:.2f}, takeoff_target_z={takeoff_target_z:.2f}, "
                f"goal_target_z={goal_target_z:.2f}"
            )

            self._enter_phase("TAKEOFF")
            return

        if self._phase == "TAKEOFF":
            err_z = takeoff_target_z - z
            vz_cmd = clamp(kp_z * err_z, -vz_max, vz_max)

            if err_z > 0.2:
                vz_cmd = clamp(vz_cmd, 0.2, vz_max)

            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if z >= takeoff_target_z - 0.15:
                self._capture_home_pose()
                self._enter_phase("HOVER_AFTER_TAKEOFF")

            return

        if self._phase == "HOVER_AFTER_TAKEOFF":
            err_z = takeoff_target_z - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if self._phase_elapsed() >= hover_after_takeoff:
                self._set_outbound_goal()
                self._enter_phase("MAPPING_TO_GOAL")

            return

        if self._phase == "FOLLOW_PLAN":
            if self._goal_reached and not continuous_mode:
                self._enter_phase("HOVER_AT_GOAL")
                return

            self._forward_latest_plan_cmd(goal_target_z, z, kp_z, vz_max)
            return

        if self._phase == "MAPPING_TO_GOAL":
            if self._goal_reached and not continuous_mode:
                if return_home_enabled:
                    self._enter_phase("HOVER_AT_GOAL")
                else:
                    self._enter_phase("LAND_AT_GOAL")
                return

            self._forward_latest_plan_cmd(goal_target_z, z, kp_z, vz_max)
            return

        if self._phase == "LAND_AT_GOAL":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)

            if self.vehicle.state.mode != "AUTO.LAND":
                if (now - self._last_mode_req_t) > 1.0:
                    self._request_mode("AUTO.LAND")
                    self._last_mode_req_t = now
                    self.get_logger().info("Requesting AUTO.LAND")
                return

            if not self.vehicle.state.armed:
                self._enter_phase("LANDED")

            return

        if self._phase == "LANDED":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            return

        if self._phase == "HOVER_AT_GOAL":
            err_z = goal_target_z - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if not return_home_enabled:
                return

            if self._phase_elapsed() < hover_at_goal:
                return

            if return_mode == "mppi":
                if self._set_home_goal():
                    self._enter_phase("BUILD_RETURN_COSTMAP")
                return

            if self._set_home_goal():
                self._enter_phase("RETURN_HOME_AVOID")

            return

        if self._phase == "BUILD_RETURN_COSTMAP":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)

            if self._phase_elapsed() >= 1.0:
                self._enter_phase("RETURN_HOME_MPPI")

            return

        if self._phase in {"RETURN_HOME_AVOID", "RETURN_HOME_MPPI"}:
            if self._goal_reached:
                self._enter_phase("HOVER_AT_HOME")
                return

            self._forward_latest_plan_cmd(home_target_z, z, kp_z, vz_max)
            return

        if self._phase == "HOVER_AT_HOME":
            err_z = home_target_z - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if self._phase_elapsed() >= hover_at_home:
                self._enter_phase("DONE")

            return

        if self._phase == "DONE":
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            return


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()