#!/usr/bin/env python3
import argparse
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("flight_success_waiter")
    state = {"phase": "", "goal": False, "armed": False, "connected": False, "pose": None}
    node.create_subscription(String, "/drone1/mission/phase", lambda m: state.update(phase=m.data), 10)
    node.create_subscription(Bool, "/drone1/mission/goal_reached", lambda m: state.update(goal=bool(m.data) or state["goal"]), 10)
    node.create_subscription(State, "/mavros/state", lambda m: state.update(armed=m.armed, connected=m.connected), 10)
    node.create_subscription(PoseStamped, "/mavros/local_position/pose", lambda m: state.update(pose=[m.pose.position.x, m.pose.position.y, m.pose.position.z]), qos_profile_sensor_data)

    started = time.monotonic()
    goal_at = None
    success = False
    while time.monotonic() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if state["goal"] and goal_at is None:
            goal_at = time.monotonic()
        # mission/phase is transition-only and a late subscriber can miss the
        # HOVER_AT_GOAL message. goal_reached is the persistent planner result,
        # so it is sufficient for outbound-run success.
        if goal_at is not None:
            success = True
            break
    result = dict(state)
    result["success"] = success
    result["elapsed_sec"] = time.monotonic() - started
    result["goal_elapsed_sec"] = None if goal_at is None else goal_at - started
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
