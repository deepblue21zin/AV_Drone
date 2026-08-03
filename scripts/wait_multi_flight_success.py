#!/usr/bin/env python3
import argparse
import json
import sys
import time

import rclpy
from std_msgs.msg import Bool, String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--vehicle-count", type=int, default=3)
    parser.add_argument("--progress-interval", type=float, default=10.0)
    args = parser.parse_args()

    if args.vehicle_count < 1 or args.vehicle_count > 3:
        parser.error("--vehicle-count must be between 1 and 3")
    if args.progress_interval <= 0.0:
        parser.error("--progress-interval must be positive")

    rclpy.init()
    node = rclpy.create_node("multi_flight_success_waiter")
    states = {
        name: {"phase": "", "started": False, "goal": False}
        for name in (f"drone{index}" for index in range(1, args.vehicle_count + 1))
    }

    for name in states:
        def update_phase(msg, drone=name):
            states[drone]["phase"] = msg.data
            if msg.data == "MPPI_GO":
                states[drone]["started"] = True

        def update_goal(msg, drone=name):
            if states[drone]["started"] and bool(msg.data):
                states[drone]["goal"] = True

        node.create_subscription(
            String,
            f"/{name}/mission/phase",
            update_phase,
            10,
        )
        node.create_subscription(
            Bool,
            f"/{name}/mission/goal_reached",
            update_goal,
            10,
        )

    started = time.monotonic()
    next_progress = started
    while time.monotonic() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now >= next_progress:
            details = ", ".join(
                f"{name}={state['phase'] or 'WAITING'}"
                f"/started={str(state['started']).lower()}"
                f"/goal={str(state['goal']).lower()}"
                for name, state in states.items()
            )
            print(
                f"[progress {now - started:.1f}/{args.timeout:.1f}s] {details}",
                file=sys.stderr,
                flush=True,
            )
            next_progress = now + args.progress_interval
        if all(state["goal"] for state in states.values()):
            result = {
                "success": True,
                "elapsed_sec": time.monotonic() - started,
                "drones": states,
            }
            print(json.dumps(result))
            node.destroy_node()
            rclpy.shutdown()
            return

    result = {
        "success": False,
        "elapsed_sec": time.monotonic() - started,
        "drones": states,
    }
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)


if __name__ == "__main__":
    main()
