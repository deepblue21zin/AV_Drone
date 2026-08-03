#!/usr/bin/env python3
import argparse
import json
import time

import rclpy
from std_msgs.msg import Bool, String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=240.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("multi_flight_success_waiter")
    states = {
        name: {"phase": "", "goal": False}
        for name in ("drone1", "drone2")
    }

    for name in states:
        node.create_subscription(
            String,
            f"/{name}/mission/phase",
            lambda msg, drone=name: states[drone].update(phase=msg.data),
            10,
        )
        node.create_subscription(
            Bool,
            f"/{name}/mission/goal_reached",
            lambda msg, drone=name: states[drone].update(
                goal=states[drone]["goal"] or bool(msg.data)
            ),
            10,
        )

    started = time.monotonic()
    while time.monotonic() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
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
