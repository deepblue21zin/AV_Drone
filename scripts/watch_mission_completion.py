#!/usr/bin/env python3
"""Observe simulation terminal phases; these are NOT landed/disarmed states."""

import argparse
import json
from pathlib import Path
import time


class CompletionTracker:
    def __init__(self, targets, hold_sec=5, stale_sec=3):
        if not targets or hold_sec <= 0 or stale_sec <= 0:
            raise ValueError("Nonempty targets and positive timing limits required")
        self.targets = targets
        self.hold_sec = hold_sec
        self.stale_sec = stale_sec
        self.seen = {}
        self.since = None

    def update(self, topic, phase, now):
        self.seen[topic] = (phase, now)

    def complete(self, now):
        matched = all(topic in self.seen and self.seen[topic][0] == phase and
                      now - self.seen[topic][1] <= self.stale_sec
                      for topic, phase in self.targets.items())
        if not matched:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.hold_sec


def main():
    import rclpy
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", required=True)
    args = parser.parse_args()
    policy = json.loads(Path(args.policy).read_text())
    tracker = CompletionTracker(policy["targets"])
    rclpy.init()
    node = rclpy.create_node("guarded_experiment_completion")
    subscriptions = []
    for topic in tracker.targets:
        subscriptions.append(node.create_subscription(
            String, topic, lambda message, key=topic: tracker.update(key, message.data, time.monotonic()), 10))
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if tracker.complete(now):
                print("[mission guard] all requested terminal phases held for 5 seconds", flush=True)
                return 0
            missing = [topic for topic in tracker.targets if topic not in tracker.seen]
            stale = [topic for topic, (_, seen) in tracker.seen.items() if now - seen > 10]
            if (missing and now - started > 120) or stale:
                print(f"[mission guard] missing/stale phase heartbeat: {missing or stale}", flush=True)
                return 1
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
