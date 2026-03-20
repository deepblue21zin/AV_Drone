#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanWaiter(Node):
    def __init__(self, topic: str):
        super().__init__('wait_for_scan_sample')
        self.received = False
        self.create_subscription(LaserScan, topic, self._on_scan, 10)

    def _on_scan(self, _msg: LaserScan):
        self.received = True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/drone1/scan')
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = ScanWaiter(args.topic)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.received:
                return 0
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
