#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('node_name')
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('wait_for_ros_node')
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            names = {f'/{name}' for name in node.get_node_names()}
            if args.node_name in names:
                return 0
            rclpy.spin_once(node, timeout_sec=0.2)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
