#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class ForwardProgressMonitor(Node):
    def __init__(self, stall_seconds, min_progress, max_altitude):
        super().__init__("forward_progress_monitor")
        self.stall_seconds = stall_seconds
        self.min_progress = min_progress
        self.max_altitude = max_altitude
        self.phase = ""
        self.best_x = None
        self.last_progress = None
        self.failure = None
        self.last_report = 0.0
        self.create_subscription(String, "/drone1/mission/phase", self.on_phase, 10)
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.on_pose,
            qos_profile_sensor_data,
        )

    def on_phase(self, message):
        self.phase = message.data
        if self.phase == "MAPPING_TO_GOAL" and self.last_progress is None:
            self.last_progress = time.monotonic()

    def on_pose(self, message):
        now = time.monotonic()
        position = message.pose.position
        orientation = message.pose.orientation
        norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2
            + orientation.z ** 2 + orientation.w ** 2
        )

        if position.z > self.max_altitude or position.z < -0.5:
            self.failure = "altitude_out_of_range z={:.3f}".format(position.z)
            return
        if not 0.8 <= norm <= 1.2:
            self.failure = "invalid_attitude_norm qnorm={:.3f}".format(norm)
            return

        if self.phase == "MAPPING_TO_GOAL":
            if self.best_x is None or position.x >= self.best_x + self.min_progress:
                self.best_x = position.x
                self.last_progress = now
            elif self.last_progress is not None and now - self.last_progress >= self.stall_seconds:
                self.failure = "forward_stall x={:.3f} best_x={:.3f} seconds={:.1f}".format(
                    position.x, self.best_x, now - self.last_progress
                )

        if now - self.last_report >= 1.0:
            self.last_report = now
            print(
                "phase={} x={:.3f} y={:.3f} z={:.3f} best_x={}".format(
                    self.phase,
                    position.x,
                    position.y,
                    position.z,
                    "-" if self.best_x is None else "{:.3f}".format(self.best_x),
                ),
                flush=True,
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stall-seconds", type=float, default=3.0)
    parser.add_argument("--min-progress", type=float, default=0.05)
    parser.add_argument("--max-altitude", type=float, default=6.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = ForwardProgressMonitor(
        args.stall_seconds, args.min_progress, args.max_altitude
    )
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and node.failure is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.failure:
        print("MONITOR_FAILURE {}".format(node.failure), flush=True)
        return 2
    print("MONITOR_TIMEOUT", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
