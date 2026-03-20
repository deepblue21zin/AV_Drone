#!/usr/bin/env python3
import math
import re
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


FLOAT_PAT = re.compile(r"^(angle_min|angle_max|angle_step|range_min|range_max):\s+(.+)$")
INT_PAT = re.compile(r"^(count):\s+(\d+)$")
RANGE_PAT = re.compile(r"^ranges:\s+(.+)$")
INTENSITY_PAT = re.compile(r"^intensities:\s+(.+)$")
FRAME_PAT = re.compile(r'^frame:\s+"(.+)"$')
SEC_PAT = re.compile(r"^\s*sec:\s+(\d+)$")
NSEC_PAT = re.compile(r"^\s*nsec:\s+(\d+)$")


def parse_number(value: str) -> float:
    value = value.strip()
    if value == "inf":
        return math.inf
    if value == "-inf":
        return -math.inf
    if value == "nan":
        return math.nan
    return float(value)


class ScanFrame:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.header_sec: Optional[int] = None
        self.header_nsec: Optional[int] = None
        self.frame_id = "drone1/lidar_link"
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_step = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.count = 0
        self.ranges: list[float] = []
        self.intensities: list[float] = []

    def complete(self) -> bool:
        return self.count > 0 and len(self.ranges) >= self.count and len(self.intensities) >= self.count

    def to_msg(self) -> LaserScan:
        msg = LaserScan()
        if self.header_sec is not None:
            msg.header.stamp.sec = self.header_sec
        if self.header_nsec is not None:
            msg.header.stamp.nanosec = self.header_nsec
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max)
        msg.angle_increment = float(self.angle_step)
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = self.ranges[: self.count]
        msg.intensities = self.intensities[: self.count]
        return msg


class GzScanStdinBridge(Node):
    def __init__(self) -> None:
        super().__init__("gz_scan_stdin_bridge")
        self.publisher = self.create_publisher(LaserScan, "/drone1/scan", 10)
        self.frame = ScanFrame()

    def flush_if_complete(self) -> None:
        if self.frame.complete():
            self.publisher.publish(self.frame.to_msg())
            self.frame.reset()

    def process_line(self, line: str) -> None:
        if not line:
            return

        if line.startswith("header {") and (self.frame.ranges or self.frame.intensities):
            self.flush_if_complete()

        match = SEC_PAT.match(line)
        if match:
            self.frame.header_sec = int(match.group(1))
            return

        match = NSEC_PAT.match(line)
        if match:
            self.frame.header_nsec = int(match.group(1))
            return

        match = FRAME_PAT.match(line)
        if match:
            frame = match.group(1)
            # Convert Gazebo scoped names into a ROS-friendly frame id.
            self.frame.frame_id = frame.replace("::", "/")
            return

        match = FLOAT_PAT.match(line)
        if match:
            key, value = match.groups()
            parsed = parse_number(value)
            if key == "angle_min":
                self.frame.angle_min = parsed
            elif key == "angle_max":
                self.frame.angle_max = parsed
            elif key == "angle_step":
                self.frame.angle_step = parsed
            elif key == "range_min":
                self.frame.range_min = parsed
            elif key == "range_max":
                self.frame.range_max = parsed
            return

        match = INT_PAT.match(line)
        if match:
            self.frame.count = int(match.group(2))
            return

        match = RANGE_PAT.match(line)
        if match:
            self.frame.ranges.append(parse_number(match.group(1)))
            return

        match = INTENSITY_PAT.match(line)
        if match:
            self.frame.intensities.append(parse_number(match.group(1)))
            self.flush_if_complete()


def main() -> int:
    rclpy.init()
    node = GzScanStdinBridge()
    try:
        for raw in sys.stdin:
            node.process_line(raw.rstrip("\n"))
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
