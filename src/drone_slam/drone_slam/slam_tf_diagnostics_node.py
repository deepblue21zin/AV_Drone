#!/usr/bin/env python3

import csv
import json
import math
import os
import time
from collections import Counter
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def quaternion_yaw(quaternion) -> float:
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_map_pose(
    map_to_odom: Tuple[float, float, float],
    odom_to_base: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Compose planar map->odom and odom->base poses."""
    tx, ty, tf_yaw = map_to_odom
    ox, oy, odom_yaw = odom_to_base
    cos_yaw, sin_yaw = math.cos(tf_yaw), math.sin(tf_yaw)
    return (
        tx + cos_yaw * ox - sin_yaw * oy,
        ty + sin_yaw * ox + cos_yaw * oy,
        wrap_angle(tf_yaw + odom_yaw),
    )


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SlamTfDiagnosticsNode(Node):
    """Record the SLAM correction separately from raw odometry."""

    FIELDNAMES = [
        "t_sec",
        "tf_stamp_sec",
        "tf_stamp_nanosec",
        "map_to_odom_x",
        "map_to_odom_y",
        "map_to_odom_yaw_rad",
        "map_to_odom_yaw_deg",
        "odom_x",
        "odom_y",
        "odom_yaw_rad",
        "slam_map_x",
        "slam_map_y",
        "slam_map_yaw_rad",
        "correction_translation_m",
        "tf_delta_ms",
        "odom_age_ms",
        "scan_age_ms",
        "event_flags",
    ]

    def __init__(self) -> None:
        super().__init__("slam_tf_diagnostics")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("scan_frame_id", "lidar_link")
        self.declare_parameter("run_id", os.environ.get("RUN_ID", "manual"))
        self.declare_parameter("vehicle_id", "drone1")
        self.declare_parameter(
            "artifacts_root", "/workspace/AV_Drone/artifacts"
        )
        self.declare_parameter("gap_threshold_sec", 0.20)
        self.declare_parameter("translation_jump_threshold_m", 0.25)
        self.declare_parameter("yaw_jump_threshold_deg", 0.50)

        self._map_frame = str(self.get_parameter("map_frame_id").value)
        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._scan_frame = str(self.get_parameter("scan_frame_id").value)
        self._gap_threshold_ns = int(
            float(self.get_parameter("gap_threshold_sec").value) * 1e9
        )
        self._translation_jump = float(
            self.get_parameter("translation_jump_threshold_m").value
        )
        self._yaw_jump = math.radians(
            float(self.get_parameter("yaw_jump_threshold_deg").value)
        )

        run_id = str(self.get_parameter("run_id").value)
        vehicle_id = str(self.get_parameter("vehicle_id").value)
        root = Path(str(self.get_parameter("artifacts_root").value))
        self._output_dir = root / run_id / vehicle_id
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output_dir / "slam_tf_diagnostics.csv"
        self._summary_path = self._output_dir / "slam_tf_diagnostics_summary.json"
        self._stream = self._csv_path.open("w", newline="")
        self._writer = csv.DictWriter(self._stream, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._stream.flush()

        self._first_tf_ns: Optional[int] = None
        self._last_tf_ns: Optional[int] = None
        self._last_scan_ns: Optional[int] = None
        self._last_odom_ns: Optional[int] = None
        self._latest_odom: Optional[Tuple[float, float, float]] = None
        self._previous_correction: Optional[Tuple[float, float, float]] = None
        self._pending_events = set()
        self._event_counts: Counter[str] = Counter()
        self._duplicate_stamp_counts: Counter[str] = Counter()
        self._row_count = 0
        self._max_translation = 0.0
        self._max_abs_yaw = 0.0

        self.create_subscription(
            TFMessage,
            str(self.get_parameter("tf_topic").value),
            self._on_tf,
            qos_profile_sensor_data,
        )
        self.create_timer(5.0, self._write_summary)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"SLAM TF diagnostics: {self._map_frame}->{self._odom_frame} "
            f"to {self._csv_path}"
        )

    @staticmethod
    def _normalized_frame(frame_id: str) -> str:
        return str(frame_id).lstrip("/")

    def _add_event(self, event: str) -> None:
        self._pending_events.add(event)
        self._event_counts[event] += 1

    def _on_odom(self, message: Odometry) -> None:
        if self._normalized_frame(message.header.frame_id) != self._odom_frame:
            self._add_event("ODOM_FRAME_MISMATCH")
        if (
            message.child_frame_id
            and self._normalized_frame(message.child_frame_id) != self._base_frame
        ):
            self._add_event("ODOM_CHILD_FRAME_MISMATCH")
        current_ns = stamp_ns(message.header.stamp)
        if self._last_odom_ns is not None:
            if current_ns < self._last_odom_ns:
                self._add_event("ODOM_NON_MONOTONIC")
            elif current_ns == self._last_odom_ns:
                self._duplicate_stamp_counts["odom"] += 1
        self._last_odom_ns = current_ns
        position = message.pose.pose.position
        self._latest_odom = (
            float(position.x),
            float(position.y),
            quaternion_yaw(message.pose.pose.orientation),
        )

    def _on_scan(self, message: LaserScan) -> None:
        if self._normalized_frame(message.header.frame_id) != self._scan_frame:
            self._add_event("SCAN_FRAME_MISMATCH")
        current_ns = stamp_ns(message.header.stamp)
        if self._last_scan_ns is not None:
            delta_ns = current_ns - self._last_scan_ns
            if delta_ns < 0:
                self._add_event("SCAN_NON_MONOTONIC")
            elif delta_ns == 0:
                self._duplicate_stamp_counts["scan"] += 1
            elif delta_ns > self._gap_threshold_ns:
                self._add_event("SCAN_GAP")
        self._last_scan_ns = current_ns

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            parent = self._normalized_frame(transform.header.frame_id)
            child = self._normalized_frame(transform.child_frame_id)
            if parent != self._map_frame or child != self._odom_frame:
                continue
            self._record_transform(transform)

    def _record_transform(self, transform) -> None:
        current_ns = stamp_ns(transform.header.stamp)
        if self._first_tf_ns is None:
            self._first_tf_ns = current_ns
        delta_ns = None if self._last_tf_ns is None else current_ns - self._last_tf_ns
        if delta_ns is not None:
            if delta_ns < 0:
                self._add_event("TF_NON_MONOTONIC")
            elif delta_ns == 0:
                self._duplicate_stamp_counts["tf"] += 1
                return
            elif delta_ns > self._gap_threshold_ns:
                self._add_event("TF_GAP")

        translation = transform.transform.translation
        correction = (
            float(translation.x),
            float(translation.y),
            quaternion_yaw(transform.transform.rotation),
        )
        if self._previous_correction is not None:
            dx = correction[0] - self._previous_correction[0]
            dy = correction[1] - self._previous_correction[1]
            dyaw = wrap_angle(correction[2] - self._previous_correction[2])
            if math.hypot(dx, dy) > self._translation_jump:
                self._add_event("CORRECTION_TRANSLATION_JUMP")
            if abs(dyaw) > self._yaw_jump:
                self._add_event("CORRECTION_YAW_JUMP")
        self._previous_correction = correction
        self._last_tf_ns = current_ns

        odom = self._latest_odom
        slam_pose = compose_map_pose(correction, odom) if odom else None
        correction_translation = math.hypot(correction[0], correction[1])
        self._max_translation = max(self._max_translation, correction_translation)
        self._max_abs_yaw = max(self._max_abs_yaw, abs(correction[2]))
        flags = ";".join(sorted(self._pending_events))
        self._pending_events.clear()
        row = {
            "t_sec": (current_ns - self._first_tf_ns) / 1e9,
            "tf_stamp_sec": int(transform.header.stamp.sec),
            "tf_stamp_nanosec": int(transform.header.stamp.nanosec),
            "map_to_odom_x": correction[0],
            "map_to_odom_y": correction[1],
            "map_to_odom_yaw_rad": correction[2],
            "map_to_odom_yaw_deg": math.degrees(correction[2]),
            "odom_x": odom[0] if odom else "",
            "odom_y": odom[1] if odom else "",
            "odom_yaw_rad": odom[2] if odom else "",
            "slam_map_x": slam_pose[0] if slam_pose else "",
            "slam_map_y": slam_pose[1] if slam_pose else "",
            "slam_map_yaw_rad": slam_pose[2] if slam_pose else "",
            "correction_translation_m": correction_translation,
            "tf_delta_ms": "" if delta_ns is None else delta_ns / 1e6,
            "odom_age_ms": (
                "" if self._last_odom_ns is None else (current_ns - self._last_odom_ns) / 1e6
            ),
            "scan_age_ms": (
                "" if self._last_scan_ns is None else (current_ns - self._last_scan_ns) / 1e6
            ),
            "event_flags": flags,
        }
        self._writer.writerow(row)
        self._stream.flush()
        self._row_count += 1

    def _write_summary(self) -> None:
        summary = {
            "rows": self._row_count,
            "map_frame_id": self._map_frame,
            "odom_frame_id": self._odom_frame,
            "base_frame_id": self._base_frame,
            "scan_frame_id": self._scan_frame,
            "max_correction_translation_m": self._max_translation,
            "max_abs_correction_yaw_deg": math.degrees(self._max_abs_yaw),
            "event_counts": dict(sorted(self._event_counts.items())),
            "duplicate_stamp_counts": dict(
                sorted(self._duplicate_stamp_counts.items())
            ),
            "csv_path": str(self._csv_path),
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        self._summary_path.write_text(json.dumps(summary, indent=2))

    def destroy_node(self):
        self._write_summary()
        self._stream.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamTfDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
