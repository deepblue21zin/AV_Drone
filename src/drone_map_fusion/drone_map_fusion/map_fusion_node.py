#!/usr/bin/env python3

import csv
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt64
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .occupancy_grid_utils import (
    GridSpec,
    fuse_projected_grids,
    project_occupancy_grid,
    quaternion_yaw,
)


@dataclass
class SourceState:
    name: str
    topic: str
    frame: str
    transform: Tuple[float, float, float]
    confidence: float
    message: Optional[OccupancyGrid] = None
    received_monotonic: Optional[float] = None


class MapFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("map_fusion")
        self.declare_parameter("global_frame", "swarm_map")
        self.declare_parameter("global_map_topic", "/swarm/global_map")
        self.declare_parameter("map_version_topic", "/swarm/map_version")
        self.declare_parameter("fusion_status_topic", "/swarm/fusion_status")
        self.declare_parameter("source_names", ["drone1", "drone2"])
        self.declare_parameter(
            "source_topics", ["/drone1/slam/map", "/drone2/slam/map"]
        )
        self.declare_parameter("source_frames", ["drone1/map", "drone2/map"])
        self.declare_parameter("source_x", [0.0, 0.0])
        self.declare_parameter("source_y", [-3.0, 3.0])
        self.declare_parameter("source_yaw", [0.0, 0.0])
        self.declare_parameter("source_confidence", [1.0, 1.0])
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("min_x", -5.0)
        self.declare_parameter("max_x", 45.0)
        self.declare_parameter("min_y", -12.0)
        self.declare_parameter("max_y", 12.0)
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("map_timeout_sec", 2.0)
        self.declare_parameter("free_threshold", 25)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("artifacts_root", "/workspace/AV_Drone/artifacts")
        self.declare_parameter("run_id", os.environ.get("RUN_ID", "manual"))

        self._global_frame = str(self.get_parameter("global_frame").value)
        self._spec = GridSpec(
            resolution=float(self.get_parameter("resolution").value),
            min_x=float(self.get_parameter("min_x").value),
            max_x=float(self.get_parameter("max_x").value),
            min_y=float(self.get_parameter("min_y").value),
            max_y=float(self.get_parameter("max_y").value),
        )
        if self._spec.resolution <= 0.0 or self._spec.width <= 0 or self._spec.height <= 0:
            raise ValueError("invalid global grid bounds or resolution")

        names = list(self.get_parameter("source_names").value)
        topics = list(self.get_parameter("source_topics").value)
        frames = list(self.get_parameter("source_frames").value)
        xs = list(self.get_parameter("source_x").value)
        ys = list(self.get_parameter("source_y").value)
        yaws = list(self.get_parameter("source_yaw").value)
        confidences = list(self.get_parameter("source_confidence").value)
        lengths = {len(names), len(topics), len(frames), len(xs), len(ys), len(yaws), len(confidences)}
        if len(lengths) != 1 or not names:
            raise ValueError("all map source parameter arrays must be non-empty and equal length")
        self._sources = [
            SourceState(
                name=str(names[i]),
                topic=str(topics[i]),
                frame=str(frames[i]),
                transform=(float(xs[i]), float(ys[i]), float(yaws[i])),
                confidence=float(confidences[i]),
            )
            for i in range(len(names))
        ]
        self._version = 0
        self._last_state = "WAITING_MAPS"
        self._latencies_ms: List[float] = []
        self._state_counts = {"WAITING_MAPS": 0, "HEALTHY": 0, "LOCAL_ONLY_FALLBACK": 0}

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("global_map_topic").value), map_qos
        )
        self._version_pub = self.create_publisher(
            UInt64, str(self.get_parameter("map_version_topic").value), 10
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("fusion_status_topic").value), 10
        )
        for index, source in enumerate(self._sources):
            self.create_subscription(
                OccupancyGrid,
                source.topic,
                lambda msg, source_index=index: self._on_map(source_index, msg),
                map_qos,
            )

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self._metrics_path = self._initialize_metrics()
        self._summary_path = self._metrics_path.parent / "swarm_summary.json"
        hz = max(0.1, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self._fuse)

    def _publish_static_transforms(self) -> None:
        transforms = []
        stamp = self.get_clock().now().to_msg()
        for source in self._sources:
            x, y, yaw = source.transform
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self._global_frame
            msg.child_frame_id = source.frame
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.rotation.z = math.sin(0.5 * yaw)
            msg.transform.rotation.w = math.cos(0.5 * yaw)
            transforms.append(msg)
        self._static_broadcaster.sendTransform(transforms)

    def _initialize_metrics(self) -> Path:
        root = Path(str(self.get_parameter("artifacts_root").value))
        run_id = str(self.get_parameter("run_id").value)
        run_dir = root / run_id
        run_dir.mkdir(parents=True, exist_ok=True)
        path = run_dir / "fusion_metrics.csv"
        if not path.exists():
            with path.open("w", newline="", encoding="utf-8") as stream:
                csv.writer(stream).writerow(
                    [
                        "timestamp",
                        "map_version",
                        "state",
                        "active_sources",
                        "stale_sources",
                        "max_input_age_ms",
                        "fusion_latency_ms",
                        "observed_cells",
                        "conflict_cells",
                        "conflict_ratio",
                    ]
                )
        return path

    def _on_map(self, source_index: int, msg: OccupancyGrid) -> None:
        expected = self._sources[source_index].frame
        if msg.header.frame_id and msg.header.frame_id != expected:
            self.get_logger().error(
                f"Rejecting {self._sources[source_index].name} map: frame "
                f"{msg.header.frame_id!r} != {expected!r}"
            )
            return
        self._sources[source_index].message = msg
        self._sources[source_index].received_monotonic = time.monotonic()

    def _fuse(self) -> None:
        started = time.perf_counter()
        now = time.monotonic()
        timeout = max(0.1, float(self.get_parameter("map_timeout_sec").value))
        active = []
        stale = []
        ages_ms = []
        for source in self._sources:
            if source.message is None or source.received_monotonic is None:
                stale.append(source.name)
                continue
            age = now - source.received_monotonic
            if age > timeout:
                stale.append(source.name)
                continue
            active.append(source)
            ages_ms.append(age * 1000.0)

        if not active:
            self._last_state = "WAITING_MAPS"
            self._publish_status(active, stale, 0.0, 0, 0, 0.0)
            return

        projected = []
        weights = []
        for source in active:
            msg = source.message
            assert msg is not None
            origin = msg.info.origin
            projected.append(
                project_occupancy_grid(
                    msg.data,
                    int(msg.info.width),
                    int(msg.info.height),
                    float(msg.info.resolution),
                    (
                        float(origin.position.x),
                        float(origin.position.y),
                        quaternion_yaw(
                            origin.orientation.x,
                            origin.orientation.y,
                            origin.orientation.z,
                            origin.orientation.w,
                        ),
                    ),
                    source.transform,
                    self._spec,
                    int(self.get_parameter("free_threshold").value),
                    int(self.get_parameter("occupied_threshold").value),
                )
            )
            age = now - float(source.received_monotonic)
            freshness = max(0.05, 1.0 - age / timeout)
            weights.append(source.confidence * freshness)

        grid, conflict_count, observed_count = fuse_projected_grids(
            projected, weights, self._spec
        )
        latency_ms = (time.perf_counter() - started) * 1000.0
        self._version += 1
        self._last_state = "HEALTHY" if len(active) == len(self._sources) else "LOCAL_ONLY_FALLBACK"

        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._global_frame
        message.info.map_load_time = message.header.stamp
        message.info.resolution = self._spec.resolution
        message.info.width = self._spec.width
        message.info.height = self._spec.height
        message.info.origin.position.x = self._spec.min_x
        message.info.origin.position.y = self._spec.min_y
        message.info.origin.orientation.w = 1.0
        message.data = grid.reshape(-1).tolist()
        self._map_pub.publish(message)
        self._version_pub.publish(UInt64(data=self._version))
        self._publish_status(
            active,
            stale,
            latency_ms,
            observed_count,
            conflict_count,
            max(ages_ms, default=0.0),
        )

    def _publish_status(
        self,
        active: List[SourceState],
        stale: List[str],
        latency_ms: float,
        observed_count: int,
        conflict_count: int,
        max_age_ms: float,
    ) -> None:
        ratio = conflict_count / max(1, observed_count)
        self._state_counts[self._last_state] = self._state_counts.get(self._last_state, 0) + 1
        if latency_ms > 0.0:
            self._latencies_ms.append(latency_ms)
        status = DiagnosticStatus()
        status.name = "swarm/map_fusion"
        status.hardware_id = "central"
        status.level = (
            DiagnosticStatus.OK
            if self._last_state == "HEALTHY"
            else DiagnosticStatus.WARN
        )
        status.message = self._last_state
        status.values = [
            KeyValue(key="map_version", value=str(self._version)),
            KeyValue(key="active_sources", value=",".join(s.name for s in active)),
            KeyValue(key="stale_sources", value=",".join(stale)),
            KeyValue(key="max_input_age_ms", value=f"{max_age_ms:.3f}"),
            KeyValue(key="fusion_latency_ms", value=f"{latency_ms:.3f}"),
            KeyValue(key="observed_cells", value=str(observed_count)),
            KeyValue(key="conflict_cells", value=str(conflict_count)),
            KeyValue(key="conflict_ratio", value=f"{ratio:.8f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_pub.publish(array)
        with self._metrics_path.open("a", newline="", encoding="utf-8") as stream:
            csv.writer(stream).writerow(
                [
                    time.time(),
                    self._version,
                    self._last_state,
                    ";".join(s.name for s in active),
                    ";".join(stale),
                    max_age_ms,
                    latency_ms,
                    observed_count,
                    conflict_count,
                    ratio,
                ]
            )
        ordered = sorted(self._latencies_ms)
        p95 = (
            ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
            if ordered
            else None
        )
        summary = {
            "map_version": self._version,
            "state": self._last_state,
            "source_count": len(self._sources),
            "active_sources": [source.name for source in active],
            "stale_sources": stale,
            "fusion_samples": len(self._latencies_ms),
            "fusion_latency_p95_ms": p95,
            "last_conflict_ratio": ratio,
            "state_counts": self._state_counts,
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        temporary = self._summary_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        temporary.replace(self._summary_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
