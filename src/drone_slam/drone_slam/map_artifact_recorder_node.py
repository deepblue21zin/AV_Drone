#!/usr/bin/env python3

import json
import math
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MapArtifactRecorderNode(Node):
    """Save a selected local map under the shared run artifact directory."""

    def __init__(self) -> None:
        super().__init__("map_artifact_recorder")
        self.declare_parameter("map_topic", "slam/map")
        self.declare_parameter("mission_phase_topic", "mission/phase")
        self.declare_parameter("save_on_phase", "LANDED")
        self.declare_parameter("artifacts_root", "/workspace/AV_Drone/artifacts")
        self.declare_parameter("run_id", os.environ.get("RUN_ID", "manual"))
        self.declare_parameter("vehicle_id", "drone1")
        self.declare_parameter("map_source", "slam")
        self.declare_parameter("save_interval_sec", 0.0)
        self.declare_parameter("save_history", False)

        self._latest_map: Optional[OccupancyGrid] = None
        self._saved = False
        self._save_count = 0
        self._started_monotonic = time.monotonic()
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_phase_topic").value),
            self._on_phase,
            10,
        )
        interval = float(self.get_parameter("save_interval_sec").value)
        if interval > 0.0:
            self.create_timer(max(1.0, interval), self._save_latest)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_phase(self, msg: String) -> None:
        if self._saved or msg.data != str(self.get_parameter("save_on_phase").value):
            return
        if self._latest_map is None:
            self.get_logger().warn("Map save requested before any map was received")
            return
        self._save(self._latest_map)
        self._saved = True

    def _save_latest(self) -> None:
        if self._latest_map is not None:
            self._save(self._latest_map)

    def _save(self, msg: OccupancyGrid) -> None:
        root = Path(str(self.get_parameter("artifacts_root").value))
        run_id = str(self.get_parameter("run_id").value)
        vehicle_id = str(self.get_parameter("vehicle_id").value)
        source = str(self.get_parameter("map_source").value)
        out_dir = root / run_id / vehicle_id / "maps"
        out_dir.mkdir(parents=True, exist_ok=True)
        self._save_count += 1
        extra_meta = {
            "snapshot_index": self._save_count,
            "elapsed_sec": float(time.monotonic() - self._started_monotonic),
        }

        self._write_bundle(msg, out_dir, source, source, extra_meta)
        if bool(self.get_parameter("save_history").value):
            history_dir = out_dir / "history"
            history_dir.mkdir(parents=True, exist_ok=True)
            snapshot_name = f"{source}_{self._save_count:04d}"
            self._write_bundle(msg, history_dir, snapshot_name, source, extra_meta)
        self.get_logger().info(
            f"Saved {source} map artifacts (snapshot {self._save_count}) to {out_dir}"
        )

    @staticmethod
    def _write_bundle(
        msg: OccupancyGrid,
        out_dir: Path,
        basename: str,
        source: str,
        extra_meta: dict,
    ) -> None:
        """Write one self-contained occupancy-grid artifact bundle."""

        height, width = int(msg.info.height), int(msg.info.width)
        grid = np.asarray(msg.data, dtype=np.int8).reshape((height, width))
        np.save(out_dir / f"{basename}_grid.npy", grid)

        pgm = np.full(grid.shape, 205, dtype=np.uint8)
        pgm[grid == 0] = 254
        occupied = grid > 0
        pgm[occupied] = np.clip(254 - grid[occupied] * 2.54, 0, 253).astype(np.uint8)
        pgm_path = out_dir / f"{basename}.pgm"
        with pgm_path.open("wb") as stream:
            stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            stream.write(np.flipud(pgm).tobytes())

        orientation = msg.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        meta = {
            "source": source,
            "frame_id": msg.header.frame_id,
            "map_stamp_sec": int(msg.header.stamp.sec),
            "map_stamp_nanosec": int(msg.header.stamp.nanosec),
            "resolution": float(msg.info.resolution),
            "width": width,
            "height": height,
            "origin": {
                "x": float(msg.info.origin.position.x),
                "y": float(msg.info.origin.position.y),
                "z": float(msg.info.origin.position.z),
                "yaw": float(origin_yaw),
            },
            "coverage": float(np.count_nonzero(grid >= 0) / max(1, grid.size)),
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            **extra_meta,
        }
        (out_dir / f"{basename}_meta.json").write_text(
            json.dumps(meta, indent=2), encoding="utf-8"
        )
        yaml_text = (
            f"image: {basename}.pgm\n"
            f"resolution: {msg.info.resolution}\n"
            f"origin: [{msg.info.origin.position.x}, {msg.info.origin.position.y}, "
            f"{origin_yaw}]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
        (out_dir / f"{basename}.yaml").write_text(yaml_text, encoding="utf-8")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapArtifactRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
