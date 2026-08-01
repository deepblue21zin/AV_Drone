#!/usr/bin/env python3

import json
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String


class MapSaverNode(Node):
    """Saves the live /map occupancy grid to disk once the mission reaches a target phase.

    Writes a fixed, overwrite-on-each-run pair of files (<map_file_basename>_grid.npy +
    <map_file_basename>_meta.json) under maps/<scenario_name>/, so downstream experiments
    can load a stable path instead of having to know which timestamped
    artifacts/<run_id>/ produced it. map_file_basename defaults to "map".
    """

    def __init__(self) -> None:
        super().__init__("map_saver")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("mission_phase_topic", "/drone1/mission/phase")
        self.declare_parameter("save_on_phase", "LANDED")
        self.declare_parameter("maps_root", "/workspace/AV_Drone/maps")
        self.declare_parameter("scenario_name", "single_drone_obstacle_demo")
        self.declare_parameter("map_file_basename", "map")

        self._latest_map: Optional[OccupancyGrid] = None
        self._saved = False

        map_topic = str(self.get_parameter("map_topic").value)
        phase_topic = str(self.get_parameter("mission_phase_topic").value)

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 1)
        self.create_subscription(String, phase_topic, self._on_phase, 10)

        self.get_logger().info(
            f"Map saver ready: map={map_topic}, phase={phase_topic}, "
            f"save_on_phase={self.get_parameter('save_on_phase').value}"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_phase(self, msg: String) -> None:
        if self._saved:
            return

        target_phase = str(self.get_parameter("save_on_phase").value)
        if msg.data != target_phase:
            return

        if self._latest_map is None:
            self.get_logger().warn(
                f"Phase reached {target_phase} but no /map message received yet; cannot save."
            )
            return

        self._save_map(self._latest_map)
        self._saved = True

    def _save_map(self, grid: OccupancyGrid) -> None:
        scenario_name = str(self.get_parameter("scenario_name").value)
        maps_root = Path(str(self.get_parameter("maps_root").value))
        basename = str(self.get_parameter("map_file_basename").value)
        out_dir = maps_root / scenario_name
        out_dir.mkdir(parents=True, exist_ok=True)

        width = int(grid.info.width)
        height = int(grid.info.height)
        data = np.array(grid.data, dtype=np.int8).reshape(height, width)

        grid_path = out_dir / f"{basename}_grid.npy"
        np.save(grid_path, data)

        meta = {
            "resolution": float(grid.info.resolution),
            "width": width,
            "height": height,
            "origin_x": float(grid.info.origin.position.x),
            "origin_y": float(grid.info.origin.position.y),
            "origin_z": float(grid.info.origin.position.z),
            "frame_id": grid.header.frame_id,
            "scenario_name": scenario_name,
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
        }
        meta_path = out_dir / f"{basename}_meta.json"
        meta_path.write_text(json.dumps(meta, indent=2))

        self.get_logger().info(
            f"Saved map: {grid_path} ({width}x{height}, {grid.info.resolution:.3f} m/cell), "
            f"meta={meta_path}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()
