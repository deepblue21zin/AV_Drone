"""Reproject frozen scan-to-keyframe geometry into occupancy grids."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict

import numpy as np
import yaml

from .data import ExperimentDataset
from .se2 import between, compose, transform_points


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    @property
    def width(self) -> int:
        return int(math.ceil((self.max_x - self.min_x) / self.resolution))

    @property
    def height(self) -> int:
        return int(math.ceil((self.max_y - self.min_y) / self.resolution))


def grid_spec_from_config(config: dict) -> GridSpec:
    mapping = config.get("map") or {}
    min_x, max_x, min_y, max_y = [float(value) for value in mapping["bounds"]]
    return GridSpec(
        resolution=float(mapping.get("resolution_m", 0.1)),
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
    )


def corrected_scan_poses(
    dataset: ExperimentDataset,
    condition: str,
    optimized_keyframes: np.ndarray | None = None,
) -> Dict[str, np.ndarray]:
    result = {}
    for name, vehicle in dataset.vehicles.items():
        track = dataset.keyframes[name]
        if condition == "B0":
            result[name] = vehicle.raw_poses.copy()
            continue
        if condition == "B1":
            result[name] = vehicle.known_poses.copy()
            continue
        if condition == "GT-reference":
            result[name] = vehicle.gt_poses.copy()
            continue
        if optimized_keyframes is None:
            raise ValueError(f"{condition} requires optimized keyframe poses")
        scan_poses = np.empty_like(vehicle.raw_poses)
        for scan_index, local_keyframe_index in enumerate(track.scan_to_keyframe):
            raw_keyframe = track.raw_poses[local_keyframe_index]
            local_scan = between(raw_keyframe, vehicle.raw_poses[scan_index])
            optimized = optimized_keyframes[track.node_indices[local_keyframe_index]]
            scan_poses[scan_index] = compose(optimized, local_scan)
        result[name] = scan_poses
    return result


def _world_to_cell(point: np.ndarray, spec: GridSpec):
    x = int(math.floor((float(point[0]) - spec.min_x) / spec.resolution))
    y = int(math.floor((float(point[1]) - spec.min_y) / spec.resolution))
    return x, y


def _bresenham(start, end):
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def build_occupancy_grid(
    dataset: ExperimentDataset,
    scan_poses: Dict[str, np.ndarray],
    config: dict,
) -> np.ndarray:
    mapping = config.get("map") or {}
    sampling = config.get("sampling") or {}
    spec = grid_spec_from_config(config)
    log_odds = np.zeros((spec.height, spec.width), dtype=np.float32)
    observed = np.zeros((spec.height, spec.width), dtype=bool)
    free_delta = float(mapping.get("free_log_odds", -0.4))
    occupied_delta = float(mapping.get("occupied_log_odds", 0.85))
    minimum = float(mapping.get("min_log_odds", -4.0))
    maximum = float(mapping.get("max_log_odds", 4.0))
    occupied_probability = float(mapping.get("occupied_probability", 0.65))
    scan_stride = max(1, int(sampling.get("map_scan_stride", 2)))
    beam_stride = max(1, int(sampling.get("map_beam_stride", 3)))

    for name, vehicle in dataset.vehicles.items():
        poses = scan_poses[name]
        for scan_index in range(0, len(vehicle.scans), scan_stride):
            scan = vehicle.scans[scan_index]
            pose = poses[scan_index]
            if not np.all(np.isfinite(pose)):
                continue  # Evaluation-only GT can lack support at bag boundaries.
            local_points, hits = scan.local_endpoints(beam_stride)
            world_points = transform_points(pose, local_points)
            start = _world_to_cell(pose[:2], spec)
            for endpoint, hit in zip(world_points, hits):
                line = _bresenham(start, _world_to_cell(endpoint, spec))
                free_cells = line[:-1] if hit else line
                if free_cells:
                    free = np.asarray(
                        [
                            (x, y)
                            for x, y in free_cells
                            if 0 <= x < spec.width and 0 <= y < spec.height
                        ],
                        dtype=int,
                    )
                    if len(free):
                        np.add.at(log_odds, (free[:, 1], free[:, 0]), free_delta)
                        observed[free[:, 1], free[:, 0]] = True
                if hit:
                    x, y = line[-1]
                    if 0 <= x < spec.width and 0 <= y < spec.height:
                        log_odds[y, x] += occupied_delta
                        observed[y, x] = True
            np.clip(log_odds, minimum, maximum, out=log_odds)

    probabilities = 1.0 / (1.0 + np.exp(-log_odds))
    grid = np.full(log_odds.shape, -1, dtype=np.int8)
    grid[observed] = 0
    grid[observed & (probabilities >= occupied_probability)] = 100
    return grid


def save_occupancy_grid(
    grid: np.ndarray, spec: GridSpec, output_prefix: Path
):
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    np.save(output_prefix.with_suffix(".npy"), grid)
    image = np.full(grid.shape, 205, dtype=np.uint8)
    image[grid == 0] = 254
    image[grid == 100] = 0
    pgm_path = output_prefix.with_suffix(".pgm")
    with pgm_path.open("wb") as stream:
        stream.write(f"P5\n{spec.width} {spec.height}\n255\n".encode("ascii"))
        stream.write(np.flipud(image).tobytes())
    yaml_path = output_prefix.with_suffix(".yaml")
    with yaml_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            {
                "image": pgm_path.name,
                "resolution": spec.resolution,
                "origin": [spec.min_x, spec.min_y, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            },
            stream,
            sort_keys=False,
        )
