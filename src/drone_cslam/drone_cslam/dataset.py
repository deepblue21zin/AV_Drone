"""Convert asynchronous bag streams into comparable scan-time trajectories."""

from __future__ import annotations

import math
from typing import Dict

import numpy as np

from .data import (
    ExperimentDataset,
    KeyframeTrack,
    SampledVehicle,
    VehicleBagData,
)
from .se2 import compose
from .time_series import (
    assign_nearest_keyframes,
    cumulative_progress,
    interpolate_pose_series,
    select_keyframe_indices,
    sorted_unique_pose_series,
)


def _compose_series(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    if len(first) != len(second):
        raise ValueError("pose series lengths must match")
    return np.vstack([compose(a, b) for a, b in zip(first, second)])


def sample_vehicle(
    bag_data: VehicleBagData,
    spawn_pose: np.ndarray,
    max_gap_sec: float,
    use_ground_truth: bool = True,
) -> SampledVehicle:
    scans = sorted(bag_data.scans, key=lambda scan: scan.timestamp)
    scan_times = np.asarray([scan.timestamp for scan in scans], dtype=float)
    series = {}
    validity = []
    support_starts = []
    support_ends = []
    input_series = [
        ("odom", bag_data.odom),
        ("map_to_odom", bag_data.map_to_odom),
    ]
    if use_ground_truth:
        input_series.append(("gt", bag_data.ground_truth))
    for label, records in input_series:
        times, poses = sorted_unique_pose_series(records)
        if not len(times):
            raise RuntimeError(f"{bag_data.name} has no {label} pose samples")
        support_starts.append(float(times[0]))
        support_ends.append(float(times[-1]))
        values, valid = interpolate_pose_series(
            times, poses, scan_times, max_gap_sec
        )
        series[label] = values
        validity.append(valid)
    common_start = max(support_starts)
    common_end = min(support_ends)
    eligible = (scan_times >= common_start) & (scan_times <= common_end)
    valid = np.logical_and.reduce([*validity, eligible])
    valid_indices = np.flatnonzero(valid)
    if len(valid_indices) < 2:
        raise RuntimeError(
            f"{bag_data.name} has fewer than two fully synchronized scans"
        )

    odom = series["odom"][valid]
    map_to_odom = series["map_to_odom"][valid]
    spawn = np.repeat(np.asarray(spawn_pose, dtype=float)[None, :], len(odom), axis=0)
    known = _compose_series(spawn, odom)
    raw_local = _compose_series(map_to_odom, odom)
    raw = _compose_series(spawn, raw_local)
    return SampledVehicle(
        name=bag_data.name,
        scans=[scans[index] for index in valid_indices],
        scan_times=scan_times[valid],
        raw_poses=raw,
        known_poses=known,
        gt_poses=series["gt"][valid] if use_ground_truth else np.full_like(raw, np.nan),
        valid_source_indices=valid_indices,
        dropped_scan_count=int(len(scans) - len(valid_indices)),
        trimmed_scan_count=int(np.count_nonzero(~eligible)),
        interpolation_drop_count=int(np.count_nonzero(eligible & ~valid)),
        eligible_scan_count=int(np.count_nonzero(eligible)),
    )


def build_dataset(
    bag_vehicles: Dict[str, VehicleBagData], config: dict, *, use_ground_truth=True
) -> ExperimentDataset:
    sampling = config.get("sampling") or {}
    max_gap = float(sampling.get("interpolation_max_gap_sec", 0.15))
    translation = float(sampling.get("keyframe_translation_m", 2.0))
    yaw = math.radians(float(sampling.get("keyframe_yaw_deg", 5.0)))

    sampled = {}
    tracks = {}
    node_offset = 0
    vehicle_configs = {str(item["name"]): item for item in config["vehicles"]}
    for name in vehicle_configs:
        if name not in bag_vehicles:
            raise RuntimeError(f"configured vehicle is missing from bag: {name}")
        spawn = np.asarray(vehicle_configs[name]["spawn"], dtype=float)
        if spawn.shape != (3,):
            raise ValueError(f"{name}.spawn must be [x, y, yaw]")
        vehicle = sample_vehicle(bag_vehicles[name], spawn, max_gap, use_ground_truth)
        sampled[name] = vehicle
        key_indices = select_keyframe_indices(
            vehicle.raw_poses, translation, yaw
        )
        node_indices = np.arange(
            node_offset, node_offset + len(key_indices), dtype=int
        )
        node_offset += len(key_indices)
        progress = cumulative_progress(
            (vehicle.gt_poses if use_ground_truth else vehicle.raw_poses)[key_indices]
        )
        key_times = vehicle.scan_times[key_indices]
        tracks[name] = KeyframeTrack(
            name=name,
            scan_indices=key_indices,
            times=key_times,
            raw_poses=vehicle.raw_poses[key_indices],
            known_poses=vehicle.known_poses[key_indices],
            gt_poses=vehicle.gt_poses[key_indices],
            progress=progress,
            node_indices=node_indices,
            scan_to_keyframe=assign_nearest_keyframes(
                vehicle.scan_times, key_times
            ),
        )
    return ExperimentDataset(vehicles=sampled, keyframes=tracks)
