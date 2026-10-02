"""Build local LiDAR submaps without using ground-truth poses."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .data import ExperimentDataset
from .se2 import between, transform_points
from .time_series import cumulative_progress


@dataclass(frozen=True)
class LocalSubmap:
    vehicle: str
    keyframe: int
    anchor_scan: int
    points: np.ndarray
    source_scan_count: int
    raw_progress_start_m: float
    raw_progress_end_m: float


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    """Keep one deterministic centroid per occupied voxel."""
    points = np.asarray(points, dtype=float)
    if not len(points):
        return np.empty((0, 2), dtype=float)
    voxel_m = float(voxel_m)
    if voxel_m <= 0.0:
        raise ValueError("voxel_m must be positive")
    cells = np.floor(points / voxel_m).astype(np.int64)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    x_sum = np.bincount(inverse, weights=points[:, 0])
    y_sum = np.bincount(inverse, weights=points[:, 1])
    return np.column_stack((x_sum / counts, y_sum / counts))


def build_local_submap(
    dataset: ExperimentDataset,
    vehicle_name: str,
    keyframe_index: int,
    config: dict,
) -> LocalSubmap:
    """Accumulate hit endpoints around a keyframe in its raw-SLAM frame.

    Scan selection and motion compensation use only B0/raw SLAM. GT is
    deliberately absent from this function so that the transform estimator
    cannot accidentally consume evaluation truth.
    """
    registration = config.get("registration") or {}
    half_length = float(registration.get("submap_half_length_m", 5.0))
    scan_stride = max(1, int(registration.get("submap_scan_stride", 3)))
    beam_stride = max(1, int(registration.get("submap_beam_stride", 2)))
    voxel_m = float(registration.get("submap_voxel_m", 0.20))
    if half_length <= 0.0:
        raise ValueError("registration.submap_half_length_m must be positive")

    vehicle = dataset.vehicles[vehicle_name]
    track = dataset.keyframes[vehicle_name]
    if keyframe_index < 0 or keyframe_index >= len(track.scan_indices):
        raise IndexError(f"invalid {vehicle_name} keyframe: {keyframe_index}")
    anchor_scan = int(track.scan_indices[keyframe_index])
    anchor_pose = vehicle.raw_poses[anchor_scan]
    raw_progress = cumulative_progress(vehicle.raw_poses)
    center = float(raw_progress[anchor_scan])
    selected = np.flatnonzero(np.abs(raw_progress - center) <= half_length)
    selected = selected[::scan_stride]

    clouds = []
    for scan_index in selected:
        endpoints, hits = vehicle.scans[int(scan_index)].local_endpoints(
            beam_stride
        )
        endpoints = endpoints[hits]
        if not len(endpoints):
            continue
        scan_in_anchor = between(
            anchor_pose, vehicle.raw_poses[int(scan_index)]
        )
        clouds.append(transform_points(scan_in_anchor, endpoints))
    points = (
        voxel_downsample(np.vstack(clouds), voxel_m)
        if clouds
        else np.empty((0, 2), dtype=float)
    )
    return LocalSubmap(
        vehicle=vehicle_name,
        keyframe=int(keyframe_index),
        anchor_scan=anchor_scan,
        points=points,
        source_scan_count=int(len(selected)),
        raw_progress_start_m=(
            float(raw_progress[selected[0]]) if len(selected) else center
        ),
        raw_progress_end_m=(
            float(raw_progress[selected[-1]]) if len(selected) else center
        ),
    )
