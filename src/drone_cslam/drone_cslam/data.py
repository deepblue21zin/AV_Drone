"""Data containers shared by bag loading, optimization, and evaluation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np


@dataclass(frozen=True)
class TimedPose:
    timestamp: float
    pose: np.ndarray


@dataclass(frozen=True)
class ScanRecord:
    timestamp: float
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: np.ndarray

    def local_endpoints(self, beam_stride: int = 1) -> tuple[np.ndarray, np.ndarray]:
        """Return valid endpoints and a hit mask for finite LaserScan beams."""
        stride = max(1, int(beam_stride))
        ranges = np.asarray(self.ranges[::stride], dtype=float)
        indices = np.arange(0, len(self.ranges), stride, dtype=float)
        angles = self.angle_min + indices * self.angle_increment
        finite = np.isfinite(ranges)
        clipped = np.where(finite, ranges, self.range_max)
        valid = (clipped >= self.range_min) & (clipped <= self.range_max)
        clipped = clipped[valid]
        angles = angles[valid]
        points = np.column_stack(
            (clipped * np.cos(angles), clipped * np.sin(angles))
        )
        hits = finite[valid] & (clipped < self.range_max - 1.0e-6)
        return points, hits


@dataclass
class VehicleBagData:
    name: str
    scans: List[ScanRecord] = field(default_factory=list)
    odom: List[TimedPose] = field(default_factory=list)
    ground_truth: List[TimedPose] = field(default_factory=list)
    map_to_odom: List[TimedPose] = field(default_factory=list)


@dataclass
class SampledVehicle:
    name: str
    scans: List[ScanRecord]
    scan_times: np.ndarray
    raw_poses: np.ndarray
    known_poses: np.ndarray
    gt_poses: np.ndarray
    valid_source_indices: np.ndarray
    dropped_scan_count: int
    trimmed_scan_count: int = 0
    interpolation_drop_count: int = 0
    eligible_scan_count: int = 0


@dataclass
class KeyframeTrack:
    name: str
    scan_indices: np.ndarray
    times: np.ndarray
    raw_poses: np.ndarray
    known_poses: np.ndarray
    gt_poses: np.ndarray
    progress: np.ndarray
    node_indices: np.ndarray
    scan_to_keyframe: np.ndarray


@dataclass
class ExperimentDataset:
    vehicles: Dict[str, SampledVehicle]
    keyframes: Dict[str, KeyframeTrack]
