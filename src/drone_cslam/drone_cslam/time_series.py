"""Timestamp interpolation and keyframe sampling."""

from __future__ import annotations

import numpy as np

from .se2 import interpolate_pose, wrap_angle


def sorted_unique_pose_series(records):
    """Convert TimedPose records into monotonic arrays, keeping the last duplicate."""
    latest = {float(record.timestamp): np.asarray(record.pose, dtype=float) for record in records}
    times = np.asarray(sorted(latest), dtype=float)
    if not len(times):
        return times, np.empty((0, 3), dtype=float)
    poses = np.vstack([latest[float(timestamp)] for timestamp in times])
    return times, poses


def interpolate_pose_series(
    times: np.ndarray,
    poses: np.ndarray,
    query_times: np.ndarray,
    max_gap_sec: float,
):
    """Interpolate poses and return (poses, valid_mask).

    A query is valid only when it is bracketed and both neighboring samples are
    within ``max_gap_sec``. Exact timestamp matches are always accepted.
    """
    times = np.asarray(times, dtype=float)
    poses = np.asarray(poses, dtype=float)
    query_times = np.asarray(query_times, dtype=float)
    output = np.full((len(query_times), 3), np.nan, dtype=float)
    valid = np.zeros(len(query_times), dtype=bool)
    if not len(times):
        return output, valid

    right_indices = np.searchsorted(times, query_times, side="left")
    for query_index, (query, right) in enumerate(zip(query_times, right_indices)):
        if right < len(times) and abs(times[right] - query) <= 1.0e-9:
            output[query_index] = poses[right]
            valid[query_index] = True
            continue
        if right == 0 or right >= len(times):
            continue
        left = right - 1
        left_gap = query - times[left]
        right_gap = times[right] - query
        if left_gap > max_gap_sec or right_gap > max_gap_sec:
            continue
        duration = times[right] - times[left]
        if duration <= 0.0:
            continue
        output[query_index] = interpolate_pose(
            poses[left], poses[right], left_gap / duration
        )
        valid[query_index] = True
    return output, valid


def cumulative_progress(poses: np.ndarray) -> np.ndarray:
    poses = np.asarray(poses, dtype=float)
    if not len(poses):
        return np.empty(0, dtype=float)
    increments = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(increments)))


def select_keyframe_indices(
    poses: np.ndarray,
    translation_threshold_m: float,
    yaw_threshold_rad: float,
) -> np.ndarray:
    poses = np.asarray(poses, dtype=float)
    if not len(poses):
        return np.empty(0, dtype=int)
    selected = [0]
    last = 0
    for index in range(1, len(poses) - 1):
        translation = np.linalg.norm(poses[index, :2] - poses[last, :2])
        yaw = abs(float(wrap_angle(poses[index, 2] - poses[last, 2])))
        if translation >= translation_threshold_m or yaw >= yaw_threshold_rad:
            selected.append(index)
            last = index
    if selected[-1] != len(poses) - 1:
        selected.append(len(poses) - 1)
    return np.asarray(selected, dtype=int)


def assign_nearest_keyframes(scan_times: np.ndarray, keyframe_times: np.ndarray) -> np.ndarray:
    if not len(keyframe_times):
        raise ValueError("at least one keyframe is required")
    right = np.searchsorted(keyframe_times, scan_times, side="left")
    right = np.clip(right, 0, len(keyframe_times) - 1)
    left = np.clip(right - 1, 0, len(keyframe_times) - 1)
    choose_left = np.abs(scan_times - keyframe_times[left]) <= np.abs(
        keyframe_times[right] - scan_times
    )
    return np.where(choose_left, left, right).astype(int)
