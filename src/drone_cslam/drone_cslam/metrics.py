"""Trajectory, differential-drift, map, and overlap metrics."""

from __future__ import annotations

import math
from typing import List

import numpy as np

from .data import ExperimentDataset
from .mapping import GridSpec
from .se2 import between, transform_points, wrap_angle
from .time_series import cumulative_progress


def _safe_stats(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"rmse": None, "median": None, "maximum": None}
    return {
        "rmse": float(np.sqrt(np.mean(values * values))),
        "median": float(np.median(values)),
        "maximum": float(np.max(values)),
    }


def trajectory_metrics(
    estimated: np.ndarray,
    ground_truth: np.ndarray,
    progress: np.ndarray,
    rpe_distance_m: float,
) -> dict:
    translation = np.linalg.norm(estimated[:, :2] - ground_truth[:, :2], axis=1)
    yaw = np.abs(wrap_angle(estimated[:, 2] - ground_truth[:, 2]))
    rpe_translation = []
    rpe_yaw = []
    for start in range(len(progress) - 1):
        candidates = np.flatnonzero(progress >= progress[start] + rpe_distance_m)
        if not len(candidates):
            continue
        end = int(candidates[0])
        estimated_delta = between(estimated[start], estimated[end])
        truth_delta = between(ground_truth[start], ground_truth[end])
        error = between(truth_delta, estimated_delta)
        rpe_translation.append(float(np.linalg.norm(error[:2])))
        rpe_yaw.append(abs(float(wrap_angle(error[2]))))
    return {
        "sample_count": int(len(estimated)),
        "ate_translation_m": _safe_stats(translation),
        "ate_yaw_deg": _safe_stats(np.degrees(yaw)),
        "endpoint_translation_m": float(translation[-1]),
        "endpoint_yaw_deg": float(math.degrees(yaw[-1])),
        "rpe_translation_m": _safe_stats(np.asarray(rpe_translation)),
        "rpe_yaw_deg": _safe_stats(np.degrees(np.asarray(rpe_yaw))),
    }


def differential_metrics(
    dataset: ExperimentDataset,
    estimated_keyframes: np.ndarray,
    sample_spacing_m: float,
) -> dict:
    names = list(dataset.keyframes)
    first = dataset.keyframes[names[0]]
    second = dataset.keyframes[names[1]]
    maximum = min(float(first.progress[-1]), float(second.progress[-1]))
    targets = np.arange(0.0, maximum + 1.0e-9, sample_spacing_m)
    if not len(targets) or maximum - targets[-1] > 0.25 * sample_spacing_m:
        targets = np.append(targets, maximum)
    translation = []
    yaw = []
    rows = []
    for target in targets:
        first_index = int(np.argmin(np.abs(first.progress - target)))
        second_index = int(np.argmin(np.abs(second.progress - target)))
        estimated_relative = between(
            estimated_keyframes[first.node_indices[first_index]],
            estimated_keyframes[second.node_indices[second_index]],
        )
        truth_relative = between(
            first.gt_poses[first_index], second.gt_poses[second_index]
        )
        error = between(truth_relative, estimated_relative)
        translation_error = float(np.linalg.norm(error[:2]))
        yaw_error = abs(float(wrap_angle(error[2])))
        translation.append(translation_error)
        yaw.append(yaw_error)
        rows.append(
            {
                "progress_m": float(target),
                "translation_error_m": translation_error,
                "yaw_error_deg": float(math.degrees(yaw_error)),
            }
        )
    return {
        "translation_m": _safe_stats(np.asarray(translation)),
        "yaw_deg": _safe_stats(np.degrees(np.asarray(yaw))),
        "endpoint_translation_m": float(translation[-1]),
        "endpoint_yaw_deg": float(math.degrees(yaw[-1])),
        "samples": rows,
    }


def map_metrics(grid: np.ndarray, reference: np.ndarray, spec: GridSpec) -> dict:
    common = (grid >= 0) & (reference >= 0)
    reference_observed = reference >= 0
    predicted_occupied = (grid == 100) & common
    reference_occupied = (reference == 100) & common
    true_positive = int(np.count_nonzero(predicted_occupied & reference_occupied))
    false_positive = int(np.count_nonzero(predicted_occupied & ~reference_occupied))
    false_negative = int(np.count_nonzero(~predicted_occupied & reference_occupied))
    precision = true_positive / max(1, true_positive + false_positive)
    recall = true_positive / max(1, true_positive + false_negative)
    f1 = 2.0 * precision * recall / max(1.0e-12, precision + recall)

    predicted_cells = np.argwhere(predicted_occupied)
    reference_cells = np.argwhere(reference_occupied)
    if len(predicted_cells) and len(reference_cells):
        from scipy.spatial import cKDTree

        predicted_xy = predicted_cells[:, ::-1] * spec.resolution
        reference_xy = reference_cells[:, ::-1] * spec.resolution
        forward = cKDTree(reference_xy).query(predicted_xy, k=1)[0]
        reverse = cKDTree(predicted_xy).query(reference_xy, k=1)[0]
        chamfer = float(0.5 * (np.mean(forward) + np.mean(reverse)))
    else:
        chamfer = None
    return {
        "common_observed_cells": int(np.count_nonzero(common)),
        "reference_observed_cells": int(np.count_nonzero(reference_observed)),
        "coverage_ratio": float(
            np.count_nonzero(common) / max(1, np.count_nonzero(reference_observed))
        ),
        "occupied_precision": float(precision),
        "occupied_recall": float(recall),
        "occupied_f1": float(f1),
        "occupied_chamfer_m": chamfer,
        "unknown_ratio": float(np.count_nonzero(grid < 0) / grid.size),
    }


def overlap_audit(dataset: ExperimentDataset, config: dict) -> List[dict]:
    """Estimate shared GT endpoint voxels in fixed progress bins."""
    evaluation = config.get("evaluation") or {}
    sampling = config.get("sampling") or {}
    bin_size = float(evaluation.get("overlap_bin_m", 20.0))
    voxel_size = float(evaluation.get("overlap_voxel_m", 0.5))
    beam_stride = max(1, int(sampling.get("map_beam_stride", 3)))
    names = list(dataset.vehicles)
    maximum = min(
        float(cumulative_progress(vehicle.gt_poses)[-1])
        for vehicle in dataset.vehicles.values()
    )
    rows = []
    for lower in np.arange(0.0, maximum + 1.0e-9, bin_size):
        upper = min(maximum, lower + bin_size)
        voxel_sets = {}
        for name in names:
            vehicle = dataset.vehicles[name]
            progress = cumulative_progress(vehicle.gt_poses)
            selected = np.flatnonzero((progress >= lower) & (progress < upper + 1.0e-9))
            voxels = set()
            for scan_index in selected[::2]:
                endpoints, hits = vehicle.scans[scan_index].local_endpoints(beam_stride)
                world = transform_points(
                    vehicle.gt_poses[scan_index], endpoints[hits]
                )
                for point in world:
                    voxels.add(
                        (
                            int(math.floor(point[0] / voxel_size)),
                            int(math.floor(point[1] / voxel_size)),
                        )
                    )
            voxel_sets[name] = voxels
        shared = set.intersection(*(voxel_sets[name] for name in names))
        denominator = min(len(voxel_sets[name]) for name in names)
        row = {
            "progress_start_m": float(lower),
            "progress_end_m": float(upper),
            "shared_voxels": int(len(shared)),
            "shared_ratio": float(len(shared) / max(1, denominator)),
        }
        for name in names:
            row[f"{name}_voxels"] = int(len(voxel_sets[name]))
        rows.append(row)
        if upper >= maximum:
            break
    return rows
