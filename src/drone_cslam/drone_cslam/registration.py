"""Coarse-to-fine 2D LiDAR submap registration for inter-UAV factors."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

from .se2 import inverse, transform_points, wrap_angle


@dataclass(frozen=True)
class RegistrationCandidate:
    pose: np.ndarray
    cost: float
    overlap_ratio: float
    inlier_rmse_m: float


@dataclass(frozen=True)
class RegistrationResult:
    initial_pose: np.ndarray
    pose: np.ndarray
    cost: float
    overlap_ratio: float
    inlier_rmse_m: float
    score_margin_ratio: float
    correction_translation_m: float
    correction_yaw_deg: float
    accepted: bool
    rejection_reasons: tuple[str, ...]
    candidates: tuple[RegistrationCandidate, ...]


def _sample_evenly(points: np.ndarray, maximum: int) -> np.ndarray:
    if len(points) <= maximum:
        return points
    indices = np.linspace(0, len(points) - 1, maximum, dtype=int)
    return points[indices]


class _SymmetricObjective:
    def __init__(
        self,
        source: np.ndarray,
        target: np.ndarray,
        correspondence_m: float,
        maximum_points: int,
    ):
        self.source = _sample_evenly(
            np.asarray(source, dtype=float), maximum_points
        )
        self.target = _sample_evenly(
            np.asarray(target, dtype=float), maximum_points
        )
        self.source_tree = cKDTree(self.source)
        self.target_tree = cKDTree(self.target)
        self.correspondence_m = float(correspondence_m)

    def metrics(self, pose: np.ndarray):
        target_in_source = transform_points(pose, self.target)
        source_in_target = transform_points(inverse(pose), self.source)
        first = self.source_tree.query(target_in_source, workers=1)[0]
        second = self.target_tree.query(source_in_target, workers=1)[0]
        distances = np.concatenate((first, second))
        inliers = distances <= self.correspondence_m
        overlap = float(np.mean(inliers)) if len(distances) else 0.0
        rmse = (
            float(np.sqrt(np.mean(np.square(distances[inliers]))))
            if np.any(inliers)
            else float(self.correspondence_m)
        )
        # Truncated symmetric Chamfer distance penalizes unsupported geometry
        # while remaining robust to the unobserved portion of each submap.
        cost = float(np.mean(np.minimum(distances, self.correspondence_m)))
        return cost, overlap, rmse

    def __call__(self, pose: np.ndarray) -> float:
        candidate = np.asarray(pose, dtype=float).copy()
        candidate[2] = float(wrap_angle(candidate[2]))
        return self.metrics(candidate)[0]


def _candidate(objective: _SymmetricObjective, pose: np.ndarray):
    pose = np.asarray(pose, dtype=float).copy()
    pose[2] = float(wrap_angle(pose[2]))
    cost, overlap, rmse = objective.metrics(pose)
    return RegistrationCandidate(pose, cost, overlap, rmse)


def register_submaps(
    source_points: np.ndarray,
    target_points: np.ndarray,
    initial_pose: np.ndarray,
    config: dict,
) -> RegistrationResult:
    """Estimate target-in-source SE(2) around the B0 relative-pose prior."""
    values = config.get("registration") or {}
    minimum_points = int(values.get("minimum_submap_points", 80))
    if (
        len(source_points) < minimum_points
        or len(target_points) < minimum_points
    ):
        raise ValueError(
            f"submaps need at least {minimum_points} points; "
            f"got {len(source_points)} and {len(target_points)}"
        )
    xy_window = float(values.get("search_translation_m", 3.0))
    yaw_window = math.radians(float(values.get("search_yaw_deg", 25.0)))
    xy_step = float(values.get("coarse_translation_step_m", 0.5))
    yaw_step = math.radians(float(values.get("coarse_yaw_step_deg", 2.5)))
    correspondence = float(values.get("correspondence_distance_m", 0.60))
    maximum_points = int(values.get("coarse_max_points", 700))
    top_k = max(2, int(values.get("top_k", 5)))
    hypothesis_xy = float(
        values.get("hypothesis_translation_separation_m", 0.75)
    )
    hypothesis_yaw = math.radians(
        float(values.get("hypothesis_yaw_separation_deg", 5.0))
    )
    objective = _SymmetricObjective(
        source_points, target_points, correspondence, maximum_points
    )
    initial = np.asarray(initial_pose, dtype=float)

    x_offsets = np.arange(-xy_window, xy_window + 0.5 * xy_step, xy_step)
    y_offsets = np.arange(-xy_window, xy_window + 0.5 * xy_step, xy_step)
    yaw_offsets = np.arange(
        -yaw_window, yaw_window + 0.5 * yaw_step, yaw_step
    )
    coarse: List[RegistrationCandidate] = []
    for yaw_offset in yaw_offsets:
        yaw = float(wrap_angle(initial[2] + yaw_offset))
        for x_offset in x_offsets:
            for y_offset in y_offsets:
                coarse.append(
                    _candidate(
                        objective,
                        np.array(
                            [
                                initial[0] + x_offset,
                                initial[1] + y_offset,
                                yaw,
                            ],
                            dtype=float,
                        ),
                    )
                )
    coarse.sort(key=lambda item: item.cost)

    refined: List[RegistrationCandidate] = []
    # Refine several distinct coarse minima; cylindrical geometry can produce
    # multiple yaw/translation hypotheses with similar scores.
    seeds = []
    for candidate in coarse:
        if all(
            np.linalg.norm(candidate.pose[:2] - seed.pose[:2])
            >= hypothesis_xy
            or abs(
                float(wrap_angle(candidate.pose[2] - seed.pose[2]))
            ) >= hypothesis_yaw
            for seed in seeds
        ):
            seeds.append(candidate)
        if len(seeds) >= top_k:
            break
    lower = np.array(
        [
            initial[0] - xy_window,
            initial[1] - xy_window,
            initial[2] - yaw_window,
        ]
    )
    upper = np.array(
        [
            initial[0] + xy_window,
            initial[1] + xy_window,
            initial[2] + yaw_window,
        ]
    )
    for seed in seeds:
        # Coarse candidates are wrapped to [-pi, pi], while Powell's interval
        # is continuous around initial yaw. Lift the seed into that interval.
        seed_pose = seed.pose.copy()
        seed_pose[2] = initial[2] + float(wrap_angle(seed_pose[2] - initial[2]))
        # Lifting a boundary angle can overshoot by floating-point epsilon.
        seed_pose = np.clip(seed_pose, lower, upper)
        # This is LOCAL refinement of a coarse basin, not a fresh search over
        # the entire 12 m / 60 deg box. Bounded Powell line searches over that
        # large multimodal interval can leave a near-perfect coarse solution
        # and return a much worse repeated-cylinder alignment.
        refinement_radius = np.array([xy_step, xy_step, yaw_step])
        local_lower = np.maximum(lower, seed_pose-refinement_radius)
        local_upper = np.minimum(upper, seed_pose+refinement_radius)
        result = minimize(
            objective,
            seed_pose,
            method="Powell",
            bounds=list(zip(local_lower, local_upper)),
            options={"xtol": 0.01, "ftol": 1.0e-4, "maxiter": 60},
        )
        candidate = _candidate(objective, result.x)
        # Refinement must never destroy an already observed better solution.
        refined.append(candidate if candidate.cost <= seed.cost else seed)
    refined.sort(key=lambda item: item.cost)
    distinct = []
    for candidate in refined:
        if all(
            np.linalg.norm(candidate.pose[:2] - other.pose[:2])
            >= hypothesis_xy
            or abs(
                float(wrap_angle(candidate.pose[2] - other.pose[2]))
            ) >= hypothesis_yaw
            for other in distinct
        ):
            distinct.append(candidate)
    best = distinct[0]
    if len(distinct) > 1:
        second = distinct[1]
        margin = float(
            (second.cost - best.cost) / max(best.cost, 1.0e-9)
        )
    else:
        # All independently seeded searches converged to one pose basin.
        margin = 1.0
    correction_xy = float(np.linalg.norm(best.pose[:2] - initial[:2]))
    correction_yaw = math.degrees(
        abs(float(wrap_angle(best.pose[2] - initial[2])))
    )

    gate = config.get("gate") or {}
    reasons = []
    if best.overlap_ratio < float(gate.get("minimum_overlap_ratio", 0.18)):
        reasons.append("low_overlap")
    if best.inlier_rmse_m > float(gate.get("maximum_inlier_rmse_m", 0.40)):
        reasons.append("high_residual")
    if margin < float(gate.get("minimum_score_margin_ratio", 0.02)):
        reasons.append("ambiguous_minimum")
    if correction_xy > float(
        gate.get("maximum_correction_translation_m", xy_window)
    ):
        reasons.append("translation_correction_limit")
    if correction_yaw > float(
        gate.get(
            "maximum_correction_yaw_deg", math.degrees(yaw_window)
        )
    ):
        reasons.append("yaw_correction_limit")
    return RegistrationResult(
        initial_pose=initial.copy(),
        pose=best.pose.copy(),
        cost=best.cost,
        overlap_ratio=best.overlap_ratio,
        inlier_rmse_m=best.inlier_rmse_m,
        score_margin_ratio=margin,
        correction_translation_m=correction_xy,
        correction_yaw_deg=correction_yaw,
        accepted=not reasons,
        rejection_reasons=tuple(reasons),
        candidates=tuple(distinct),
    )
