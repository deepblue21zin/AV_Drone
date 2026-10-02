"""Build baseline and noisy-oracle factor sets from synchronized keyframes."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .data import ExperimentDataset
from .pose_graph import PoseFactor, PosePrior
from .se2 import between, compose, wrap_angle


@dataclass(frozen=True)
class OraclePair:
    source_vehicle: str
    target_vehicle: str
    source_keyframe: int
    target_keyframe: int
    source_node: int
    target_node: int
    target_progress_m: float
    measurement: np.ndarray


def initial_pose_array(dataset: ExperimentDataset) -> np.ndarray:
    node_count = sum(len(track.node_indices) for track in dataset.keyframes.values())
    poses = np.empty((node_count, 3), dtype=float)
    for track in dataset.keyframes.values():
        poses[track.node_indices] = track.raw_poses
    return poses


def build_intra_factors(dataset: ExperimentDataset, config: dict):
    graph = config.get("pose_graph") or {}
    sigma = np.array(
        [
            float(graph.get("intra_translation_sigma_m", 0.15)),
            float(graph.get("intra_translation_sigma_m", 0.15)),
            math.radians(float(graph.get("intra_yaw_sigma_deg", 2.0))),
        ],
        dtype=float,
    )
    factors = []
    for track in dataset.keyframes.values():
        for index in range(len(track.node_indices) - 1):
            factors.append(
                PoseFactor(
                    source=int(track.node_indices[index]),
                    target=int(track.node_indices[index + 1]),
                    measurement=between(
                        track.raw_poses[index], track.raw_poses[index + 1]
                    ),
                    sigma=sigma.copy(),
                    kind="intra_raw_slam",
                )
            )
    return factors


def build_priors(dataset: ExperimentDataset, config: dict):
    graph = config.get("pose_graph") or {}
    sigma = np.array(
        [
            float(graph.get("prior_translation_sigma_m", 0.02)),
            float(graph.get("prior_translation_sigma_m", 0.02)),
            math.radians(float(graph.get("prior_yaw_sigma_deg", 0.2))),
        ],
        dtype=float,
    )
    vehicle_configs = {str(item["name"]): item for item in config["vehicles"]}
    priors = []
    for name, track in dataset.keyframes.items():
        measurement = np.asarray(vehicle_configs[name]["spawn"], dtype=float)
        priors.append(
            PosePrior(
                node=int(track.node_indices[0]),
                measurement=measurement,
                sigma=sigma.copy(),
            )
        )
    return priors


def _unique_progress_pairs(first_track, second_track, targets):
    selected = []
    seen = set()
    for target in targets:
        first_index = int(np.argmin(np.abs(first_track.progress - target)))
        second_index = int(np.argmin(np.abs(second_track.progress - target)))
        key = (first_index, second_index)
        if key in seen:
            continue
        seen.add(key)
        selected.append((float(target), first_index, second_index))
    return selected


def build_oracle_factors(
    dataset: ExperimentDataset,
    config: dict,
    condition: str,
) -> Tuple[List[PoseFactor], List[OraclePair]]:
    names = list(dataset.keyframes)
    if len(names) != 2:
        raise ValueError("phase-1 oracle evaluation currently requires exactly two UAVs")
    if condition not in {"O-single", "O-periodic"}:
        raise ValueError(f"unknown oracle condition: {condition}")
    first = dataset.keyframes[names[0]]
    second = dataset.keyframes[names[1]]
    maximum = min(float(first.progress[-1]), float(second.progress[-1]))
    graph = config.get("pose_graph") or {}
    spacing = float(graph.get("oracle_spacing_m", 10.0))
    if condition == "O-single":
        targets = [0.5 * maximum]
    else:
        targets = np.arange(spacing, maximum + 1.0e-9, spacing).tolist()
        if not targets or maximum - targets[-1] >= 0.5 * spacing:
            targets.append(maximum)

    sigma = np.array(
        [
            float(graph.get("oracle_translation_sigma_m", 0.05)),
            float(graph.get("oracle_translation_sigma_m", 0.05)),
            math.radians(float(graph.get("oracle_yaw_sigma_deg", 0.5))),
        ],
        dtype=float,
    )
    seed = int(graph.get("random_seed", 42)) + (1 if condition == "O-single" else 2)
    random = np.random.default_rng(seed)
    factors = []
    pairs = []
    for target, first_index, second_index in _unique_progress_pairs(
        first, second, targets
    ):
        truth = between(first.gt_poses[first_index], second.gt_poses[second_index])
        noise = random.normal(0.0, sigma)
        noise[2] = float(wrap_angle(noise[2]))
        measurement = compose(truth, noise)
        factor = PoseFactor(
            source=int(first.node_indices[first_index]),
            target=int(second.node_indices[second_index]),
            measurement=measurement,
            sigma=sigma.copy(),
            kind="oracle_inter_uav",
        )
        factors.append(factor)
        pairs.append(
            OraclePair(
                source_vehicle=names[0],
                target_vehicle=names[1],
                source_keyframe=first_index,
                target_keyframe=second_index,
                source_node=factor.source,
                target_node=factor.target,
                target_progress_m=target,
                measurement=measurement,
            )
        )
    return factors, pairs
