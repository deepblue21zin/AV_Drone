"""Odometry-prior-assisted submap retrieval and one-region offline correction.

Only scan-derived geometry and raw SLAM poses are accessed here. The initial
inter-vehicle coordinate calibration is assumed known; no online GT is used.
"""

import math
import time

import numpy as np
from scipy.spatial.distance import pdist

from .graph_builder import build_intra_factors, build_priors, initial_pose_array
from .pose_graph import PoseFactor, optimize_pose_graph
from .registration import register_submaps
from .se2 import between, compose, inverse, transform_points, wrap_angle
from .submap import build_local_submap


def shape_descriptor(points):
    """Deterministic translation/rotation-invariant pair-distance histogram."""
    points = np.asarray(points)
    indices = np.linspace(0, len(points)-1, min(len(points), 128), dtype=int)
    distances = pdist(points[indices]) if len(indices) > 1 else np.empty(0)
    hist, _ = np.histogram(distances, bins=np.linspace(0, 32, 65))
    hist = hist.astype(float)
    return hist / max(1, hist.sum())


def correction_disagreement(first, second, reference_xy):
    """Compare corrections near the shared scene, independent of world origin.

    Comparing their translation coefficients at world (0,0) couples a small
    yaw difference to a long lever arm and falsely rejects far-away scenes.
    """
    point = np.asarray(reference_xy, dtype=float).reshape(1, 2)
    distance = float(np.linalg.norm(transform_points(first, point)-transform_points(second, point)))
    yaw = abs(math.degrees(float(wrap_angle(first[2]-second[2]))))
    return distance, yaw


def retrieve_candidates(dataset, config):
    names = list(dataset.keyframes)
    if len(names) != 2:
        raise ValueError("Exactly two UAVs are required")
    settings = config["retrieval"]
    minimum_points = config["registration"].get("minimum_submap_points", 80)
    caches = {}
    for name in names:
        track = dataset.keyframes[name]
        cache = {}
        last = -math.inf
        for index, progress in enumerate(track.progress):
            if progress < settings["minimum_travel_m"] or progress-last < settings["anchor_spacing_m"]:
                continue
            submap = build_local_submap(dataset, name, index, config)
            if len(submap.points) >= minimum_points:
                cache[index] = (submap, shape_descriptor(submap.points))
                last = progress
        caches[name] = cache
    first, second = (dataset.keyframes[name] for name in names)
    candidates = []
    for i, (_, descriptor) in caches[names[0]].items():
        scored = []
        for j, (_, other) in caches[names[1]].items():
            distance = float(np.linalg.norm(first.raw_poses[i, :2]-second.raw_poses[j, :2]))
            if distance > settings["maximum_prior_distance_m"]:
                continue
            score = float(np.abs(descriptor-other).sum())
            scored.append((score, distance, j))
        ranked = sorted(scored)[:settings["top_k_per_anchor"]]
        # Optional complementary retrieval: repeated cylinders can rank a
        # nearby overlapping submap below distant look-alikes. This changes
        # recall only; registration and independent-neighbor gates still apply.
        nearest = sorted(scored, key=lambda value: (value[1], value[0], value[2]))
        seen = {item[2] for item in ranked}
        for item in nearest[:settings.get("spatial_top_k_per_anchor", 0)]:
            if item[2] not in seen:
                ranked.append(item)
                seen.add(item[2])
        for score, distance, j in ranked:
            candidates.append({"source_keyframe": i, "target_keyframe": j,
                               "descriptor_distance": score, "prior_distance_m": distance})
    candidates.sort(key=lambda row: (row["descriptor_distance"], row["prior_distance_m"],
                                     row["source_keyframe"], row["target_keyframe"]))
    return names, candidates[:settings["maximum_candidates"]], caches


def estimate(dataset, config, on_pair=None, registration_only=False):
    started = time.monotonic()
    names, candidates, caches = retrieve_candidates(dataset, config)
    first, second = (dataset.keyframes[name] for name in names)
    rows, results = [], []
    for number, candidate in enumerate(candidates):
        i, j = candidate["source_keyframe"], candidate["target_keyframe"]
        source, target = caches[names[0]][i][0], caches[names[1]][j][0]
        initial = between(first.raw_poses[i], second.raw_poses[j])
        result = register_submaps(source.points, target.points, initial, config)
        correction = compose(compose(first.raw_poses[i], result.pose), inverse(second.raw_poses[j]))
        row = {**candidate, "pair": number, "source_vehicle": names[0], "target_vehicle": names[1],
               "target_progress_m": float(first.progress[i]),
               "target_vehicle_progress_m": float(second.progress[j]),
               "source_time": float(first.times[i]), "target_time": float(second.times[j]),
               "association_source": "lidar_descriptor_with_raw_slam_spatial_prior",
               "initial_pose": initial.tolist(), "estimated_pose": result.pose.tolist(),
               "correction_world": correction.tolist(), "cost": result.cost,
               "overlap_ratio": result.overlap_ratio, "inlier_rmse_m": result.inlier_rmse_m,
               "score_margin_ratio": result.score_margin_ratio,
               "correction_translation_m": result.correction_translation_m,
               "correction_yaw_deg": result.correction_yaw_deg,
               "geometry_pass": bool(result.accepted), "accepted": False,
               "rejection_reasons": ";".join(result.rejection_reasons),
               "source_submap_points": len(source.points), "target_submap_points": len(target.points)}
        rows.append(row)
        results.append((source, target, result))
        if on_pair:
            on_pair(number+1, len(candidates), row)

    # Adjacent independent anchor pairs must support a similar map correction.
    # The resulting factors are correlated, so only one representative is used.
    consistent = []
    settings = config["consistency"]
    for row in rows:
        row["support_pairs"] = []
        if not row["geometry_pass"]:
            continue
        for other in rows:
            if not other["geometry_pass"] or row["pair"] == other["pair"]:
                continue
            ds = row["target_progress_m"]-other["target_progress_m"]
            dt = row["target_vehicle_progress_m"]-other["target_vehicle_progress_m"]
            if not (settings["minimum_anchor_separation_m"] <= abs(ds) <= settings["neighborhood_m"]
                    and settings["minimum_anchor_separation_m"] <= abs(dt) <= settings["neighborhood_m"]
                    and ds*dt > 0):
                continue
            reference = .5 * (second.raw_poses[row["target_keyframe"], :2] +
                              second.raw_poses[other["target_keyframe"], :2])
            distance, yaw = correction_disagreement(np.array(row["correction_world"]),
                                                    np.array(other["correction_world"]), reference)
            if distance <= settings["translation_m"] and yaw <= settings["yaw_deg"]:
                row["support_pairs"].append(other["pair"])
        if row["support_pairs"]:
            consistent.append(row)
        else:
            row["rejection_reasons"] = "no_neighbor_consistency"
    if registration_only:
        return {"rows": rows, "results": results,
                "consistent_pair_count": len(consistent),
                "elapsed_sec": time.monotonic()-started}
    selected = min(consistent, key=lambda row: (-row["overlap_ratio"], row["cost"], row["pair"])) if consistent else None
    for row in consistent:
        row["accepted"] = row is selected
        if row is not selected:
            row["rejection_reasons"] = "not_selected_single_region_representative"
    initial_graph = initial_pose_array(dataset)
    factors = []
    graph = config["pose_graph"]
    if selected:
        factors.append(PoseFactor(
            int(first.node_indices[selected["source_keyframe"]]),
            int(second.node_indices[selected["target_keyframe"]]),
            np.array(selected["estimated_pose"]),
            np.array([graph["registration_translation_sigma_m"]]*2 +
                     [math.radians(graph["registration_yaw_sigma_deg"])]), "lidar_no_gt_single"))
        optimized = optimize_pose_graph(initial_graph, [*build_intra_factors(dataset, config), *factors],
                                        build_priors(dataset, config), loss=graph["robust_loss"],
                                        robust_scale=graph["robust_scale"],
                                        max_function_evaluations=graph["max_function_evaluations"])
        poses = optimized.poses if optimized.success else initial_graph.copy()
        optimizer = {"success": optimized.success, "evaluations": optimized.evaluations,
                     "message": optimized.message, "accepted_factor_count": len(factors)}
    else:
        poses = initial_graph.copy()
        optimizer = {"success": True, "evaluations": 0, "message": "No validated match; B0 retained",
                     "accepted_factor_count": 0}
    return {"poses": poses, "rows": rows, "results": results, "factors": factors,
            "optimizer": optimizer, "elapsed_sec": time.monotonic()-started,
            "selected_pair": None if selected is None else selected["pair"],
            "consistent_pair_count": len(consistent)}
