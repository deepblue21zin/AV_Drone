"""Two spatially separated LiDAR-validated regions, and same-input ablations.

Neither GT nor intended route windows are used. Components are formed from
independent neighboring anchor-pair support, not from repeated registrations
of one anchor. Selection and all graph weights are shared across ablations.
"""

import math
import time

import numpy as np

from .graph_builder import build_intra_factors, build_priors, initial_pose_array
from .place_recognition import estimate
from .pose_graph import PoseFactor, optimize_pose_graph


def select_regions(rows, dataset, config):
    valid = {row["pair"]: row for row in rows
             if row["geometry_pass"] and row.get("support_pairs")}
    pending = set(valid)
    components = []
    while pending:
        queue, members = [min(pending)], set()
        while queue:
            pair = queue.pop()
            if pair in members or pair not in valid:
                continue
            members.add(pair)
            queue.extend(valid[pair]["support_pairs"])
        pending -= members
        representative = min((valid[p] for p in members),
                             key=lambda r: (-r["overlap_ratio"], r["cost"], r["pair"]))
        components.append({"members": sorted(members), "representative": representative})
    components.sort(key=lambda c: (c["representative"]["target_progress_m"],
                                    c["representative"]["pair"]))
    selected = []
    settings = config["regions"]
    tracks = list(dataset.keyframes.values())
    for component in components:
        row = component["representative"]
        if selected:
            previous = selected[-1]["representative"]
            progress_separated = all(
                row[field]-previous[field] >= settings["minimum_progress_separation_m"]
                for field in ("target_progress_m", "target_vehicle_progress_m"))
            spatially_separated = all(
                np.linalg.norm(track.raw_poses[row[field], :2]-track.raw_poses[previous[field], :2])
                >= settings["minimum_world_separation_m"]
                for track, field in zip(tracks, ("source_keyframe", "target_keyframe")))
            if not (progress_separated and spatially_separated):
                continue
        selected.append(component)
        if len(selected) == 2:
            break
    return selected, components


def estimate_two_regions(dataset, config, on_pair=None):
    started = time.monotonic()
    result = estimate(dataset, config, on_pair=on_pair, registration_only=True)
    selected, components = select_regions(result["rows"], dataset, config)
    regions = []
    factors = []
    first, second = list(dataset.keyframes.values())
    graph = config["pose_graph"]
    for index, component in enumerate(selected):
        row = component["representative"]
        label = "AB"[index]
        regions.append({"region": label, "pair": row["pair"],
                        "support_component_pairs": component["members"],
                        "source_progress_m": row["target_progress_m"],
                        "target_progress_m": row["target_vehicle_progress_m"],
                        "source_time": row["source_time"], "target_time": row["target_time"]})
        factors.append(PoseFactor(
            int(first.node_indices[row["source_keyframe"]]),
            int(second.node_indices[row["target_keyframe"]]),
            np.array(row["estimated_pose"]),
            np.array([graph["registration_translation_sigma_m"]]*2 +
                     [math.radians(graph["registration_yaw_sigma_deg"])]),
            "lidar_no_gt_region_"+label))
    selections = {"N-prior": [], "N-single-A": [0] if factors else [],
                  "N-single-B": [1] if len(factors) == 2 else [],
                  "N-double": list(range(len(factors)))}
    requested = {"N-prior": 0, "N-single-A": 1, "N-single-B": 1, "N-double": 2}
    initial = initial_pose_array(dataset)
    intra, priors = build_intra_factors(dataset, config), build_priors(dataset, config)
    poses, optimizers, condition_factors = {}, {}, {}
    for condition, indices in selections.items():
        chosen = [factors[i] for i in indices]
        condition_factors[condition] = chosen
        if chosen or condition == "N-prior":
            optimized = optimize_pose_graph(
                initial.copy(), [*intra, *chosen], priors,
                loss=graph["robust_loss"], robust_scale=graph["robust_scale"],
                max_function_evaluations=graph["max_function_evaluations"])
            poses[condition] = optimized.poses if optimized.success else initial.copy()
            optimizer = {"success": optimized.success, "evaluations": optimized.evaluations,
                         "message": optimized.message}
        else:
            poses[condition] = initial.copy()
            optimizer = {"success": False, "evaluations": 0,
                         "message": "Required validated region unavailable; B0 retained (not a valid ablation)"}
        optimizer.update(accepted_factor_count=len(chosen), requested_factor_count=requested[condition],
                         available=len(chosen) == requested[condition],
                         applied_factor_count=len(chosen) if optimizer["success"] else 0,
                         selected_pairs=[regions[i]["pair"] for i in indices])
        optimizers[condition] = optimizer
        print(f"{condition}: {optimizer}", flush=True)
    for row in result["rows"]:
        row["region"] = next((r["region"] for r in regions
                              if row["pair"] in r["support_component_pairs"]), "")
        row["accepted_conditions"] = [c for c, indices in selections.items()
                                       if any(regions[i]["pair"] == row["pair"] for i in indices)]
        row["accepted"] = "N-double" in row["accepted_conditions"]
        if row["accepted"]:
            row["rejection_reasons"] = ""
        elif row.get("support_pairs"):
            row["rejection_reasons"] = "not_selected_independent_region_representative"
    return {**result, "poses": poses["N-double"], "factors": factors,
            "optimizer": optimizers["N-double"], "selected_pair": None,
            "selected_pairs": [r["pair"] for r in regions], "regions": regions,
            "validated_component_count": len(components),
            "condition_poses": poses, "condition_factors": condition_factors,
            "condition_optimizers": optimizers, "elapsed_sec": time.monotonic()-started}
