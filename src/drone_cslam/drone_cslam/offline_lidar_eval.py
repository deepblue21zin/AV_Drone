"""Post-flight LiDAR place retrieval/correction with GT loaded only for scoring."""

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml

from .bag_reader import load_bag
from .dataset import build_dataset
from .lidar_eval import _write_json, _write_csv, _save_registration_plot, _registration_summary
from .mapping import build_occupancy_grid, corrected_scan_poses, grid_spec_from_config, save_occupancy_grid
from .metrics import trajectory_metrics, differential_metrics, map_metrics
from .oracle_eval import _condition_keyframes, _trajectory_rows, _vehicle_audit
from .place_recognition import estimate
from .region_correction import estimate_two_regions
from .se2 import between, transform_points, wrap_angle
from .time_series import interpolate_pose_series, sorted_unique_pose_series

CONDITIONS = ("B0", "B1", "N-single")


def attach_evaluation_truth(dataset, truth_records, config):
    """Attach scoring labels without changing scan selection, nodes, or estimates."""
    result = copy.deepcopy(dataset)
    support = {}
    for name, vehicle in result.vehicles.items():
        times, values = sorted_unique_pose_series(truth_records[name].ground_truth)
        poses, valid = interpolate_pose_series(times, values, vehicle.scan_times,
                                               config["sampling"]["interpolation_max_gap_sec"])
        poses[~valid] = np.nan
        vehicle.gt_poses = poses
        track = result.keyframes[name]
        track.gt_poses = poses[track.scan_indices]
        support[name] = {"gt_supported_scans": int(valid.sum()), "total_scans": len(valid),
                         "gt_supported_keyframes": int(np.isfinite(track.gt_poses).all(axis=1).sum())}
    return result, support


def spatial_overlap(dataset, config):
    """GT-only audit by world-x bins, independent of observation timestamps."""
    size = config["evaluation"]["overlap_bin_m"]
    voxel = config["evaluation"]["overlap_voxel_m"]
    bins = {}
    names = list(dataset.vehicles)
    for name, vehicle in dataset.vehicles.items():
        for index in range(0, len(vehicle.scans), 4):
            pose = vehicle.gt_poses[index]
            if not np.all(np.isfinite(pose)):
                continue
            bin_id = math.floor(pose[0]/size)
            entry = bins.setdefault(bin_id, {n: set() for n in names})
            points, hits = vehicle.scans[index].local_endpoints(3)
            cells = np.floor(transform_points(pose, points[hits])/voxel).astype(int)
            entry[name].update(map(tuple, cells.tolist()))
    rows = []
    for index, values in sorted(bins.items()):
        shared = set.intersection(*(values[name] for name in names))
        row = {"world_x_start_m": index*size, "world_x_end_m": (index+1)*size,
               "shared_voxels": len(shared),
               "shared_ratio": len(shared)/max(1, min(len(values[name]) for name in names))}
        row.update({name+"_voxels": len(values[name]) for name in names})
        rows.append(row)
    return rows


def make_plots(output, metrics, dataset, keyframes, config, registrations=None, overlap=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"B0": "#dc2626", "B1": "#d97706", "N-single": "#2563eb", "GT": "#15803d",
              "N-prior": "#64748b", "N-single-A": "#9333ea", "N-single-B": "#0891b2",
              "N-double": "#2563eb"}
    figure, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    for axis, (name, track) in zip(axes, dataset.keyframes.items()):
        axis.plot(track.gt_poses[:, 0], track.gt_poses[:, 1], color=colors["GT"], label="GT")
        for condition, poses in keyframes.items():
            path = poses[track.node_indices]
            axis.plot(path[:, 0], path[:, 1], color=colors[condition], label=condition)
        axis.set(title=name, ylabel="world y [m]", xlim=(-1, 151), ylim=(-16, 16))
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=.2)
        axis.legend(ncol=4)
    axes[-1].set_xlabel("world x [m]")
    figure.tight_layout()
    figure.savefig(output / "overview_trajectories.png", dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(2, 2, figsize=(14, 7))
    for column, (name, track) in enumerate(dataset.keyframes.items()):
        for condition, poses in keyframes.items():
            path = poses[track.node_indices]
            position = np.linalg.norm(path[:, :2]-track.gt_poses[:, :2], axis=1)
            yaw = np.degrees(np.abs(wrap_angle(path[:, 2]-track.gt_poses[:, 2])))
            axes[0, column].plot(track.progress, position, label=condition, color=colors[condition])
            axes[1, column].plot(track.progress, yaw, label=condition, color=colors[condition])
        axes[0, column].set(title=name, ylabel="absolute position error [m]")
        axes[1, column].set(xlabel="raw-SLAM accumulated distance [m]", ylabel="absolute wrapped yaw error [deg]")
        for axis in axes[:, column]:
            axis.grid(alpha=.2)
            axis.legend()
    figure.tight_layout()
    figure.savefig(output / "overview_errors.png", dpi=160)
    plt.close(figure)

    spec = grid_spec_from_config(config)
    reference = np.load(output / "maps/GT-reference.npy")
    figure, axes = plt.subplots(len(keyframes), 1, figsize=(15, 3*len(keyframes)))
    for axis, condition in zip(axes, keyframes):
        grid = np.load(output / "maps" / f"{condition}.npy")
        rgb = np.ones((*grid.shape, 3), dtype=np.float32)
        rgb[(grid < 0) & (reference < 0)] = .86
        rgb[(reference == 100) & (grid != 100)] = [0.16, .65, .25]
        rgb[(grid == 100) & (reference != 100)] = [.87, .18, .16]
        rgb[(grid == 100) & (reference == 100)] = [.10, .10, .10]
        axis.imshow(rgb, origin="lower", extent=[spec.min_x,spec.max_x,spec.min_y,spec.max_y])
        axis.set(title=f"{condition}: green=GT-only; red=estimate-only; black=coincident occupied", ylabel="world y [m]")
    axes[-1].set_xlabel("world x [m]")
    figure.tight_layout()
    figure.savefig(output / "overview_map_errors.png", dpi=160)
    plt.close(figure)

    if registrations is not None:
        figure, axes = plt.subplots(2, 1, figsize=(14, 7), gridspec_kw={"height_ratios": [2, 1]})
        for (name, track), color in zip(dataset.keyframes.items(), ("#2563eb", "#ea580c")):
            axes[0].plot(track.gt_poses[:, 0], track.gt_poses[:, 1], color=color, label=name+" GT (evaluation only)")
        tracks = list(dataset.keyframes.values())
        for row in registrations:
            if not row["accepted"]:
                continue
            a = tracks[0].gt_poses[row["source_keyframe"], :2]
            b = tracks[1].gt_poses[row["target_keyframe"], :2]
            if not np.isfinite([a, b]).all():
                continue
            axes[0].plot([a[0], b[0]], [a[1], b[1]], "ko--", markersize=6)
            axes[0].annotate(f"{row.get('region') or 'single'} / pair {row['pair']}\n"
                             f"observed {abs(row['source_time']-row['target_time']):.1f}s apart",
                             .5*(a+b), xytext=(0, 30), textcoords="offset points", ha="center",
                             arrowprops={"arrowstyle": "-", "color": "black"})
        axes[0].set(xlim=(-1, 151), ylim=(-16, 16), ylabel="world y [m]",
                    title="Selected LiDAR constraints on GT tracks: linked observations are NOT simultaneous meetings")
        axes[0].set_aspect("equal", adjustable="box")
        axes[0].legend(loc="lower left")
        if overlap:
            axes[1].bar([r["world_x_start_m"] for r in overlap], [r["shared_ratio"] for r in overlap],
                        width=config["evaluation"]["overlap_bin_m"]*.9, align="edge", color="#475569")
        axes[1].set(xlim=(-1, 151), xlabel="world x [m]", ylabel="shared hit-voxel ratio",
                    title="GT-only post-flight overlap audit; never used to choose or fit constraints")
        for axis in axes:
            axis.grid(alpha=.2)
        figure.tight_layout()
        figure.savefig(output / "overview_shared_regions.png", dpi=160)
        plt.close(figure)


def complete_two_region_ablation(optimizers):
    """Count actual applied constraints, not condition names or requested counts."""
    expected = {"N-single-A": 1, "N-single-B": 1, "N-double": 2}
    return all(optimizers.get(name, {}).get("available")
               and optimizers[name].get("success")
               and optimizers[name].get("applied_factor_count") == count
               for name, count in expected.items())


def run(bag, config_path, output_path, inference_only=False, require_complete_ablation=False):
    started = time.monotonic()
    output = Path(output_path).resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Refusing to replace previous analysis: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config_text = Path(config_path).read_text()
    config = yaml.safe_load(config_text)
    if require_complete_ablation and config.get("experiment", {}).get("correction_mode") != "two_regions":
        raise ValueError("--require-complete-ablation requires correction_mode: two_regions")
    (output / "config.yaml").write_text(config_text)
    (output / "source_snapshots").mkdir()
    source_hashes = {}
    for name in ("bag_reader.py", "dataset.py", "place_recognition.py", "region_correction.py",
                 "graph_builder.py", "pose_graph.py", "mapping.py", "registration.py", "offline_lidar_eval.py"):
        source = Path(__file__).with_name(name).read_bytes()
        source_hashes[name] = hashlib.sha256(source).hexdigest()
        (output / "source_snapshots" / name).write_bytes(source)
    for name in ("maps", "trajectories", "submaps", "debug_plots"):
        (output / name).mkdir()
    print("Reading inference inputs WITHOUT ground-truth topics", flush=True)
    observations = load_bag(bag, config, include_ground_truth=False)
    assert all(not vehicle.ground_truth for vehicle in observations.values())
    dataset = build_dataset(observations, config, use_ground_truth=False)
    def checkpoint(done, total, row):
        print(f"Registration {done}/{total}: overlap={row['overlap_ratio']:.3f} geometry_pass={row['geometry_pass']}", flush=True)
        with (output / "inference_candidates.jsonl").open("a") as stream:
            stream.write(json.dumps(row)+"\n")
    multi = config.get("experiment", {}).get("correction_mode") == "two_regions"
    primary = "N-double" if multi else "N-single"
    inferred = (estimate_two_regions if multi else estimate)(dataset, config, on_pair=checkpoint)
    condition_poses = inferred.get("condition_poses", {primary: inferred["poses"]})
    optimizers = inferred.get("condition_optimizers", {primary: inferred["optimizer"]})
    keyframes = {"B0": _condition_keyframes(dataset, "B0"),
                 "B1": _condition_keyframes(dataset, "B1"), **condition_poses}
    # Persist all estimates before GT can enter the process.
    np.savez_compressed(output / "inference_poses.npz", **keyframes)
    records = [{"source": factor.source, "target": factor.target,
                "measurement": factor.measurement.tolist(), "sigma": factor.sigma.tolist(),
                "kind": factor.kind} for factor in inferred["factors"]]
    _write_json(output / "inference_frozen.json", {"gt_topics_loaded": False, "gt_used_for_sync": False,
                "gt_used_for_association": False, "gt_used_for_measurement": False,
                "gt_used_for_optimization": False, "initial_frame_calibration_known": True,
                "selected_pair": inferred["selected_pair"], "factors": records,
                "selected_pairs": inferred.get("selected_pairs", []),
                "regions": inferred.get("regions", []), "condition_optimizers": optimizers,
                "source_sha256": source_hashes,
                "optimizer": inferred["optimizer"], "rows": inferred["rows"]})
    if multi:
        complete = complete_two_region_ablation(optimizers)
        _write_json(output / "ablation_status.json", {
            "complete": complete, "strictly_required": require_complete_ablation,
            "required_applied_counts": {"N-single-A": 1, "N-single-B": 1, "N-double": 2},
            "actual_applied_counts": {name: opt.get("applied_factor_count", 0)
                                      for name, opt in optimizers.items()},
            "gt_loaded_at_validation": False})
        if require_complete_ablation and not complete:
            raise RuntimeError("Two-region ablation incomplete: need successful A=1, B=1, A+B=2. "
                               "Inference diagnostics preserved; GT scoring and map reporting not started.")
    for condition, poses in keyframes.items():
        scan_poses = corrected_scan_poses(dataset, condition, poses if condition not in ("B0", "B1") else None)
        grid = build_occupancy_grid(dataset, scan_poses, config)
        save_occupancy_grid(grid, grid_spec_from_config(config), output / "maps" / condition)
        print(f"Saved inference map: {condition}", flush=True)

    if inference_only:
        print("Inference-only completed; no GT topics were loaded.", flush=True)
        return {"inference_only": True, "selected_pair": inferred["selected_pair"]}

    print("Inference frozen. Reading GT now, for scoring only.", flush=True)
    truth_records = load_bag(bag, config)
    evaluated, support = attach_evaluation_truth(dataset, truth_records, config)
    reference = build_occupancy_grid(evaluated, corrected_scan_poses(evaluated, "GT-reference"), config)
    spec = grid_spec_from_config(config)
    save_occupancy_grid(reference, spec, output / "maps/GT-reference")
    rows = inferred["rows"]
    names = list(evaluated.keyframes)
    for row, (source, target, result) in zip(rows, inferred["results"]):
        a = evaluated.keyframes[names[0]].gt_poses[row["source_keyframe"]]
        b = evaluated.keyframes[names[1]].gt_poses[row["target_keyframe"]]
        truth = between(a, b)
        initial_error, estimated_error = between(truth, result.initial_pose), between(truth, result.pose)
        row.update(condition=primary, initial_translation_error_m=float(np.linalg.norm(initial_error[:2])),
                   estimated_translation_error_m=float(np.linalg.norm(estimated_error[:2])),
                   initial_yaw_error_deg=float(abs(math.degrees(initial_error[2]))),
                   estimated_yaw_error_deg=float(abs(math.degrees(estimated_error[2]))))
        row["evaluation_pose_match_correct"] = bool(
            row["estimated_translation_error_m"] <= config["evaluation"]["correct_match_translation_m"] and
            row["estimated_yaw_error_deg"] <= config["evaluation"]["correct_match_yaw_deg"])
        prefix = f"pair_{row['pair']:03d}_source_{row['source_keyframe']}_target_{row['target_keyframe']}"
        np.savez_compressed(output / "submaps" / f"{prefix}.npz", source_points=source.points,
                            target_points=target.points, initial_pose=result.initial_pose,
                            estimated_pose=result.pose, ground_truth_pose=truth)
        if row["geometry_pass"] or row["pair"] < 4:
            _save_registration_plot(output / "debug_plots" / f"{prefix}.png", source.points, target.points,
                                    result.initial_pose, result.pose, f"{prefix}; used={row['accepted']}")
    _write_csv(output / "registrations.csv", rows)
    summaries = {}
    for condition in condition_poses:
        condition_rows = [{**row, "accepted": condition in row.get("accepted_conditions", [primary] if row["accepted"] else [])}
                          for row in rows]
        summaries[condition] = _registration_summary(condition_rows)
        summaries[condition]["consistent_pair_count"] = inferred["consistent_pair_count"]
    metrics = {"conditions": {}, "registration": summaries, "gt_evaluation_support": support}
    # The scoring view may drop GT-unsupported keyframes; inference stays frozen.
    scoring = copy.deepcopy(evaluated)
    for track in scoring.keyframes.values():
        valid = np.all(np.isfinite(track.gt_poses), axis=1)
        for field in ("node_indices", "gt_poses", "raw_poses", "known_poses", "progress", "times", "scan_indices"):
            setattr(track, field, getattr(track, field)[valid])
    for condition, poses in keyframes.items():
        _write_csv(output / "trajectories" / f"{condition}.csv", _trajectory_rows(scoring, condition, poses))
        grid = np.load(output / "maps" / f"{condition}.npy")
        metric = {"vehicles": {}, "map": map_metrics(grid, reference, spec),
                  "differential": differential_metrics(scoring, poses, config["evaluation"]["differential_sample_spacing_m"])}
        # Unequal route lengths mean the last common-distance sample is not
        # necessarily both vehicles' terminal pose. Report both explicitly.
        differential = metric["differential"]
        differential["last_common_progress_translation_m"] = differential["endpoint_translation_m"]
        differential["last_common_progress_yaw_deg"] = differential["endpoint_yaw_deg"]
        one, two = list(scoring.keyframes.values())
        terminal_error = between(between(one.gt_poses[-1], two.gt_poses[-1]),
                                 between(poses[one.node_indices[-1]], poses[two.node_indices[-1]]))
        differential["endpoint_translation_m"] = float(np.linalg.norm(terminal_error[:2]))
        differential["endpoint_yaw_deg"] = abs(math.degrees(float(terminal_error[2])))
        differential["endpoint_definition"] = "each_vehicle_last_gt_supported_keyframe"
        differential["sample_axis"] = "common_raw_slam_accumulated_distance"
        for name, track in scoring.keyframes.items():
            metric["vehicles"][name] = trajectory_metrics(poses[track.node_indices], track.gt_poses,
                                                          track.progress, config["evaluation"]["rpe_distance_m"])
        metrics["conditions"][condition] = metric
    selected = [row for row in rows if row["accepted"]]
    b0, new = (metrics["conditions"][name] for name in ("B0", primary))
    checks = {"inference_gt_free_with_known_initial_calibration": True,
              ("two_independent_regions_available" if multi else "one_factor_available"): len(selected) == (2 if multi else 1),
              "optimizer_succeeded": inferred["optimizer"]["success"],
              "accepted_pose_correct_on_evaluation": bool(selected) and all(r["evaluation_pose_match_correct"] for r in selected),
              "relative_endpoint_translation_improved": new["differential"]["endpoint_translation_m"] < b0["differential"]["endpoint_translation_m"],
              "relative_yaw_rmse_not_worse": new["differential"]["yaw_deg"]["rmse"] <= b0["differential"]["yaw_deg"]["rmse"],
              "map_chamfer_improved": new["map"]["occupied_chamfer_m"] is not None and b0["map"]["occupied_chamfer_m"] is not None and new["map"]["occupied_chamfer_m"] < b0["map"]["occupied_chamfer_m"]}
    metrics["phase2_gate"] = {"pass": all(checks.values()), "checks": checks}
    if multi:
        metrics["same_bag_ablation"] = {
            "all_requested_regions_available": len(selected) == 2,
            "optimizers": optimizers,
            "note": "A/B denote earlier/later validated regions, not GT or route-window labels. N-prior isolates initial-calibration PGO from inter-UAV correction.",
            "double_vs_single_endpoint_improved": {
                c: bool(len(selected) == 2 and optimizers[c]["success"] and optimizers[primary]["success"]
                        and new["differential"]["endpoint_translation_m"] < metrics["conditions"][c]["differential"]["endpoint_translation_m"])
                for c in ("N-single-A", "N-single-B")}}
    _write_json(output / "metrics.json", metrics)
    _write_csv(output / "metrics.csv", [dict(condition=c, **metrics["conditions"][c]["map"],
                     differential_endpoint_translation_m=metrics["conditions"][c]["differential"]["endpoint_translation_m"],
                     differential_endpoint_yaw_deg=metrics["conditions"][c]["differential"]["endpoint_yaw_deg"]) for c in keyframes])
    overlap = spatial_overlap(evaluated, config)
    _write_csv(output / "overlap_audit.csv", overlap)
    make_plots(output, metrics, evaluated, keyframes, config, rows, overlap)
    definitions = {"N-single": "One LiDAR-estimated constraint, selected by geometry and neighboring-pair consistency; all applied after flight"}
    if multi:
        definitions = {"N-prior": "Same graph and known-start priors, zero inter-UAV factors (calibration-only control)",
                       "N-single-A": "Only the earlier LiDAR-validated region; one representative factor",
                       "N-single-B": "Only the later independent LiDAR-validated region; one representative factor",
                       "N-double": "Both region representatives, at most two factors; check available/applied counts before interpreting"}
    manifest = {"schema_version": 3 if multi else 2, "generated_at": datetime.now(timezone.utc).isoformat(),
                "bag_path": str(Path(bag).resolve()), "config_path": str(Path(config_path).resolve()),
                "config_sha256": hashlib.sha256(config_text.encode()).hexdigest(), "conditions": list(keyframes),
                "primary_condition": primary, "requested_regions": 2 if multi else 1,
                "regions": inferred.get("regions", []),
                "validated_component_count": inferred.get("validated_component_count"),
                "source_sha256": source_hashes,
                "association_source": "lidar_descriptor_with_raw_slam_spatial_prior",
                "gt_usage": "evaluation_only", "initial_frame_calibration_known": True,
                "condition_definitions": definitions,
                "vehicles": _vehicle_audit(dataset), "optimizer": optimizers,
                "inference_elapsed_sec": inferred["elapsed_sec"], "total_elapsed_sec": time.monotonic()-started}
    _write_json(output / "manifest.json", manifest)
    (output / "report.md").write_text(
        "# Full-length offline LiDAR correction\n\n"
        "- GT is not loaded until inference poses, factors and maps have been saved.\n"
        "- Initial inter-drone frame calibration is known; unknown-start global localization is not claimed.\n"
        f"- Primary condition: {primary}; selected region representatives: {len(selected)}.\n"
        "- Adjacent pairs validate consistency; one representative per region enters the graph.\n"
        "- Single-A, single-B and double (when enabled) start from identical raw poses with identical graph weights.\n"
        "- N-prior (when enabled) isolates the effect of known-start priors without inter-UAV factors.\n"
        "- Candidate search is assisted by a bounded raw-SLAM spatial prior.\n"
        "- Graph/map generation can complete even when the scientific improvement gate fails.\n"
        f"- Scientific gate: {metrics['phase2_gate']['pass']}\n\n"
        + "\n".join(f"- {key}: {value}" for key, value in checks.items()) + "\n")
    print(json.dumps(metrics["phase2_gate"], indent=2), flush=True)
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--inference-only", action="store_true")
    parser.add_argument("--require-complete-ablation", action="store_true",
                        help="Stop before GT/maps unless A=1, B=1 and A+B=2 are all actually applied")
    args = parser.parse_args()
    run(args.bag, args.config, args.output, args.inference_only, args.require_complete_ablation)


if __name__ == "__main__":
    main()
