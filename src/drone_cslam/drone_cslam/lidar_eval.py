"""Phase-2 controlled LiDAR relative-pose experiment.

Ground truth selects comparable keyframe pairs and scores the result.  The
submap builder, initial guess, registration objective, and pose-graph factor
measurement do not receive the GT relative transform.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from . import __version__
from .bag_reader import load_bag
from .dataset import build_dataset
from .graph_builder import (
    build_intra_factors,
    build_priors,
    initial_pose_array,
)
from .mapping import (
    build_occupancy_grid,
    corrected_scan_poses,
    grid_spec_from_config,
    save_occupancy_grid,
)
from .metrics import (
    differential_metrics,
    map_metrics,
    overlap_audit,
    trajectory_metrics,
)
from .oracle_eval import (
    _condition_keyframes,
    _trajectory_rows,
    _vehicle_audit,
)
from .pose_graph import PoseFactor, optimize_pose_graph
from .registration import register_submaps
from .se2 import between, transform_points, wrap_angle
from .submap import build_local_submap


CONDITIONS = ("B0", "B1", "R-single", "R-periodic")


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


def _write_json(path: Path, value):
    with path.open("w", encoding="utf-8") as stream:
        json.dump(
            value,
            stream,
            indent=2,
            sort_keys=True,
            default=_json_default,
        )
        stream.write("\n")


def _write_csv(path: Path, rows):
    rows = list(rows)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _pair_indices(dataset, config, condition):
    names = list(dataset.keyframes)
    if len(names) != 2:
        raise ValueError(
            "phase-2 controlled registration requires exactly two UAVs"
        )
    first = dataset.keyframes[names[0]]
    second = dataset.keyframes[names[1]]
    maximum = min(float(first.progress[-1]), float(second.progress[-1]))
    spacing = float(
        (config.get("pose_graph") or {}).get("factor_spacing_m", 10.0)
    )
    if condition == "R-single":
        targets = [0.5 * maximum]
    elif condition == "R-periodic":
        targets = np.arange(spacing, maximum + 1.0e-9, spacing).tolist()
        if not targets or maximum - targets[-1] >= 0.5 * spacing:
            targets.append(maximum)
    else:
        raise ValueError(f"unknown registration condition: {condition}")
    output = []
    seen = set()
    for target in targets:
        first_index = int(np.argmin(np.abs(first.progress - target)))
        second_index = int(np.argmin(np.abs(second.progress - target)))
        key = (first_index, second_index)
        if key in seen:
            continue
        seen.add(key)
        output.append((float(target), first_index, second_index))
    return names, output


def _save_registration_plot(path, source, target, initial, estimate, title):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    initial_points = transform_points(initial, target)
    estimated_points = transform_points(estimate, target)
    figure, axis = plt.subplots(figsize=(8.0, 7.0))
    axis.scatter(
        source[:, 0],
        source[:, 1],
        s=2,
        c="#2563eb",
        label="UAV1 submap",
    )
    axis.scatter(
        initial_points[:, 0], initial_points[:, 1], s=2, c="#f59e0b",
        alpha=0.45, label="UAV2 with B0 initial",
    )
    axis.scatter(
        estimated_points[:, 0], estimated_points[:, 1], s=2, c="#dc2626",
        alpha=0.55, label="UAV2 after LiDAR registration",
    )
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.2)
    axis.set_xlabel("source-keyframe x [m]")
    axis.set_ylabel("source-keyframe y [m]")
    axis.set_title(title)
    axis.legend(markerscale=4, loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def _statistics(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"mean": None, "median": None, "maximum": None}
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "maximum": float(np.max(values)),
    }


def _registration_summary(rows):
    selected = len(rows)
    accepted_rows = [row for row in rows if row["accepted"]]
    return {
        "selected_pair_count": selected,
        "accepted_factor_count": len(accepted_rows),
        "acceptance_ratio": float(len(accepted_rows) / max(1, selected)),
        "initial_translation_error_m": _statistics(
            [row["initial_translation_error_m"] for row in rows]
        ),
        "estimated_translation_error_m": _statistics(
            [row["estimated_translation_error_m"] for row in rows]
        ),
        "initial_yaw_error_deg": _statistics(
            [row["initial_yaw_error_deg"] for row in rows]
        ),
        "estimated_yaw_error_deg": _statistics(
            [row["estimated_yaw_error_deg"] for row in rows]
        ),
        "accepted_translation_error_m": _statistics(
            [row["estimated_translation_error_m"] for row in accepted_rows]
        ),
        "accepted_yaw_error_deg": _statistics(
            [row["estimated_yaw_error_deg"] for row in accepted_rows]
        ),
    }


def _report(manifest, metrics, registration_summaries):
    lines = [
        "# Phase-2 Controlled LiDAR Relative-Pose Report",
        "",
        f"- Bag: `{manifest['bag_path']}`",
        "- Pair association: **GT progress (controlled experiment only)**",
        "- Factor measurement: **LiDAR submap registration; GT not provided**",
        "- Scientific gate: **"
        f"{'PASS' if metrics['phase2_gate']['pass'] else 'FAIL'}**",
        "",
        "## Registration",
        "",
        "| Condition | selected | accepted | initial median [m] | "
        "estimated median [m] | estimated median yaw [deg] |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for condition in ("R-single", "R-periodic"):
        summary = registration_summaries[condition]
        lines.append(
            f"| {condition} | {summary['selected_pair_count']} | "
            f"{summary['accepted_factor_count']} | "
            f"{summary['initial_translation_error_m']['median']:.4f} | "
            f"{summary['estimated_translation_error_m']['median']:.4f} | "
            f"{summary['estimated_yaw_error_deg']['median']:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Corrected trajectory/map comparison",
            "",
            "| Condition | differential endpoint [m] | map Chamfer [m] | "
            "occupied F1 |",
            "|---|---:|---:|---:|",
        ]
    )
    for condition in CONDITIONS:
        values = metrics["conditions"][condition]
        chamfer = values["map"]["occupied_chamfer_m"]
        chamfer_text = "n/a" if chamfer is None else f"{chamfer:.4f}"
        lines.append(
            f"| {condition} | "
            f"{values['differential']['endpoint_translation_m']:.4f} | "
            f"{chamfer_text} | {values['map']['occupied_f1']:.4f} |"
        )
    lines.extend(["", "## Gate", ""])
    for name, passed in metrics["phase2_gate"]["checks"].items():
        lines.append(f"- {'PASS' if passed else 'FAIL'}: `{name}`")
    lines.extend(
        [
            "",
            "> This is not global place recognition. GT still chooses the "
            "keyframe pair. The controlled result tests only whether LiDAR "
            "can replace the oracle transform.",
            "",
        ]
    )
    return "\n".join(lines)


def run_experiment(
    bag_path: str,
    config_path: str,
    output_path: str,
    overwrite=False,
):
    config_file = Path(config_path).expanduser().resolve()
    config_text = config_file.read_text(encoding="utf-8")
    config = yaml.safe_load(config_text) or {}
    output = Path(output_path).expanduser().resolve()
    if output.exists() and any(output.iterdir()) and not overwrite:
        raise FileExistsError(
            f"output directory is not empty: {output}; pass --overwrite "
            "to replace outputs"
        )
    output.mkdir(parents=True, exist_ok=True)
    submap_dir = output / "submaps"
    plot_dir = output / "debug_plots"
    trajectory_dir = output / "trajectories"
    map_dir = output / "maps"
    for directory in (submap_dir, plot_dir, trajectory_dir, map_dir):
        directory.mkdir(exist_ok=True)

    dataset = build_dataset(load_bag(bag_path, config), config)
    initial_graph = initial_pose_array(dataset)
    intra = build_intra_factors(dataset, config)
    priors = build_priors(dataset, config)
    graph_config = config.get("pose_graph") or {}
    sigma = np.array(
        [
            float(graph_config.get("registration_translation_sigma_m", 0.15)),
            float(graph_config.get("registration_translation_sigma_m", 0.15)),
            math.radians(
                float(graph_config.get("registration_yaw_sigma_deg", 1.5))
            ),
        ]
    )

    registration_rows = []
    candidate_rows = []
    factor_records = []
    factors_by_condition = {}
    optimized_by_condition = {}
    optimizer_records = {}
    summaries = {}
    for condition in ("R-single", "R-periodic"):
        names, pairs = _pair_indices(dataset, config, condition)
        first = dataset.keyframes[names[0]]
        second = dataset.keyframes[names[1]]
        factors = []
        condition_rows = []
        for pair_index, pair in enumerate(pairs):
            target_progress, first_index, second_index = pair
            source = build_local_submap(dataset, names[0], first_index, config)
            target = build_local_submap(
                dataset, names[1], second_index, config
            )
            initial_relative = between(
                first.raw_poses[first_index], second.raw_poses[second_index]
            )
            truth_relative = between(
                first.gt_poses[first_index], second.gt_poses[second_index]
            )
            result = register_submaps(
                source.points, target.points, initial_relative, config
            )
            initial_error = between(truth_relative, initial_relative)
            estimated_error = between(truth_relative, result.pose)
            prefix = f"{condition}_{pair_index:02d}_{target_progress:06.1f}m"
            np.savez_compressed(
                submap_dir / f"{prefix}.npz",
                source_points=source.points,
                target_points=target.points,
                initial_pose=initial_relative,
                estimated_pose=result.pose,
                ground_truth_pose=truth_relative,
            )
            _save_registration_plot(
                plot_dir / f"{prefix}.png",
                source.points,
                target.points,
                initial_relative,
                result.pose,
                f"{condition}: GT-associated pair at {target_progress:.1f} m",
            )
            row = {
                "condition": condition,
                "pair": pair_index,
                "target_progress_m": target_progress,
                "association_source": "ground_truth_progress",
                "source_vehicle": names[0],
                "target_vehicle": names[1],
                "source_keyframe": first_index,
                "target_keyframe": second_index,
                "source_node": int(first.node_indices[first_index]),
                "target_node": int(second.node_indices[second_index]),
                "source_submap_points": len(source.points),
                "target_submap_points": len(target.points),
                "initial_x": float(initial_relative[0]),
                "initial_y": float(initial_relative[1]),
                "initial_yaw_deg": math.degrees(float(initial_relative[2])),
                "estimated_x": float(result.pose[0]),
                "estimated_y": float(result.pose[1]),
                "estimated_yaw_deg": math.degrees(float(result.pose[2])),
                "gt_x": float(truth_relative[0]),
                "gt_y": float(truth_relative[1]),
                "gt_yaw_deg": math.degrees(float(truth_relative[2])),
                "initial_translation_error_m": float(
                    np.linalg.norm(initial_error[:2])
                ),
                "initial_yaw_error_deg": math.degrees(
                    abs(float(wrap_angle(initial_error[2])))
                ),
                "estimated_translation_error_m": float(
                    np.linalg.norm(estimated_error[:2])
                ),
                "estimated_yaw_error_deg": math.degrees(
                    abs(float(wrap_angle(estimated_error[2])))
                ),
                "cost": result.cost,
                "overlap_ratio": result.overlap_ratio,
                "inlier_rmse_m": result.inlier_rmse_m,
                "score_margin_ratio": result.score_margin_ratio,
                "correction_translation_m": result.correction_translation_m,
                "correction_yaw_deg": result.correction_yaw_deg,
                "accepted": result.accepted,
                "rejection_reasons": ";".join(result.rejection_reasons),
            }
            registration_rows.append(row)
            condition_rows.append(row)
            for rank, candidate in enumerate(result.candidates):
                candidate_rows.append(
                    {
                        "condition": condition,
                        "pair": pair_index,
                        "rank": rank + 1,
                        "x": float(candidate.pose[0]),
                        "y": float(candidate.pose[1]),
                        "yaw_deg": math.degrees(float(candidate.pose[2])),
                        "cost": candidate.cost,
                        "overlap_ratio": candidate.overlap_ratio,
                        "inlier_rmse_m": candidate.inlier_rmse_m,
                    }
                )
            if result.accepted:
                factor = PoseFactor(
                    source=int(first.node_indices[first_index]),
                    target=int(second.node_indices[second_index]),
                    measurement=result.pose.copy(),
                    sigma=sigma.copy(),
                    kind="lidar_inter_uav_gt_associated",
                )
                factors.append(factor)
                factor_records.append(
                    {
                        "condition": condition,
                        "source": factor.source,
                        "target": factor.target,
                        "measurement": factor.measurement.tolist(),
                        "sigma": factor.sigma.tolist(),
                        "association_source": "ground_truth_progress",
                        "measurement_source": "lidar_submap_registration",
                    }
                )
        factors_by_condition[condition] = factors
        summaries[condition] = _registration_summary(condition_rows)
        if factors:
            optimized = optimize_pose_graph(
                initial_graph,
                [*intra, *factors],
                priors,
                loss=str(graph_config.get("robust_loss", "huber")),
                robust_scale=float(graph_config.get("robust_scale", 1.5)),
                max_function_evaluations=int(
                    graph_config.get("max_function_evaluations", 300)
                ),
            )
            optimized_by_condition[condition] = optimized.poses
            optimizer_records[condition] = {
                "success": optimized.success,
                "cost": optimized.cost,
                "evaluations": optimized.evaluations,
                "message": optimized.message,
                "accepted_factor_count": len(factors),
            }
        else:
            optimized_by_condition[condition] = initial_graph.copy()
            optimizer_records[condition] = {
                "success": True,
                "cost": None,
                "evaluations": 0,
                "message": "no accepted factors; R condition equals B0",
                "accepted_factor_count": 0,
            }

    _write_csv(output / "registrations.csv", registration_rows)
    _write_csv(output / "candidates.csv", candidate_rows)
    factor_path = output / "estimated_factors.jsonl"
    with factor_path.open("w", encoding="utf-8") as stream:
        for record in factor_records:
            stream.write(json.dumps(record) + "\n")
    _write_json(output / "relative_pose_metrics.json", summaries)

    keyframes = {
        "B0": _condition_keyframes(dataset, "B0"),
        "B1": _condition_keyframes(dataset, "B1"),
        **optimized_by_condition,
    }
    evaluation = config.get("evaluation") or {}
    spec = grid_spec_from_config(config)
    reference_grid = build_occupancy_grid(
        dataset, corrected_scan_poses(dataset, "GT-reference"), config
    )
    save_occupancy_grid(reference_grid, spec, map_dir / "GT-reference")
    metrics = {"conditions": {}, "registration": summaries}
    metric_rows = []
    for condition in CONDITIONS:
        poses = keyframes[condition]
        _write_csv(
            trajectory_dir / f"{condition}.csv",
            _trajectory_rows(dataset, condition, poses),
        )
        scan_poses = corrected_scan_poses(
            dataset,
            condition,
            poses if condition.startswith("R-") else None,
        )
        grid = build_occupancy_grid(dataset, scan_poses, config)
        save_occupancy_grid(grid, spec, map_dir / condition)
        condition_metrics = {
            "vehicles": {},
            "differential": differential_metrics(
                dataset,
                poses,
                float(evaluation.get("differential_sample_spacing_m", 5.0)),
            ),
            "map": map_metrics(grid, reference_grid, spec),
        }
        for name, track in dataset.keyframes.items():
            condition_metrics["vehicles"][name] = trajectory_metrics(
                poses[track.node_indices],
                track.gt_poses,
                track.progress,
                float(evaluation.get("rpe_distance_m", 10.0)),
            )
        metrics["conditions"][condition] = condition_metrics
        metric_rows.append(
            {
                "condition": condition,
                "differential_endpoint_translation_m": condition_metrics[
                    "differential"
                ]["endpoint_translation_m"],
                "differential_endpoint_yaw_deg": condition_metrics[
                    "differential"
                ]["endpoint_yaw_deg"],
                **condition_metrics["map"],
            }
        )
    periodic = summaries["R-periodic"]
    initial_median = periodic["initial_translation_error_m"]["median"]
    estimated_median = periodic["estimated_translation_error_m"]["median"]
    checks = {
        "gt_not_used_for_factor_measurement": True,
        "periodic_has_accepted_factor": periodic["accepted_factor_count"] > 0,
        "periodic_median_relative_translation_improved": (
            estimated_median is not None
            and initial_median is not None
            and estimated_median < initial_median
        ),
        "periodic_optimizer_succeeded": optimizer_records[
            "R-periodic"
        ]["success"],
    }
    metrics["phase2_gate"] = {
        "pass": bool(all(checks.values())),
        "checks": checks,
    }
    _write_json(output / "metrics.json", metrics)
    _write_csv(output / "metrics.csv", metric_rows)
    _write_csv(output / "overlap_audit.csv", overlap_audit(dataset, config))

    manifest = {
        "schema_version": 1,
        "tool_version": __version__,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "bag_path": str(Path(bag_path).expanduser().resolve()),
        "config_path": str(config_file),
        "config_sha256": hashlib.sha256(
            config_text.encode("utf-8")
        ).hexdigest(),
        "conditions": list(CONDITIONS),
        "association_source": "ground_truth_progress",
        "factor_measurement_source": "lidar_submap_registration",
        "ground_truth_transform_available_to_estimator": False,
        "vehicles": _vehicle_audit(dataset),
        "optimizer": optimizer_records,
    }
    _write_json(output / "manifest.json", manifest)
    (output / "report.md").write_text(
        _report(manifest, metrics, summaries), encoding="utf-8"
    )
    return metrics["phase2_gate"]


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "GT-associated, LiDAR-measured inter-UAV relative-pose "
            "experiment."
        )
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    arguments = _parse_args(argv)
    try:
        gate = run_experiment(
            arguments.bag,
            arguments.config,
            arguments.output,
            arguments.overwrite,
        )
    except Exception as exc:
        print(f"lidar_relative_eval failed: {exc}", file=sys.stderr)
        return 2
    status = "PASS" if gate["pass"] else "FAIL"
    print(f"phase-2 LiDAR evaluation complete: {status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
