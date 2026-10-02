"""Command-line entry point for the phase-1 oracle correction experiment."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

from . import __version__
from .bag_reader import load_bag
from .dataset import build_dataset
from .graph_builder import (
    build_intra_factors,
    build_oracle_factors,
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
from .pose_graph import optimize_pose_graph


CONDITIONS = ("B0", "B1", "O-single", "O-periodic")


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
            allow_nan=False,
        )
        stream.write("\n")


def _write_csv(path: Path, rows):
    rows = list(rows)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _keyframe_rows(dataset):
    rows = []
    for name, track in dataset.keyframes.items():
        for local_index, node in enumerate(track.node_indices):
            row = {
                "vehicle": name,
                "keyframe": local_index,
                "node": int(node),
                "scan_index": int(track.scan_indices[local_index]),
                "timestamp": float(track.times[local_index]),
                "progress_m": float(track.progress[local_index]),
            }
            for label, poses in (
                ("raw", track.raw_poses),
                ("known", track.known_poses),
                ("gt", track.gt_poses),
            ):
                row[f"{label}_x"] = float(poses[local_index, 0])
                row[f"{label}_y"] = float(poses[local_index, 1])
                row[f"{label}_yaw"] = float(poses[local_index, 2])
            rows.append(row)
    return rows


def _condition_keyframes(dataset, condition, graph_poses=None):
    node_count = sum(len(track.node_indices) for track in dataset.keyframes.values())
    output = np.empty((node_count, 3), dtype=float)
    if graph_poses is not None:
        output[:] = graph_poses
        return output
    for track in dataset.keyframes.values():
        if condition == "B0":
            output[track.node_indices] = track.raw_poses
        elif condition == "B1":
            output[track.node_indices] = track.known_poses
        elif condition == "GT-reference":
            output[track.node_indices] = track.gt_poses
        else:
            raise ValueError(f"missing keyframe source for {condition}")
    return output


def _trajectory_rows(dataset, condition, keyframe_poses):
    rows = []
    for name, track in dataset.keyframes.items():
        poses = keyframe_poses[track.node_indices]
        for local_index, pose in enumerate(poses):
            rows.append(
                {
                    "condition": condition,
                    "vehicle": name,
                    "keyframe": local_index,
                    "node": int(track.node_indices[local_index]),
                    "timestamp": float(track.times[local_index]),
                    "progress_m": float(track.progress[local_index]),
                    "x": float(pose[0]),
                    "y": float(pose[1]),
                    "yaw": float(pose[2]),
                    "gt_x": float(track.gt_poses[local_index, 0]),
                    "gt_y": float(track.gt_poses[local_index, 1]),
                    "gt_yaw": float(track.gt_poses[local_index, 2]),
                }
            )
    return rows


def _factor_json(condition, factor):
    return {
        "condition": condition,
        "type": factor.kind,
        "source": int(factor.source),
        "target": int(factor.target),
        "measurement": factor.measurement.tolist(),
        "sigma": factor.sigma.tolist(),
    }


def _calculate_gate(metrics, config, optimizers, vehicle_audit):
    thresholds = ((config.get("evaluation") or {}).get("phase1_gate") or {})
    b0_diff = metrics["conditions"]["B0"]["differential"][
        "endpoint_translation_m"
    ]
    periodic_diff = metrics["conditions"]["O-periodic"]["differential"][
        "endpoint_translation_m"
    ]
    b0_chamfer = metrics["conditions"]["B0"]["map"]["occupied_chamfer_m"]
    periodic_chamfer = metrics["conditions"]["O-periodic"]["map"][
        "occupied_chamfer_m"
    ]
    b0_f1 = metrics["conditions"]["B0"]["map"]["occupied_f1"]
    periodic_f1 = metrics["conditions"]["O-periodic"]["map"]["occupied_f1"]
    single_diff = metrics["conditions"]["O-single"]["differential"][
        "endpoint_translation_m"
    ]

    differential_improvement = (
        (b0_diff - periodic_diff) / b0_diff if b0_diff > 1.0e-12 else 0.0
    )
    if b0_chamfer is None or periodic_chamfer is None or b0_chamfer <= 1.0e-12:
        chamfer_improvement = None
    else:
        chamfer_improvement = (b0_chamfer - periodic_chamfer) / b0_chamfer
    f1_drop = b0_f1 - periodic_f1
    checks = {
        "synchronized_drop_within_limit": all(
            audit["drop_ratio"]
            <= float(thresholds.get("max_sync_drop_ratio", 0.01))
            for audit in vehicle_audit.values()
        ),
        "optimizers_converged": all(result.success for result in optimizers.values()),
        "differential_endpoint_improved": differential_improvement
        >= float(thresholds.get("differential_endpoint_improvement_min", 0.30)),
        "chamfer_improved": chamfer_improvement is not None
        and chamfer_improvement
        >= float(thresholds.get("chamfer_improvement_min", 0.20)),
        "occupied_f1_preserved": f1_drop
        <= float(thresholds.get("occupied_f1_allowed_drop", 0.02)),
        "periodic_better_than_single": periodic_diff < single_diff,
    }
    return {
        "pass": bool(all(checks.values())),
        "checks": checks,
        "differential_endpoint_improvement": float(differential_improvement),
        "chamfer_improvement": chamfer_improvement,
        "occupied_f1_drop": float(f1_drop),
    }


def _vehicle_audit(dataset):
    audit = {}
    for name, vehicle in dataset.vehicles.items():
        source_count = len(vehicle.scans) + vehicle.dropped_scan_count
        audit[name] = {
            "source_scans": source_count,
            "common_support_scans": vehicle.eligible_scan_count,
            "valid_scans": len(vehicle.scans),
            "dropped_scans": vehicle.dropped_scan_count,
            "startup_trimmed_scans": vehicle.trimmed_scan_count,
            "interpolation_dropped_scans": vehicle.interpolation_drop_count,
            "drop_ratio": vehicle.interpolation_drop_count
            / max(1, vehicle.eligible_scan_count),
            "keyframes": len(dataset.keyframes[name].node_indices),
        }
    return audit


def _report_markdown(manifest, metrics, gate, oracle_pairs):
    lines = [
        "# Phase-1 Oracle Multi-UAV Drift Correction Report",
        "",
        f"- Bag: `{manifest['bag_path']}`",
        f"- Generated: `{manifest['generated_at']}`",
        f"- Scientific gate: **{'PASS' if gate['pass'] else 'FAIL'}**",
        "",
        "## Data audit",
        "",
        "| UAV | source scans | common support | valid | startup trimmed | "
        "sync dropped | sync drop ratio | keyframes |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, audit in manifest["vehicles"].items():
        lines.append(
            f"| {name} | {audit['source_scans']} | {audit['common_support_scans']} | "
            f"{audit['valid_scans']} | {audit['startup_trimmed_scans']} | "
            f"{audit['interpolation_dropped_scans']} | {audit['drop_ratio']:.4f} | "
            f"{audit['keyframes']} |"
        )
    lines.extend(
        [
            "",
            "## Main comparison",
            "",
            "| Condition | Differential endpoint [m] | Map Chamfer [m] | Occupied F1 |",
            "|---|---:|---:|---:|",
        ]
    )
    for condition in CONDITIONS:
        values = metrics["conditions"][condition]
        chamfer = values["map"]["occupied_chamfer_m"]
        chamfer_text = "n/a" if chamfer is None else f"{chamfer:.4f}"
        lines.append(
            f"| {condition} | {values['differential']['endpoint_translation_m']:.4f} | "
            f"{chamfer_text} | {values['map']['occupied_f1']:.4f} |"
        )
    lines.extend(
        [
            "",
            "## Oracle factors",
            "",
            f"- O-single: {len(oracle_pairs['O-single'])}",
            f"- O-periodic: {len(oracle_pairs['O-periodic'])}",
            "",
            "## Gate checks",
            "",
        ]
    )
    for name, passed in gate["checks"].items():
        lines.append(f"- {'PASS' if passed else 'FAIL'}: `{name}`")
    lines.extend(
        [
            "",
            "> GT is used only to construct and evaluate oracle factors. This report does not "
            "claim an automatic inter-UAV place-recognition result.",
            "",
        ]
    )
    return "\n".join(lines)


def run_experiment(bag_path: str, config_path: str, output_path: str, overwrite=False):
    config_file = Path(config_path).expanduser().resolve()
    with config_file.open("r", encoding="utf-8") as stream:
        config_text = stream.read()
    config = yaml.safe_load(config_text) or {}
    output = Path(output_path).expanduser().resolve()
    if output.exists() and any(output.iterdir()) and not overwrite:
        raise FileExistsError(
            f"output directory is not empty: {output}; pass --overwrite to replace named outputs"
        )
    output.mkdir(parents=True, exist_ok=True)
    trajectory_dir = output / "trajectories"
    map_dir = output / "maps"
    trajectory_dir.mkdir(exist_ok=True)
    map_dir.mkdir(exist_ok=True)

    bag_vehicles = load_bag(bag_path, config)
    dataset = build_dataset(bag_vehicles, config)
    initial = initial_pose_array(dataset)
    intra = build_intra_factors(dataset, config)
    priors = build_priors(dataset, config)
    graph_config = config.get("pose_graph") or {}

    keyframes = {
        "B0": _condition_keyframes(dataset, "B0"),
        "B1": _condition_keyframes(dataset, "B1"),
    }
    oracle_pairs = {}
    oracle_factors = {}
    optimizers = {}
    for condition in ("O-single", "O-periodic"):
        inter, pairs = build_oracle_factors(dataset, config, condition)
        oracle_pairs[condition] = pairs
        oracle_factors[condition] = inter
        result = optimize_pose_graph(
            initial,
            [*intra, *inter],
            priors,
            loss=str(graph_config.get("robust_loss", "huber")),
            robust_scale=float(graph_config.get("robust_scale", 1.5)),
            max_function_evaluations=int(
                graph_config.get("max_function_evaluations", 300)
            ),
        )
        optimizers[condition] = result
        keyframes[condition] = result.poses

    _write_csv(output / "keyframes.csv", _keyframe_rows(dataset))
    factor_path = output / "factors.jsonl"
    with factor_path.open("w", encoding="utf-8") as stream:
        for condition in ("O-single", "O-periodic"):
            for factor in intra:
                stream.write(json.dumps(_factor_json(condition, factor)) + "\n")
            for factor in oracle_factors[condition]:
                stream.write(json.dumps(_factor_json(condition, factor)) + "\n")
        for prior in priors:
            stream.write(
                json.dumps(
                    {
                        "condition": "all",
                        "type": "spawn_prior",
                        "node": int(prior.node),
                        "measurement": prior.measurement.tolist(),
                        "sigma": prior.sigma.tolist(),
                    }
                )
                + "\n"
            )

    evaluation = config.get("evaluation") or {}
    rpe_distance = float(evaluation.get("rpe_distance_m", 10.0))
    differential_spacing = float(
        evaluation.get("differential_sample_spacing_m", 5.0)
    )
    metrics = {"conditions": {}}
    spec = grid_spec_from_config(config)
    gt_scan_poses = corrected_scan_poses(dataset, "GT-reference")
    reference_grid = build_occupancy_grid(dataset, gt_scan_poses, config)
    save_occupancy_grid(reference_grid, spec, map_dir / "GT-reference")

    flat_metric_rows = []
    for condition in CONDITIONS:
        condition_keyframes = keyframes[condition]
        _write_csv(
            trajectory_dir / f"{condition}.csv",
            _trajectory_rows(dataset, condition, condition_keyframes),
        )
        scan_poses = corrected_scan_poses(
            dataset,
            condition,
            condition_keyframes if condition.startswith("O-") else None,
        )
        grid = build_occupancy_grid(dataset, scan_poses, config)
        save_occupancy_grid(grid, spec, map_dir / condition)
        condition_metrics = {
            "vehicles": {},
            "differential": differential_metrics(
                dataset, condition_keyframes, differential_spacing
            ),
            "map": map_metrics(grid, reference_grid, spec),
        }
        for name, track in dataset.keyframes.items():
            condition_metrics["vehicles"][name] = trajectory_metrics(
                condition_keyframes[track.node_indices],
                track.gt_poses,
                track.progress,
                rpe_distance,
            )
        metrics["conditions"][condition] = condition_metrics
        flat_metric_rows.append(
            {
                "condition": condition,
                "differential_endpoint_translation_m": condition_metrics[
                    "differential"
                ]["endpoint_translation_m"],
                "differential_endpoint_yaw_deg": condition_metrics["differential"][
                    "endpoint_yaw_deg"
                ],
                **condition_metrics["map"],
            }
        )

    overlap_rows = overlap_audit(dataset, config)
    _write_csv(output / "overlap_audit.csv", overlap_rows)
    vehicle_audit = _vehicle_audit(dataset)
    gate = _calculate_gate(metrics, config, optimizers, vehicle_audit)
    metrics["phase1_gate"] = gate
    _write_json(output / "metrics.json", metrics)
    _write_csv(output / "metrics.csv", flat_metric_rows)

    manifest = {
        "schema_version": 1,
        "tool_version": __version__,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "bag_path": str(Path(bag_path).expanduser().resolve()),
        "config_path": str(config_file),
        "config_sha256": hashlib.sha256(config_text.encode("utf-8")).hexdigest(),
        "conditions": list(CONDITIONS),
        "vehicles": vehicle_audit,
        "optimizer": {
            condition: {
                "success": result.success,
                "cost": result.cost,
                "optimality": result.optimality,
                "evaluations": result.evaluations,
                "message": result.message,
                "oracle_factor_count": len(oracle_pairs[condition]),
            }
            for condition, result in optimizers.items()
        },
    }
    _write_json(output / "manifest.json", manifest)
    (output / "report.md").write_text(
        _report_markdown(manifest, metrics, gate, oracle_pairs), encoding="utf-8"
    )
    return gate


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Evaluate noisy GT-relative inter-UAV factors on a ROS 2 bag."
    )
    parser.add_argument("--bag", required=True, help="ROS 2 bag directory")
    parser.add_argument("--config", required=True, help="oracle phase-1 YAML config")
    parser.add_argument("--output", required=True, help="artifact output directory")
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace named outputs in a non-empty output directory",
    )
    return parser.parse_args(argv)


def main(argv=None):
    arguments = _parse_args(argv)
    try:
        gate = run_experiment(
            arguments.bag,
            arguments.config,
            arguments.output,
            overwrite=arguments.overwrite,
        )
    except Exception as exc:
        print(f"oracle_drift_eval failed: {exc}", file=sys.stderr)
        return 2
    print(
        f"phase-1 oracle evaluation complete: {'PASS' if gate['pass'] else 'FAIL'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
