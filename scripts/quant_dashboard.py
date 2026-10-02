#!/usr/bin/env python3
"""Experiment browser with editable labels; measured artifacts stay read-only."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
from dashboard_catalog import (
    CatalogError, CatalogConflict, GROUPS, active_ids, catalog_exists,
    discover_experiments, display_label, entry_for, load_catalog, save_entries,
)


def run_label(repo_root: Path, run_id: str) -> str:
    source, _, vehicle = str(run_id).partition("/")
    label = display_label(load_catalog(repo_root), source)
    return f"{label} · {vehicle}" if vehicle else label


def dashboard_filter_enabled(repo_root: Path) -> bool:
    return catalog_exists(repo_root) or (repo_root / "experiments/dashboard_active_runs.txt").exists()


CORE_METRICS = [
    "success",
    "mission_time_s",
    "final_goal_distance_m",
    "actual_path_length_m",
    "min_obstacle_distance_m",
    "compute_latency_ms_p95",
]

SUPPLEMENTARY_METRICS = [
    "runtime_s",
    "outbound_time_s",
    "return_time_s",
    "total_path_length_m",
    "outbound_path_length_m",
    "return_path_length_m",
    "straight_line_distance_m",
    "path_efficiency",
    "safety_intervention_count",
    "safety_event_count",
    "control_effort",
    "command_smoothness",
    "planning_time_ms_p50",
    "planning_time_ms_p95",
    "replan_count",
    "planner_cmd_count",
    "safe_cmd_count",
    "pose_count",
    "pose_period_p99_s",
    "map_coverage",
]

KPI_GUIDE = [
    {
        "kpi": "success",
        "direction": "higher is better",
        "paper_use": "Goal reach success rate by planner condition.",
        "target_note": "Primary KPI. Failed runs remain in the dataset.",
    },
    {
        "kpi": "mission_time_s",
        "direction": "lower is better",
        "paper_use": "How long the run took under the same map, start, and goal.",
        "target_note": "For success runs this is time-to-goal; for failed runs it usually reflects timeout/stop time.",
    },
    {
        "kpi": "final_goal_distance_m",
        "direction": "lower is better",
        "paper_use": "Remaining distance to the goal at the end of the run.",
        "target_note": "Keeps failed/partial runs interpretable without relying only on pass/fail.",
    },
    {
        "kpi": "actual_path_length_m",
        "direction": "lower is better",
        "paper_use": "Measured flown trajectory length.",
        "target_note": "Use mainly for runs that made meaningful progress or reached the goal.",
    },
    {
        "kpi": "min_obstacle_distance_m",
        "direction": "higher is better",
        "paper_use": "Closest observed clearance to obstacles.",
        "target_note": "Simple safety margin. If LiDAR noise dominates, report it as a limitation.",
    },
    {
        "kpi": "compute_latency_ms_p95",
        "direction": "lower is better",
        "paper_use": "95th-percentile planner computation time.",
        "target_note": "Shows whether the method is real-time capable. Empty until planners log latency.",
    },
]

SUPPLEMENTARY_KPI_GUIDE = [
    {
        "kpi": "path_efficiency",
        "direction": "higher is better",
        "paper_use": "straight_line_distance_m / actual_path_length_m.",
        "target_note": "Useful after checking success/final_goal_distance first.",
    },
    {
        "kpi": "safety_intervention_count",
        "direction": "lower is better",
        "paper_use": "How often the safety layer blocked or slowed commands.",
        "target_note": "Debug signal for planner/safety disagreement.",
    },
    {
        "kpi": "control_effort",
        "direction": "lower is better",
        "paper_use": "Integrated command energy proxy.",
        "target_note": "Secondary smoothness/stability indicator.",
    },
    {
        "kpi": "command_smoothness",
        "direction": "lower is better",
        "paper_use": "Average command change between consecutive safe commands.",
        "target_note": "Secondary command stability indicator.",
    },
    {
        "kpi": "planning_time_ms_p50 / p95",
        "direction": "lower is better",
        "paper_use": "Planner compute latency distribution.",
        "target_note": "Only populated when planners publish/log latency.",
    },
    {
        "kpi": "pose_count / pose_period_p99_s",
        "direction": "context dependent",
        "paper_use": "Trajectory sampling health.",
        "target_note": "Useful for validating rosbag data quality.",
    },
]

ORACLE_CONDITIONS = ["B0", "B1", "O-single", "O-periodic"]
ORACLE_COLORS = {
    "GT": "#111827",
    "B0": "#dc2626",
    "B1": "#2563eb",
    "O-single": "#f59e0b",
    "O-periodic": "#16a34a",
}
LIDAR_CONDITIONS = ["B0", "B1", "R-single", "R-periodic"]
LIDAR_COLORS = {
    "GT": "#111827",
    "B0": "#dc2626",
    "B1": "#2563eb",
    "R-single": "#f59e0b",
    "R-periodic": "#16a34a",
    "N-single": "#7c3aed",
    "N-prior": "#64748b",
    "N-single-A": "#9333ea",
    "N-single-B": "#0891b2",
    "N-double": "#2563eb",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default=".", help="Repository root directory.")
    parser.add_argument(
        "--check-data",
        action="store_true",
        help="Validate and summarize available quantification data without importing Streamlit.",
    )
    return parser.parse_args()


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="") as handle:
        return list(csv.DictReader(handle))


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def remove_prefix(text: str, prefix: str) -> str:
    """Python 3.8-compatible str.removeprefix."""
    return text[len(prefix) :] if text.startswith(prefix) else text


def remove_suffix(text: str, suffix: str) -> str:
    """Python 3.8-compatible str.removesuffix."""
    return text[: -len(suffix)] if suffix and text.endswith(suffix) else text


def load_run_reports(repo_root: Path) -> list[dict[str, Any]]:
    """Load optional human-readable implementation reports for dashboard runs."""
    report_root = repo_root / "experiments" / "run_reports"
    reports = []
    if not report_root.exists():
        return reports
    for path in sorted(report_root.glob("*.json")):
        report = read_json(path)
        if not report.get("source_run_id"):
            continue
        report["report_path"] = str(path)
        reports.append(report)
    return reports


def find_map_debug_runs(repo_root: Path) -> list[Path]:
    """Return run roots that contain at least one recorded occupancy grid."""
    artifacts_root = repo_root / "artifacts"
    if not artifacts_root.exists():
        return []
    runs = {
        path.parents[2]
        for path in artifacts_root.glob("*/*/maps/*_meta.json")
        if path.with_name(path.name.replace("_meta.json", "_grid.npy")).exists()
    }
    if dashboard_filter_enabled(repo_root):
        active_runs = dashboard_active_run_names(repo_root)
        runs = {run for run in runs if run.name in active_runs}
    return sorted(
        runs,
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )


def discover_map_snapshots(run_root: Path) -> list[dict[str, Any]]:
    """Discover latest and periodic map bundles for one multi-vehicle run."""
    snapshots: list[dict[str, Any]] = []
    for maps_dir in sorted(run_root.glob("*/maps")):
        vehicle_id = maps_dir.parent.name
        candidates = [*(maps_dir.glob("*_meta.json")), *(maps_dir.glob("history/*_meta.json"))]
        for meta_path in sorted(candidates):
            basename = remove_suffix(meta_path.name, "_meta.json")
            grid_path = meta_path.with_name(f"{basename}_grid.npy")
            if not grid_path.exists():
                continue
            meta = read_json(meta_path)
            source = str(meta.get("source") or basename.rsplit("_", 1)[0])
            is_history = meta_path.parent.name == "history"
            snapshots.append({
                "vehicle_id": vehicle_id,
                "source": source,
                "layer_id": f"{vehicle_id}/{source}",
                "stage": "history" if is_history else "latest",
                "snapshot_index": as_int(meta.get("snapshot_index")),
                "elapsed_sec": as_float(meta.get("elapsed_sec")),
                "grid_path": grid_path,
                "meta_path": meta_path,
                "pgm_path": meta_path.with_name(f"{basename}.pgm"),
                "meta": meta,
            })
    return snapshots


def dashboard_active_run_names(repo_root: Path) -> set[str]:
    """Catalog is authoritative when present; keep the legacy allowlist fallback."""
    if catalog_exists(repo_root):
        return active_ids(repo_root)
    include_path = repo_root / "experiments" / "dashboard_active_runs.txt"
    if not include_path.exists():
        return set()
    return {
        line.strip()
        for line in include_path.read_text().splitlines()
        if line.strip() and not line.strip().startswith("#")
    }


def find_oracle_runs(repo_root: Path) -> list[Path]:
    """Return oracle_correction directories containing a complete metric file."""
    artifacts_root = repo_root / "artifacts"
    if not artifacts_root.exists():
        return []
    active_runs = dashboard_active_run_names(repo_root)
    candidates = [
        metrics_path.parent
        for metrics_path in artifacts_root.glob(
            "*/oracle_correction/metrics.json"
        )
        if (metrics_path.parent / "manifest.json").exists()
    ]
    if dashboard_filter_enabled(repo_root):
        candidates = [
            path for path in candidates if path.parent.name in active_runs
        ]
    return sorted(
        candidates,
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )


def find_lidar_registration_runs(repo_root: Path) -> list[Path]:
    """Return complete controlled LiDAR-registration artifact directories."""
    artifacts_root = repo_root / "artifacts"
    if not artifacts_root.exists():
        return []
    active_runs = dashboard_active_run_names(repo_root)
    candidates = [
        metrics_path.parent
        for metrics_path in artifacts_root.glob(
            "*/lidar_registration/metrics.json"
        )
        if (metrics_path.parent / "manifest.json").exists()
    ]
    if dashboard_filter_enabled(repo_root):
        candidates = [
            path for path in candidates if path.parent.name in active_runs
        ]
    return sorted(
        candidates,
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )


def lidar_metric_rows(metrics: dict[str, Any]) -> list[dict[str, Any]]:
    """Flatten correction metrics, keeping two-region ablations in reading order."""
    rows = []
    conditions = metrics.get("conditions") or {}
    order = list(conditions)
    if "N-double" in conditions:
        preferred = ("B0", "B1", "N-prior", "N-single-A", "N-single-B", "N-double")
        order = [c for c in preferred if c in conditions] + [c for c in order if c not in preferred]
    for condition in order:
        values = conditions.get(condition) or {}
        differential = values.get("differential") or {}
        map_metrics = values.get("map") or {}
        rows.append({
            "condition": condition,
            "endpoint_translation_m": differential.get(
                "endpoint_translation_m"
            ),
            "endpoint_yaw_deg": differential.get("endpoint_yaw_deg"),
            "translation_rmse_m": (
                differential.get("translation_m") or {}
            ).get("rmse"),
            "yaw_rmse_deg": (differential.get("yaw_deg") or {}).get(
                "rmse"
            ),
            "occupied_chamfer_m": map_metrics.get("occupied_chamfer_m"),
            "occupied_f1": map_metrics.get("occupied_f1"),
        })
    return rows


def lidar_differential_rows(metrics: dict[str, Any]) -> list[dict[str, Any]]:
    """Flatten progress-indexed relative errors for Phase 2 conditions."""
    rows = []
    conditions = metrics.get("conditions") or {}
    for condition in conditions:
        differential = (conditions.get(condition) or {}).get(
            "differential"
        ) or {}
        for sample in differential.get("samples") or []:
            rows.append({
                "condition": condition,
                "progress_m": sample.get("progress_m"),
                "translation_error_m": sample.get(
                    "translation_error_m"
                ),
                "yaw_error_deg": sample.get("yaw_error_deg"),
            })
    return rows


def lidar_registration_rows(artifact_dir: Path) -> list[dict[str, Any]]:
    """Load registration diagnostics with numeric and boolean types."""
    numeric_fields = {
        "pair",
        "target_progress_m",
        "source_keyframe",
        "target_keyframe",
        "source_submap_points",
        "target_submap_points",
        "initial_translation_error_m",
        "initial_yaw_error_deg",
        "estimated_translation_error_m",
        "estimated_yaw_error_deg",
        "cost",
        "overlap_ratio",
        "inlier_rmse_m",
        "score_margin_ratio",
        "correction_translation_m",
        "correction_yaw_deg",
    }
    rows = []
    for source in read_csv_rows(artifact_dir / "registrations.csv"):
        row: dict[str, Any] = dict(source)
        for field in numeric_fields:
            value = source.get(field)
            try:
                row[field] = float(value) if value not in {None, ""} else None
            except (TypeError, ValueError):
                row[field] = None
        row["accepted"] = str(source.get("accepted", "")).lower() == "true"
        rows.append(row)
    return rows


def lidar_trajectory_rows(artifact_dir: Path) -> list[dict[str, Any]]:
    """Load Phase 2 estimated and GT keyframe trajectories."""
    rows: list[dict[str, Any]] = []
    conditions = list((read_json(artifact_dir / "metrics.json").get("conditions") or {}).keys()) or LIDAR_CONDITIONS
    for condition in conditions:
        path = artifact_dir / "trajectories" / f"{condition}.csv"
        for source in read_csv_rows(path):
            try:
                common = {
                    "vehicle": str(source["vehicle"]),
                    "keyframe": int(source["keyframe"]),
                    "progress_m": float(source["progress_m"]),
                }
                rows.append({
                    **common,
                    "series": condition,
                    "x": float(source["x"]),
                    "y": float(source["y"]),
                })
                if condition == "B0":
                    rows.append({
                        **common,
                        "series": "GT",
                        "x": float(source["gt_x"]),
                        "y": float(source["gt_y"]),
                    })
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def oracle_metric_rows(metrics: dict[str, Any]) -> list[dict[str, Any]]:
    """Flatten the primary Phase-1 metrics into one row per condition."""
    rows = []
    conditions = metrics.get("conditions") or {}
    ordered = [name for name in ORACLE_CONDITIONS if name in conditions]
    ordered.extend(sorted(set(conditions) - set(ordered)))
    for condition in ordered:
        values = conditions.get(condition) or {}
        differential = values.get("differential") or {}
        map_metrics = values.get("map") or {}
        rows.append({
            "condition": condition,
            "endpoint_translation_m": differential.get(
                "endpoint_translation_m"
            ),
            "endpoint_yaw_deg": differential.get("endpoint_yaw_deg"),
            "translation_rmse_m": (
                differential.get("translation_m") or {}
            ).get("rmse"),
            "yaw_rmse_deg": (differential.get("yaw_deg") or {}).get(
                "rmse"
            ),
            "occupied_chamfer_m": map_metrics.get("occupied_chamfer_m"),
            "occupied_f1": map_metrics.get("occupied_f1"),
            "occupied_precision": map_metrics.get("occupied_precision"),
            "occupied_recall": map_metrics.get("occupied_recall"),
            "coverage_ratio": map_metrics.get("coverage_ratio"),
        })
    return rows


def oracle_differential_rows(metrics: dict[str, Any]) -> list[dict[str, Any]]:
    """Flatten progress-indexed inter-UAV drift samples for plotting."""
    rows = []
    conditions = metrics.get("conditions") or {}
    for condition in ORACLE_CONDITIONS:
        differential = (conditions.get(condition) or {}).get(
            "differential"
        ) or {}
        for sample in differential.get("samples") or []:
            rows.append({
                "condition": condition,
                "progress_m": sample.get("progress_m"),
                "translation_error_m": sample.get(
                    "translation_error_m"
                ),
                "yaw_error_deg": sample.get("yaw_error_deg"),
            })
    return rows


def oracle_trajectory_rows(artifact_dir: Path) -> list[dict[str, Any]]:
    """Load estimated and GT keyframe trajectories in a long chart format."""
    rows: list[dict[str, Any]] = []
    for condition in ORACLE_CONDITIONS:
        path = artifact_dir / "trajectories" / f"{condition}.csv"
        for source in read_csv_rows(path):
            try:
                common = {
                    "vehicle": str(source["vehicle"]),
                    "keyframe": int(source["keyframe"]),
                    "progress_m": float(source["progress_m"]),
                }
                rows.append({
                    **common,
                    "series": condition,
                    "x": float(source["x"]),
                    "y": float(source["y"]),
                })
                if condition == "B0":
                    rows.append({
                        **common,
                        "series": "GT",
                        "x": float(source["gt_x"]),
                        "y": float(source["gt_y"]),
                    })
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def oracle_factor_rows(artifact_dir: Path) -> list[dict[str, Any]]:
    """Load inter-UAV factors and attach their GT endpoint coordinates."""
    nodes: dict[int, dict[str, str]] = {}
    for row in read_csv_rows(artifact_dir / "keyframes.csv"):
        try:
            nodes[int(row["node"])] = row
        except (KeyError, TypeError, ValueError):
            continue
    rows = []
    factors_path = artifact_dir / "factors.jsonl"
    if not factors_path.exists():
        return rows
    for line in factors_path.read_text().splitlines():
        try:
            factor = json.loads(line)
        except json.JSONDecodeError:
            continue
        if factor.get("type") != "oracle_inter_uav":
            continue
        source_id = int(factor["source"])
        target_id = int(factor["target"])
        source = nodes.get(source_id)
        target = nodes.get(target_id)
        if source is None or target is None:
            continue
        measurement = factor.get("measurement") or [None, None, None]
        rows.append({
            "condition": str(factor.get("condition", "")),
            "source_node": source_id,
            "target_node": target_id,
            "source_x": float(source["gt_x"]),
            "source_y": float(source["gt_y"]),
            "target_x": float(target["gt_x"]),
            "target_y": float(target["gt_y"]),
            "measurement_x": measurement[0],
            "measurement_y": measurement[1],
            "measurement_yaw": measurement[2],
        })
    return rows


def oracle_gate_rows(metrics: dict[str, Any]) -> list[dict[str, str]]:
    """Convert the Phase-1 gate dictionary into a display table."""
    checks = (metrics.get("phase1_gate") or {}).get("checks") or {}
    return [
        {
            "check": key.replace("_", " "),
            "result": "PASS" if bool(value) else "FAIL",
        }
        for key, value in checks.items()
    ]


def occupancy_snapshot_points(
    snapshot: dict[str, Any],
    world_from_local: dict[str, float],
    include_non_occupied: bool = False,
    max_points: int = 20000,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Convert occupied-grid cell centers to world coordinates for plotting."""
    grid = np.load(snapshot["grid_path"], allow_pickle=False)
    meta = snapshot["meta"]
    if grid.ndim != 2:
        raise ValueError(f"Expected a 2-D occupancy grid, got {grid.shape}")
    observed_rows, observed_cols = np.nonzero(grid >= 0)
    observed_values = grid[observed_rows, observed_cols].astype(np.int16)
    plot_mask = observed_values >= 65
    if include_non_occupied:
        plot_mask = np.ones(observed_values.shape, dtype=bool)
    rows = observed_rows[plot_mask]
    cols = observed_cols[plot_mask]
    values = observed_values[plot_mask]
    unsampled_count = int(values.size)
    if values.size > max_points:
        selected = np.linspace(0, values.size - 1, max_points, dtype=np.int64)
        rows, cols, values = rows[selected], cols[selected], values[selected]

    resolution = float(meta["resolution"])
    origin = meta.get("origin") or {}
    origin_x = float(origin.get("x", 0.0))
    origin_y = float(origin.get("y", 0.0))
    origin_yaw = float(origin.get("yaw", 0.0))
    grid_x = (cols.astype(np.float64) + 0.5) * resolution
    grid_y = (rows.astype(np.float64) + 0.5) * resolution
    cos_o, sin_o = math.cos(origin_yaw), math.sin(origin_yaw)
    frame_x = origin_x + cos_o * grid_x - sin_o * grid_y
    frame_y = origin_y + sin_o * grid_x + cos_o * grid_y
    tx = float(world_from_local["x"])
    ty = float(world_from_local["y"])
    yaw = float(world_from_local["yaw"])
    cos_t, sin_t = math.cos(yaw), math.sin(yaw)
    world_x = tx + cos_t * frame_x - sin_t * frame_y
    world_y = ty + sin_t * frame_x + cos_t * frame_y
    layer_id = str(snapshot["layer_id"])
    records = [
        {
            "x": float(x),
            "y": float(y),
            "occupancy": int(value),
            "cell_state": (
                "occupied" if value >= 65 else "free" if value <= 25 else "uncertain"
            ),
            "layer": layer_id,
        }
        for x, y, value in zip(world_x, world_y, values)
    ]
    summary = {
        "layer": layer_id,
        "frame_id": str(meta.get("frame_id", "")),
        "snapshot": snapshot.get("snapshot_index"),
        "elapsed_sec": snapshot.get("elapsed_sec"),
        "resolution_m": resolution,
        "observed_cells": int(observed_values.size),
        "occupied_cells": int(np.count_nonzero(observed_values >= 65)),
        "free_cells": int(np.count_nonzero(observed_values <= 25)),
        "coverage_pct": float(meta.get("coverage", 0.0)) * 100.0,
        "plotted_cells": int(values.size),
        "eligible_cells": unsampled_count,
        "transform_x": tx,
        "transform_y": ty,
        "transform_yaw": yaw,
    }
    return records, summary


def project_snapshot_to_target(
    snapshot: dict[str, Any],
    world_from_local: dict[str, float],
    target_snapshot: dict[str, Any],
) -> np.ndarray:
    """Project a saved local grid into the exact raster of a saved fused grid."""
    source_grid = np.load(snapshot["grid_path"], allow_pickle=False)
    source_meta = snapshot["meta"]
    target_meta = target_snapshot["meta"]
    target_grid = np.load(target_snapshot["grid_path"], allow_pickle=False)
    if source_grid.ndim != 2 or target_grid.ndim != 2:
        raise ValueError("source and target occupancy grids must be 2-D")

    target_height, target_width = target_grid.shape
    target_resolution = float(target_meta["resolution"])
    target_origin = target_meta.get("origin") or {}
    target_x = float(target_origin.get("x", 0.0))
    target_y = float(target_origin.get("y", 0.0))
    target_yaw = float(target_origin.get("yaw", 0.0))

    observed_rows, observed_cols = np.nonzero(source_grid >= 0)
    values = source_grid[observed_rows, observed_cols].astype(np.float64)
    resolution = float(source_meta["resolution"])
    origin = source_meta.get("origin") or {}
    origin_x = float(origin.get("x", 0.0))
    origin_y = float(origin.get("y", 0.0))
    origin_yaw = float(origin.get("yaw", 0.0))
    grid_x = (observed_cols.astype(np.float64) + 0.5) * resolution
    grid_y = (observed_rows.astype(np.float64) + 0.5) * resolution
    cos_o, sin_o = math.cos(origin_yaw), math.sin(origin_yaw)
    frame_x = origin_x + cos_o * grid_x - sin_o * grid_y
    frame_y = origin_y + sin_o * grid_x + cos_o * grid_y

    yaw = float(world_from_local["yaw"])
    cos_t, sin_t = math.cos(yaw), math.sin(yaw)
    world_x = (
        float(world_from_local["x"]) + cos_t * frame_x - sin_t * frame_y
    )
    world_y = (
        float(world_from_local["y"]) + sin_t * frame_x + cos_t * frame_y
    )
    dx, dy = world_x - target_x, world_y - target_y
    cos_g, sin_g = math.cos(target_yaw), math.sin(target_yaw)
    target_frame_x = cos_g * dx + sin_g * dy
    target_frame_y = -sin_g * dx + cos_g * dy
    cols = np.floor(target_frame_x / target_resolution).astype(np.int64)
    rows = np.floor(target_frame_y / target_resolution).astype(np.int64)
    valid = (
        (cols >= 0)
        & (cols < target_width)
        & (rows >= 0)
        & (rows < target_height)
    )
    flat = rows[valid] * target_width + cols[valid]
    projected = np.full(target_height * target_width, -1, dtype=np.int16)
    if flat.size:
        counts = np.bincount(flat, minlength=projected.size)
        sums = np.bincount(flat, weights=values[valid], minlength=projected.size)
        unique = np.flatnonzero(counts)
        projected[unique] = np.rint(sums[unique] / counts[unique]).astype(np.int16)
    return projected.reshape((target_height, target_width))


def dilate_boolean_mask(mask: np.ndarray, radius: int) -> np.ndarray:
    """Expand a 2-D mask by a square pixel radius for raster visibility."""
    if mask.ndim != 2:
        raise ValueError("mask must be 2-D")
    radius = int(radius)
    if radius < 0:
        raise ValueError("radius must be non-negative")
    if radius == 0:
        return mask.astype(bool, copy=True)
    source = mask.astype(bool, copy=False)
    padded = np.pad(source, radius, mode="constant", constant_values=False)
    expanded = np.zeros_like(source, dtype=bool)
    height, width = source.shape
    for row_offset in range(2 * radius + 1):
        for col_offset in range(2 * radius + 1):
            expanded |= padded[
                row_offset : row_offset + height,
                col_offset : col_offset + width,
            ]
    return expanded


def projected_conflict_points(
    sources: list[tuple[dict[str, Any], dict[str, float]]],
    target_snapshot: dict[str, Any],
    max_points: int = 5000,
) -> tuple[list[dict[str, float]], dict[str, int]]:
    """Locate free/occupied source disagreements in the fused grid coordinates."""
    target_meta = target_snapshot["meta"]
    target_grid = np.load(target_snapshot["grid_path"], allow_pickle=False)
    height, width = target_grid.shape
    target_resolution = float(target_meta["resolution"])
    target_origin = target_meta.get("origin") or {}
    target_x = float(target_origin.get("x", 0.0))
    target_y = float(target_origin.get("y", 0.0))
    target_yaw = float(target_origin.get("yaw", 0.0))
    free_votes = np.zeros(height * width, dtype=np.int16)
    occupied_votes = np.zeros(height * width, dtype=np.int16)
    contribution_votes = np.zeros(height * width, dtype=np.int16)

    for snapshot, transform in sources:
        projected = project_snapshot_to_target(snapshot, transform, target_snapshot)
        flat = np.flatnonzero(projected.reshape(-1) >= 0)
        if not flat.size:
            continue
        values = projected.reshape(-1)[flat]
        contribution_votes[flat] += 1
        free_votes[flat[values <= 25]] += 1
        occupied_votes[flat[values >= 65]] += 1

    conflict = np.flatnonzero((free_votes > 0) & (occupied_votes > 0))
    overlap = np.flatnonzero(contribution_votes > 1)
    total_conflicts = int(conflict.size)
    if conflict.size > max_points:
        conflict = conflict[np.linspace(0, conflict.size - 1, max_points, dtype=np.int64)]
    rows, cols = conflict // width, conflict % width
    gx = (cols.astype(np.float64) + 0.5) * target_resolution
    gy = (rows.astype(np.float64) + 0.5) * target_resolution
    cos_g, sin_g = math.cos(target_yaw), math.sin(target_yaw)
    xs = target_x + cos_g * gx - sin_g * gy
    ys = target_y + sin_g * gx + cos_g * gy
    points = [{"x": float(x), "y": float(y)} for x, y in zip(xs, ys)]
    return points, {
        "projected_observed_cells": int(np.count_nonzero(contribution_votes)),
        "source_overlap_cells": int(overlap.size),
        "conflict_cells": total_conflicts,
    }


def as_float(value: Any) -> float | None:
    if value in {None, ""}:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def as_int(value: Any) -> int | None:
    number = as_float(value)
    if number is None:
        return None
    return int(number)


def as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if value in {None, ""}:
        return None
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "pass", "success"}:
        return True
    if text in {"false", "0", "no", "fail", "failure"}:
        return False
    return None


def first_non_empty(*values: Any, default: str = "") -> Any:
    for value in values:
        if value not in {None, "", "unknown"}:
            return value
    return default


def resolve_repo_path(repo_root: Path, value: Any) -> Path | None:
    if value in {None, ""}:
        return None
    text = str(value).strip()
    if not text:
        return None
    if text.startswith("/workspace/AV_Drone/"):
        return repo_root / remove_prefix(text, "/workspace/AV_Drone/")
    path = Path(text)
    if path.is_absolute():
        return path
    return repo_root / path


def local_to_world(
    local_x: float,
    local_y: float,
    transform: dict[str, float],
) -> tuple[float, float]:
    """Apply the recorded world-from-local planar transform."""
    yaw = float(transform["yaw"])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        float(transform["x"]) + cos_yaw * local_x - sin_yaw * local_y,
        float(transform["y"]) + sin_yaw * local_x + cos_yaw * local_y,
    )


def _normalized_world_from_local(value: Any) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None
    try:
        transform = {
            "x": float(value["x"]),
            "y": float(value["y"]),
            "yaw": float(value.get("yaw", 0.0)),
        }
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in transform.values()):
        return None
    return transform


def resolve_world_from_local(
    repo_root: Path,
    metadata: dict[str, Any],
    summary: dict[str, Any],
    vehicle_id: str,
) -> tuple[dict[str, float] | None, str]:
    """Resolve a coordinate transform from metadata or a snapshotted manifest."""
    has_frame_contract = bool(
        first_non_empty(
            metadata.get("trajectory_frame_id"), summary.get("trajectory_frame_id")
        )
        and first_non_empty(
            metadata.get("world_frame_id"), summary.get("world_frame_id")
        )
    )
    direct_value = metadata.get("world_from_local")
    if not isinstance(direct_value, dict):
        direct_value = summary.get("world_from_local")
    direct = _normalized_world_from_local(direct_value)
    if has_frame_contract and direct is not None:
        return direct, "metadata"

    snapshot_files = metadata.get("config_snapshot_files") or {}
    candidates = [
        snapshot_files.get("scenario_manifest"),
        metadata.get("scenario_manifest_path"),
        summary.get("scenario_manifest_path"),
    ]
    for candidate in candidates:
        manifest_path = resolve_repo_path(repo_root, candidate)
        if manifest_path is None or not manifest_path.is_file():
            continue
        try:
            manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            continue
        for vehicle in manifest.get("vehicles", []):
            if str(vehicle.get("name", "")) != str(vehicle_id):
                continue
            spawn = vehicle.get("spawn", [])
            if not isinstance(spawn, list) or len(spawn) < 4:
                continue
            transform = _normalized_world_from_local(
                {"x": spawn[0], "y": spawn[1], "yaw": spawn[3]}
            )
            if transform is not None:
                return transform, "scenario_manifest"
    return None, "unavailable"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def infer_world_path(repo_root: Path, world_name: str, scenario_id: str, *candidates: Any) -> str:
    for candidate in candidates:
        path = resolve_repo_path(repo_root, candidate)
        if path and path.exists():
            return str(path)

    names = []
    if world_name not in {"", "unknown"}:
        names.append(world_name)
    if "obstacle_demo" in scenario_id:
        names.append("obstacle_demo")
    if "random_corridor" in scenario_id:
        names.append("random_corridor_generated")

    for name in names:
        for suffix in [".world", ".sdf"]:
            path = repo_root / "sim_assets" / "worlds" / f"{name}{suffix}"
            if path.exists():
                return str(path)
            gz_path = repo_root / "sim_assets" / "gz" / "worlds" / f"{name}{suffix}"
            if gz_path.exists():
                return str(gz_path)

    default_path = repo_root / "sim_assets" / "worlds" / "obstacle_demo.world"
    return str(default_path) if default_path.exists() else ""


def has_artifact_evidence(path: Path) -> bool:
    return any(
        (path / filename).exists()
        for filename in [
            "metadata.json",
            "summary.json",
            "paper_metrics.json",
            "rosbag_metrics.json",
            "trajectory_from_rosbag.csv",
            "trajectory.csv",
        ]
    )


def classify_algorithm(record: dict[str, Any]) -> str:
    text = " ".join(
        str(record.get(key) or "").lower()
        for key in [
            "condition_id",
            "condition",
            "planner_name",
            "planner_family",
            "baseline_name",
            "run_id",
        ]
    )
    if "fgm" in text or "follow_the_gap" in text or "gap" in text or "baseline_single_goal" in text:
        return "FGM"
    if "mppi" in text or "sampling_mpc" in text:
        return "MPPI"
    if "astar" in text or "a*" in text:
        return "A*"
    return "Unknown"


def build_trajectory_label(record: dict[str, Any]) -> str:
    algorithm = str(record.get("algorithm_label") or classify_algorithm(record))
    condition = str(record.get("condition_id") or "unknown")
    run_id = str(record.get("run_id") or "")
    planner = str(record.get("planner_name") or "")
    if planner and planner != "unknown":
        return f"{algorithm} | {condition} | {planner} | {run_id}"
    return f"{algorithm} | {condition} | {run_id}"


def parse_pose(text: str | None) -> tuple[float, float, float]:
    parts = [float(item) for item in str(text or "").split()]
    while len(parts) < 6:
        parts.append(0.0)
    return parts[0], parts[1], parts[5]


def parse_size(text: str | None) -> tuple[float, float, float]:
    parts = [float(item) for item in str(text or "").split()]
    while len(parts) < 3:
        parts.append(0.0)
    return parts[0], parts[1], parts[2]


def model_uri_path(repo_root: Path, uri: str) -> Path | None:
    if not uri.startswith("model://"):
        return resolve_repo_path(repo_root, uri)
    model_name = remove_prefix(uri, "model://").strip("/")
    for base in [repo_root / "sim_assets" / "models", repo_root / "sim_assets" / "gz" / "models"]:
        path = base / model_name / "model.sdf"
        if path.exists():
            return path
    return None


def cylinder_radius_from_model(repo_root: Path, uri: str, fallback: float = 0.5) -> float:
    model_path = model_uri_path(repo_root, uri)
    if not model_path:
        return fallback
    try:
        root = ET.parse(model_path).getroot()
        radius = root.findtext(".//cylinder/radius")
        return float(radius) if radius is not None else fallback
    except Exception:
        return fallback


def rotated_box_points(cx: float, cy: float, yaw: float, sx: float, sy: float) -> list[tuple[float, float]]:
    half_x = sx / 2.0
    half_y = sy / 2.0
    corners = [(-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y)]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    points = []
    for x, y in corners:
        points.append((cx + cos_yaw * x - sin_yaw * y, cy + sin_yaw * x + cos_yaw * y))
    points.append(points[0])
    return points


def circle_points(cx: float, cy: float, radius: float, segments: int = 32) -> list[tuple[float, float]]:
    points = []
    for idx in range(segments + 1):
        theta = 2.0 * math.pi * idx / segments
        points.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta)))
    return points


def load_world_geometry(repo_root: Path, world_path: Path) -> dict[str, list[dict[str, Any]]]:
    rect_rows: list[dict[str, Any]] = []
    outline_rows: list[dict[str, Any]] = []

    root = ET.parse(world_path).getroot()

    def append_outline(name: str, kind: str, points: list[tuple[float, float]]) -> None:
        for order, (x, y) in enumerate(points):
            outline_rows.append({
                "geometry_id": name,
                "kind": kind,
                "order": order,
                "x": x,
                "y": y,
            })

    for model in root.findall(".//model"):
        name = model.attrib.get("name", "model")
        pose_x, pose_y, yaw = parse_pose(model.findtext("pose"))
        box_size = model.findtext(".//box/size")
        if box_size:
            sx, sy, _ = parse_size(box_size)
            kind = "floor" if "floor" in name or "ground" in name else "wall"
            if abs(yaw) < 1e-6:
                rect_rows.append({
                    "geometry_id": name,
                    "kind": kind,
                    "x1": pose_x - sx / 2.0,
                    "x2": pose_x + sx / 2.0,
                    "y1": pose_y - sy / 2.0,
                    "y2": pose_y + sy / 2.0,
                })
            else:
                append_outline(name, kind, rotated_box_points(pose_x, pose_y, yaw, sx, sy))

        cylinder = model.find(".//cylinder")
        if cylinder is not None:
            radius_text = cylinder.findtext("radius")
            radius = float(radius_text) if radius_text else 0.5
            append_outline(name, "obstacle", circle_points(pose_x, pose_y, radius))

    for include in root.findall(".//include"):
        name = include.findtext("name") or include.attrib.get("name", "include")
        uri = include.findtext("uri") or ""
        pose_x, pose_y, _ = parse_pose(include.findtext("pose"))
        if not uri.startswith("model://"):
            continue
        if "cylinder" not in name and "cylinder" not in uri and "pole" not in name:
            continue
        radius = cylinder_radius_from_model(repo_root, uri, fallback=0.5)
        append_outline(name, "obstacle", circle_points(pose_x, pose_y, radius))

    return {"rects": rect_rows, "outlines": outline_rows}


def final_goal_distance_from_trajectory(trajectory_path: Path, goal_x: float | None, goal_y: float | None) -> float | None:
    if goal_x is None or goal_y is None or not trajectory_path.exists():
        return None
    last_xy = None
    try:
        with trajectory_path.open("r", newline="") as handle:
            for row in csv.DictReader(handle):
                try:
                    last_xy = (float(row["x"]), float(row["y"]))
                except (KeyError, TypeError, ValueError):
                    continue
    except Exception:
        return None
    if last_xy is None:
        return None
    return math.hypot(last_xy[0] - goal_x, last_xy[1] - goal_y)


def artifact_dirs(repo_root: Path) -> list[Path]:
    artifacts_root = repo_root / "artifacts"
    if not artifacts_root.exists():
        return []
    active_runs = dashboard_active_run_names(repo_root)
    candidates = set()
    for path in artifacts_root.glob("*_drone*"):
        if path.is_dir() and has_artifact_evidence(path):
            candidates.add(path)
    for filename in ["rosbag_metrics.json", "paper_metrics.json"]:
        for path in [*artifacts_root.glob(f"*/{filename}"), *artifacts_root.glob(f"*/*/{filename}")]:
            if path.parent.is_dir() and has_artifact_evidence(path.parent):
                candidates.add(path.parent)
    if dashboard_filter_enabled(repo_root):
        candidates = {
            path
            for path in candidates
            if path.name in active_runs or path.parent.name in active_runs
        }
    return sorted(candidates, key=lambda item: str(item.relative_to(artifacts_root)))


def load_artifact_records(repo_root: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for artifact_dir in artifact_dirs(repo_root):
        metadata = read_json(artifact_dir / "metadata.json")
        summary = read_json(artifact_dir / "summary.json")
        paper = read_json(artifact_dir / "paper_metrics.json")
        rosbag = read_json(artifact_dir / "rosbag_metrics.json")
        phase = read_json(artifact_dir / "phase_summary.json")
        slam = read_json(artifact_dir / "slam_summary.json")
        swarm = read_json(artifact_dir.parent / "swarm_summary.json")

        source_run_id = first_non_empty(
            rosbag.get("run_id"),
            paper.get("run_id"),
            metadata.get("run_id"),
            summary.get("run_id"),
            artifact_dir.name,
        )
        vehicle_id = first_non_empty(metadata.get("drone_name"), artifact_dir.name)
        nested_vehicle_artifact = (
            artifact_dir.parent.name == str(source_run_id)
            and artifact_dir.name == str(vehicle_id)
        )
        run_id = (
            f"{source_run_id}/{vehicle_id}"
            if nested_vehicle_artifact
            else source_run_id
        )
        condition_id = first_non_empty(
            rosbag.get("condition_id"),
            rosbag.get("condition"),
            paper.get("condition_id"),
            paper.get("condition"),
            metadata.get("condition_id"),
            metadata.get("experiment_condition"),
            summary.get("condition_id"),
            summary.get("experiment_condition"),
            default="unknown",
        )
        scenario_id = first_non_empty(
            rosbag.get("scenario_id"),
            rosbag.get("scenario"),
            paper.get("scenario_id"),
            paper.get("scenario"),
            metadata.get("scenario_id"),
            metadata.get("scenario_name"),
            summary.get("scenario_id"),
            summary.get("scenario_name"),
            default="unknown",
        )
        success = as_bool(
            first_non_empty(
                rosbag.get("success"),
                rosbag.get("return_success"),
                paper.get("success"),
                paper.get("return_success"),
            )
        )
        if success is None:
            success = as_bool(summary.get("goal_reached"))
        success_code = first_non_empty(
            rosbag.get("success_code"),
            paper.get("success_code"),
            summary.get("failure_code"),
        )
        failure_code = first_non_empty(
            rosbag.get("failure_code"),
            paper.get("failure_code"),
            summary.get("failure_code"),
        )
        result = (
            "in_progress"
            if str(success_code).strip().lower() in {"in_progress", "running"}
            else ("pass" if success else "fail")
        )

        mission_time = as_float(
            first_non_empty(
                rosbag.get("mission_time_s"),
                rosbag.get("outbound_time_s"),
                paper.get("mission_time_s"),
                paper.get("outbound_time_s"),
                summary.get("outbound_time_s"),
                paper.get("runtime_s"),
                summary.get("runtime_s"),
            )
        )
        actual_path_length = as_float(
            first_non_empty(
                rosbag.get("actual_path_length_m"),
                rosbag.get("outbound_path_length_m"),
                rosbag.get("total_path_length_m"),
                paper.get("actual_path_length_m"),
                paper.get("outbound_path_length_m"),
                paper.get("total_path_length_m"),
                summary.get("outbound_path_length_m"),
                summary.get("total_path_length_m"),
            )
        )
        straight_line_distance = as_float(first_non_empty(rosbag.get("straight_line_distance_m"), paper.get("straight_line_distance_m")))
        planned_path_length = as_float(first_non_empty(rosbag.get("planned_path_length_m"), paper.get("planned_path_length_m")))
        path_efficiency = as_float(first_non_empty(rosbag.get("path_efficiency"), paper.get("path_efficiency")))
        if path_efficiency is None and actual_path_length:
            if planned_path_length:
                path_efficiency = planned_path_length / actual_path_length
            elif straight_line_distance:
                path_efficiency = straight_line_distance / actual_path_length

        return_path_length = as_float(first_non_empty(rosbag.get("return_path_length_m"), paper.get("return_path_length_m")))
        straight_home_distance = as_float(first_non_empty(rosbag.get("straight_line_home_distance_m"), paper.get("straight_line_home_distance_m")))
        return_efficiency = as_float(first_non_empty(rosbag.get("return_path_efficiency"), paper.get("return_path_efficiency")))
        if return_efficiency is None and return_path_length and straight_home_distance:
            return_efficiency = straight_home_distance / return_path_length

        world_name = first_non_empty(
            rosbag.get("world_name"),
            paper.get("world_name"),
            metadata.get("world_name"),
            metadata.get("px4_gz_world"),
            summary.get("px4_gz_world"),
            default="unknown",
        )
        scenario_manifest_path = first_non_empty(
            rosbag.get("scenario_manifest_path"),
            paper.get("scenario_manifest_path"),
            metadata.get("scenario_manifest_path"),
            summary.get("scenario_manifest_path"),
        )
        snapshot_files = metadata.get("config_snapshot_files") or {}
        world_from_local, world_transform_source = resolve_world_from_local(
            repo_root,
            metadata,
            summary,
            str(vehicle_id),
        )
        world_snapshot_path = first_non_empty(
            rosbag.get("world_snapshot_path"),
            paper.get("world_snapshot_path"),
            metadata.get("world_snapshot_path"),
            summary.get("world_snapshot_path"),
            snapshot_files.get("world"),
        )
        map_path = infer_world_path(
            repo_root,
            str(world_name),
            str(scenario_id),
            world_snapshot_path,
            rosbag.get("world_path"),
            paper.get("world_path"),
            metadata.get("world_path"),
            summary.get("world_path"),
        )
        world_sha256 = first_non_empty(
            rosbag.get("world_sha256"),
            paper.get("world_sha256"),
            metadata.get("world_sha256"),
            summary.get("world_sha256"),
        )
        map_file_sha256 = ""
        if map_path:
            try:
                map_file_sha256 = file_sha256(Path(map_path))
            except Exception:
                map_file_sha256 = ""
        world_hash_verified = bool(
            world_sha256
            and map_file_sha256
            and str(world_sha256) == str(map_file_sha256)
        )
        goal_x = as_float(first_non_empty(rosbag.get("goal_x"), paper.get("goal_x"), metadata.get("goal_x"), summary.get("goal_x")))
        goal_y = as_float(first_non_empty(rosbag.get("goal_y"), paper.get("goal_y"), metadata.get("goal_y"), summary.get("goal_y")))
        if goal_x is None and "obstacle_demo" in str(scenario_id):
            goal_x = 140.0
        if goal_y is None and "obstacle_demo" in str(scenario_id):
            goal_y = 0.0
        trajectory_path = (
            artifact_dir / "trajectory_from_rosbag.csv"
            if (artifact_dir / "trajectory_from_rosbag.csv").exists()
            else artifact_dir / "trajectory.csv"
        )
        final_goal_distance = as_float(
            first_non_empty(
                rosbag.get("final_goal_distance_m"),
                paper.get("final_goal_distance_m"),
                summary.get("final_goal_distance_m"),
            )
        )
        if final_goal_distance is None:
            final_goal_distance = final_goal_distance_from_trajectory(trajectory_path, goal_x, goal_y)
        compute_latency_ms_p95 = as_float(
            first_non_empty(
                rosbag.get("compute_latency_ms_p95"),
                rosbag.get("planning_time_ms_p95"),
                paper.get("compute_latency_ms_p95"),
                paper.get("planning_time_ms_p95"),
            )
        )

        record = {
            "run_id": run_id,
            "source_run_id": source_run_id,
            "vehicle_id": vehicle_id,
            "artifact_path": str(artifact_dir),
            "condition_id": condition_id,
            "scenario_id": scenario_id,
            "world_name": world_name,
            "map_path": map_path,
            "world_snapshot_path": world_snapshot_path,
            "world_sha256": world_sha256,
            "map_file_sha256": map_file_sha256,
            "world_hash_verified": world_hash_verified,
            "scenario_manifest_path": scenario_manifest_path,
            "trajectory_frame_id": first_non_empty(
                metadata.get("trajectory_frame_id"),
                summary.get("trajectory_frame_id"),
                default="local",
            ),
            "world_frame_id": first_non_empty(
                metadata.get("world_frame_id"),
                summary.get("world_frame_id"),
                default="world",
            ),
            "world_from_local": world_from_local,
            "world_transform_source": world_transform_source,
            "planner_name": first_non_empty(
                rosbag.get("planner_name"),
                paper.get("planner_name"),
                metadata.get("planner_name"),
                summary.get("planner_name"),
                default="unknown",
            ),
            "planner_family": first_non_empty(
                rosbag.get("planner_family"),
                paper.get("planner_family"),
                metadata.get("planner_family"),
                summary.get("planner_family"),
                default="unknown",
            ),
            "map_source": first_non_empty(
                rosbag.get("map_source"),
                paper.get("map_source"),
                metadata.get("map_source"),
                summary.get("map_source"),
                default="unknown",
            ),
            "seed": as_int(
                first_non_empty(
                    rosbag.get("seed"),
                    rosbag.get("experiment_seed"),
                    paper.get("seed"),
                    paper.get("experiment_seed"),
                    metadata.get("experiment_seed"),
                    summary.get("experiment_seed"),
                )
            ),
            "experiment_stage": first_non_empty(
                rosbag.get("experiment_stage"),
                paper.get("experiment_stage"),
                metadata.get("experiment_stage"),
                summary.get("experiment_stage"),
                default="legacy",
            ),
            "trial_index": as_int(
                first_non_empty(
                    rosbag.get("trial_index"),
                    paper.get("trial_index"),
                    metadata.get("trial_index"),
                    summary.get("trial_index"),
                )
            ),
            "return_mode": first_non_empty(
                rosbag.get("return_mode"),
                paper.get("return_mode"),
                metadata.get("return_mode"),
                summary.get("return_mode"),
                default="unknown",
            ),
            "mapping_enabled": first_non_empty(
                rosbag.get("mapping_enabled"),
                paper.get("mapping_enabled"),
                metadata.get("mapping_enabled"),
                default="",
            ),
            "started_at": first_non_empty(metadata.get("started_at"), summary.get("started_at")),
            "git_commit": first_non_empty(metadata.get("git_commit"), summary.get("git_commit")),
            "git_branch": first_non_empty(metadata.get("git_branch"), summary.get("git_branch")),
            "git_dirty": first_non_empty(metadata.get("git_dirty"), summary.get("git_dirty")),
            "success": success,
            "result": result,
            "success_code": success_code,
            "failure_code": failure_code,
            "runtime_s": as_float(first_non_empty(rosbag.get("runtime_s"), paper.get("runtime_s"), summary.get("runtime_s"))),
            "mission_time_s": mission_time,
            "outbound_time_s": as_float(first_non_empty(rosbag.get("outbound_time_s"), paper.get("outbound_time_s"), summary.get("outbound_time_s"))),
            "final_goal_distance_m": final_goal_distance,
            "actual_path_length_m": actual_path_length,
            "straight_line_distance_m": straight_line_distance,
            "planned_path_length_m": planned_path_length,
            "path_efficiency": path_efficiency,
            "return_time_s": as_float(first_non_empty(rosbag.get("return_time_s"), paper.get("return_time_s"), summary.get("return_time_s"))),
            "outbound_path_length_m": as_float(first_non_empty(rosbag.get("outbound_path_length_m"), paper.get("outbound_path_length_m"), summary.get("outbound_path_length_m"))),
            "return_path_length_m": return_path_length,
            "return_path_efficiency": return_efficiency,
            "total_path_length_m": as_float(
                first_non_empty(rosbag.get("total_path_length_m"), paper.get("total_path_length_m"), summary.get("total_path_length_m"))
            ),
            "min_obstacle_distance_m": as_float(
                first_non_empty(
                    rosbag.get("return_min_obstacle_distance_m"),
                    rosbag.get("min_obstacle_distance_m"),
                    paper.get("return_min_obstacle_distance_m"),
                    paper.get("min_obstacle_distance_m"),
                    summary.get("closest_obstacle_m"),
                )
            ),
            "safety_intervention_count": as_float(
                first_non_empty(
                    rosbag.get("safety_intervention_count"),
                    paper.get("safety_intervention_count"),
                    summary.get("safety_intervention_count"),
                    summary.get("safety_event_count"),
                )
            ),
            "safety_event_count": as_float(
                first_non_empty(
                    rosbag.get("safety_event_count"),
                    paper.get("safety_event_count"),
                    summary.get("safety_event_count"),
                )
            ),
            "control_effort": as_float(first_non_empty(rosbag.get("control_effort"), paper.get("control_effort"), summary.get("control_effort"))),
            "command_smoothness": as_float(first_non_empty(rosbag.get("command_smoothness"), paper.get("command_smoothness"))),
            "planning_time_ms_p50": as_float(first_non_empty(rosbag.get("planning_time_ms_p50"), paper.get("planning_time_ms_p50"))),
            "planning_time_ms_p95": as_float(first_non_empty(rosbag.get("planning_time_ms_p95"), paper.get("planning_time_ms_p95"))),
            "global_planning_time_ms_p95": as_float(
                first_non_empty(
                    rosbag.get("global_planning_time_ms_p95"),
                    paper.get("global_planning_time_ms_p95"),
                    summary.get("global_planning_time_ms_p95"),
                )
            ),
            "compute_latency_ms_p95": compute_latency_ms_p95,
            "replan_count": as_float(first_non_empty(rosbag.get("replan_count"), paper.get("replan_count"))),
            "escape_count": as_float(first_non_empty(rosbag.get("escape_count"), paper.get("escape_count"), summary.get("escape_count"))),
            "planner_cmd_count": as_float(first_non_empty(rosbag.get("planner_cmd_count"), paper.get("planner_cmd_count"))),
            "safe_cmd_count": as_float(first_non_empty(rosbag.get("safe_cmd_count"), paper.get("safe_cmd_count"))),
            "pose_count": as_float(first_non_empty(rosbag.get("pose_count"), paper.get("pose_count"))),
            "pose_period_p99_s": as_float(first_non_empty(rosbag.get("pose_period_p99_s"), paper.get("pose_period_p99_s"))),
            "map_coverage": as_float(
                first_non_empty(
                    rosbag.get("map_coverage"),
                    rosbag.get("map_coverage_final"),
                    rosbag.get("slam_coverage"),
                    paper.get("map_coverage"),
                    paper.get("map_coverage_final"),
                    paper.get("slam_coverage"),
                    summary.get("slam_coverage"),
                    slam.get("coverage"),
                )
            ),
            "fusion_state": swarm.get("state"),
            "fusion_map_version": as_int(swarm.get("map_version")),
            "fusion_source_count": as_int(swarm.get("source_count")),
            "fusion_latency_p95_ms": as_float(swarm.get("fusion_latency_p95_ms")),
            "fusion_conflict_ratio": as_float(swarm.get("last_conflict_ratio")),
            "localization_ok_rate": as_float(first_non_empty(rosbag.get("localization_ok_rate"), paper.get("localization_ok_rate"))),
            "mission_phase": first_non_empty(summary.get("mission_phase"), paper.get("mission_phase")),
            "rosbag_path": first_non_empty(rosbag.get("rosbag_path"), paper.get("rosbag_path"), metadata.get("rosbag_path")),
            "paper_metrics_path": str(artifact_dir / "paper_metrics.json"),
            "rosbag_metrics_path": str(artifact_dir / "rosbag_metrics.json"),
            "summary_path": str(artifact_dir / "summary.json"),
            "trajectory_path": str(trajectory_path),
            "phase_summary_path": str(artifact_dir / "phase_summary.json"),
            "phase_count": len(phase.get("phase_timeline") or []),
        }
        record["algorithm_label"] = first_non_empty(
            rosbag.get("algorithm_label"),
            paper.get("algorithm_label"),
            metadata.get("algorithm_label"),
            default=classify_algorithm(record),
        )
        record["trajectory_label"] = first_non_empty(
            rosbag.get("trajectory_label"),
            paper.get("trajectory_label"),
            metadata.get("trajectory_label"),
            default=build_trajectory_label(record),
        )
        record["experiment_label"] = run_label(repo_root, str(source_run_id))
        if catalog_exists(repo_root):
            record["trajectory_label"] = run_label(repo_root, str(run_id))
        records.append(record)
    return records


def load_registry_records(repo_root: Path) -> dict[str, list[dict[str, str]]]:
    experiments_dir = repo_root / "experiments"
    return {
        "index": read_csv_rows(experiments_dir / "index.csv"),
        "ledger": read_csv_rows(experiments_dir / "ledger.csv"),
        "scenario_table": read_csv_rows(experiments_dir / "scenario_table.csv"),
        "summary_table": read_csv_rows(experiments_dir / "paper_outputs" / "summary_table.csv"),
    }


def metric_summary(records: list[dict[str, Any]], metric_names: list[str]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[str(record.get("condition_id") or "unknown")].append(record)

    rows: list[dict[str, Any]] = []
    for condition_id, condition_records in sorted(grouped.items()):
        for metric in metric_names:
            if metric == "success":
                values = [1.0 if record.get("success") else 0.0 for record in condition_records]
            else:
                values = [
                    as_float(record.get(metric))
                    for record in condition_records
                    if as_float(record.get(metric)) is not None
                ]
            clean = [value for value in values if value is not None]
            if not clean:
                rows.append({
                    "condition_id": condition_id,
                    "metric": metric,
                    "runs": len(condition_records),
                    "valid": 0,
                    "mean": None,
                    "std": None,
                    "min": None,
                    "max": None,
                })
                continue
            mean = sum(clean) / len(clean)
            variance = sum((value - mean) ** 2 for value in clean) / len(clean)
            rows.append({
                "condition_id": condition_id,
                "metric": metric,
                "runs": len(condition_records),
                "valid": len(clean),
                "mean": mean,
                "std": math.sqrt(variance),
                "min": min(clean),
                "max": max(clean),
            })
    return rows


def check_data(repo_root: Path) -> int:
    artifacts = load_artifact_records(repo_root)
    registry = load_registry_records(repo_root)
    run_reports = load_run_reports(repo_root)
    oracle_runs = find_oracle_runs(repo_root)
    lidar_registration_runs = find_lidar_registration_runs(repo_root)
    print(f"repo_root: {repo_root}")
    print(f"artifact_records: {len(artifacts)}")
    print(f"registry_index_rows: {len(registry['index'])}")
    print(f"ledger_rows: {len(registry['ledger'])}")
    print(f"scenario_table_rows: {len(registry['scenario_table'])}")
    print(f"summary_table_rows: {len(registry['summary_table'])}")
    print(f"run_reports: {len(run_reports)}")
    print(f"oracle_runs: {len(oracle_runs)}")
    print(f"lidar_registration_runs: {len(lidar_registration_runs)}")
    conditions = sorted({str(record.get("condition_id")) for record in artifacts})
    scenarios = sorted({str(record.get("scenario_id")) for record in artifacts})
    unknown_conditions = sum(1 for record in artifacts if record.get("condition_id") == "unknown")
    unknown_scenarios = sum(1 for record in artifacts if record.get("scenario_id") == "unknown")
    print(f"conditions: {', '.join(conditions) if conditions else '-'}")
    print(f"scenarios: {', '.join(scenarios) if scenarios else '-'}")
    print(f"unknown_condition_records: {unknown_conditions}")
    print(f"unknown_scenario_records: {unknown_scenarios}")
    return 0


def render_oracle_correction_tab(repo_root: Path, pd: Any, st: Any, artifact_dir=None) -> None:
    """Render saved Phase-1 oracle drift-correction artifacts."""
    st.subheader("Oracle 기반 Multi-UAV drift 보정")
    st.caption(
        "동일 rosbag의 scan을 B0, B1, O-single, O-periodic pose로 재투영해 "
        "상대 drift와 지도 오차를 비교합니다. GT는 oracle factor 생성과 "
        "평가에만 사용됩니다."
    )
    oracle_runs = [artifact_dir] if artifact_dir is not None else find_oracle_runs(repo_root)
    if not oracle_runs:
        st.info(
            "`artifacts/<RUN_ID>/oracle_correction/metrics.json` 형식의 "
            "분석 결과가 없습니다. active run 목록도 확인하세요."
        )
        return

    if artifact_dir is None:
        artifact_dir = st.selectbox(
            "Oracle run", oracle_runs,
            format_func=lambda path: run_label(repo_root, path.parent.name),
            key="oracle_correction_run",
        )
    metrics = read_json(artifact_dir / "metrics.json")
    manifest = read_json(artifact_dir / "manifest.json")
    if not metrics.get("conditions"):
        st.error(f"조건별 지표를 읽지 못했습니다: {artifact_dir}")
        return

    gate = metrics.get("phase1_gate") or {}
    gate_pass = bool(gate.get("pass", False))
    metric_rows = oracle_metric_rows(metrics)
    differential_rows = oracle_differential_rows(metrics)
    trajectory_rows = oracle_trajectory_rows(artifact_dir)
    factor_rows = oracle_factor_rows(artifact_dir)

    maximum_progress = max(
        [
            float(row["progress_m"])
            for row in differential_rows
            if row.get("progress_m") is not None
        ],
        default=0.0,
    )
    vehicles = manifest.get("vehicles") or {}
    sync_drop = max(
        [
            float(values.get("drop_ratio", 0.0))
            for values in vehicles.values()
        ],
        default=0.0,
    )
    periodic_factors = int(
        ((manifest.get("optimizer") or {}).get("O-periodic") or {}).get(
            "oracle_factor_count", 0
        )
    )
    endpoint_improvement = gate.get("differential_endpoint_improvement")
    chamfer_improvement = gate.get("chamfer_improvement")

    kpi_columns = st.columns(6)
    kpi_columns[0].metric("Formal gate", "PASS" if gate_pass else "FAIL")
    kpi_columns[1].metric("공통 진행 거리", f"{maximum_progress:.1f} m")
    kpi_columns[2].metric("Periodic factors", periodic_factors)
    kpi_columns[3].metric("최대 sync drop", f"{sync_drop * 100.0:.2f}%")
    kpi_columns[4].metric(
        "종단 상대오차 개선",
        "-" if endpoint_improvement is None else f"{float(endpoint_improvement) * 100.0:.1f}%",
    )
    kpi_columns[5].metric(
        "Chamfer 개선",
        "-" if chamfer_improvement is None else f"{float(chamfer_improvement) * 100.0:.1f}%",
    )
    if gate_pass:
        st.success("설정된 Phase-1 acceptance gate를 통과했습니다.")
    else:
        st.warning(
            "Formal gate를 통과하지 못했습니다. 이 run은 파이프라인 검증과 "
            "feasibility signal로만 해석해야 합니다."
        )

    overview_tab, drift_tab, trajectory_tab, map_tab, factor_tab, raw_tab = st.tabs([
        "Summary",
        "Relative Drift",
        "Trajectories",
        "Maps",
        "Factors",
        "Raw Artifact",
    ])

    with overview_tab:
        st.markdown("#### 조건별 핵심 지표")
        metric_df = pd.DataFrame(metric_rows)
        st.dataframe(metric_df, use_container_width=True, hide_index=True)

        gate_col, audit_col = st.columns(2)
        with gate_col:
            st.markdown("#### Gate checks")
            gate_df = pd.DataFrame(oracle_gate_rows(metrics))
            st.dataframe(gate_df, use_container_width=True, hide_index=True)
        with audit_col:
            st.markdown("#### Data audit")
            audit_rows = []
            for vehicle, values in vehicles.items():
                audit_rows.append({
                    "vehicle": vehicle,
                    "source_scans": values.get("source_scans"),
                    "common_support": values.get("common_support_scans"),
                    "valid_scans": values.get("valid_scans"),
                    "startup_trimmed": values.get("startup_trimmed_scans"),
                    "sync_dropped": values.get("interpolation_dropped_scans"),
                    "keyframes": values.get("keyframes"),
                })
            st.dataframe(
                pd.DataFrame(audit_rows), use_container_width=True, hide_index=True
            )

        summary_image = artifact_dir / "presentation" / "07_metric_summary.png"
        if summary_image.exists():
            st.image(str(summary_image), use_container_width=True)

    with drift_tab:
        st.markdown("#### 진행 거리별 inter-UAV differential error")
        drift_df = pd.DataFrame(differential_rows)
        if drift_df.empty:
            st.info("저장된 differential sample이 없습니다.")
        else:
            try:
                import altair as alt

                color = alt.Color(
                    "condition:N",
                    title="Condition",
                    scale=alt.Scale(
                        domain=ORACLE_CONDITIONS,
                        range=[ORACLE_COLORS[name] for name in ORACLE_CONDITIONS],
                    ),
                )
                translation_chart = (
                    alt.Chart(drift_df)
                    .mark_line(point=True, strokeWidth=2.5)
                    .encode(
                        x=alt.X("progress_m:Q", title="common progress [m]"),
                        y=alt.Y(
                            "translation_error_m:Q",
                            title="differential translation error [m]",
                            scale=alt.Scale(zero=True),
                        ),
                        color=color,
                        tooltip=[
                            "condition",
                            "progress_m",
                            "translation_error_m",
                        ],
                    )
                    .properties(height=390)
                    .interactive()
                )
                yaw_chart = (
                    alt.Chart(drift_df)
                    .mark_line(point=True, strokeWidth=2.5)
                    .encode(
                        x=alt.X("progress_m:Q", title="common progress [m]"),
                        y=alt.Y(
                            "yaw_error_deg:Q",
                            title="differential yaw error [deg]",
                            scale=alt.Scale(zero=True),
                        ),
                        color=color,
                        tooltip=["condition", "progress_m", "yaw_error_deg"],
                    )
                    .properties(height=390)
                    .interactive()
                )
                chart_columns = st.columns(2)
                chart_columns[0].altair_chart(
                    translation_chart, use_container_width=True
                )
                chart_columns[1].altair_chart(
                    yaw_chart, use_container_width=True
                )
            except Exception as exc:
                st.warning(f"Altair chart 생성 실패: {exc}")
                st.dataframe(
                    drift_df, use_container_width=True, hide_index=True
                )
        st.caption(
            "오차가 있는지는 B0 곡선이 진행하면서 증가하는지로 확인하고, "
            "보정 효과는 O-periodic이 후반 구간에서 B0보다 낮은지로 판단합니다. "
            "translation과 yaw를 반드시 함께 보세요."
        )

    with trajectory_tab:
        st.markdown("#### GT와 조건별 keyframe trajectory")
        trajectory_df = pd.DataFrame(trajectory_rows)
        if trajectory_df.empty:
            st.info("trajectory CSV를 읽지 못했습니다.")
        else:
            available_vehicles = sorted(
                trajectory_df["vehicle"].astype(str).unique().tolist()
            )
            selected_vehicles = st.multiselect(
                "Vehicles",
                available_vehicles,
                default=available_vehicles,
                key="oracle_trajectory_vehicles",
            )
            shown = trajectory_df[
                trajectory_df["vehicle"].isin(selected_vehicles)
            ].copy()
            try:
                import altair as alt

                series_order = ["GT", *ORACLE_CONDITIONS]
                chart = (
                    alt.Chart(shown)
                    .mark_line(point=True, strokeWidth=2.2)
                    .encode(
                        x=alt.X("x:Q", title="x [m]"),
                        y=alt.Y("y:Q", title="y [m]"),
                        color=alt.Color(
                            "series:N",
                            title="Trajectory",
                            scale=alt.Scale(
                                domain=series_order,
                                range=[ORACLE_COLORS[name] for name in series_order],
                            ),
                        ),
                        detail="series:N",
                        order="keyframe:Q",
                        tooltip=[
                            "vehicle",
                            "series",
                            "keyframe",
                            "progress_m",
                            "x",
                            "y",
                        ],
                    )
                    .properties(width=480, height=390)
                    .facet(column=alt.Column("vehicle:N", title=None))
                    .resolve_scale(x="independent", y="independent")
                )
                st.altair_chart(chart, use_container_width=True)
            except Exception as exc:
                st.warning(f"Trajectory chart 생성 실패: {exc}")
                st.dataframe(
                    shown, use_container_width=True, hide_index=True
                )

    with map_tab:
        st.markdown("#### 조건별 occupancy map")
        map_conditions = [*ORACLE_CONDITIONS, "GT-reference"]
        map_tabs = st.tabs(map_conditions)
        for condition, condition_tab in zip(map_conditions, map_tabs):
            with condition_tab:
                map_path = artifact_dir / "maps" / f"{condition}.pgm"
                if map_path.exists():
                    st.image(
                        str(map_path),
                        caption=condition,
                        use_container_width=True,
                    )
                else:
                    st.info(f"{map_path.name}이 없습니다.")
        overlay_image = (
            artifact_dir / "presentation" / "06_map_error_overlay.png"
        )
        if overlay_image.exists():
            st.markdown("#### GT 대비 TP/FP/FN")
            st.image(str(overlay_image), use_container_width=True)
        else:
            st.caption(
                "TP/FP/FN 그림은 presentation renderer 실행 후 표시됩니다."
            )

    with factor_tab:
        st.markdown("#### Oracle inter-UAV factors")
        factor_df = pd.DataFrame(factor_rows)
        if factor_df.empty:
            st.info("저장된 oracle factor가 없습니다.")
        else:
            st.dataframe(
                factor_df, use_container_width=True, hide_index=True
            )
        factor_image = artifact_dir / "presentation" / "04_oracle_factors.png"
        if factor_image.exists():
            st.image(str(factor_image), use_container_width=True)
        overlap_rows = read_csv_rows(artifact_dir / "overlap_audit.csv")
        if overlap_rows:
            st.markdown("#### 실제 scan overlap audit")
            st.dataframe(
                pd.DataFrame(overlap_rows),
                use_container_width=True,
                hide_index=True,
            )

    with raw_tab:
        report_path = artifact_dir / "report.md"
        if report_path.exists():
            st.markdown(report_path.read_text())
        with st.expander("manifest.json"):
            st.json(manifest)
        with st.expander("metrics.json"):
            st.json(metrics)
        st.caption(f"Artifact: `{artifact_dir}`")


def render_lidar_registration_tab(repo_root: Path, pd: Any, st: Any, artifact_dir=None) -> None:
    """Render legacy GT-associated and evaluation-only-GT LiDAR artifacts."""
    st.subheader("LiDAR 기반 inter-UAV 상대 pose 추정")
    runs = [artifact_dir] if artifact_dir is not None else find_lidar_registration_runs(repo_root)
    if not runs:
        st.info(
            "`artifacts/<RUN_ID>/lidar_registration/metrics.json` 형식의 "
            "완료된 분석 결과가 없습니다."
        )
        return
    if artifact_dir is None:
        artifact_dir = st.selectbox(
            "LiDAR registration run", runs,
            format_func=lambda path: run_label(repo_root, path.parent.name),
            key="lidar_registration_run",
        )
    expanded = artifact_dir.parent / "lidar_registration_retrieval_expanded"
    if artifact_dir.name == "lidar_registration" and (expanded / "manifest.json").exists():
        variants = {"기본 검색 · 형상 후보": artifact_dir,
                    "추가 진단 · 형상 + 가까운 후보": expanded}
        choice = st.selectbox("같은 rosbag의 검색 설정", list(variants),
                              key="lidar_variant_" + artifact_dir.parent.name)
        artifact_dir = variants[choice]
        st.caption("추가 진단은 같은 기록에서 후보 검색만 확장한 탐색적 분석입니다. 기존 합격 기준은 유지하며 별도 결과로 보존합니다.")
    metrics = read_json(artifact_dir / "metrics.json")
    manifest = read_json(artifact_dir / "manifest.json")
    no_gt = manifest.get("gt_usage") == "evaluation_only"
    conditions = list(metrics.get("conditions") or {})
    correction = manifest.get("primary_condition", "N-single" if no_gt else "R-periodic")
    if no_gt:
        st.info(
            "이번 결과: GT는 평가 전용입니다. LiDAR 형상 특징과 raw-SLAM 위치 범위로 후보를 찾고, "
            f"이웃 후보의 일관성을 검사한 뒤 구간별 대표 제약으로 비행 종료 후 보정합니다({correction}). "
            "초기 드론 간 좌표계 정렬은 알려져 있다고 가정합니다."
        )
        st.caption("B0=raw SLAM, B1=초기 정렬+외부 odometry. N 계열은 GT 없이 추정하며, GT는 정답 표시용입니다.")
        if correction == "N-double":
            st.markdown("**같은 rosbag 비교:** N-single-A=앞 구간만 · N-single-B=뒤 구간만 · N-double=두 구간 모두. N-prior는 드론 간 제약 없이 초기 정렬만 적용한 대조군입니다.")
            status = manifest.get("optimizer") or {}
            st.dataframe(pd.DataFrame([
                {"조건": name, "요청 제약": opt.get("requested_factor_count"),
                 "채택 제약": opt.get("accepted_factor_count"), "실제 적용": opt.get("applied_factor_count"),
                 "조건 성립": opt.get("available"), "최적화 성공": opt.get("success")}
                for name, opt in status.items()]), hide_index=True, use_container_width=True)
            if not (status.get(correction) or {}).get("available"):
                st.warning("두 독립 구간의 정합 제약을 모두 확보하지 못했습니다. N-double이라는 이름만으로 2회 보정 성공으로 해석하지 마세요.")
    else:
        st.caption(
            "기존 controlled 실험: GT로 비교할 keyframe pair를 선택합니다. 상대 pose 값은 "
            "LiDAR 정합으로 계산하지만 자동 장소 인식 결과는 아닙니다."
        )
    registrations = lidar_registration_rows(artifact_dir)
    if not metrics.get("conditions"):
        st.error(f"조건별 지표를 읽지 못했습니다: {artifact_dir}")
        return

    gate = metrics.get("phase2_gate") or {}
    summaries = metrics.get("registration") or {}
    periodic = summaries.get(correction) or {}
    selected = int(periodic.get("selected_pair_count", 0))
    accepted = int(periodic.get("accepted_factor_count", 0))
    initial_median = (
        periodic.get("initial_translation_error_m") or {}
    ).get("median")
    estimated_median = (
        periodic.get("estimated_translation_error_m") or {}
    ).get("median")
    if no_gt:
        # Rejected retrieval candidates are not correction measurements. Show
        # the quality of the factor actually used, not all failed candidates.
        used = [row for row in registrations if row.get("accepted")]
        before = [row["initial_translation_error_m"] for row in used
                  if row.get("initial_translation_error_m") is not None]
        after = [row["estimated_translation_error_m"] for row in used
                 if row.get("estimated_translation_error_m") is not None]
        initial_median = float(pd.Series(before, dtype=float).median()) if before else None
        estimated_median = float(pd.Series(after, dtype=float).median()) if after else None
    b0_endpoint = (
        (metrics.get("conditions") or {}).get("B0") or {}
    ).get("differential", {}).get("endpoint_translation_m")
    periodic_endpoint = (
        (metrics.get("conditions") or {}).get(correction) or {}
    ).get("differential", {}).get("endpoint_translation_m")

    columns = st.columns(6)
    columns[0].metric("종합 개선 판정", "PASS" if gate.get("pass") else "FAIL")
    columns[1].metric("검사 후보 pair" if no_gt else "선택 pair", selected)
    columns[2].metric("채택 factor", accepted)
    columns[3].metric(
        "채택 pair 보정 전" if no_gt else "초기 median 오차",
        "-" if initial_median is None else f"{float(initial_median):.3f} m",
    )
    columns[4].metric(
        "채택 pair 정합 후" if no_gt else "등록 median 오차",
        "-" if estimated_median is None else f"{float(estimated_median):.3f} m",
    )
    endpoint_text = "-"
    if b0_endpoint is not None and periodic_endpoint is not None:
        endpoint_text = f"{float(b0_endpoint):.3f} → {float(periodic_endpoint):.3f} m"
    columns[5].metric("종단 상대오차", endpoint_text)
    if gate.get("pass"):
        st.success("설정한 평가 기준을 통과했습니다. 반복 실험에서의 일반화는 별도 검증이 필요합니다.")
    else:
        st.warning(
            "종합 평가 기준을 모두 통과하지는 못했습니다. 분석 실행 실패와는 다릅니다. "
            "아래에서 위치·yaw·지도 중 어떤 항목이 개선 또는 악화됐는지 확인하세요."
        )

    summary_tab, registration_tab, drift_tab, map_tab, raw_tab = st.tabs([
        "Summary",
        "Registration",
        "Drift & Trajectory",
        "Maps",
        "Raw Artifact",
    ])
    with summary_tab:
        st.markdown("#### 조건별 보정 결과")
        st.dataframe(
            pd.DataFrame(lidar_metric_rows(metrics)),
            use_container_width=True,
            hide_index=True,
        )
        gate_rows = [
            {"check": name, "result": "PASS" if passed else "FAIL"}
            for name, passed in (gate.get("checks") or {}).items()
        ]
        st.markdown("#### Gate checks")
        st.dataframe(
            pd.DataFrame(gate_rows),
            use_container_width=True,
            hide_index=True,
        )
        if no_gt:
            st.info("해석 범위: 고정 경로·초기 좌표 정렬 기지·오프라인 공통 관측 보정입니다. 통신 최적화나 미지 초기 위치 전역 정합을 입증한 결과는 아닙니다.")
            for name in ("overview_shared_regions.png", "overview_trajectories.png", "overview_errors.png", "overview_map_errors.png"):
                if (artifact_dir / name).exists():
                    st.image(str(artifact_dir / name), use_container_width=True)
            audit = read_csv_rows(artifact_dir / "overlap_audit.csv")
            if audit:
                st.markdown("#### 실제 공통 관측 구간 — GT로 사후 평가, 보정 입력 아님")
                st.dataframe(pd.DataFrame(audit), use_container_width=True, hide_index=True)
        else:
            st.info("논문 해석 범위: GT association을 사용한 controlled registration이며 자동 장소 인식 성능을 의미하지 않습니다.")

    with registration_tab:
        registration_df = pd.DataFrame(registrations)
        st.markdown("#### Pair별 상대 pose 추정과 confidence")
        st.dataframe(
            registration_df,
            use_container_width=True,
            hide_index=True,
        )
        if registrations:
            error_rows = []
            for row in registrations:
                common = {
                    "condition": row.get("condition"),
                    "progress_m": row.get("target_progress_m"),
                    "accepted": row.get("accepted"),
                }
                error_rows.extend([
                    {
                        **common,
                        "source": "B0 initial",
                        "translation_error_m": row.get(
                            "initial_translation_error_m"
                        ),
                    },
                    {
                        **common,
                        "source": "LiDAR estimate",
                        "translation_error_m": row.get(
                            "estimated_translation_error_m"
                        ),
                    },
                ])
            try:
                import altair as alt

                chart = (
                    alt.Chart(pd.DataFrame(error_rows))
                    .mark_line(point=True, strokeWidth=2.3)
                    .encode(
                        x=alt.X("progress_m:Q", title="raw-SLAM source progress [m]" if no_gt else "GT-associated progress [m]"),
                        y=alt.Y(
                            "translation_error_m:Q",
                            title="relative translation error [m]",
                        ),
                        color=alt.Color("source:N", title="Transform"),
                        strokeDash=alt.StrokeDash(
                            "condition:N", title="Condition"
                        ),
                        tooltip=[
                            "condition",
                            "progress_m",
                            "source",
                            "translation_error_m",
                            "accepted",
                        ],
                    )
                    .properties(height=390)
                    .interactive()
                )
                st.altair_chart(chart, use_container_width=True)
            except Exception as exc:
                st.warning(f"Registration chart 생성 실패: {exc}")

        images = sorted((artifact_dir / "debug_plots").glob("*.png"))
        if images:
            st.markdown("#### 서브맵 정합 전/후")
            image_columns = st.columns(2)
            for index, image_path in enumerate(images):
                image_columns[index % 2].image(
                    str(image_path),
                    caption=image_path.stem,
                    use_container_width=True,
                )

    with drift_tab:
        if no_gt:
            st.caption("곡선의 가로축은 공통 raw-SLAM 누적거리입니다. 상단 종단 오차는 두 드론 각각의 마지막 GT 평가 가능 pose를 비교하므로, 길이가 다른 경로에서는 곡선의 마지막 표본과 다를 수 있습니다.")
        drift_rows = lidar_differential_rows(metrics)
        if drift_rows:
            try:
                import altair as alt

                drift_df = pd.DataFrame(drift_rows)
                chart = (
                    alt.Chart(drift_df)
                    .mark_line(point=True, strokeWidth=2.3)
                    .encode(
                        x=alt.X("progress_m:Q", title="common progress [m]"),
                        y=alt.Y(
                            "translation_error_m:Q",
                            title="differential translation error [m]",
                        ),
                        color=alt.Color(
                            "condition:N",
                            scale=alt.Scale(
                                domain=conditions,
                                range=[
                                    LIDAR_COLORS.get(name, "#64748b")
                                    for name in conditions
                                ],
                            ),
                        ),
                        tooltip=[
                            "condition",
                            "progress_m",
                            "translation_error_m",
                            "yaw_error_deg",
                        ],
                    )
                    .properties(height=390)
                    .interactive()
                )
                st.altair_chart(chart, use_container_width=True)
                yaw_chart = chart.encode(y=alt.Y("yaw_error_deg:Q", title="absolute wrapped relative yaw error [deg]"))
                st.altair_chart(yaw_chart, use_container_width=True)
            except Exception as exc:
                st.warning(f"Drift chart 생성 실패: {exc}")

        trajectory_rows = lidar_trajectory_rows(artifact_dir)
        if trajectory_rows:
            try:
                import altair as alt

                trajectory_df = pd.DataFrame(trajectory_rows)
                series = ["GT", *conditions]
                trajectory_chart = (
                    alt.Chart(trajectory_df)
                    .mark_line(point=True, strokeWidth=2.0)
                    .encode(
                        x=alt.X("x:Q", title="x [m]"),
                        y=alt.Y("y:Q", title="y [m]"),
                        color=alt.Color(
                            "series:N",
                            scale=alt.Scale(
                                domain=series,
                                range=[LIDAR_COLORS.get(name, "#64748b") for name in series],
                            ),
                        ),
                        detail="series:N",
                        order="keyframe:Q",
                        tooltip=["vehicle", "series", "progress_m", "x", "y"],
                    )
                    .properties(width=480, height=390)
                    .facet(column=alt.Column("vehicle:N", title=None))
                )
                st.altair_chart(trajectory_chart, use_container_width=True)
            except Exception as exc:
                st.warning(f"Trajectory chart 생성 실패: {exc}")

    with map_tab:
        map_conditions = [*conditions, "GT-reference"]
        map_tabs = st.tabs(map_conditions)
        for condition, condition_tab in zip(map_conditions, map_tabs):
            with condition_tab:
                map_path = artifact_dir / "maps" / f"{condition}.pgm"
                if map_path.exists():
                    st.image(
                        str(map_path),
                        caption=condition,
                        use_container_width=True,
                    )
                else:
                    st.info(f"{map_path.name}이 없습니다.")

    with raw_tab:
        report_path = artifact_dir / "report.md"
        if report_path.exists():
            st.markdown(report_path.read_text())
        with st.expander("manifest.json"):
            st.json(manifest)
        with st.expander("metrics.json"):
            st.json(metrics)
        st.caption(f"Artifact: `{artifact_dir}`")


def render_legacy_dashboard(repo_root: Path) -> None:
    try:
        import pandas as pd
        import streamlit as st
    except Exception as exc:
        raise SystemExit(
            "Streamlit dashboard dependencies are missing. Install with: "
            "python3 -m pip install -r requirements-dashboard.txt"
        ) from exc

    st.subheader("기존 상세 분석 도구")
    st.warning("과거 경로계획·지도 디버그 기능입니다. 다른 경로/조건의 평균은 통제된 성능 비교가 아닙니다. 기본 화면은 왼쪽 '실험 보기'에서 확인하세요.")

    artifacts = load_artifact_records(repo_root)
    registry = load_registry_records(repo_root)
    run_reports = load_run_reports(repo_root)
    df = pd.DataFrame(artifacts)

    with st.sidebar:
        st.header("Data")
        st.write(f"Repo root: `{repo_root}`")
        st.write(f"Artifact runs: `{len(artifacts)}`")
        if st.button("Refresh"):
            st.rerun()

        if df.empty:
            st.warning("No artifact records found.")
            return

        scenarios = sorted(df["scenario_id"].dropna().unique().tolist())
        algorithms = sorted(df["algorithm_label"].dropna().unique().tolist())
        conditions = sorted(df["condition_id"].dropna().unique().tolist())
        stages = sorted(df["experiment_stage"].dropna().unique().tolist())
        results = sorted(df["result"].dropna().unique().tolist())
        selected_scenarios = st.multiselect("Scenario", scenarios, default=scenarios)
        selected_algorithms = st.multiselect("Algorithm", algorithms, default=algorithms)
        selected_conditions = st.multiselect("Condition", conditions, default=conditions)
        selected_stages = st.multiselect("Stage", stages, default=stages)
        selected_results = st.multiselect("Result", results, default=results)
        run_query = st.text_input("실험 이름 / ID 검색", "")

    filtered = df[
        df["scenario_id"].isin(selected_scenarios)
        & df["algorithm_label"].isin(selected_algorithms)
        & df["condition_id"].isin(selected_conditions)
        & df["experiment_stage"].isin(selected_stages)
        & df["result"].isin(selected_results)
    ].copy()
    if run_query:
        searchable = filtered["run_id"].astype(str).map(lambda rid: run_label(repo_root, rid) + " " + rid)
        filtered = filtered[searchable.str.contains(run_query, case=False, na=False, regex=False)]

    total_runs = len(filtered)
    completed = filtered[filtered["result"] != "in_progress"]
    success_rate = float(completed["success"].fillna(False).mean() * 100.0) if not completed.empty else math.nan
    mean_mission_time = filtered["mission_time_s"].dropna().mean() if "mission_time_s" in filtered else math.nan
    mean_path_length = filtered["actual_path_length_m"].dropna().mean() if "actual_path_length_m" in filtered else math.nan

    kpi1, kpi2, kpi3, kpi4 = st.columns(4)
    kpi1.metric("Runs", total_runs)
    kpi2.metric("Success rate (completed)", "-" if math.isnan(success_rate) else f"{success_rate:.1f}%")
    kpi3.metric("Mean mission time", "-" if math.isnan(mean_mission_time) else f"{mean_mission_time:.2f}s")
    kpi4.metric("Mean path length", "-" if math.isnan(mean_path_length) else f"{mean_path_length:.2f}m")

    unknown_conditions = int((df["condition_id"] == "unknown").sum())
    unknown_scenarios = int((df["scenario_id"] == "unknown").sum())
    if unknown_conditions or unknown_scenarios:
        st.warning(
            "Some historical artifacts do not contain quantification schema fields: "
            f"unknown condition={unknown_conditions}, unknown scenario={unknown_scenarios}. "
            "Future A*-vs-MPPI runs should write condition_id/scenario_id into metadata and paper_metrics."
        )

    tabs = st.tabs([
        "Overview",
        "Compare",
        "Trajectory Overlay",
        "Runs",
        "Ledger",
        "Artifacts",
        "SLAM & Fusion Debug",
        "Oracle Drift Correction",
        "LiDAR Registration",
        "Implementation Report",
    ])

    with tabs[0]:
        st.subheader("Core KPI 기준")
        st.dataframe(pd.DataFrame(KPI_GUIDE), use_container_width=True)
        st.subheader("Supplementary KPI 기준")
        st.dataframe(
            pd.DataFrame(SUPPLEMENTARY_KPI_GUIDE),
            use_container_width=True,
        )
        st.subheader("Condition counts")
        if not filtered.empty:
            counts = filtered.groupby(["scenario_id", "algorithm_label", "condition_id", "result"]).size().reset_index(name="runs")
            st.dataframe(counts, use_container_width=True)
        st.subheader("Existing registry tables")
        c1, c2 = st.columns(2)
        c1.write("experiments/index.csv")
        c1.dataframe(
            pd.DataFrame(registry["index"]), use_container_width=True
        )
        c2.write("experiments/scenario_table.csv")
        c2.dataframe(
            pd.DataFrame(registry["scenario_table"]),
            use_container_width=True,
        )

    with tabs[1]:
        st.subheader("Core KPI summary by condition")
        core_summary_df = pd.DataFrame(metric_summary(filtered.to_dict("records"), CORE_METRICS))
        st.dataframe(core_summary_df, use_container_width=True)

        st.subheader("Supplementary KPI summary by condition")
        supplementary_metrics = st.multiselect(
            "Supplementary metrics",
            SUPPLEMENTARY_METRICS,
            default=SUPPLEMENTARY_METRICS,
        )
        supplementary_summary_df = pd.DataFrame(metric_summary(filtered.to_dict("records"), supplementary_metrics))
        st.dataframe(
            supplementary_summary_df, use_container_width=True
        )

        numeric_summary = pd.concat([core_summary_df, supplementary_summary_df], ignore_index=True)
        numeric_summary = numeric_summary[numeric_summary["mean"].notna()].copy()
        if not numeric_summary.empty:
            selected_metric = st.selectbox("Chart metric", sorted(numeric_summary["metric"].unique()))
            chart_df = numeric_summary[numeric_summary["metric"] == selected_metric][["condition_id", "mean"]]
            chart_df = chart_df.set_index("condition_id")
            st.bar_chart(chart_df)

    with tabs[2]:
        st.subheader("Trajectory overlay")
        if filtered.empty:
            st.info("No runs match the current filter.")
        else:
            # Keep the initial render bounded while still showing both vehicles
            # from the latest multi-UAV experiment by default.
            default_runs = filtered["run_id"].astype(str).tail(2).tolist()
            run_label_lookup = dict(
                zip(
                    filtered["run_id"].astype(str).tolist(),
                    filtered["trajectory_label"].astype(str).tolist(),
                )
            )
            selected_runs = st.multiselect(
                "Runs to overlay",
                filtered["run_id"].astype(str).tolist(),
                default=default_runs,
                format_func=lambda run_id: run_label_lookup.get(str(run_id), str(run_id)),
            )
            selected_records = filtered[filtered["run_id"].astype(str).isin(selected_runs)].copy()
            default_world = repo_root / "sim_assets" / "worlds" / "obstacle_demo.world"
            map_candidates = [
                str(path)
                for path in sorted({
                    Path(str(value))
                    for value in selected_records.get("map_path", [])
                    if str(value).strip() and Path(str(value)).exists()
                })
            ]
            if default_world.exists() and str(default_world) not in map_candidates:
                map_candidates.append(str(default_world))
            render_map = st.checkbox("Render world map", value=True)
            selected_map_path = ""
            if render_map:
                if map_candidates:
                    selected_map_path = st.selectbox("World map file", map_candidates)
                else:
                    selected_map_path = st.text_input("World map file", str(default_world))

            selected_map_sha256 = ""
            if render_map and selected_map_path:
                try:
                    selected_map_sha256 = file_sha256(Path(selected_map_path))
                    st.caption(f"World SHA-256: `{selected_map_sha256}`")
                except Exception as exc:
                    st.error(f"Could not hash world map `{selected_map_path}`: {exc}")

            if render_map and selected_map_sha256:
                recorded_hashes = {
                    str(value)
                    for value in selected_records.get("world_sha256", [])
                    if str(value).strip()
                }
                mismatched = selected_records[
                    selected_records["world_sha256"].astype(str).str.strip().ne("")
                    & selected_records["world_sha256"].astype(str).ne(selected_map_sha256)
                ]
                unverified = selected_records[
                    selected_records["world_sha256"].astype(str).str.strip().eq("")
                ]
                if recorded_hashes and mismatched.empty:
                    st.success("Selected verified runs use the displayed world snapshot.")
                if not mismatched.empty:
                    st.error(
                        "Excluded runs whose recorded world hash differs from the displayed map: "
                        + ", ".join(mismatched["run_id"].astype(str).tolist())
                    )
                if not unverified.empty:
                    st.warning(
                        "Showing legacy runs without a recorded world hash as unverified: "
                        + ", ".join(unverified["run_id"].astype(str).tolist())
                    )
                selected_records = selected_records[
                    selected_records["world_sha256"].astype(str).str.strip().eq("")
                    | selected_records["world_sha256"].astype(str).eq(selected_map_sha256)
                ].copy()

            load_trajectory_data = st.checkbox(
                "Load selected trajectory data",
                value=False,
                help="Enable after selecting runs. Large live CSV files are sampled before plotting.",
            )
            trajectory_records = (
                selected_records
                if load_trajectory_data
                else selected_records.iloc[0:0]
            )
            trajectory_frames = []
            missing_world_transforms = []
            per_run_max_points = max(500, 5000 // max(1, len(trajectory_records)))
            for _, record in trajectory_records.iterrows():
                trajectory_path = Path(str(record.get("trajectory_path", "")))
                if not trajectory_path.exists():
                    continue
                try:
                    frame = pd.read_csv(trajectory_path)
                except Exception:
                    continue
                if frame.empty or not {"x", "y"}.issubset(frame.columns):
                    continue
                frame = frame.copy()
                frame["x"] = pd.to_numeric(frame["x"], errors="coerce")
                frame["y"] = pd.to_numeric(frame["y"], errors="coerce")
                frame = frame.dropna(subset=["x", "y"])
                if frame.empty:
                    continue
                if len(frame) > per_run_max_points:
                    sample_indices = [
                        round(index * (len(frame) - 1) / (per_run_max_points - 1))
                        for index in range(per_run_max_points)
                    ]
                    frame = frame.iloc[sample_indices].copy()
                frame["local_x"] = frame["x"]
                frame["local_y"] = frame["y"]
                transform = record.get("world_from_local")
                if isinstance(transform, dict):
                    yaw = float(transform["yaw"])
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    frame["world_x"] = (
                        float(transform["x"])
                        + cos_yaw * frame["local_x"]
                        - sin_yaw * frame["local_y"]
                    )
                    frame["world_y"] = (
                        float(transform["y"])
                        + sin_yaw * frame["local_x"]
                        + cos_yaw * frame["local_y"]
                    )
                    frame["x"] = frame["world_x"]
                    frame["y"] = frame["world_y"]
                    frame["coordinate_frame"] = str(
                        record.get("world_frame_id", "world")
                    )
                elif render_map:
                    missing_world_transforms.append(str(record["run_id"]))
                    continue
                else:
                    frame["world_x"] = None
                    frame["world_y"] = None
                    frame["coordinate_frame"] = str(
                        record.get("trajectory_frame_id", "local")
                    )
                frame["run_id"] = str(record["run_id"])
                frame["condition_id"] = str(record["condition_id"])
                frame["algorithm_label"] = str(record.get("algorithm_label", "Unknown"))
                frame["planner_name"] = str(record.get("planner_name", "unknown"))
                frame["trajectory_label"] = str(record.get("trajectory_label", record["run_id"]))
                trajectory_frames.append(frame)

            if missing_world_transforms:
                st.warning(
                    "Skipped map overlay for runs without a local-to-world transform: "
                    + ", ".join(sorted(set(missing_world_transforms)))
                )
            if not load_trajectory_data:
                st.info("Enable 'Load selected trajectory data' to render the overlay.")
            elif not trajectory_frames:
                st.warning("Selected runs do not have readable trajectory.csv files.")
            else:
                trajectory_df = pd.concat(trajectory_frames, ignore_index=True)
                max_points = 5000
                if len(trajectory_df) > max_points:
                    stride = max(1, len(trajectory_df) // max_points)
                    trajectory_df = trajectory_df.iloc[::stride].copy()

                chart_df = trajectory_df[
                    [
                        col
                        for col in [
                            "x",
                            "y",
                            "condition_id",
                            "run_id",
                            "algorithm_label",
                            "planner_name",
                            "trajectory_label",
                            "mission_phase",
                            "coordinate_frame",
                            "local_x",
                            "local_y",
                            "world_x",
                            "world_y",
                        ]
                        if col in trajectory_df.columns
                    ]
                ].dropna(subset=["x", "y"])
                chart_df = chart_df.copy()
                chart_df["x"] = pd.to_numeric(chart_df["x"], errors="coerce")
                chart_df["y"] = pd.to_numeric(chart_df["y"], errors="coerce")
                chart_df = chart_df.dropna(subset=["x", "y"])
                chart_df["run_label"] = chart_df["trajectory_label"].astype(str)
                try:
                    import altair as alt

                    rect_df = pd.DataFrame()
                    outline_df = pd.DataFrame()
                    map_path = Path(selected_map_path) if selected_map_path else None
                    if render_map and map_path and map_path.exists():
                        try:
                            geometry = load_world_geometry(repo_root, map_path)
                            rect_df = pd.DataFrame(geometry["rects"])
                            outline_df = pd.DataFrame(geometry["outlines"])
                            st.caption(f"Map overlay: `{map_path}`")
                        except Exception as exc:
                            st.warning(f"Could not render world map `{map_path}`: {exc}")

                    x_values = chart_df["x"].dropna().tolist()
                    y_values = chart_df["y"].dropna().tolist()
                    if not rect_df.empty:
                        x_values.extend(rect_df["x1"].dropna().tolist())
                        x_values.extend(rect_df["x2"].dropna().tolist())
                        y_values.extend(rect_df["y1"].dropna().tolist())
                        y_values.extend(rect_df["y2"].dropna().tolist())
                    if not outline_df.empty:
                        x_values.extend(outline_df["x"].dropna().tolist())
                        y_values.extend(outline_df["y"].dropna().tolist())

                    x_domain = None
                    y_domain = None
                    if x_values and y_values:
                        x_min = min(x_values)
                        x_max = max(x_values)
                        y_min = min(y_values)
                        y_max = max(y_values)
                        x_pad = max((x_max - x_min) * 0.03, 1.0)
                        y_pad = max((y_max - y_min) * 0.08, 1.0)
                        x_domain = [x_min - x_pad, x_max + x_pad]
                        y_domain = [y_min - y_pad, y_max + y_pad]

                    x_axis = alt.X("x:Q", title="x [m]", scale=alt.Scale(domain=x_domain) if x_domain else alt.Undefined)
                    y_axis = alt.Y("y:Q", title="y [m]", scale=alt.Scale(domain=y_domain) if y_domain else alt.Undefined)
                    layers = []

                    if not rect_df.empty:
                        floor_df = rect_df[rect_df["kind"] == "floor"].copy()
                        wall_df = rect_df[rect_df["kind"] != "floor"].copy()
                        if not floor_df.empty:
                            layers.append(
                                alt.Chart(floor_df)
                                .mark_rect(color="#eef2f7", opacity=0.9)
                                .encode(
                                    x=alt.X("x1:Q", title="x [m]", scale=alt.Scale(domain=x_domain) if x_domain else alt.Undefined),
                                    x2="x2:Q",
                                    y=alt.Y("y1:Q", title="y [m]", scale=alt.Scale(domain=y_domain) if y_domain else alt.Undefined),
                                    y2="y2:Q",
                                    tooltip=["geometry_id", "kind"],
                                )
                            )
                        if not wall_df.empty:
                            layers.append(
                                alt.Chart(wall_df)
                                .mark_rect(color="#9ca3af", opacity=0.72)
                                .encode(
                                    x=alt.X("x1:Q", title="x [m]", scale=alt.Scale(domain=x_domain) if x_domain else alt.Undefined),
                                    x2="x2:Q",
                                    y=alt.Y("y1:Q", title="y [m]", scale=alt.Scale(domain=y_domain) if y_domain else alt.Undefined),
                                    y2="y2:Q",
                                    tooltip=["geometry_id", "kind"],
                                )
                            )

                    if not outline_df.empty:
                        layers.append(
                            alt.Chart(outline_df)
                            .mark_line(color="#111827", strokeWidth=1.1, opacity=0.85)
                            .encode(
                                x=x_axis,
                                y=y_axis,
                                detail="geometry_id:N",
                                order="order:Q",
                                tooltip=["geometry_id", "kind"],
                            )
                        )

                    trajectory_layer = (
                        alt.Chart(chart_df)
                        .mark_line(point=False, strokeWidth=2.4)
                        .encode(
                            x=x_axis,
                            y=y_axis,
                            color=alt.Color("run_label:N", title="Trajectory"),
                            detail="run_id:N",
                            tooltip=[
                                "run_label",
                                "algorithm_label",
                                "planner_name",
                                "run_id",
                                "condition_id",
                                "mission_phase",
                                "coordinate_frame",
                                "local_x",
                                "local_y",
                                "world_x",
                                "world_y",
                                "x",
                                "y",
                            ],
                        )
                    )
                    layers.append(trajectory_layer)

                    marker_rows = []
                    for run_id, group in chart_df.groupby("run_id", sort=False):
                        if group.empty:
                            continue
                        first = group.iloc[0]
                        last = group.iloc[-1]
                        marker_rows.append({"run_label": first["run_label"], "marker": "start", "x": first["x"], "y": first["y"]})
                        marker_rows.append({"run_label": last["run_label"], "marker": "end", "x": last["x"], "y": last["y"]})
                    marker_df = pd.DataFrame(marker_rows)
                    if not marker_df.empty:
                        layers.append(
                            alt.Chart(marker_df)
                            .mark_point(filled=True, size=90)
                            .encode(
                                x=x_axis,
                                y=y_axis,
                                color=alt.Color("run_label:N", title="Trajectory"),
                                shape=alt.Shape("marker:N", title="Marker"),
                                tooltip=["run_label", "marker", "x", "y"],
                            )
                        )

                    chart = alt.layer(*layers).properties(height=620).interactive()
                    st.altair_chart(chart, use_container_width=True)
                except Exception:
                    st.scatter_chart(chart_df, x="x", y="y", color="run_label")

                st.dataframe(
                    trajectory_df[
                        [
                            col
                            for col in [
                                "run_id",
                                "algorithm_label",
                                "condition_id",
                                "planner_name",
                                "trajectory_label",
                                "mission_phase",
                                "coordinate_frame",
                                "t_sec",
                                "local_x",
                                "local_y",
                                "world_x",
                                "world_y",
                                "x",
                                "y",
                                "z",
                                "speed_mps",
                                "nearest_obstacle_m",
                            ]
                            if col in trajectory_df.columns
                        ]
                    ].tail(500),
                    use_container_width=True,
                )

    with tabs[3]:
        st.subheader("Runs")
        identity_cols = [
            "run_id",
            "vehicle_id",
            "algorithm_label",
            "trajectory_label",
            "scenario_id",
            "condition_id",
            "planner_name",
            "planner_family",
            "map_source",
            "result",
            "experiment_stage",
            "trial_index",
            "seed",
        ]
        core_cols = [
            "mission_time_s",
            "final_goal_distance_m",
            "actual_path_length_m",
            "min_obstacle_distance_m",
            "compute_latency_ms_p95",
            "global_planning_time_ms_p95",
        ]
        supplementary_cols = [
            "runtime_s",
            "outbound_time_s",
            "return_time_s",
            "total_path_length_m",
            "outbound_path_length_m",
            "return_path_length_m",
            "straight_line_distance_m",
            "path_efficiency",
            "safety_intervention_count",
            "safety_event_count",
            "control_effort",
            "command_smoothness",
            "planning_time_ms_p50",
            "planning_time_ms_p95",
            "replan_count",
            "planner_cmd_count",
            "safe_cmd_count",
            "pose_count",
            "pose_period_p99_s",
            "map_coverage",
            "fusion_state",
            "fusion_map_version",
            "fusion_source_count",
            "fusion_latency_p95_ms",
            "fusion_conflict_ratio",
            "artifact_path",
        ]
        st.write("Core KPI per run")
        st.dataframe(
            filtered[
                [
                    col
                    for col in identity_cols + core_cols
                    if col in filtered.columns
                ]
            ],
            use_container_width=True,
        )
        st.write("Supplementary KPI per run")
        st.dataframe(
            filtered[
                [
                    col
                    for col in identity_cols + supplementary_cols
                    if col in filtered.columns
                ]
            ],
            use_container_width=True,
        )

    with tabs[4]:
        st.subheader("Ledger")
        st.dataframe(
            pd.DataFrame(registry["ledger"]), use_container_width=True
        )

    with tabs[5]:
        st.subheader("Artifact drill-down")
        if filtered.empty:
            st.info("No runs match the current filter.")
            return
        selected_run = st.selectbox("Run", filtered["run_id"].astype(str).tolist(),
                                    format_func=lambda rid: run_label(repo_root, rid))
        record = filtered[filtered["run_id"].astype(str) == selected_run].iloc[0].to_dict()
        st.json(record)
        artifact_path = Path(str(record["artifact_path"]))
        for label, filename in [
            ("paper_metrics.json", "paper_metrics.json"),
            ("summary.json", "summary.json"),
            ("metadata.json", "metadata.json"),
            ("phase_summary.json", "phase_summary.json"),
        ]:
            with st.expander(label):
                st.json(read_json(artifact_path / filename))
        swarm_summary_path = artifact_path.parent / "swarm_summary.json"
        if swarm_summary_path.exists():
            with st.expander("swarm_summary.json"):
                st.json(read_json(swarm_summary_path))

    with tabs[6]:
        st.subheader("SLAM 생성 과정 및 맵 융합 디버그")
        st.caption(
            "각 드론의 로컬 SLAM 맵을 기록된 world_from_local 변환으로 world 좌표에 투영하고, "
            "같은 시점의 중앙 융합 맵과 겹쳐 표시합니다."
        )
        map_run_roots = find_map_debug_runs(repo_root)
        if not map_run_roots:
            st.info("저장된 맵 스냅샷이 없습니다. 새 실행부터 30초 간격 이력이 기록됩니다.")
        else:
            debug_run_root = st.selectbox(
                "Map run",
                map_run_roots,
                format_func=lambda path: run_label(repo_root, path.name),
            )
            snapshots = discover_map_snapshots(debug_run_root)
            layer_ids = sorted({str(item["layer_id"]) for item in snapshots})
            default_layers = [
                layer for layer in layer_ids
                if layer.endswith("/slam")
                or layer.endswith("/known_pose")
                or "/fused_" in layer
            ]
            if not default_layers:
                default_layers = layer_ids
            selected_layers = st.multiselect(
                "Layers",
                layer_ids,
                default=default_layers,
                help="known_pose_reference는 SLAM 오차를 비교할 때 선택하면 됩니다.",
            )
            history_indices = sorted({
                int(item["snapshot_index"])
                for item in snapshots
                if item["stage"] == "history" and item["snapshot_index"] is not None
            })
            snapshot_options = ["latest", *[f"history #{index:04d}" for index in history_indices]]
            selected_stage = st.selectbox(
                "Snapshot",
                snapshot_options,
                help="각 recorder가 약 30초마다 같은 순번의 스냅샷을 저장합니다.",
            )
            selected_index = (
                None if selected_stage == "latest" else int(selected_stage.rsplit("#", 1)[1])
            )
            selected_snapshots = []
            for layer in selected_layers:
                candidates = [item for item in snapshots if item["layer_id"] == layer]
                if selected_index is None:
                    candidates = [item for item in candidates if item["stage"] == "latest"]
                else:
                    candidates = [
                        item for item in candidates
                        if item["stage"] == "history"
                        and item["snapshot_index"] == selected_index
                    ]
                if candidates:
                    selected_snapshots.append(candidates[-1])

            if not selected_snapshots:
                st.warning("선택한 순번에 해당하는 맵 레이어가 없습니다.")
            else:
                include_non_occupied = st.checkbox(
                    "Free/uncertain cells도 표시",
                    value=False,
                    help="기본값은 장애물(occupancy >= 65)만 표시해 겹침을 선명하게 봅니다.",
                )
                max_points_per_layer = st.selectbox(
                    "Layer당 최대 표시 셀",
                    [5000, 10000, 20000, 40000],
                    index=1,
                )
                run_records = [
                    record for record in artifacts
                    if str(record.get("source_run_id")) == debug_run_root.name
                ]
                transforms: dict[str, dict[str, float]] = {"swarm": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
                transform_sources: dict[str, str] = {"swarm": "global map (identity)"}
                for record in run_records:
                    vehicle_id = str(record.get("vehicle_id", ""))
                    transform = _normalized_world_from_local(record.get("world_from_local"))
                    if transform is not None:
                        transforms[vehicle_id] = transform
                        transform_sources[vehicle_id] = str(record.get("world_transform_source", "metadata"))
                for snapshot in selected_snapshots:
                    vehicle_id = str(snapshot["vehicle_id"])
                    if vehicle_id in transforms:
                        continue
                    metadata = read_json(debug_run_root / vehicle_id / "metadata.json")
                    transform = _normalized_world_from_local(metadata.get("world_from_local"))
                    if transform is not None:
                        transforms[vehicle_id] = transform
                        transform_sources[vehicle_id] = "metadata.json"

                point_rows: list[dict[str, Any]] = []
                snapshot_summary_rows = []
                plotted_snapshots = []
                missing_transforms = []
                for snapshot in selected_snapshots:
                    vehicle_id = str(snapshot["vehicle_id"])
                    transform = transforms.get(vehicle_id)
                    if transform is None:
                        missing_transforms.append(vehicle_id)
                        continue
                    try:
                        rows, map_summary = occupancy_snapshot_points(
                            snapshot,
                            transform,
                            include_non_occupied=include_non_occupied,
                            max_points=int(max_points_per_layer),
                        )
                    except Exception as exc:
                        st.error(f"{snapshot['layer_id']} 로드 실패: {exc}")
                        continue
                    point_rows.extend(rows)
                    map_summary["transform_source"] = transform_sources.get(vehicle_id, "unknown")
                    snapshot_summary_rows.append(map_summary)
                    plotted_snapshots.append(snapshot)
                if missing_transforms:
                    st.warning(
                        "world_from_local 변환이 없어 제외된 vehicle: "
                        + ", ".join(sorted(set(missing_transforms)))
                    )

                conflict_rows: list[dict[str, float]] = []
                conflict_summary: dict[str, int] = {}
                fused_snapshots = [
                    item for item in plotted_snapshots
                    if str(item["vehicle_id"]) == "swarm" or str(item["source"]).startswith("fused_")
                ]
                fused_input_source = ""
                if fused_snapshots:
                    fused_input_source = str(
                        fused_snapshots[0]["source"]
                    )
                    fused_input_source = remove_prefix(
                        fused_input_source, "fused_"
                    )
                source_snapshots = [
                    (item, transforms[str(item["vehicle_id"])])
                    for item in plotted_snapshots
                    if str(item["vehicle_id"]) != "swarm"
                    and (
                        not fused_input_source
                        or str(item["source"]) == fused_input_source
                    )
                    and not str(item["source"]).startswith("known_pose_reference")
                ]
                diagnostic_slam_snapshots = [
                    (item, transforms[str(item["vehicle_id"])])
                    for item in plotted_snapshots
                    if str(item["vehicle_id"]) != "swarm"
                    and str(item["source"]) == "slam"
                    and fused_input_source != "slam"
                ]
                if fused_snapshots and len(source_snapshots) >= 2:
                    try:
                        conflict_rows, conflict_summary = projected_conflict_points(
                            source_snapshots, fused_snapshots[0]
                        )
                    except Exception as exc:
                        st.warning(f"공간 충돌 셀 계산 실패: {exc}")

                if snapshot_summary_rows:
                    summary_df = pd.DataFrame(snapshot_summary_rows)
                    s1, s2, s3, s4, s5 = st.columns(5)
                    s1.metric("표시 레이어", len(snapshot_summary_rows))
                    s2.metric("융합 입력 맵", len(source_snapshots))
                    s3.metric("Raw SLAM 진단", len(diagnostic_slam_snapshots))
                    s4.metric("입력 중첩 셀", conflict_summary.get("source_overlap_cells", "-"))
                    s5.metric("입력 충돌 셀", conflict_summary.get("conflict_cells", "-"))
                    st.dataframe(summary_df, use_container_width=True)

                if fused_snapshots and source_snapshots:
                    try:
                        import io
                        from PIL import Image, ImageDraw, ImageFont

                        target_snapshot = fused_snapshots[0]
                        fused_grid = np.load(
                            target_snapshot["grid_path"], allow_pickle=False
                        )
                        target_meta = target_snapshot["meta"]
                        target_origin = target_meta.get("origin") or {}
                        target_yaw = float(target_origin.get("yaw", 0.0))
                        if abs(target_yaw) > 1e-6:
                            raise ValueError(
                                "raster view currently requires a zero-yaw fused grid"
                            )
                        resolution = float(target_meta["resolution"])
                        origin_x = float(target_origin.get("x", 0.0))
                        origin_y = float(target_origin.get("y", 0.0))
                        extent = [
                            origin_x,
                            origin_x + fused_grid.shape[1] * resolution,
                            origin_y,
                            origin_y + fused_grid.shape[0] * resolution,
                        ]
                        base = np.full((*fused_grid.shape, 3), 173, dtype=np.uint8)
                        base[fused_grid >= 0] = (255, 255, 255)
                        base[(fused_grid > 25) & (fused_grid < 65)] = (209, 209, 209)
                        base[fused_grid >= 65] = (8, 8, 8)

                        vehicle_palette = {
                            "drone1": (25, 118, 210),
                            "drone2": (245, 124, 0),
                            "drone3": (46, 125, 50),
                            "drone4": (142, 36, 170),
                        }
                        coverage_count = np.zeros(fused_grid.shape, dtype=np.int16)
                        source_occupied_masks: dict[str, np.ndarray] = {}
                        for source_snapshot, transform in source_snapshots:
                            vehicle_id = str(source_snapshot["vehicle_id"])
                            color = np.asarray(
                                vehicle_palette.get(vehicle_id, (0, 137, 123)),
                                dtype=np.float32,
                            )
                            projected = project_snapshot_to_target(
                                source_snapshot, transform, target_snapshot
                            )
                            observed = projected >= 0
                            coverage_count[observed] += 1
                            base[observed] = (
                                base[observed].astype(np.float32) * 0.84 + color * 0.16
                            ).astype(np.uint8)
                            source_occupied_masks[vehicle_id] = projected >= 65

                        overlap = coverage_count > 1
                        overlap_color = np.asarray((123, 31, 162), dtype=np.float32)
                        base[overlap] = (
                            base[overlap].astype(np.float32) * 0.88
                            + overlap_color * 0.12
                        ).astype(np.uint8)

                        # A 0.1 m occupied cell is only one pixel in the saved map.
                        # Draw a per-source color halo first, then restore a thicker
                        # black fused core so the map remains visible under 4 px paths.
                        for vehicle_id, occupied in source_occupied_masks.items():
                            color = np.asarray(
                                vehicle_palette.get(vehicle_id, (0, 137, 123)),
                                dtype=np.float32,
                            )
                            occupied_halo = dilate_boolean_mask(occupied, radius=2)
                            base[occupied_halo] = (
                                base[occupied_halo].astype(np.float32) * 0.15
                                + color * 0.85
                            ).astype(np.uint8)
                        raw_slam_vehicle_ids = []
                        for raw_snapshot, transform in diagnostic_slam_snapshots:
                            vehicle_id = str(raw_snapshot["vehicle_id"])
                            color = np.asarray(
                                vehicle_palette.get(vehicle_id, (0, 137, 123)),
                                dtype=np.float32,
                            )
                            projected = project_snapshot_to_target(
                                raw_snapshot, transform, target_snapshot
                            )
                            raw_halo = dilate_boolean_mask(
                                projected >= 65, radius=1
                            )
                            base[raw_halo] = (
                                base[raw_halo].astype(np.float32) * 0.18
                                + color * 0.82
                            ).astype(np.uint8)
                            raw_slam_vehicle_ids.append(vehicle_id)
                        fused_occupied = dilate_boolean_mask(
                            fused_grid >= 65, radius=1
                        )
                        base[fused_occupied] = (8, 8, 8)

                        map_image = Image.fromarray(np.flipud(base), mode="RGB")
                        left_margin, top_margin = 72, 48
                        bottom_margin, right_margin = 92, 24
                        canvas = Image.new(
                            "RGB",
                            (
                                map_image.width + left_margin + right_margin,
                                map_image.height + top_margin + bottom_margin,
                            ),
                            "white",
                        )
                        canvas.paste(map_image, (left_margin, top_margin))
                        draw = ImageDraw.Draw(canvas)
                        try:
                            title_font = ImageFont.load_default(size=18)
                            label_font = ImageFont.load_default(size=14)
                            small_font = ImageFont.load_default(size=12)
                        except TypeError:
                            title_font = label_font = small_font = ImageFont.load_default()

                        draw.text(
                            (left_margin, 14),
                            f"2-UAV Map Inputs, Raw SLAM and Fusion - {debug_run_root.name} / {selected_stage}",
                            fill=(20, 20, 20),
                            font=title_font,
                        )
                        map_left, map_top = left_margin, top_margin
                        map_right = map_left + map_image.width - 1
                        map_bottom = map_top + map_image.height - 1
                        draw.rectangle(
                            (map_left, map_top, map_right, map_bottom),
                            outline=(45, 45, 45),
                            width=2,
                        )

                        def world_to_pixel(x_value: float, y_value: float) -> tuple[int, int]:
                            pixel_x = map_left + int(round((x_value - origin_x) / resolution))
                            pixel_y = map_top + map_image.height - 1 - int(
                                round((y_value - origin_y) / resolution)
                            )
                            return pixel_x, pixel_y

                        x_tick = math.ceil(extent[0] / 20.0) * 20.0
                        while x_tick <= extent[1]:
                            px, _ = world_to_pixel(x_tick, origin_y)
                            draw.line((px, map_top, px, map_bottom), fill=(190, 190, 190), width=1)
                            draw.text(
                                (px - 10, map_bottom + 8),
                                f"{x_tick:g}",
                                fill=(40, 40, 40),
                                font=small_font,
                            )
                            x_tick += 20.0
                        y_tick = math.ceil(extent[2] / 5.0) * 5.0
                        while y_tick <= extent[3]:
                            _, py = world_to_pixel(origin_x, y_tick)
                            draw.line((map_left, py, map_right, py), fill=(205, 205, 205), width=1)
                            draw.text(
                                (10, py - 7),
                                f"{y_tick:g}",
                                fill=(40, 40, 40),
                                font=small_font,
                            )
                            y_tick += 5.0
                        draw.text(
                            ((map_left + map_right) // 2 - 35, map_bottom + 30),
                            "World X (m)",
                            fill=(25, 25, 25),
                            font=label_font,
                        )
                        draw.text(
                            (6, map_top - 22),
                            "World Y (m)",
                            fill=(25, 25, 25),
                            font=label_font,
                        )

                        snapshot_elapsed = max(
                            [
                                float(item["elapsed_sec"])
                                for item in plotted_snapshots
                                if item.get("elapsed_sec") is not None
                            ],
                            default=None,
                        )
                        source_vehicle_ids = {
                            str(item[0]["vehicle_id"]) for item in source_snapshots
                        }
                        trajectory_legend = []
                        for record in run_records:
                            vehicle_id = str(record.get("vehicle_id", ""))
                            if vehicle_id not in source_vehicle_ids:
                                continue
                            trajectory_path = Path(str(record.get("trajectory_path", "")))
                            transform = transforms.get(vehicle_id)
                            if not trajectory_path.exists() or transform is None:
                                continue
                            trajectory = pd.read_csv(trajectory_path)
                            if trajectory.empty or not {"x", "y"}.issubset(trajectory.columns):
                                continue
                            for column in ["x", "y", "t_sec"]:
                                if column in trajectory:
                                    trajectory[column] = pd.to_numeric(
                                        trajectory[column], errors="coerce"
                                    )
                            trajectory = trajectory.dropna(subset=["x", "y"])
                            if (
                                selected_index is not None
                                and snapshot_elapsed is not None
                                and "t_sec" in trajectory
                            ):
                                trajectory = trajectory[
                                    trajectory["t_sec"] <= snapshot_elapsed
                                ]
                            if trajectory.empty:
                                continue
                            yaw = float(transform["yaw"])
                            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
                            local_x = trajectory["x"].to_numpy(dtype=float)
                            local_y = trajectory["y"].to_numpy(dtype=float)
                            world_x = (
                                float(transform["x"])
                                + cos_yaw * local_x
                                - sin_yaw * local_y
                            )
                            world_y = (
                                float(transform["y"])
                                + sin_yaw * local_x
                                + cos_yaw * local_y
                            )
                            color = vehicle_palette.get(vehicle_id, (0, 137, 123))
                            path_pixels = [
                                world_to_pixel(float(x), float(y))
                                for x, y in zip(world_x, world_y)
                            ]
                            if len(path_pixels) > 1:
                                draw.line(path_pixels, fill=color, width=4, joint="curve")
                            start_px, start_py = path_pixels[0]
                            end_px, end_py = path_pixels[-1]
                            draw.ellipse(
                                (start_px - 7, start_py - 7, start_px + 7, start_py + 7),
                                fill=color,
                                outline="white",
                                width=2,
                            )
                            draw.line(
                                (end_px - 7, end_py - 7, end_px + 7, end_py + 7),
                                fill="white",
                                width=6,
                            )
                            draw.line(
                                (end_px - 7, end_py + 7, end_px + 7, end_py - 7),
                                fill="white",
                                width=6,
                            )
                            draw.line(
                                (end_px - 7, end_py - 7, end_px + 7, end_py + 7),
                                fill=color,
                                width=3,
                            )
                            draw.line(
                                (end_px - 7, end_py + 7, end_px + 7, end_py - 7),
                                fill=color,
                                width=3,
                            )
                            goal_x = as_float(record.get("goal_x"))
                            goal_y = as_float(record.get("goal_y"))
                            if goal_x is not None and goal_y is not None:
                                goal_world_x, goal_world_y = local_to_world(
                                    goal_x, goal_y, transform
                                )
                                goal_px, goal_py = world_to_pixel(
                                    goal_world_x, goal_world_y
                                )
                                star_points = []
                                for star_index in range(10):
                                    angle = -math.pi / 2.0 + star_index * math.pi / 5.0
                                    radius = 10 if star_index % 2 == 0 else 4
                                    star_points.append(
                                        (
                                            goal_px + int(radius * math.cos(angle)),
                                            goal_py + int(radius * math.sin(angle)),
                                        )
                                    )
                                draw.polygon(
                                    star_points, fill=color, outline=(20, 20, 20)
                                )
                            trajectory_legend.append((vehicle_id, color))

                        legend_y = map_bottom + 53
                        legend_items = [
                            ("Unknown", (173, 173, 173)),
                            ("Fused free", (255, 255, 255)),
                            ("Fused occupied", (8, 8, 8)),
                            ("Source overlap", (123, 31, 162)),
                        ]
                        legend_items.extend(
                            (f"{vehicle_id} fusion input + trajectory", color)
                            for vehicle_id, color in trajectory_legend
                        )
                        if raw_slam_vehicle_ids:
                            legend_items.append(
                                ("Colored offset line = raw SLAM diagnostic", (190, 70, 190))
                            )
                        legend_x = map_left
                        for label, color in legend_items:
                            draw.rectangle(
                                (legend_x, legend_y, legend_x + 18, legend_y + 12),
                                fill=color,
                                outline=(90, 90, 90),
                            )
                            draw.text(
                                (legend_x + 24, legend_y - 2),
                                label,
                                fill=(35, 35, 35),
                                font=small_font,
                            )
                            legend_x += 205

                        st.markdown("#### 융합 입력·Raw SLAM·비행 경로")
                        image_buffer = io.BytesIO()
                        canvas.save(image_buffer, format="PNG")
                        st.image(
                            image_buffer.getvalue(), use_container_width=True
                        )
                        st.download_button(
                            "현재 SLAM 융합 그림 PNG 다운로드",
                            data=image_buffer.getvalue(),
                            file_name=f"{debug_run_root.name}_{selected_stage.replace(' ', '_')}_slam_fusion.png",
                            mime="image/png",
                            key=f"slam_png_{debug_run_root.name}_{selected_stage}",
                        )
                        st.caption(
                            "○=실제 시작점, X=선택 시점 실제 위치, ★=설정 목표점. "
                            "파랑/주황 음영은 실제 융합 입력 영역, 두꺼운 검정은 fused occupied map, "
                            "보라색은 입력 중첩입니다. known-pose 융합 run에서 검정 맵 밖으로 벗어난 "
                            "기체색 선은 별도로 기록한 raw scan-matching SLAM drift입니다."
                        )
                    except Exception as exc:
                        st.warning(f"SLAM raster view 렌더링 실패: {exc}")

                if point_rows:
                    try:
                        import altair as alt

                        points_df = pd.DataFrame(point_rows)
                        rect_df = pd.DataFrame()
                        outline_df = pd.DataFrame()
                        world_candidates = [
                            Path(str(record.get("map_path"))) for record in run_records
                            if str(record.get("map_path", "")).strip()
                            and Path(str(record.get("map_path"))).exists()
                        ]
                        if world_candidates:
                            try:
                                geometry = load_world_geometry(repo_root, world_candidates[0])
                                rect_df = pd.DataFrame(geometry["rects"])
                                outline_df = pd.DataFrame(geometry["outlines"])
                                st.caption(f"World reference: `{world_candidates[0]}`")
                            except Exception as exc:
                                st.warning(f"World geometry 로드 실패: {exc}")

                        x_values = points_df["x"].tolist()
                        y_values = points_df["y"].tolist()
                        if not outline_df.empty:
                            x_values.extend(outline_df["x"].tolist())
                            y_values.extend(outline_df["y"].tolist())
                        x_min, x_max = min(x_values), max(x_values)
                        y_min, y_max = min(y_values), max(y_values)
                        x_pad = max(1.0, (x_max - x_min) * 0.03)
                        y_pad = max(1.0, (y_max - y_min) * 0.08)
                        x_axis = alt.X("x:Q", title="world x [m]", scale=alt.Scale(domain=[x_min - x_pad, x_max + x_pad]))
                        y_axis = alt.Y("y:Q", title="world y [m]", scale=alt.Scale(domain=[y_min - y_pad, y_max + y_pad]))
                        chart_layers = []
                        if not rect_df.empty:
                            walls = rect_df[rect_df["kind"] != "floor"]
                            if not walls.empty:
                                chart_layers.append(
                                    alt.Chart(walls).mark_rect(color="#94a3b8", opacity=0.24).encode(
                                        x=alt.X("x1:Q", scale=alt.Scale(domain=[x_min - x_pad, x_max + x_pad])),
                                        x2="x2:Q",
                                        y=alt.Y("y1:Q", scale=alt.Scale(domain=[y_min - y_pad, y_max + y_pad])),
                                        y2="y2:Q",
                                        tooltip=["geometry_id", "kind"],
                                    )
                                )
                        if not outline_df.empty:
                            chart_layers.append(
                                alt.Chart(outline_df).mark_line(color="#334155", strokeWidth=1.1).encode(
                                    x=x_axis, y=y_axis, detail="geometry_id:N", order="order:Q",
                                    tooltip=["geometry_id", "kind"],
                                )
                            )
                        chart_layers.append(
                            alt.Chart(points_df).mark_point(filled=True, size=13).encode(
                                x=x_axis,
                                y=y_axis,
                                color=alt.Color("layer:N", title="Map layer"),
                                shape=alt.Shape("cell_state:N", title="Cell state"),
                                opacity=alt.condition(
                                    alt.datum.cell_state == "occupied", alt.value(0.62), alt.value(0.08)
                                ),
                                tooltip=["layer", "cell_state", "occupancy", "x", "y"],
                            )
                        )
                        if conflict_rows:
                            conflict_df = pd.DataFrame(conflict_rows)
                            chart_layers.append(
                                alt.Chart(conflict_df).mark_point(
                                    shape="cross", color="#dc2626", size=55, strokeWidth=1.8
                                ).encode(x=x_axis, y=y_axis, tooltip=["x", "y"])
                            )
                        chart = alt.layer(*chart_layers).properties(height=640).interactive()
                        st.altair_chart(chart, use_container_width=True)
                        if conflict_rows:
                            st.caption(
                                "빨간 + 표시는 선택한 로컬 입력들에서 free(≤25)와 occupied(≥65)가 "
                                "동시에 투표된 위치입니다. 저장 시점 차이 때문에 런타임 지표와 소폭 다를 수 있습니다."
                            )
                    except Exception as exc:
                        st.warning(f"맵 오버레이 렌더링 실패: {exc}")

                with st.expander("선택 스냅샷 원본 PGM"):
                    image_columns = st.columns(min(3, max(1, len(plotted_snapshots))))
                    for index, snapshot in enumerate(plotted_snapshots):
                        if snapshot["pgm_path"].exists():
                            image_columns[index % len(image_columns)].image(
                                str(snapshot["pgm_path"]),
                                caption=str(snapshot["layer_id"]),
                                use_container_width=True,
                            )

            fusion_metrics_path = debug_run_root / "fusion_metrics.csv"
            if fusion_metrics_path.exists():
                st.markdown("#### 런타임 융합 상태")
                try:
                    fusion_df = pd.read_csv(fusion_metrics_path)
                    if not fusion_df.empty:
                        fusion_df["map_version"] = pd.to_numeric(fusion_df["map_version"], errors="coerce")
                        for column in ["fusion_latency_ms", "conflict_ratio", "max_input_age_ms"]:
                            fusion_df[column] = pd.to_numeric(fusion_df[column], errors="coerce")
                        c1, c2 = st.columns(2)
                        c1.line_chart(fusion_df.set_index("map_version")[["fusion_latency_ms", "max_input_age_ms"]])
                        c2.line_chart(fusion_df.set_index("map_version")[["conflict_ratio"]])
                        st.dataframe(
                            fusion_df.tail(300), use_container_width=True
                        )
                except Exception as exc:
                    st.warning(f"fusion_metrics.csv 로드 실패: {exc}")

            with st.expander("실제 융합 계산 방식"):
                st.markdown(
                    "1. 각 OccupancyGrid 셀 중심에 맵 원점의 위치·yaw를 적용합니다.  \n"
                    "2. manifest의 `world_from_local = (spawn x, y, yaw)`를 적용해 `swarm_map` 좌표로 옮깁니다.  \n"
                    "3. 0.1 m 전역 격자에 투영하고, source confidence와 최신도(freshness)를 가중치로 사용합니다.  \n"
                    "4. 확률을 log-odds로 바꿔 합산한 뒤 다시 0~100 occupancy로 변환합니다.  \n"
                    "5. 같은 전역 셀에 free(≤25)와 occupied(≥65) 입력이 함께 있으면 conflict로 기록합니다."
                )

    with tabs[7]:
        render_oracle_correction_tab(repo_root, pd, st)

    with tabs[8]:
        render_lidar_registration_tab(repo_root, pd, st)

    with tabs[9]:
        st.subheader("구현 및 실험 보고서")
        visible_run_ids = set(filtered["source_run_id"].astype(str).tolist())
        visible_reports = [
            report
            for report in run_reports
            if str(report.get("source_run_id")) in visible_run_ids
        ]
        if not visible_reports:
            st.info("현재 필터에 대응하는 구현 보고서가 없습니다.")
        else:
            report_by_run = {
                str(report["source_run_id"]): report for report in visible_reports
            }
            report_run_id = st.selectbox(
                "Report run",
                sorted(report_by_run),
                format_func=lambda rid: run_label(repo_root, rid),
            )
            report = report_by_run[report_run_id]
            st.markdown(f"### {report.get('title', report_run_id)}")
            status_col, scope_col = st.columns(2)
            status_col.metric("검증 상태", str(report.get("status", "unknown")))
            scope_col.metric("검증 범위", str(report.get("scope", "unknown")))
            if report.get("executive_summary"):
                st.info(str(report["executive_summary"]))

            st.markdown("#### 사용한 SLAM 구성")
            st.json(report.get("slam_stack", {}))

            st.markdown("#### 문제 → 원인 → 해결 → 결과")
            st.dataframe(
                pd.DataFrame(report.get("changes", [])),
                use_container_width=True,
            )

            st.markdown("#### 정량 검증")
            st.dataframe(
                pd.DataFrame(report.get("validation", [])),
                use_container_width=True,
            )

            st.markdown("#### 테스트")
            st.dataframe(
                pd.DataFrame(report.get("tests", [])),
                use_container_width=True,
            )

            report_artifact = repo_root / "artifacts" / report_run_id
            if report_artifact.exists():
                st.markdown("#### 저장된 지도 비교")
                for vehicle in report.get("vehicles", []):
                    vehicle_id = str(vehicle)
                    st.markdown(f"**{vehicle_id}**")
                    image_columns = st.columns(2)
                    comparison_sources = ["slam", "known_pose_reference"]
                    reference_path = (
                        report_artifact
                        / vehicle_id
                        / "maps"
                        / "known_pose_reference.pgm"
                    )
                    if not reference_path.exists():
                        comparison_sources[1] = "known_pose"
                    for column, source in zip(image_columns, comparison_sources):
                        image_path = (
                            report_artifact
                            / vehicle_id
                            / "maps"
                            / f"{source}.pgm"
                        )
                        if image_path.exists():
                            column.image(
                                str(image_path),
                                caption=f"{vehicle_id} / {source}",
                                use_container_width=True,
                            )

            st.markdown("#### 남은 위험과 해석 주의사항")
            for item in report.get("remaining_risks", []):
                st.markdown(f"- {item}")
            for item in report.get("notes", []):
                st.caption(str(item))


def _reset_catalog_drafts(st):
    for key in list(st.session_state):
        if str(key).startswith("catalog_edit_"):
            del st.session_state[key]


def _change_catalog_visibility(repo_root, run_id, visibility, revision, st):
    prefix = "catalog_edit_" + run_id + "_"
    changes = {"visibility": visibility}
    if visibility == "archive":
        if not st.session_state.get(prefix + "confirm"):
            st.session_state["catalog_error"] = "보관함 이동 확인을 선택해 주세요."
            return
        changes["archive_reason"] = st.session_state.get(prefix + "reason", "")
    try:
        save_entries(repo_root, {run_id: changes}, revision)
        st.session_state.pop(prefix + "revision", None)
        st.session_state["catalog_flash"] = ("보관함으로 옮겼습니다. 원본은 보존되며 복원할 수 있습니다."
                                               if visibility == "archive" else "기본 목록으로 복원했습니다.")
    except (CatalogError, OSError) as exc:
        st.session_state["catalog_error"] = str(exc)


def render_catalog_editor(repo_root, run_id, catalog, st, allow_archive=True):
    """Form edits metadata only. Stable IDs and all scientific values are read-only."""
    prefix = "catalog_edit_" + run_id + "_"
    revision = st.session_state.setdefault(prefix + "revision", catalog["revision"])
    entry = entry_for(catalog, run_id)
    st.caption("표시 이름·목적·메모만 변경합니다. 실험 ID, 폴더명, rosbag, 계산된 지표는 바뀌지 않습니다.")
    with st.form("label_form_" + run_id):
        label = st.text_input("실험 이름", value=entry["label"], max_chars=120, key=prefix + "label")
        purpose = st.text_area("무엇을 확인하려는 실험인가?", value=entry["purpose"], max_chars=2000,
                               height=90, key=prefix + "purpose")
        group = st.selectbox("실험 분류", GROUPS,
                             index=GROUPS.index(entry["group"]) if entry["group"] in GROUPS else len(GROUPS)-1,
                             key=prefix + "group")
        notes = st.text_area("해석·주의사항 메모", value=entry["notes"], max_chars=4000,
                             height=90, key=prefix + "notes")
        submitted = st.form_submit_button("이름·설명 저장", type="primary")
    if submitted:
        try:
            save_entries(repo_root, {run_id: {"label": label, "purpose": purpose, "group": group, "notes": notes}}, revision)
            st.session_state.pop(prefix + "revision", None)
            st.session_state["catalog_flash"] = "실험 이름과 설명을 저장했습니다. 모든 선택 목록에 반영됩니다."
            st.rerun()
        except (CatalogError, OSError) as exc:
            st.error(str(exc))
    st.button("저장된 내용 다시 불러오기", key=prefix + "reload", on_click=_reset_catalog_drafts, args=(st,))
    if not allow_archive:
        st.caption("목록에서 삭제하거나 복원하려면 왼쪽 '이름·보관 관리' 메뉴를 사용하세요.")
        return
    with st.expander("기본 목록에서 삭제 / 보관함에서 복원"):
        st.caption("삭제는 기본 목록에서 제외하는 동작입니다. 원본 데이터는 그대로 두며 보관함에서 복원할 수 있습니다.")
        if entry["visibility"] == "archive":
            st.write("보관 사유: " + entry["archive_reason"])
            st.button("기본 목록으로 복원", key=prefix + "restore", on_click=_change_catalog_visibility,
                      args=(repo_root, run_id, "active", revision, st))
        else:
            reason = st.text_input("목록에서 제외하는 이유", key=prefix + "reason", max_chars=1000)
            confirmed = st.checkbox("원본은 보존하고 이 실험을 보관함으로 옮깁니다.", key=prefix + "confirm")
            st.button("기본 목록에서 삭제 → 보관함", key=prefix + "archive", disabled=not confirmed,
                      on_click=_change_catalog_visibility, args=(repo_root, run_id, "archive", revision, st))


def experiment_table(inventory, catalog):
    return [{"실험 이름": display_label(catalog, rid), "분류": entry_for(catalog, rid)["group"],
             "목적": entry_for(catalog, rid)["purpose"],
             "목록": "기본 목록" if entry_for(catalog, rid)["visibility"] == "active" else "보관함",
             "보관 사유": entry_for(catalog, rid)["archive_reason"], "원본 ID": rid}
            for rid in inventory]


def render_experiment_summary(repo_root, run, catalog, pd, st):
    run_id, root = run["run_id"], run["path"]
    report = next((r for r in load_run_reports(repo_root) if r["source_run_id"] == run_id), {})
    host = read_json(repo_root / "runtime/sim" / run_id / "host.result.json")
    if report.get("executive_summary"):
        st.info(report["executive_summary"].replace("LiDAR Registration 탭에서 같은 run을 선택한다", "이 실험의 LiDAR 상세에서 확인한다"))
    analysis = next((root / kind for kind in ("lidar_registration", "oracle_correction")
                     if kind in run["analyses"]), None)
    if analysis:
        manifest = read_json(analysis / "manifest.json")
        gt_free = manifest.get("gt_usage") == "evaluation_only"
        if gt_free:
            st.caption("GT 사용: 평가 전용 · 초기 좌표 정렬은 기지 가정 · 비행 후 오프라인 보정 · 실시간 만남/통신 횟수 실험 아님")
        elif analysis.name == "lidar_registration":
            st.warning("이 과거 LiDAR 실험은 GT로 비교할 pair를 선택했습니다. 'GT 없이 장소를 찾은 결과'로 해석하면 안 됩니다.")
        else:
            st.warning("Oracle은 GT에서 상대 pose 제약을 만드는 이상적 비교 조건입니다. 실제 센서만으로 보정한 결과가 아닙니다.")
        metrics = read_json(analysis / "metrics.json")
        if host:
            cols = st.columns(3)
            cols[0].metric("기록 종료", "정상 완료" if host.get("reason") == "mission_complete" else host.get("reason", "미확인"))
            cols[1].metric("기체 수", len(run["vehicles"]))
            primary = manifest.get("primary_condition", "N-single")
            optimizer = (manifest.get("optimizer") or {}).get(primary) or {}
            applied = optimizer.get("applied_factor_count", optimizer.get("accepted_factor_count") if optimizer.get("success") else 0)
            cols[2].metric("적용한 LiDAR 제약", "-" if not optimizer else f"{applied}개")
        if manifest.get("primary_condition") == "N-double":
            st.info("두 구간 비교: N-single-A=앞 구간만 / N-single-B=뒤 구간만 / N-double=두 구간 모두. N-prior는 초기 정렬만 적용한 대조군입니다. 'LiDAR 상세'에서 구간 확보·최적화 성공 여부를 먼저 확인하세요.")
            primary_opt = (manifest.get("optimizer") or {}).get("N-double") or {}
            status = read_json(analysis / "ablation_status.json")
            if status:
                counts = status.get("actual_applied_counts") or {}
                st.caption(f"실제 적용: 앞 구간만 {counts.get('N-single-A', 0)}개 · 뒤 구간만 {counts.get('N-single-B', 0)}개 · 두 구간 모두 {counts.get('N-double', 0)}개")
                if status.get("complete"):
                    st.success("1회·2회 비교 성립: 서로 떨어진 A/B 제약을 각각 검증했고 A+B에 2개 모두 적용했습니다. 어느 조건이 더 정확한지는 아래 지표로 별도 판단합니다.")
            if not primary_opt.get("available"):
                st.warning(f"2구간 비교 미성립: 필요한 제약 2개 중 {primary_opt.get('accepted_factor_count', 0)}개만 확보했습니다. N-double은 실제 2회 보정 결과가 아니며, 미성립 조건의 숫자는 fallback 참고값입니다.")
        gate = metrics.get("phase2_gate") or metrics.get("phase1_gate") or {}
        if gate:
            if gate.get("pass"):
                st.success("보정 평가: 설정된 종합 기준 통과. 다른 경로/반복 실행에서 일반화 검증은 별도입니다.")
            else:
                st.warning("보정 평가: 일부 기준 미달. 기록·분석 실행 실패와 다릅니다. 개선된 지표와 악화된 지표를 함께 보세요.")
        rows = lidar_metric_rows(metrics)
        if manifest.get("primary_condition") == "N-double":
            for row in rows:
                opt = (manifest.get("optimizer") or {}).get(row["condition"])
                row["조건 상태"] = ("기준" if opt is None else
                                   "미성립 · fallback" if not opt.get("available") else
                                   "유효" if opt.get("success") else "최적화 실패 · fallback")
                row["실제 적용 제약"] = 0 if opt is None else opt.get("applied_factor_count", 0)
        rename = {"condition": "조건", "endpoint_translation_m": "종단 상대 위치 오차 [m] ↓",
                  "endpoint_yaw_deg": "종단 상대 yaw 오차 [°] ↓", "translation_rmse_m": "상대 위치 RMSE [m] ↓",
                  "yaw_rmse_deg": "상대 yaw RMSE [°] ↓", "occupied_chamfer_m": "지도 Chamfer [m] ↓",
                  "occupied_f1": "지도 F1 ↑"}
        st.subheader("같은 기록에서 보정 전·후 비교")
        st.dataframe(pd.DataFrame(rows).rename(columns=rename).round(3), use_container_width=True, hide_index=True)
        st.caption("↓ 작을수록 좋음 / ↑ 클수록 좋음. 상대 관계가 좋아져도 절대 지도 정확도가 좋아진 것은 아닙니다.")
        with st.expander("B0 · B1 · N / R / O 조건 설명"):
            st.markdown("- **B0**: raw SLAM. 드론 간 보정 없음.\n- **B1**: 초기 정렬 + 외부 MAVROS/PX4 odometry.\n- **N-single**: GT 없이 찾은 LiDAR 제약 1개로 보정. 초기 정렬은 기지.\n- **R-single / R-periodic**: GT로 고른 pair를 LiDAR로 정합한 과거 controlled 실험.\n- **O-single / O-periodic**: GT에서 만든 상대 pose 제약을 한 번 / 여러 번 적용.\n- **GT**: 평가용 정답. Oracle 조건에서는 제약 생성에도 사용.")
            st.button("조건·그래프 해석 가이드 열기", on_click=_open_experiment_guide, args=(st,))
        previews = {"공통 관측 위치": "overview_shared_regions.png", "전체 지도 오차": "overview_map_errors.png", "GT 대비 이동 궤적": "overview_trajectories.png",
                    "절대 위치·yaw 오차": "overview_errors.png"}
        previews = {label: filename for label, filename in previews.items() if (analysis / filename).exists()}
        if previews:
            view = st.radio("결과 그림", list(previews), horizontal=True, key="preview_" + run_id)
            st.image(str(analysis / previews[view]), use_container_width=True)
            if view == "전체 지도 오차":
                st.caption("초록: GT에만 점유 / 빨강: 추정에만 점유 / 검정: 점유 일치 / 회색: 둘 다 미관측. GT 지도도 동일 LiDAR 관측으로 만든 평가 기준입니다.")
            elif view == "공통 관측 위치":
                st.caption("위 그래프의 A/B 연결선은 실제 채택된 보정 제약만 표시합니다. 아래 막대는 채택 여부와 관계없이 공통 관측이 있었던 구간입니다. 동시에 만난 위치가 아니며, GT 궤적·공통 점유셀은 사후 평가에만 사용합니다.")
    elif report:
        st.dataframe(pd.DataFrame(report.get("validation", [])), use_container_width=True, hide_index=True)
    else:
        st.info("이 실행에는 완성된 보정 분석이 없습니다. '비행 기록·지도'에서 기록을 확인하고 목적을 메모해 주세요.")
    if entry_for(catalog, run_id)["notes"]:
        st.subheader("해석 메모")
        st.write(entry_for(catalog, run_id)["notes"])


def render_recorded_maps(repo_root, run, pd, st):
    st.caption("비행 중 저장한 원본 지도입니다. 오프라인 N 계열 보정 지도는 'LiDAR 상세'에서 확인하세요.")
    snapshots = discover_map_snapshots(run["path"])
    if snapshots:
        stages = sorted({item["snapshot_index"] for item in snapshots if item["stage"] == "history" and item["snapshot_index"] is not None})
        options = ["최종", *[f"기록 #{index:04d}" for index in stages]]
        choice = st.select_slider("지도 기록 시점", options, value="최종", key="map_time_" + run["run_id"])
        index = None if choice == "최종" else int(choice.split("#")[1])
        selected = [item for item in snapshots if (item["stage"] == "latest" if index is None else
                                                    item["stage"] == "history" and item["snapshot_index"] == index)]
        st.caption("각 기록 시점은 recorder별로 조금 다를 수 있습니다. 아래 이미지는 각 지도 좌표계이며, 정렬 중첩 디버그는 기존 상세 도구에서 제공합니다.")
        columns = st.columns(2)
        for number, item in enumerate(selected):
            if item["pgm_path"].exists():
                columns[number % 2].image(str(item["pgm_path"]), caption=f"{item['layer_id']} · {item.get('elapsed_sec')}s", use_container_width=True)
    else:
        st.info("이 실행에는 저장된 지도 스냅샷이 없습니다.")
    records = []
    paths = [run["path"] / name for name in run["vehicles"]] or [run["path"]]
    for path in paths:
        summary = read_json(path / "summary.json")
        records.append({"기체": path.name, "임무 상태": summary.get("mission_phase", "미기록"),
                        "목표 도달": summary.get("goal_reached", "미기록")})
    st.dataframe(pd.DataFrame(records), use_container_width=True, hide_index=True)


def render_report_files(repo_root, run, st):
    report = next((r for r in load_run_reports(repo_root) if r["source_run_id"] == run["run_id"]), {})
    if report:
        st.subheader(report.get("title", "실험 보고서"))
        st.write(report.get("executive_summary", ""))
        for risk in report.get("remaining_risks", []):
            st.write("• " + risk)
        st.download_button("실험 보고서 JSON 다운로드", json.dumps(report, ensure_ascii=False, indent=2),
                           file_name=run["run_id"] + "_report.json", mime="application/json")
    for kind in run["analyses"]:
        root = run["path"] / kind
        with st.expander(kind + " · 지표/그림 다운로드"):
            for path in [root / "metrics.csv", root / "registrations.csv", root / "report.md",
                         *sorted(root.glob("overview_*.png"))]:
                if path.is_file():
                    st.download_button(path.name, path.read_bytes(), file_name=path.name, key=str(path))
    with st.expander("원본 ID · 저장 위치 · 변경 이력"):
        st.code(run["run_id"], language=None)
        st.code(str(run["path"]), language=None)
        bag = repo_root / "rosbags" / (run["run_id"] + "_slam_debug")
        if bag.is_dir():
            st.code(str(bag), language=None)
        st.caption("이름 저장 이력: experiments/dashboard_catalog_history/. 표시 이름을 바꿔도 원본 경로는 유지됩니다.")


def _open_experiment_guide(st):
    st.session_state["workspace_page"] = "실험 해석 가이드"


def render_experiment_guide(repo_root: Path, st) -> None:
    """One Markdown source for the in-app README and its downloadable copy."""
    path = repo_root / "docs/slam_experiment_guide.md"
    try:
        content = path.read_text(encoding="utf-8")
    except OSError:
        st.error("가이드 원문을 읽을 수 없습니다: docs/slam_experiment_guide.md")
        return
    st.caption("B0/B1부터 그래프 읽는 방법까지 · 현재 구현 기준 README")
    st.download_button("가이드 Markdown 다운로드", content, file_name=path.name,
                       mime="text/markdown", key="download_experiment_guide")
    st.markdown(content)


def render_dashboard(repo_root: Path) -> None:
    import pandas as pd
    import streamlit as st

    st.set_page_config(page_title="AV_Drone · 실험 노트", page_icon="📊", layout="wide")
    try:
        catalog = load_catalog(repo_root)
        inventory = discover_experiments(repo_root)
    except CatalogError as exc:
        st.error(str(exc))
        st.stop()
    with st.sidebar:
        st.title("AV_Drone")
        st.caption("실험 기록 · 보정 결과 · 해석")
        page = st.radio("메뉴", ("실험 보기", "실험 해석 가이드", "이름·보관 관리", "기존 상세 분석 도구"), key="workspace_page")
        if st.button("목록 새로고침", on_click=_reset_catalog_drafts, args=(st,)):
            st.rerun()
        active = sum(entry_for(catalog, rid)["visibility"] == "active" for rid in inventory)
        st.caption(f"기본 목록 {active}개 · 보관함 {len(inventory)-active}개")
    if "catalog_flash" in st.session_state:
        st.success(st.session_state.pop("catalog_flash"))
    if "catalog_error" in st.session_state:
        st.error(st.session_state.pop("catalog_error"))
    if page == "실험 해석 가이드":
        render_experiment_guide(repo_root, st)
        return
    if page == "기존 상세 분석 도구":
        render_legacy_dashboard(repo_root)
        return
    if not inventory:
        st.info("실험 기록이 없습니다. 새 기록이 생기면 목록에 표시됩니다.")
        return
    if page == "이름·보관 관리":
        st.title("실험 이름·보관 관리")
        st.caption("분석 목적별로 정리합니다. 성능이 나쁜 결과도 비교 자료이며, 실패했다는 이유만으로 제외하지 않습니다.")
        scope = st.radio("관리할 목록", ("기본 목록", "보관함", "전체"), horizontal=True, key="manage_scope")
        query = st.text_input("이름·목적·ID 검색", key="manage_search")
    else:
        with st.sidebar:
            scope = st.radio("실험 목록", ("기본 목록", "보관함", "전체"), key="browse_scope")
            query = st.text_input("이름·목적·ID 검색", key="browse_search")
    desired = {"기본 목록": "active", "보관함": "archive"}.get(scope)
    visible = [rid for rid in inventory if (desired is None or entry_for(catalog, rid)["visibility"] == desired)
               and query.casefold() in " ".join([rid, *[str(value) for value in entry_for(catalog, rid).values()]]).casefold()]
    visible.sort(key=lambda rid: inventory[rid]["modified"], reverse=True)
    if not visible:
        st.info("해당 목록에 실험이 없습니다. 검색을 지우거나 보관함/전체 목록을 선택해 주세요.")
        return
    if page == "이름·보관 관리":
        st.dataframe(pd.DataFrame(experiment_table({rid: inventory[rid] for rid in visible}, catalog)),
                     use_container_width=True, hide_index=True)
        previous = st.session_state.get("manage_run")
        selected = st.selectbox("편집할 실험", visible, index=visible.index(previous) if previous in visible else 0,
                                format_func=lambda rid: display_label(catalog, rid), key="manage_run")
        render_catalog_editor(repo_root, selected, catalog, st)
        st.download_button("전체 라벨·보관 목록 백업", json.dumps(catalog, ensure_ascii=False, indent=2),
                           file_name="dashboard_catalog.json", mime="application/json")
        return
    with st.sidebar:
        previous = st.session_state.get("workspace_run")
        selected = st.selectbox("실험 선택", visible, index=visible.index(previous) if previous in visible else 0,
                                format_func=lambda rid: display_label(catalog, rid), key="workspace_run")
    entry, run = entry_for(catalog, selected), inventory[selected]
    st.title(entry["label"])
    st.caption(entry["group"] + " · 선택한 실험 하나의 기록만 표시합니다.")
    st.write(entry["purpose"])
    if entry["visibility"] == "archive":
        st.warning("보관 중: " + entry["archive_reason"])
    with st.expander("✏️ 실험 이름·목적 편집"):
        render_catalog_editor(repo_root, selected, catalog, st, allow_archive=False)
    sections = ["실험 요약"]
    if "lidar_registration" in run["analyses"]:
        sections.append("LiDAR 상세")
    if "oracle_correction" in run["analyses"]:
        sections.append("Oracle 상세")
    sections += ["비행 기록·지도", "보고서·파일"]
    section = st.radio("보기", sections, horizontal=True, key="view_" + selected)
    if section == "실험 요약":
        render_experiment_summary(repo_root, run, catalog, pd, st)
    elif section == "LiDAR 상세":
        render_lidar_registration_tab(repo_root, pd, st, run["path"] / "lidar_registration")
    elif section == "Oracle 상세":
        render_oracle_correction_tab(repo_root, pd, st, run["path"] / "oracle_correction")
    elif section == "비행 기록·지도":
        render_recorded_maps(repo_root, run, pd, st)
    else:
        render_report_files(repo_root, run, st)


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    if args.check_data:
        return check_data(repo_root)
    render_dashboard(repo_root)
    return 0


if __name__ == "__main__":
    main()
