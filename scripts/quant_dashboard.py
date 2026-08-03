#!/usr/bin/env python3
"""Read-only Streamlit dashboard for A* vs MPPI path-planning quantification."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any


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
        return repo_root / text.removeprefix("/workspace/AV_Drone/")
    path = Path(text)
    if path.is_absolute():
        return path
    return repo_root / path


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
    model_name = uri.removeprefix("model://").strip("/")
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
    include_path = repo_root / "experiments" / "dashboard_active_runs.txt"
    active_runs = set()
    if include_path.exists():
        active_runs = {
            line.strip()
            for line in include_path.read_text().splitlines()
            if line.strip() and not line.strip().startswith("#")
        }
    candidates = set()
    for path in artifacts_root.glob("*_drone*"):
        if path.is_dir() and has_artifact_evidence(path):
            candidates.add(path)
    for filename in ["rosbag_metrics.json", "paper_metrics.json"]:
        for path in artifacts_root.rglob(filename):
            if path.parent.is_dir() and has_artifact_evidence(path.parent):
                candidates.add(path.parent)
    if active_runs:
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
    print(f"repo_root: {repo_root}")
    print(f"artifact_records: {len(artifacts)}")
    print(f"registry_index_rows: {len(registry['index'])}")
    print(f"ledger_rows: {len(registry['ledger'])}")
    print(f"scenario_table_rows: {len(registry['scenario_table'])}")
    print(f"summary_table_rows: {len(registry['summary_table'])}")
    conditions = sorted({str(record.get("condition_id")) for record in artifacts})
    scenarios = sorted({str(record.get("scenario_id")) for record in artifacts})
    unknown_conditions = sum(1 for record in artifacts if record.get("condition_id") == "unknown")
    unknown_scenarios = sum(1 for record in artifacts if record.get("scenario_id") == "unknown")
    print(f"conditions: {', '.join(conditions) if conditions else '-'}")
    print(f"scenarios: {', '.join(scenarios) if scenarios else '-'}")
    print(f"unknown_condition_records: {unknown_conditions}")
    print(f"unknown_scenario_records: {unknown_scenarios}")
    return 0


def render_dashboard(repo_root: Path) -> None:
    try:
        import pandas as pd
        import streamlit as st
    except Exception as exc:
        raise SystemExit(
            "Streamlit dashboard dependencies are missing. Install with: "
            "python3 -m pip install -r requirements-dashboard.txt"
        ) from exc

    st.set_page_config(page_title="AV_Drone A* vs MPPI Quantification", layout="wide")
    st.title("AV_Drone A* vs MPPI Path Planning Quantification")
    st.caption(
        "Read-only dashboard for comparing graph-search shortest-path planning "
        "and sampling-based MPPI control under matched world, start, goal, and seed conditions."
    )

    artifacts = load_artifact_records(repo_root)
    registry = load_registry_records(repo_root)
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
        run_query = st.text_input("Run ID contains", "")

    filtered = df[
        df["scenario_id"].isin(selected_scenarios)
        & df["algorithm_label"].isin(selected_algorithms)
        & df["condition_id"].isin(selected_conditions)
        & df["experiment_stage"].isin(selected_stages)
        & df["result"].isin(selected_results)
    ].copy()
    if run_query:
        filtered = filtered[filtered["run_id"].astype(str).str.contains(run_query, case=False, na=False)]

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

    tabs = st.tabs(["Overview", "Compare", "Trajectory Overlay", "Runs", "Ledger", "Artifacts"])

    with tabs[0]:
        st.subheader("Core KPI 기준")
        st.dataframe(pd.DataFrame(KPI_GUIDE), width="stretch")
        st.subheader("Supplementary KPI 기준")
        st.dataframe(pd.DataFrame(SUPPLEMENTARY_KPI_GUIDE), width="stretch")
        st.subheader("Condition counts")
        if not filtered.empty:
            counts = filtered.groupby(["scenario_id", "algorithm_label", "condition_id", "result"]).size().reset_index(name="runs")
            st.dataframe(counts, width="stretch")
        st.subheader("Existing registry tables")
        c1, c2 = st.columns(2)
        c1.write("experiments/index.csv")
        c1.dataframe(pd.DataFrame(registry["index"]), width="stretch")
        c2.write("experiments/scenario_table.csv")
        c2.dataframe(pd.DataFrame(registry["scenario_table"]), width="stretch")

    with tabs[1]:
        st.subheader("Core KPI summary by condition")
        core_summary_df = pd.DataFrame(metric_summary(filtered.to_dict("records"), CORE_METRICS))
        st.dataframe(core_summary_df, width="stretch")

        st.subheader("Supplementary KPI summary by condition")
        supplementary_metrics = st.multiselect(
            "Supplementary metrics",
            SUPPLEMENTARY_METRICS,
            default=SUPPLEMENTARY_METRICS,
        )
        supplementary_summary_df = pd.DataFrame(metric_summary(filtered.to_dict("records"), supplementary_metrics))
        st.dataframe(supplementary_summary_df, width="stretch")

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
                if len(frame) > per_run_max_points:
                    sample_indices = [
                        round(index * (len(frame) - 1) / (per_run_max_points - 1))
                        for index in range(per_run_max_points)
                    ]
                    frame = frame.iloc[sample_indices].copy()
                frame["run_id"] = str(record["run_id"])
                frame["condition_id"] = str(record["condition_id"])
                frame["algorithm_label"] = str(record.get("algorithm_label", "Unknown"))
                frame["planner_name"] = str(record.get("planner_name", "unknown"))
                frame["trajectory_label"] = str(record.get("trajectory_label", record["run_id"]))
                trajectory_frames.append(frame)

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
                    st.altair_chart(chart, width="stretch")
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
                                "t_sec",
                                "x",
                                "y",
                                "z",
                                "speed_mps",
                                "nearest_obstacle_m",
                            ]
                            if col in trajectory_df.columns
                        ]
                    ].tail(500),
                    width="stretch",
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
        st.dataframe(filtered[[col for col in identity_cols + core_cols if col in filtered.columns]], width="stretch")
        st.write("Supplementary KPI per run")
        st.dataframe(filtered[[col for col in identity_cols + supplementary_cols if col in filtered.columns]], width="stretch")

    with tabs[4]:
        st.subheader("Ledger")
        st.dataframe(pd.DataFrame(registry["ledger"]), width="stretch")

    with tabs[5]:
        st.subheader("Artifact drill-down")
        if filtered.empty:
            st.info("No runs match the current filter.")
            return
        selected_run = st.selectbox("Run", filtered["run_id"].astype(str).tolist())
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


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    if args.check_data:
        return check_data(repo_root)
    render_dashboard(repo_root)
    return 0


if __name__ == "__main__":
    main()
