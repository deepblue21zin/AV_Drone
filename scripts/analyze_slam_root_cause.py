#!/usr/bin/env python3

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_ids", nargs="+", help="Artifact run identifiers")
    parser.add_argument("--artifacts-root", default="artifacts")
    parser.add_argument("--output", default="")
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text()) if path.exists() else {}


def grid_points(root: Path, stem: str, threshold: int = 65) -> np.ndarray:
    grid_path = root / f"{stem}_grid.npy"
    meta_path = root / f"{stem}_meta.json"
    if not grid_path.exists() or not meta_path.exists():
        return np.empty((0, 2), dtype=np.float64)
    grid = np.load(grid_path, allow_pickle=False)
    meta = read_json(meta_path)
    rows, cols = np.nonzero(grid >= threshold)
    resolution = float(meta["resolution"])
    origin = meta.get("origin") or {}
    origin_yaw = float(origin.get("yaw", 0.0))
    gx = (cols.astype(np.float64) + 0.5) * resolution
    gy = (rows.astype(np.float64) + 0.5) * resolution
    cos_yaw, sin_yaw = math.cos(origin_yaw), math.sin(origin_yaw)
    return np.column_stack((
        float(origin.get("x", 0.0)) + cos_yaw * gx - sin_yaw * gy,
        float(origin.get("y", 0.0)) + sin_yaw * gx + cos_yaw * gy,
    ))


def nearest_distances(
    query: np.ndarray, reference: np.ndarray, chunk: int = 256
) -> np.ndarray:
    if not len(query) or not len(reference):
        return np.asarray([], dtype=np.float64)
    result = []
    for start in range(0, len(query), chunk):
        points = query[start : start + chunk]
        distance_squared = (
            (points[:, None, 0] - reference[None, :, 0]) ** 2
            + (points[:, None, 1] - reference[None, :, 1]) ** 2
        )
        result.append(np.sqrt(np.min(distance_squared, axis=1)))
    return np.concatenate(result)


def alignment_metrics(query: np.ndarray, reference: np.ndarray) -> dict[str, Any]:
    distances = nearest_distances(query, reference)
    if not len(distances):
        return {"occupied_cells": int(len(query))}
    return {
        "occupied_cells": int(len(query)),
        "within_0p3_pct": float(np.mean(distances <= 0.3000001) * 100.0),
        "within_0p5_pct": float(np.mean(distances <= 0.5000001) * 100.0),
        "within_1p0_pct": float(np.mean(distances <= 1.0000001) * 100.0),
        "median_distance_m": float(np.median(distances)),
        "p90_distance_m": float(np.percentile(distances, 90)),
    }


def load_manifest(vehicle_root: Path) -> dict[str, Any]:
    snapshots = sorted(
        (vehicle_root / "config_snapshots").glob("scenario_manifest_*.yaml")
    )
    return yaml.safe_load(snapshots[-1].read_text()) if snapshots else {}


def analyze_tf(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"rows": 0, "missing": True}
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return {"rows": 0}
    yaw = np.asarray([float(row["map_to_odom_yaw_deg"]) for row in rows])
    translation = np.asarray(
        [float(row["correction_translation_m"]) for row in rows]
    )
    flags = [
        flag
        for row in rows
        for flag in str(row.get("event_flags", "")).split(";")
        if flag
    ]
    return {
        "rows": len(rows),
        "final_yaw_deg": float(yaw[-1]),
        "max_abs_yaw_deg": float(np.max(np.abs(yaw))),
        "final_translation_m": float(translation[-1]),
        "max_translation_m": float(np.max(translation)),
        "event_counts": {flag: flags.count(flag) for flag in sorted(set(flags))},
    }


def analyze_vehicle(run_root: Path, vehicle: str) -> dict[str, Any]:
    vehicle_root = run_root / vehicle
    maps_root = vehicle_root / "maps"
    manifest = load_manifest(vehicle_root)
    vehicle_config = next(
        (item for item in manifest.get("vehicles", []) if item.get("name") == vehicle),
        {},
    )
    slam_latest = grid_points(maps_root, "slam")
    reference_stem = "known_pose_reference"
    reference_latest = grid_points(maps_root, reference_stem)
    if not len(reference_latest):
        reference_stem = "known_pose"
        reference_latest = grid_points(maps_root, reference_stem)
    latest = alignment_metrics(slam_latest, reference_latest)
    history = []
    for meta_path in sorted((maps_root / "history").glob("slam_*_meta.json")):
        stem = meta_path.name.removesuffix("_meta.json")
        index = stem.rsplit("_", 1)[-1]
        reference_stem = f"known_pose_reference_{index}"
        reference_root = maps_root / "history"
        if not (reference_root / f"{reference_stem}_grid.npy").exists():
            reference_stem = f"known_pose_{index}"
        if not (reference_root / f"{reference_stem}_grid.npy").exists():
            reference_stem = "known_pose_reference"
            reference_root = maps_root
        if not (reference_root / f"{reference_stem}_grid.npy").exists():
            reference_stem = "known_pose"
        metrics = alignment_metrics(
            grid_points(maps_root / "history", stem),
            grid_points(reference_root, reference_stem),
        )
        meta = read_json(meta_path)
        metrics.update({
            "snapshot_index": int(meta.get("snapshot_index", int(index))),
            "elapsed_sec": float(meta.get("elapsed_sec", 0.0)),
        })
        history.append(metrics)
    onset = next(
        (
            row
            for row in history
            if row.get("within_0p3_pct", 100.0) < 80.0
        ),
        None,
    )
    summary = read_json(vehicle_root / "summary.json")
    invalid_reasons = []
    if not summary.get("connected", False):
        invalid_reasons.append("FCU_DISCONNECT")
    if not len(slam_latest):
        invalid_reasons.append("SLAM_MAP_MISSING")
    if not len(reference_latest):
        invalid_reasons.append("REFERENCE_MAP_MISSING")
    return {
        "vehicle_id": vehicle,
        "valid": not invalid_reasons,
        "invalid_reasons": invalid_reasons,
        "spawn_y": float((vehicle_config.get("spawn") or [0, 0])[1]),
        "use_scan_matching": bool(
            (manifest.get("slam") or {}).get("use_scan_matching", True)
        ),
        "slam_parameters": manifest.get("slam") or {},
        "latest_alignment": latest,
        "drift_onset": onset,
        "history": history,
        "tf_diagnostics": analyze_tf(
            vehicle_root / "slam_tf_diagnostics.csv"
        ),
    }


def main() -> None:
    args = parse_args()
    artifacts_root = Path(args.artifacts_root).resolve()
    results = {"runs": []}
    for run_id in args.run_ids:
        run_root = artifacts_root / run_id
        if not run_root.exists():
            raise FileNotFoundError(run_root)
        results["runs"].append({
            "run_id": run_id,
            "vehicles": [
                analyze_vehicle(run_root, vehicle)
                for vehicle in ("drone1", "drone2")
            ],
        })
    text = json.dumps(results, ensure_ascii=False, indent=2)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
