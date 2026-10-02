#!/usr/bin/env python3
"""Render dependency-light PNGs for the SLAM root-cause report."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from quant_dashboard import (
    dilate_boolean_mask,
    discover_map_snapshots,
    project_snapshot_to_target,
    read_json,
)


COLORS = {"drone1": (25, 118, 210), "drone2": (245, 124, 0)}


def font(size: int) -> ImageFont.ImageFont:
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def blend(base: np.ndarray, mask: np.ndarray, color: tuple[int, int, int], alpha: float) -> None:
    base[mask] = (
        base[mask].astype(np.float32) * (1.0 - alpha)
        + np.asarray(color, dtype=np.float32) * alpha
    ).astype(np.uint8)


def transforms(run_root: Path) -> dict[str, dict[str, float]]:
    result = {"swarm": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
    for vehicle in ("drone1", "drone2"):
        metadata = read_json(run_root / vehicle / "metadata.json")
        value = metadata.get("world_from_local") or {}
        result[vehicle] = {
            "x": float(value.get("x", 0.0)),
            "y": float(value.get("y", 0.0)),
            "yaw": float(value.get("yaw", 0.0)),
        }
    return result


def render_map(run_root: Path, output: Path) -> None:
    snapshots = discover_map_snapshots(run_root)
    latest = [item for item in snapshots if item["stage"] == "latest"]
    target = next(item for item in latest if item["source"] == "fused_known_pose")
    known = {
        item["vehicle_id"]: item
        for item in latest
        if item["source"] == "known_pose"
    }
    raw = {
        item["vehicle_id"]: item
        for item in latest
        if item["source"] == "slam"
    }
    tf = transforms(run_root)
    grid = np.load(target["grid_path"], allow_pickle=False)
    meta = target["meta"]
    resolution = float(meta["resolution"])
    origin = meta.get("origin") or {}
    origin_x = float(origin.get("x", 0.0))
    origin_y = float(origin.get("y", 0.0))

    base = np.full((*grid.shape, 3), 173, dtype=np.uint8)
    base[grid >= 0] = (255, 255, 255)
    base[(grid > 25) & (grid < 65)] = (209, 209, 209)
    base[grid >= 65] = (8, 8, 8)

    for vehicle, snapshot in known.items():
        projected = project_snapshot_to_target(snapshot, tf[vehicle], target)
        blend(base, projected >= 0, COLORS[vehicle], 0.12)
    for vehicle, snapshot in raw.items():
        projected = project_snapshot_to_target(snapshot, tf[vehicle], target)
        raw_halo = dilate_boolean_mask(projected >= 65, radius=1)
        blend(base, raw_halo, COLORS[vehicle], 0.88)
    base[dilate_boolean_mask(grid >= 65, radius=1)] = (8, 8, 8)

    image = Image.fromarray(np.flipud(base), mode="RGB")
    left, top, bottom = 70, 58, 90
    canvas = Image.new("RGB", (image.width + left + 30, image.height + top + bottom), "white")
    canvas.paste(image, (left, top))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (left, 16),
        "Operational fused odometry map vs raw scan-matching SLAM",
        fill=(20, 20, 20),
        font=font(18),
    )

    def pixel(x: float, y: float) -> tuple[int, int]:
        return (
            left + int(round((x - origin_x) / resolution)),
            top + image.height - 1 - int(round((y - origin_y) / resolution)),
        )

    for x_tick in range(0, 151, 20):
        px, _ = pixel(float(x_tick), origin_y)
        draw.line((px, top, px, top + image.height - 1), fill=(205, 205, 205), width=1)
        draw.text((px - 9, top + image.height + 8), str(x_tick), fill=(40, 40, 40), font=font(12))
    for y_tick in range(-15, 16, 5):
        _, py = pixel(origin_x, float(y_tick))
        draw.line((left, py, left + image.width - 1, py), fill=(215, 215, 215), width=1)
        draw.text((20, py - 6), str(y_tick), fill=(40, 40, 40), font=font(12))

    for vehicle in ("drone1", "drone2"):
        rows = list(csv.DictReader((run_root / vehicle / "trajectory.csv").open()))
        transform = tf[vehicle]
        cos_yaw, sin_yaw = math.cos(transform["yaw"]), math.sin(transform["yaw"])
        points = []
        for row in rows[:: max(1, len(rows) // 2500)]:
            lx, ly = float(row["x"]), float(row["y"])
            wx = transform["x"] + cos_yaw * lx - sin_yaw * ly
            wy = transform["y"] + sin_yaw * lx + cos_yaw * ly
            points.append(pixel(wx, wy))
        if len(points) > 1:
            draw.line(points, fill=COLORS[vehicle], width=4, joint="curve")
        draw.ellipse(
            (points[0][0] - 6, points[0][1] - 6, points[0][0] + 6, points[0][1] + 6),
            fill=COLORS[vehicle],
            outline="white",
            width=2,
        )
        end = points[-1]
        draw.line((end[0] - 7, end[1] - 7, end[0] + 7, end[1] + 7), fill=COLORS[vehicle], width=3)
        draw.line((end[0] - 7, end[1] + 7, end[0] + 7, end[1] - 7), fill=COLORS[vehicle], width=3)

    legend_y = top + image.height + 48
    legends = [
        ("black: operational fused-known-pose occupied", (8, 8, 8)),
        ("blue: drone1 trajectory / raw SLAM offset", COLORS["drone1"]),
        ("orange: drone2 trajectory / raw SLAM offset", COLORS["drone2"]),
    ]
    x = left
    for label, color in legends:
        draw.rectangle((x, legend_y, x + 18, legend_y + 12), fill=color)
        draw.text((x + 24, legend_y - 2), label, fill=(35, 35, 35), font=font(12))
        x += 430
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


def render_drift(run_root: Path, output: Path) -> None:
    width, height = 1200, 650
    canvas = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(canvas)
    draw.text((55, 18), "Raw slam_toolbox map->odom drift during the stable fusion run", fill=(20, 20, 20), font=font(18))
    panels = [(70, 70, 1140, 330, "Translation correction [m]"), (70, 375, 1140, 610, "Yaw correction [deg]")]
    series = {}
    max_t = 1.0
    for vehicle in ("drone1", "drone2"):
        rows = list(csv.DictReader((run_root / vehicle / "slam_tf_diagnostics.csv").open()))
        t0 = float(rows[0]["t_sec"])
        values = [
            (
                float(row["t_sec"]) - t0,
                float(row["correction_translation_m"]),
                float(row["map_to_odom_yaw_deg"]),
            )
            for row in rows
        ]
        series[vehicle] = values
        max_t = max(max_t, values[-1][0])
    maxima = [
        max(value[1] for values in series.values() for value in values),
        max(abs(value[2]) for values in series.values() for value in values),
    ]
    for panel_index, (x0, y0, x1, y1, label) in enumerate(panels):
        max_value = max(1.0, math.ceil(maxima[panel_index]))
        draw.rectangle((x0, y0, x1, y1), outline=(80, 80, 80), width=2)
        draw.text((x0, y0 - 24), label, fill=(30, 30, 30), font=font(14))
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            py = int(y1 - fraction * (y1 - y0))
            draw.line((x0, py, x1, py), fill=(225, 225, 225), width=1)
            draw.text((20, py - 6), f"{fraction * max_value:.1f}", fill=(80, 80, 80), font=font(11))
        for vehicle, values in series.items():
            points = []
            for t, translation, yaw in values[:: max(1, len(values) // 3000)]:
                value = translation if panel_index == 0 else abs(yaw)
                px = int(x0 + t / max_t * (x1 - x0))
                py = int(y1 - min(value, max_value) / max_value * (y1 - y0))
                points.append((px, py))
            if len(points) > 1:
                draw.line(points, fill=COLORS[vehicle], width=3)
        draw.text((x1 - 60, y1 + 10), f"{max_t:.0f}s", fill=(60, 60, 60), font=font(11))
    draw.text((70, 622), "blue=drone1, orange=drone2; operational fusion does not consume these drifting transforms", fill=(50, 50, 50), font=font(12))
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


def render_matrix(matrix_path: Path, output: Path) -> None:
    data = json.loads(matrix_path.read_text())["runs"]
    labels = {
        "2026-08-04_01-29-29_slam_diag_baseline_r2": "baseline r2",
        "2026-08-04_01-44-06_slam_diag_lane_swap": "lane swap",
        "2026-08-04_01-48-49_slam_diag_scan_matching_off": "scan match OFF",
        "2026-08-04_02-09-55_slam_diag_angle_penalty": "angle penalty",
        "2026-08-04_02-18-18_slam_diag_odom_penalty": "odom penalty 45m",
        "2026-08-04_02-23-53_slam_diag_odom_penalty_full": "odom penalty 137m",
        "2026-08-04_02-43-51_slam_diag_hard_odom_medium": "hard odom 70m",
        "2026-08-04_03-00-47_slam_diag_stable_fusion_full": "final raw SLAM",
    }
    runs = [run for run in data if run["run_id"] in labels]
    width, height = 1280, 650
    canvas = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(canvas)
    draw.text((55, 18), "Raw SLAM occupied cells within 0.3 m of odometry reference", fill=(20, 20, 20), font=font(18))
    x0, y0, x1, y1 = 70, 70, 1230, 540
    draw.rectangle((x0, y0, x1, y1), outline=(70, 70, 70), width=2)
    for pct in range(0, 101, 20):
        py = int(y1 - pct / 100.0 * (y1 - y0))
        draw.line((x0, py, x1, py), fill=(225, 225, 225), width=1)
        draw.text((25, py - 6), f"{pct}%", fill=(60, 60, 60), font=font(11))
    group_width = (x1 - x0) / len(runs)
    for index, run in enumerate(runs):
        center = x0 + (index + 0.5) * group_width
        for offset, vehicle in ((-18, run["vehicles"][0]), (18, run["vehicles"][1])):
            value = float(vehicle["latest_alignment"].get("within_0p3_pct", 0.0))
            bar_x0 = int(center + offset - 14)
            bar_x1 = int(center + offset + 14)
            bar_y = int(y1 - value / 100.0 * (y1 - y0))
            color = COLORS[vehicle["vehicle_id"]]
            draw.rectangle((bar_x0, bar_y, bar_x1, y1), fill=color)
            draw.text((bar_x0 - 3, bar_y - 16), f"{value:.0f}", fill=color, font=font(10))
        label = labels[run["run_id"]]
        draw.text((int(center - 55), y1 + 16), label, fill=(40, 40, 40), font=font(10))
    draw.rectangle((70, 590, 86, 602), fill=COLORS["drone1"])
    draw.text((94, 587), "drone1", fill=(40, 40, 40), font=font(12))
    draw.rectangle((180, 590, 196, 602), fill=COLORS["drone2"])
    draw.text((204, 587), "drone2", fill=(40, 40, 40), font=font(12))
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--artifacts-root", default="artifacts")
    parser.add_argument("--output-dir", default="experiments/run_reports/assets")
    args = parser.parse_args()
    run_root = Path(args.artifacts_root) / args.run_id
    output = Path(args.output_dir)
    render_map(run_root, output / "2026-08-04_stable_fusion_raw_slam_debug.png")
    render_drift(run_root, output / "2026-08-04_raw_slam_tf_drift.png")
    render_matrix(Path(args.matrix), output / "2026-08-04_slam_experiment_matrix.png")


if __name__ == "__main__":
    main()
