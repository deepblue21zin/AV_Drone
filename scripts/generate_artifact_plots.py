#!/usr/bin/env python3

import argparse
import csv
import json
import math
from pathlib import Path
from xml.sax.saxutils import escape


NUMERIC_FIELDS = [
    "t_sec",
    "pose_count",
    "scan_count",
    "planner_cmd_count",
    "safe_cmd_count",
    "current_obstacle_m",
    "nearest_obstacle_m",
    "safety_event_count",
]

WIDTH = 960
HEIGHT = 540
PLOT_LEFT = 90
PLOT_RIGHT = 40
PLOT_TOP = 60
PLOT_BOTTOM = 70
PLOT_WIDTH = WIDTH - PLOT_LEFT - PLOT_RIGHT
PLOT_HEIGHT = HEIGHT - PLOT_TOP - PLOT_BOTTOM


def latest_artifact(artifacts_root: Path) -> Path:
    candidates = sorted(
        [p for p in artifacts_root.glob("*_drone1") if p.is_dir()],
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(f"No artifact directory found under {artifacts_root}")
    return candidates[0]


def parse_float(value: str):
    text = str(value).strip()
    if text in {"", "None"}:
        return None
    if text in {"inf", ".inf", "Infinity"}:
        return math.inf
    return float(text)


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() == "true"


def load_metrics(metrics_path: Path):
    rows = []
    with metrics_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for key, value in row.items():
                if key in NUMERIC_FIELDS:
                    parsed[key] = parse_float(value)
                elif key in {"connected", "armed", "goal_reached"}:
                    parsed[key] = parse_bool(value)
                else:
                    parsed[key] = value
            rows.append(parsed)
    if not rows:
        raise ValueError(f"No metric rows found in {metrics_path}")
    return rows


def load_json(path: Path):
    return json.loads(path.read_text())


def svg_root(title: str, body: str) -> str:
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">'
        f'<rect x="0" y="0" width="{WIDTH}" height="{HEIGHT}" fill="#ffffff"/>'
        f'<text x="{WIDTH/2:.0f}" y="32" text-anchor="middle" font-size="24" font-family="Arial" fill="#17212b">{escape(title)}</text>'
        f"{body}</svg>"
    )


def value_to_y(value: float, min_y: float, max_y: float) -> float:
    if max_y <= min_y:
        return PLOT_TOP + PLOT_HEIGHT / 2
    ratio = (value - min_y) / (max_y - min_y)
    return PLOT_TOP + PLOT_HEIGHT - ratio * PLOT_HEIGHT


def value_to_x(value: float, min_x: float, max_x: float) -> float:
    if max_x <= min_x:
        return PLOT_LEFT + PLOT_WIDTH / 2
    ratio = (value - min_x) / (max_x - min_x)
    return PLOT_LEFT + ratio * PLOT_WIDTH


def build_axes(min_x: float, max_x: float, min_y: float, max_y: float, x_label: str, y_label: str) -> str:
    parts = [
        f'<line x1="{PLOT_LEFT}" y1="{PLOT_TOP}" x2="{PLOT_LEFT}" y2="{PLOT_TOP + PLOT_HEIGHT}" stroke="#4f5b66" stroke-width="1.5"/>',
        f'<line x1="{PLOT_LEFT}" y1="{PLOT_TOP + PLOT_HEIGHT}" x2="{PLOT_LEFT + PLOT_WIDTH}" y2="{PLOT_TOP + PLOT_HEIGHT}" stroke="#4f5b66" stroke-width="1.5"/>',
        f'<text x="{PLOT_LEFT + PLOT_WIDTH / 2:.0f}" y="{HEIGHT - 20}" text-anchor="middle" font-size="16" font-family="Arial" fill="#3f4d4a">{escape(x_label)}</text>',
        f'<text x="24" y="{PLOT_TOP + PLOT_HEIGHT / 2:.0f}" transform="rotate(-90 24 {PLOT_TOP + PLOT_HEIGHT / 2:.0f})" text-anchor="middle" font-size="16" font-family="Arial" fill="#3f4d4a">{escape(y_label)}</text>',
    ]
    for step in range(6):
        x_value = min_x + (max_x - min_x) * step / 5 if max_x > min_x else min_x
        x = value_to_x(x_value, min_x, max_x)
        parts.append(f'<line x1="{x:.1f}" y1="{PLOT_TOP}" x2="{x:.1f}" y2="{PLOT_TOP + PLOT_HEIGHT}" stroke="#dde5ec" stroke-width="1"/>')
        parts.append(f'<text x="{x:.1f}" y="{PLOT_TOP + PLOT_HEIGHT + 22}" text-anchor="middle" font-size="12" font-family="Arial" fill="#607080">{x_value:.1f}</text>')
    for step in range(6):
        y_value = min_y + (max_y - min_y) * step / 5 if max_y > min_y else min_y
        y = value_to_y(y_value, min_y, max_y)
        parts.append(f'<line x1="{PLOT_LEFT}" y1="{y:.1f}" x2="{PLOT_LEFT + PLOT_WIDTH}" y2="{y:.1f}" stroke="#dde5ec" stroke-width="1"/>')
        parts.append(f'<text x="{PLOT_LEFT - 10}" y="{y + 4:.1f}" text-anchor="end" font-size="12" font-family="Arial" fill="#607080">{y_value:.2f}</text>')
    return "".join(parts)


def polyline(xs, ys, min_x, max_x, min_y, max_y, color, stroke_width=2.5):
    points = []
    for x, y in zip(xs, ys):
        if y is None or not math.isfinite(y):
            continue
        points.append(
            f"{value_to_x(x, min_x, max_x):.1f},{value_to_y(y, min_y, max_y):.1f}"
        )
    if len(points) < 2:
        return ""
    return f'<polyline fill="none" stroke="{color}" stroke-width="{stroke_width}" points="{" ".join(points)}"/>'


def legend(items):
    x = WIDTH - 260
    y = 56
    parts = []
    for idx, (label, color) in enumerate(items):
        yy = y + idx * 22
        parts.append(f'<line x1="{x}" y1="{yy}" x2="{x + 18}" y2="{yy}" stroke="{color}" stroke-width="3"/>')
        parts.append(f'<text x="{x + 26}" y="{yy + 4}" font-size="13" font-family="Arial" fill="#3f4d4a">{escape(label)}</text>')
    return "".join(parts)


def save_svg(path: Path, content: str):
    path.write_text(content)
    return path


def save_counts_plot(rows, plot_dir: Path):
    xs = [row["t_sec"] for row in rows]
    series = [
        ("pose_count", "#1e88e5"),
        ("scan_count", "#ef6c00"),
        ("planner_cmd_count", "#43a047"),
        ("safe_cmd_count", "#6a1b9a"),
    ]
    ys = [[row[name] for row in rows] for name, _ in series]
    max_y = max(max(values) for values in ys)
    body = build_axes(min(xs), max(xs), 0, max_y, "Runtime [s]", "Cumulative count")
    for (name, color), values in zip(series, ys):
        body += polyline(xs, values, min(xs), max(xs), 0, max_y, color)
    body += legend(series)
    return save_svg(plot_dir / "counts_over_time.svg", svg_root("Runtime Message Counts", body))


def save_obstacle_plot(rows, plot_dir: Path):
    xs = [row["t_sec"] for row in rows]
    current = [row["current_obstacle_m"] if row["current_obstacle_m"] not in {None, math.inf} else None for row in rows]
    nearest = [row["nearest_obstacle_m"] if row["nearest_obstacle_m"] not in {None, math.inf} else None for row in rows]
    finite_values = [value for value in current + nearest if value is not None]
    max_y = max(finite_values) if finite_values else 1.0
    body = build_axes(min(xs), max(xs), 0, max_y, "Runtime [s]", "Distance [m]")
    body += polyline(xs, current, min(xs), max(xs), 0, max_y, "#ef6c00")
    body += polyline(xs, nearest, min(xs), max(xs), 0, max_y, "#1e88e5")
    body += legend([("current_obstacle_m", "#ef6c00"), ("closest_obstacle_m", "#1e88e5")])
    return save_svg(plot_dir / "obstacle_distance.svg", svg_root("Obstacle Distance Over Time", body))


def save_phase_plot(rows, plot_dir: Path):
    xs = [row["t_sec"] for row in rows]
    phases = [row["mission_phase"] for row in rows]
    unique_phases = []
    for phase in phases:
        if phase not in unique_phases:
            unique_phases.append(phase)
    mapping = {phase: idx for idx, phase in enumerate(unique_phases)}
    phase_values = [mapping[phase] for phase in phases]
    goal_values = [1.0 if row["goal_reached"] else 0.0 for row in rows]
    max_y = max(phase_values) + 1 if phase_values else 1
    body = build_axes(min(xs), max(xs), 0, max_y, "Runtime [s]", "Phase index")
    body += polyline(xs, phase_values, min(xs), max(xs), 0, max_y, "#1e88e5")
    body += polyline(xs, goal_values, min(xs), max(xs), 0, max_y, "#ef6c00")
    body += legend([("mission_phase", "#1e88e5"), ("goal_reached", "#ef6c00")])
    labels = "".join(
        f'<text x="{WIDTH - 260}" y="{160 + idx * 18}" font-size="12" font-family="Arial" fill="#3f4d4a">{idx}: {escape(phase)}</text>'
        for idx, phase in enumerate(unique_phases)
    )
    body += labels
    return save_svg(plot_dir / "mission_timeline.svg", svg_root("Mission Phase Timeline", body))


def save_latency_plot(summary, plot_dir: Path):
    pose_values = [
        float(summary.get("pose_period_mean_s") or 0.0) * 1000.0,
        float(summary.get("pose_period_p99_s") or 0.0) * 1000.0,
        float(summary.get("pose_period_worst_s") or 0.0) * 1000.0,
    ]
    scan_values = [
        float(summary.get("scan_period_mean_s") or 0.0) * 1000.0,
        float(summary.get("scan_period_p99_s") or 0.0) * 1000.0,
        float(summary.get("scan_period_worst_s") or 0.0) * 1000.0,
    ]
    categories = ["mean", "p99", "worst"]
    max_y = max(pose_values + scan_values + [1.0])
    body = build_axes(0, len(categories), 0, max_y, "Statistic", "Period [ms]")
    for idx, category in enumerate(categories):
        base_x = PLOT_LEFT + (idx + 0.5) * (PLOT_WIDTH / len(categories))
        bar_width = 30
        pose_h = (pose_values[idx] / max_y) * PLOT_HEIGHT if max_y > 0 else 0
        scan_h = (scan_values[idx] / max_y) * PLOT_HEIGHT if max_y > 0 else 0
        body += f'<rect x="{base_x - 40:.1f}" y="{PLOT_TOP + PLOT_HEIGHT - pose_h:.1f}" width="{bar_width}" height="{pose_h:.1f}" fill="#1e88e5"/>'
        body += f'<rect x="{base_x + 10:.1f}" y="{PLOT_TOP + PLOT_HEIGHT - scan_h:.1f}" width="{bar_width}" height="{scan_h:.1f}" fill="#ef6c00"/>'
        body += f'<text x="{base_x:.1f}" y="{PLOT_TOP + PLOT_HEIGHT + 22}" text-anchor="middle" font-size="12" font-family="Arial" fill="#607080">{category}</text>'
    body += legend([("pose_period_ms", "#1e88e5"), ("scan_period_ms", "#ef6c00")])
    return save_svg(plot_dir / "loop_period_summary.svg", svg_root("Loop Period Summary", body))


def write_manifest(artifact_dir: Path, plot_dir: Path, plot_paths, metadata, summary):
    manifest = {
        "artifact_dir": str(artifact_dir),
        "scenario_name": metadata.get("scenario_name", summary.get("scenario_name", "unknown")),
        "git_commit": metadata.get("git_commit", summary.get("git_commit", "unknown")),
        "plots": [str(path) for path in plot_paths],
    }
    (plot_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))


def main():
    parser = argparse.ArgumentParser(description="Generate portable SVG plots from a drone artifact directory.")
    parser.add_argument("--artifact", default="", help="Artifact directory path. If omitted, use the latest artifact.")
    parser.add_argument("--artifacts-root", default="artifacts", help="Artifacts root directory.")
    parser.add_argument("--output-root", default="experiments/plots", help="Directory where generated plot folders are stored.")
    args = parser.parse_args()

    artifact_dir = Path(args.artifact) if args.artifact else latest_artifact(Path(args.artifacts_root))
    metrics_path = artifact_dir / "metrics.csv"
    summary_path = artifact_dir / "summary.json"
    metadata_path = artifact_dir / "metadata.json"
    if not metrics_path.exists():
        raise FileNotFoundError(f"Missing metrics file: {metrics_path}")
    if not summary_path.exists():
        raise FileNotFoundError(f"Missing summary file: {summary_path}")
    if not metadata_path.exists():
        raise FileNotFoundError(f"Missing metadata file: {metadata_path}")

    rows = load_metrics(metrics_path)
    summary = load_json(summary_path)
    metadata = load_json(metadata_path)
    plot_dir = Path(args.output_root) / artifact_dir.name
    plot_dir.mkdir(parents=True, exist_ok=True)

    plots = [
        save_counts_plot(rows, plot_dir),
        save_obstacle_plot(rows, plot_dir),
        save_phase_plot(rows, plot_dir),
        save_latency_plot(summary, plot_dir),
    ]
    write_manifest(artifact_dir, plot_dir, plots, metadata, summary)
    for path in plots:
        print(path)


if __name__ == "__main__":
    main()
