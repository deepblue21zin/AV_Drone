#!/usr/bin/env python3

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def load_json(path: Path):
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def load_trajectory(path: Path):
    if not path.exists():
        return []
    with path.open("r", newline="") as handle:
        return list(csv.DictReader(handle))


def as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def condition_records(artifacts_root: Path):
    records = []
    for metrics_path in sorted(artifacts_root.glob("*_drone*/paper_metrics.json")):
        metrics = load_json(metrics_path)
        if not metrics:
            continue
        artifact_dir = metrics_path.parent
        records.append({
            "artifact_dir": artifact_dir,
            "metrics": metrics,
            "trajectory": load_trajectory(artifact_dir / "trajectory.csv"),
        })
    return records


def plot_trajectories(records, output_dir, plt):
    grouped = defaultdict(list)
    for record in records:
        grouped[record["metrics"].get("condition") or "unknown"].append(record)

    for condition, condition_records_ in grouped.items():
        plt.figure(figsize=(8, 5))
        for record in condition_records_:
            trajectory = record["trajectory"]
            if not trajectory:
                continue
            xs = [as_float(row.get("x")) for row in trajectory]
            ys = [as_float(row.get("y")) for row in trajectory]
            label = record["metrics"].get("run_id", record["artifact_dir"].name)
            plt.plot(xs, ys, linewidth=1.2, label=label)
        plt.title(f"Trajectory - {condition}")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.axis("equal")
        plt.grid(True, alpha=0.3)
        if len(condition_records_) <= 8:
            plt.legend(fontsize=7)
        plt.tight_layout()
        plt.savefig(output_dir / f"trajectory_{condition}.png", dpi=160)
        plt.close()


def plot_metric_bars(records, output_dir, plt):
    grouped = defaultdict(list)
    for record in records:
        grouped[record["metrics"].get("condition") or "unknown"].append(record["metrics"])

    for metric, filename, title in [
        ("return_time_s", "return_time_comparison.png", "Return Time"),
        ("total_path_length_m", "path_length_comparison.png", "Total Path Length"),
        ("min_obstacle_distance_m", "min_distance_comparison.png", "Minimum Obstacle Distance"),
        ("safety_event_count", "safety_event_comparison.png", "Safety Event Count"),
    ]:
        labels = []
        means = []
        for condition, rows in sorted(grouped.items()):
            values = [as_float(row.get(metric), None) for row in rows]
            values = [value for value in values if value is not None]
            if not values:
                continue
            labels.append(condition)
            means.append(sum(values) / len(values))
        if not labels:
            continue
        plt.figure(figsize=(8, 4))
        plt.bar(labels, means)
        plt.title(title)
        plt.xticks(rotation=20, ha="right")
        plt.grid(axis="y", alpha=0.3)
        plt.tight_layout()
        plt.savefig(output_dir / filename, dpi=160)
        plt.close()


def main():
    parser = argparse.ArgumentParser(description="Generate paper figures from AV_Drone artifacts.")
    parser.add_argument("--artifacts-root", default="artifacts")
    parser.add_argument("--output-dir", default="experiments/paper_outputs/figures")
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required to generate figures: {exc}") from exc

    artifacts_root = Path(args.artifacts_root)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    records = condition_records(artifacts_root)
    plot_trajectories(records, output_dir, plt)
    plot_metric_bars(records, output_dir, plt)
    print(f"Wrote figures under {output_dir}")


if __name__ == "__main__":
    main()
