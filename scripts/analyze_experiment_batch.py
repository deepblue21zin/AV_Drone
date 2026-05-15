#!/usr/bin/env python3

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


DEFAULT_METRICS = [
    "outbound_time_s",
    "return_time_s",
    "total_path_length_m",
    "return_path_length_m",
    "min_obstacle_distance_m",
    "safety_event_count",
    "escape_count",
    "map_coverage",
]


def load_json(path: Path):
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def as_float(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def summarize(values):
    clean = [v for v in values if v is not None]
    if not clean:
        return {"count": 0, "mean": "", "std": "", "min": "", "max": ""}
    mean = sum(clean) / len(clean)
    variance = sum((v - mean) ** 2 for v in clean) / len(clean)
    return {
        "count": len(clean),
        "mean": mean,
        "std": math.sqrt(variance),
        "min": min(clean),
        "max": max(clean),
    }


def write_csv(path: Path, rows, headers):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=headers)
        writer.writeheader()
        for row in rows:
            writer.writerow({header: row.get(header, "") for header in headers})


def write_markdown(path: Path, rows, headers):
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(header, "")) for header in headers) + " |")
    path.write_text("\n".join(lines) + "\n")


def fmt(value):
    if value == "":
        return ""
    if isinstance(value, float):
        return f"{value:.4f}"
    return value


def main():
    parser = argparse.ArgumentParser(description="Aggregate AV_Drone paper_metrics.json files.")
    parser.add_argument("--artifacts-root", default="artifacts")
    parser.add_argument("--output-dir", default="experiments/paper_outputs")
    parser.add_argument("--metrics", nargs="*", default=DEFAULT_METRICS)
    args = parser.parse_args()

    artifacts_root = Path(args.artifacts_root)
    output_dir = Path(args.output_dir)
    records = []
    for metrics_path in sorted(artifacts_root.glob("*_drone*/paper_metrics.json")):
        payload = load_json(metrics_path)
        if not payload:
            continue
        payload["_artifact_path"] = str(metrics_path.parent)
        records.append(payload)

    by_condition = defaultdict(list)
    for record in records:
        by_condition[record.get("condition") or "unknown"].append(record)

    rows = []
    for condition, condition_records in sorted(by_condition.items()):
        success_values = [1.0 if record.get("success") else 0.0 for record in condition_records]
        success_summary = summarize(success_values)
        rows.append({
            "condition": condition,
            "metric": "success_rate",
            "runs": len(condition_records),
            "valid": success_summary["count"],
            "mean": fmt(success_summary["mean"]),
            "std": fmt(success_summary["std"]),
            "min": fmt(success_summary["min"]),
            "max": fmt(success_summary["max"]),
        })
        for metric in args.metrics:
            values = [as_float(record.get(metric)) for record in condition_records]
            summary = summarize(values)
            rows.append({
                "condition": condition,
                "metric": metric,
                "runs": len(condition_records),
                "valid": summary["count"],
                "mean": fmt(summary["mean"]),
                "std": fmt(summary["std"]),
                "min": fmt(summary["min"]),
                "max": fmt(summary["max"]),
            })

    headers = ["condition", "metric", "runs", "valid", "mean", "std", "min", "max"]
    write_csv(output_dir / "summary_table.csv", rows, headers)
    write_markdown(output_dir / "summary_table.md", rows, headers)
    print(f"Wrote {output_dir / 'summary_table.csv'}")
    print(f"Wrote {output_dir / 'summary_table.md'}")


if __name__ == "__main__":
    main()
