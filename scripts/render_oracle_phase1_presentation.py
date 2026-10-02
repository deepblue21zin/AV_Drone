#!/usr/bin/env python3
"""Render presentation-ready figures from a drone_cslam phase-1 artifact."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import (  # noqa: E402
    FancyArrowPatch,
    FancyBboxPatch,
    Patch,
)


CONDITION_ORDER = ["B0", "B1", "O-single", "O-periodic"]
COLORS = {
    "GT": "#111827",
    "B0": "#dc2626",
    "B1": "#2563eb",
    "O-single": "#f59e0b",
    "O-periodic": "#16a34a",
}
DISPLAY_NAMES = {
    "B0": "B0: raw SLAM",
    "B1": "B1: GNSS-aided PX4 odometry",
    "O-single": "Oracle: single factor",
    "O-periodic": "Oracle: periodic factors",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--artifact",
        type=Path,
        required=True,
        help="Path to an oracle_correction artifact directory",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output directory (default: ARTIFACT/presentation)",
    )
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def set_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 11,
            "axes.titlesize": 14,
            "axes.labelsize": 11,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "grid.linestyle": "--",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
        }
    )


def save_figure(fig: plt.Figure, output: Path, dpi: int) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def numeric_row(row: dict[str, str]) -> dict[str, float | str]:
    converted: dict[str, float | str] = {}
    for key, value in row.items():
        try:
            converted[key] = float(value)
        except (TypeError, ValueError):
            converted[key] = value
    return converted


def load_trajectories(
    artifact: Path,
) -> dict[str, dict[str, list[dict[str, float | str]]]]:
    trajectories: dict[str, dict[str, list[dict[str, float | str]]]] = {}
    for condition in CONDITION_ORDER:
        path = artifact / "trajectories" / f"{condition}.csv"
        if not path.exists():
            continue
        by_vehicle: dict[str, list[dict[str, float | str]]] = {}
        for raw_row in read_csv(path):
            row = numeric_row(raw_row)
            by_vehicle.setdefault(str(row["vehicle"]), []).append(row)
        for rows in by_vehicle.values():
            rows.sort(key=lambda item: float(item["timestamp"]))
        trajectories[condition] = by_vehicle
    return trajectories


def load_factors(artifact: Path) -> list[dict]:
    factors = []
    path = artifact / "factors.jsonl"
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                factors.append(json.loads(line))
    return factors


def load_map(artifact: Path, condition: str) -> np.ndarray:
    return np.load(artifact / "maps" / f"{condition}.npy")


def wrap_radians(angle: np.ndarray | float) -> np.ndarray:
    """Wrap an angle or angle array to [-pi, pi)."""
    values = np.asarray(angle, dtype=float)
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def absolute_yaw_error_deg(rows: list[dict[str, float | str]]) -> np.ndarray:
    """Return per-keyframe global-frame yaw error against GT."""
    estimated = np.asarray([float(row["yaw"]) for row in rows], dtype=float)
    ground_truth = np.asarray([float(row["gt_yaw"]) for row in rows], dtype=float)
    return np.degrees(np.abs(wrap_radians(estimated - ground_truth)))


def start_aligned_relative_yaw_drift_deg(
    by_vehicle: dict[str, list[dict[str, float | str]]],
    targets: list[float],
) -> np.ndarray:
    """Return inter-UAV yaw drift after removing the initial relative offset."""
    vehicles = sorted(by_vehicle)
    if len(vehicles) != 2:
        raise ValueError("start-aligned relative yaw requires exactly two UAVs")
    first_rows = by_vehicle[vehicles[0]]
    second_rows = by_vehicle[vehicles[1]]
    first_progress = np.asarray(
        [float(row["progress_m"]) for row in first_rows], dtype=float
    )
    second_progress = np.asarray(
        [float(row["progress_m"]) for row in second_rows], dtype=float
    )
    signed_errors = []
    for target in targets:
        first = first_rows[int(np.argmin(np.abs(first_progress - target)))]
        second = second_rows[int(np.argmin(np.abs(second_progress - target)))]
        estimated_relative = float(second["yaw"]) - float(first["yaw"])
        truth_relative = float(second["gt_yaw"]) - float(first["gt_yaw"])
        signed_errors.append(float(wrap_radians(estimated_relative - truth_relative)))
    signed_errors_array = np.asarray(signed_errors, dtype=float)
    return np.degrees(
        np.abs(wrap_radians(signed_errors_array - signed_errors_array[0]))
    )


def observed_crop(grids: list[np.ndarray], margin: int = 10) -> tuple[slice, slice]:
    observed = np.zeros_like(grids[0], dtype=bool)
    for grid in grids:
        observed |= grid >= 0
    ys, xs = np.where(observed)
    if len(xs) == 0:
        return slice(0, grids[0].shape[0]), slice(0, grids[0].shape[1])
    y0 = max(int(ys.min()) - margin, 0)
    y1 = min(int(ys.max()) + margin + 1, grids[0].shape[0])
    x0 = max(int(xs.min()) - margin, 0)
    x1 = min(int(xs.max()) + margin + 1, grids[0].shape[1])
    return slice(y0, y1), slice(x0, x1)


def load_map_spec(artifact: Path) -> tuple[float, float, float]:
    # The generated YAML format is deliberately simple; avoid a PyYAML dependency.
    yaml_path = artifact / "maps" / "GT-reference.yaml"
    resolution = 0.1
    origin_x = -10.0
    origin_y = -20.0
    lines = yaml_path.read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        if line.startswith("resolution:"):
            resolution = float(line.split(":", 1)[1].strip())
        elif line.startswith("origin:"):
            value = line.split(":", 1)[1].strip()
            if value.startswith("["):
                values = value.strip("[]").split(",")
                origin_x, origin_y = float(values[0]), float(values[1])
            else:
                origin_x = float(lines[index + 1].split("-", 1)[1].strip())
                origin_y = float(lines[index + 2].split("-", 1)[1].strip())
    return resolution, origin_x, origin_y


def crop_extent(
    crop: tuple[slice, slice], resolution: float, origin_x: float, origin_y: float
) -> list[float]:
    ys, xs = crop
    return [
        origin_x + int(xs.start) * resolution,
        origin_x + int(xs.stop) * resolution,
        origin_y + int(ys.start) * resolution,
        origin_y + int(ys.stop) * resolution,
    ]


def render_pipeline(output: Path, dpi: int) -> None:
    fig, ax = plt.subplots(figsize=(15, 7.2))
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 8)
    ax.axis("off")

    boxes = [
        (0.4, 4.4, 2.2, 1.45, "Gazebo / PX4", "2 UAV + LiDAR\nGT odometry"),
        (3.1, 4.4, 2.2, 1.45, "ROS 2 bag", "scan / odom / TF\nGT / diagnostics"),
        (5.8, 4.4, 2.2, 1.45, "Dataset", "time sync\nkeyframe sampling"),
        (8.5, 4.4, 2.2, 1.45, "Factor graph", "intra-UAV odometry\ninter-UAV oracle"),
        (11.2, 4.4, 2.2, 1.45, "SE(2) PGO", "SciPy least squares\nB0 / B1 / Oracle"),
        (8.5, 1.55, 2.2, 1.45, "Map reprojection", "pose-corrected scans\noccupancy grid"),
        (11.2, 1.55, 2.2, 1.45, "Evaluation", "relative drift\nChamfer / occupied F1"),
    ]
    fills = ["#e0f2fe", "#e0f2fe", "#ede9fe", "#fef3c7", "#dcfce7", "#f3e8ff", "#fee2e2"]
    for (x, y, width, height, title, body), fill in zip(boxes, fills):
        patch = FancyBboxPatch(
            (x, y),
            width,
            height,
            boxstyle="round,pad=0.08,rounding_size=0.12",
            linewidth=1.5,
            edgecolor="#334155",
            facecolor=fill,
        )
        ax.add_patch(patch)
        ax.text(x + width / 2, y + height * 0.68, title, ha="center", va="center", fontsize=13, fontweight="bold")
        ax.text(x + width / 2, y + height * 0.28, body, ha="center", va="center", fontsize=10, color="#334155")

    arrows = [
        ((2.62, 5.12), (3.08, 5.12)),
        ((5.32, 5.12), (5.78, 5.12)),
        ((8.02, 5.12), (8.48, 5.12)),
        ((10.72, 5.12), (11.18, 5.12)),
        ((12.3, 4.35), (9.62, 3.05)),
        ((10.72, 2.27), (11.18, 2.27)),
    ]
    for start, end in arrows:
        ax.add_patch(
            FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=16, linewidth=1.8, color="#475569")
        )

    ax.text(7.5, 7.35, "Phase-1 Oracle Drift-Correction Pipeline", ha="center", fontsize=22, fontweight="bold", color="#0f172a")
    ax.text(
        7.5,
        6.85,
        "GT is used only to create an upper-bound inter-UAV constraint and to evaluate the result",
        ha="center",
        fontsize=12,
        color="#475569",
    )
    ax.text(
        6.95,
        0.55,
        "Scientific question: if a correct inter-UAV match existed, could it reduce one-way relative drift?",
        ha="center",
        fontsize=13,
        color="#166534",
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#f0fdf4", "edgecolor": "#86efac"},
    )
    save_figure(fig, output / "01_pipeline.png", dpi)


def render_trajectories(artifact: Path, output: Path, dpi: int) -> None:
    trajectories = load_trajectories(artifact)
    if "B0" not in trajectories:
        raise FileNotFoundError("B0 trajectory is required to recover GT samples")
    vehicles = sorted(trajectories["B0"])
    fig, axes = plt.subplots(1, len(vehicles), figsize=(15, 6.2), squeeze=False)
    for ax, vehicle in zip(axes[0], vehicles):
        gt_rows = trajectories["B0"][vehicle]
        ax.plot(
            [float(row["gt_x"]) for row in gt_rows],
            [float(row["gt_y"]) for row in gt_rows],
            color=COLORS["GT"],
            linewidth=3.0,
            label="Ground truth",
            zorder=2,
        )
        for condition in CONDITION_ORDER:
            rows = trajectories.get(condition, {}).get(vehicle, [])
            if not rows:
                continue
            ax.plot(
                [float(row["x"]) for row in rows],
                [float(row["y"]) for row in rows],
                color=COLORS[condition],
                linewidth=2.0,
                marker="o",
                markersize=3.2,
                label=DISPLAY_NAMES[condition],
                alpha=0.9,
            )
        ax.set_title(vehicle.replace("drone", "UAV "))
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("auto")
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle("Estimated Keyframe Trajectories vs Ground Truth", fontsize=20, fontweight="bold", y=1.01)
    fig.tight_layout(rect=[0, 0.08, 1, 0.96])
    save_figure(fig, output / "02_trajectories.png", dpi)


def render_differential_error(metrics: dict, output: Path, dpi: int) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(15, 6.3), sharex=True)
    for condition in CONDITION_ORDER:
        samples = metrics["conditions"][condition]["differential"].get("samples", [])
        if not samples:
            continue
        progress = [float(sample["progress_m"]) for sample in samples]
        translation = [float(sample["translation_error_m"]) for sample in samples]
        yaw = [float(sample["yaw_error_deg"]) for sample in samples]
        axes[0].plot(
            progress,
            translation,
            color=COLORS[condition],
            linewidth=2.8,
            marker="o",
            markersize=6,
            label=DISPLAY_NAMES[condition],
        )
        axes[1].plot(
            progress,
            yaw,
            color=COLORS[condition],
            linewidth=2.8,
            marker="o",
            markersize=6,
            label=DISPLAY_NAMES[condition],
        )
    panels = [
        (
            axes[0],
            "Inter-UAV Relative Translation",
            "inter-UAV relative translation error [m]",
        ),
        (
            axes[1],
            "Inter-UAV Relative Yaw",
            "inter-UAV relative yaw error [deg]",
        ),
    ]
    for ax, title, ylabel in panels:
        ax.set_xlabel("Common one-way progress [m]")
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontweight="bold")
        ax.set_ylim(bottom=0)
        ax.text(
            0.98,
            0.96,
            "lower is better",
            transform=ax.transAxes,
            ha="right",
            va="top",
            color="#64748b",
            fontsize=10,
        )
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle(
        "Inter-UAV Relative Error Along the Shared One-Way Route",
        fontsize=20,
        fontweight="bold",
        y=1.02,
    )
    fig.text(
        0.5,
        0.96,
        "Relative yaw compares (UAV 2 - UAV 1) against GT; it is not per-UAV global yaw error.",
        ha="center",
        color="#475569",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.08, 1, 0.93])
    save_figure(fig, output / "03_differential_error.png", dpi)


def render_global_yaw_error(artifact: Path, output: Path, dpi: int) -> None:
    trajectories = load_trajectories(artifact)
    if "B0" not in trajectories:
        raise FileNotFoundError("B0 trajectory is required to identify the UAVs")
    vehicles = sorted(trajectories["B0"])
    maximum_error = max(
        float(np.max(absolute_yaw_error_deg(rows)))
        for condition in CONDITION_ORDER
        for rows in trajectories.get(condition, {}).values()
        if rows
    )
    fig, axes = plt.subplots(
        1,
        len(vehicles),
        figsize=(15, 6.3),
        squeeze=False,
        sharey=True,
    )
    for ax, vehicle in zip(axes[0], vehicles):
        for condition in CONDITION_ORDER:
            rows = trajectories.get(condition, {}).get(vehicle, [])
            if not rows:
                continue
            progress = [float(row["progress_m"]) for row in rows]
            error = absolute_yaw_error_deg(rows)
            ax.plot(
                progress,
                error,
                color=COLORS[condition],
                linewidth=2.8,
                marker="o",
                markersize=5,
                label=DISPLAY_NAMES[condition],
            )
        ax.set_title(
            vehicle.replace("drone", "UAV "),
            fontweight="bold",
        )
        ax.set_xlabel("Per-UAV one-way progress [m]")
        ax.set_ylabel("global-frame yaw error vs GT [deg]")
        ax.set_ylim(0, maximum_error * 1.10)
        ax.text(
            0.98,
            0.96,
            "lower is better",
            transform=ax.transAxes,
            ha="right",
            va="top",
            color="#64748b",
            fontsize=10,
        )
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="lower center",
        ncol=4,
        bbox_to_anchor=(0.5, -0.01),
    )
    fig.suptitle(
        "Per-UAV Global-Frame Yaw Error Against Ground Truth",
        fontsize=20,
        fontweight="bold",
        y=1.02,
    )
    fig.text(
        0.5,
        0.96,
        "Error = |wrap(estimated yaw - GT yaw)|. Oracle conditions also include the configured spawn-yaw priors.",
        ha="center",
        color="#475569",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.08, 1, 0.93])
    save_figure(fig, output / "03a_global_yaw_error.png", dpi)


def render_start_aligned_relative_yaw_drift(
    artifact: Path,
    metrics: dict,
    output: Path,
    dpi: int,
) -> None:
    trajectories = load_trajectories(artifact)
    samples = metrics["conditions"]["B0"]["differential"].get("samples", [])
    targets = [float(sample["progress_m"]) for sample in samples]
    if not targets:
        raise ValueError("B0 differential samples are required for yaw-drift targets")

    fig, ax = plt.subplots(figsize=(15, 6.3))
    for condition in CONDITION_ORDER:
        by_vehicle = trajectories.get(condition, {})
        if not by_vehicle:
            continue
        drift = start_aligned_relative_yaw_drift_deg(by_vehicle, targets)
        ax.plot(
            targets,
            drift,
            color=COLORS[condition],
            linewidth=2.8,
            marker="o",
            markersize=6,
            label=DISPLAY_NAMES[condition],
        )
    ax.set_xlabel("Common one-way progress [m]")
    ax.set_ylabel("start-aligned inter-UAV relative yaw drift [deg]")
    ax.set_ylim(bottom=0)
    ax.text(
        0.98,
        0.96,
        "lower means more stable",
        transform=ax.transAxes,
        ha="right",
        va="top",
        color="#64748b",
        fontsize=10,
    )
    ax.legend(loc="upper left", ncol=2)
    fig.suptitle(
        "Inter-UAV Relative Yaw Drift After Removing the Initial Offset",
        fontsize=20,
        fontweight="bold",
        y=1.02,
    )
    fig.text(
        0.5,
        0.96,
        "Every condition starts at 0 deg; this diagnoses stability after removing initial bias, not global accuracy.",
        ha="center",
        color="#475569",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.03, 1, 0.93])
    save_figure(fig, output / "03b_start_aligned_relative_yaw_drift.png", dpi)


def render_oracle_factors(artifact: Path, output: Path, dpi: int) -> None:
    keyframes = [numeric_row(row) for row in read_csv(artifact / "keyframes.csv")]
    by_node = {int(float(row["node"])): row for row in keyframes}
    by_vehicle: dict[str, list[dict[str, float | str]]] = {}
    for row in keyframes:
        by_vehicle.setdefault(str(row["vehicle"]), []).append(row)
    for rows in by_vehicle.values():
        rows.sort(key=lambda item: float(item["timestamp"]))
    factors = load_factors(artifact)

    fig, axes = plt.subplots(1, 2, figsize=(15, 6.3), sharex=True, sharey=True)
    for ax, condition in zip(axes, ["O-single", "O-periodic"]):
        for vehicle, rows in sorted(by_vehicle.items()):
            ax.plot(
                [float(row["gt_x"]) for row in rows],
                [float(row["gt_y"]) for row in rows],
                linewidth=2.4,
                marker="o",
                markersize=4,
                label=vehicle,
            )
        oracle = [
            factor
            for factor in factors
            if factor.get("condition") == condition and factor.get("type") == "oracle_inter_uav"
        ]
        for index, factor in enumerate(oracle):
            source = by_node[int(factor["source"])]
            target = by_node[int(factor["target"])]
            x_values = [float(source["gt_x"]), float(target["gt_x"])]
            y_values = [float(source["gt_y"]), float(target["gt_y"])]
            ax.plot(x_values, y_values, color="#9333ea", linewidth=3.2, linestyle="--", zorder=5)
            ax.scatter(x_values, y_values, color="#9333ea", s=54, zorder=6)
            ax.text(np.mean(x_values), np.mean(y_values), f"  C{index + 1}", color="#7e22ce", fontweight="bold")
        ax.set_title(f"{DISPLAY_NAMES[condition]}\n{len(oracle)} inter-UAV constraint(s)")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    handles = [
        Line2D([0], [0], color="#1f77b4", lw=2.4, marker="o", label="UAV 1 GT keyframes"),
        Line2D([0], [0], color="#ff7f0e", lw=2.4, marker="o", label="UAV 2 GT keyframes"),
        Line2D([0], [0], color="#9333ea", lw=3.2, ls="--", label="Oracle inter-UAV factor"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle("Where Inter-UAV Constraints Enter the Graph", fontsize=20, fontweight="bold", y=1.01)
    fig.tight_layout(rect=[0, 0.08, 1, 0.96])
    save_figure(fig, output / "04_oracle_factors.png", dpi)


def render_map_comparison(artifact: Path, output: Path, dpi: int) -> None:
    grids = {condition: load_map(artifact, condition) for condition in CONDITION_ORDER}
    gt = load_map(artifact, "GT-reference")
    crop = observed_crop([gt, *grids.values()])
    resolution, origin_x, origin_y = load_map_spec(artifact)
    extent = crop_extent(crop, resolution, origin_x, origin_y)
    occupancy_cmap = ListedColormap(["#ffffff", "#e2e8f0", "#111827"])

    fig, axes = plt.subplots(2, 2, figsize=(15, 8.3), sharex=True, sharey=True)
    for ax, condition in zip(axes.ravel(), CONDITION_ORDER):
        grid = grids[condition][crop]
        display = np.zeros_like(grid, dtype=np.uint8)
        display[grid == 0] = 1
        display[grid == 100] = 2
        ax.imshow(display, origin="lower", extent=extent, interpolation="nearest", cmap=occupancy_cmap, vmin=0, vmax=2)
        gt_occupied = (gt[crop] == 100).astype(float)
        if gt_occupied.max() > 0:
            ax.contour(gt_occupied, levels=[0.5], colors=["#06b6d4"], linewidths=0.8, origin="lower", extent=extent)
        ax.set_title(DISPLAY_NAMES[condition], color=COLORS[condition], fontweight="bold")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    fig.legend(
        handles=[
            Patch(facecolor="#111827", label="estimated occupied"),
            Patch(facecolor="#e2e8f0", label="estimated free"),
            Line2D([0], [0], color="#06b6d4", lw=2, label="GT occupied outline"),
        ],
        loc="lower center",
        ncol=3,
        bbox_to_anchor=(0.5, -0.01),
    )
    fig.suptitle("Reprojected Occupancy Maps", fontsize=20, fontweight="bold", y=1.01)
    fig.tight_layout(rect=[0, 0.06, 1, 0.96])
    save_figure(fig, output / "05_map_comparison.png", dpi)


def render_map_error_overlay(artifact: Path, output: Path, dpi: int) -> None:
    grids = {condition: load_map(artifact, condition) for condition in CONDITION_ORDER}
    gt = load_map(artifact, "GT-reference")
    crop = observed_crop([gt, *grids.values()])
    resolution, origin_x, origin_y = load_map_spec(artifact)
    extent = crop_extent(crop, resolution, origin_x, origin_y)
    error_cmap = ListedColormap(["#ffffff", "#e5e7eb", "#16a34a", "#dc2626", "#2563eb"])

    fig, axes = plt.subplots(2, 2, figsize=(15, 8.3), sharex=True, sharey=True)
    gt_crop = gt[crop]
    for ax, condition in zip(axes.ravel(), CONDITION_ORDER):
        estimate = grids[condition][crop]
        display = np.zeros_like(estimate, dtype=np.uint8)
        common = (estimate >= 0) & (gt_crop >= 0)
        est_occ = estimate == 100
        gt_occ = gt_crop == 100
        display[common] = 1
        display[common & est_occ & gt_occ] = 2
        display[common & est_occ & ~gt_occ] = 3
        display[common & ~est_occ & gt_occ] = 4
        ax.imshow(display, origin="lower", extent=extent, interpolation="nearest", cmap=error_cmap, vmin=0, vmax=4)
        ax.set_title(DISPLAY_NAMES[condition], color=COLORS[condition], fontweight="bold")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    fig.legend(
        handles=[
            Patch(facecolor="#16a34a", label="true positive occupied"),
            Patch(facecolor="#dc2626", label="false positive occupied"),
            Patch(facecolor="#2563eb", label="missed GT occupied"),
            Patch(facecolor="#e5e7eb", label="common observed support"),
        ],
        loc="lower center",
        ncol=4,
        bbox_to_anchor=(0.5, -0.01),
    )
    fig.suptitle("Occupancy Error Overlay Against GT Reference", fontsize=20, fontweight="bold", y=1.01)
    fig.tight_layout(rect=[0, 0.06, 1, 0.96])
    save_figure(fig, output / "06_map_error_overlay.png", dpi)


def render_metric_summary(metrics: dict, output: Path, dpi: int) -> None:
    endpoint = []
    chamfer = []
    f1 = []
    for condition in CONDITION_ORDER:
        values = metrics["conditions"][condition]
        endpoint.append(float(values["differential"]["endpoint_translation_m"]))
        chamfer.append(float(values["map"]["occupied_chamfer_m"]))
        f1.append(float(values["map"]["occupied_f1"]))
    colors = [COLORS[condition] for condition in CONDITION_ORDER]
    short_names = ["B0", "B1", "Single", "Periodic"]

    fig, axes = plt.subplots(1, 3, figsize=(15, 6.2))
    panels = [
        (endpoint, "Endpoint relative drift", "error [m]", "lower is better", "{:.3f}"),
        (chamfer, "Map Chamfer distance", "distance [m]", "lower is better", "{:.3f}"),
        (f1, "Occupied-cell F1", "F1 score", "higher is better", "{:.3f}"),
    ]
    for ax, (values, title, ylabel, direction, value_format) in zip(axes, panels):
        bars = ax.bar(short_names, values, color=colors, width=0.68)
        ax.set_title(title, fontweight="bold")
        ax.set_ylabel(ylabel)
        ax.set_ylim(0, max(values) * 1.28 if max(values) > 0 else 1)
        ax.text(0.98, 0.97, direction, transform=ax.transAxes, ha="right", va="top", color="#64748b", fontsize=9)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), value_format.format(value), ha="center", va="bottom", fontsize=10, fontweight="bold")
        ax.tick_params(axis="x", rotation=12)
    fig.suptitle("Phase-1 Oracle Quantitative Summary", fontsize=20, fontweight="bold", y=1.01)
    fig.tight_layout()
    save_figure(fig, output / "07_metric_summary.png", dpi)


def render_gate_summary(metrics: dict, output: Path, dpi: int) -> None:
    gate = metrics.get("phase1_gate", {})
    checks = gate.get("checks", {})
    rows = []
    for key, value in checks.items():
        if isinstance(value, dict):
            passed = bool(value.get("passed", False))
        else:
            passed = bool(value)
        readable = key.replace("_", " ").capitalize()
        rows.append((readable, "PASS" if passed else "FAIL"))

    fig, ax = plt.subplots(figsize=(14, 7.5))
    ax.axis("off")
    overall_pass = bool(gate.get("pass", False))
    headline = "FORMAL GATE: PASS" if overall_pass else "FORMAL GATE: NOT YET PASSED"
    headline_color = "#166534" if overall_pass else "#991b1b"
    headline_fill = "#dcfce7" if overall_pass else "#fee2e2"
    ax.text(
        0.5,
        0.92,
        headline,
        ha="center",
        va="center",
        fontsize=23,
        fontweight="bold",
        color=headline_color,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": headline_fill, "edgecolor": headline_color},
    )
    ax.text(
        0.5,
        0.81,
        "Use this run as pipeline validation and a feasibility signal — not as a final algorithm claim.",
        ha="center",
        fontsize=13,
        color="#334155",
    )
    if rows:
        table = ax.table(
            cellText=[[name, status] for name, status in rows],
            colLabels=["Acceptance check", "Result"],
            colWidths=[0.72, 0.20],
            cellLoc="left",
            colLoc="left",
            bbox=[0.09, 0.18, 0.82, 0.53],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(12)
        for (row_index, column_index), cell in table.get_celld().items():
            cell.set_edgecolor("#cbd5e1")
            if row_index == 0:
                cell.set_facecolor("#e2e8f0")
                cell.set_text_props(fontweight="bold", color="#0f172a")
            elif column_index == 1:
                status = rows[row_index - 1][1]
                cell.set_facecolor("#dcfce7" if status == "PASS" else "#fee2e2")
                cell.set_text_props(fontweight="bold", color="#166534" if status == "PASS" else "#991b1b")
            else:
                cell.set_facecolor("#f8fafc")
    ax.text(
        0.5,
        0.08,
        "Next: complete the full route, run multiple seeds, and replace the oracle with scan-derived constraints.",
        ha="center",
        fontsize=13,
        color="#475569",
    )
    save_figure(fig, output / "08_gate_summary.png", dpi)


def main() -> None:
    args = parse_args()
    artifact = args.artifact.resolve()
    output = (args.output or artifact / "presentation").resolve()
    metrics_path = artifact / "metrics.json"
    if not metrics_path.exists():
        raise FileNotFoundError(f"Missing phase-1 metrics: {metrics_path}")
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))

    set_style()
    render_pipeline(output, args.dpi)
    render_trajectories(artifact, output, args.dpi)
    render_differential_error(metrics, output, args.dpi)
    render_global_yaw_error(artifact, output, args.dpi)
    render_start_aligned_relative_yaw_drift(artifact, metrics, output, args.dpi)
    render_oracle_factors(artifact, output, args.dpi)
    render_map_comparison(artifact, output, args.dpi)
    render_map_error_overlay(artifact, output, args.dpi)
    render_metric_summary(metrics, output, args.dpi)
    render_gate_summary(metrics, output, args.dpi)
    print(f"Rendered 10 figures to {output}")


if __name__ == "__main__":
    main()
