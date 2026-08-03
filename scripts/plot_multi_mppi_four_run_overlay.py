#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

from plot_multi_mppi_trajectory import DRONES, load_cylinders, load_paths


RUN_STYLES = ("-", "--", "-.", ":")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--experiment-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--runs",
        type=int,
        default=None,
        help="Number of run_XX directories to include (default: auto-discover)",
    )
    args = parser.parse_args()

    cylinders = load_cylinders(args.world)
    experiment_dir = Path(args.experiment_dir)
    world_name = Path(args.world).stem
    all_paths = {}

    if args.runs is not None:
        if args.runs < 1:
            parser.error("--runs must be positive")
        run_names = [f"run_{number:02d}" for number in range(1, args.runs + 1)]
    else:
        run_names = sorted(path.name for path in experiment_dir.glob("run_[0-9]*") if path.is_dir())
    if not run_names:
        raise RuntimeError("No run_XX directories found")

    for run_name in run_names:
        all_paths[run_name] = load_paths(str(experiment_dir / run_name / "rosbag"))

    fig, ax = plt.subplots(figsize=(16, 5.2), dpi=180)
    ax.add_collection(
        PatchCollection(
            [Circle((x, y), radius) for x, y, radius in cylinders],
            facecolor="#606870",
            edgecolor="#20252a",
            linewidth=0.3,
            alpha=0.78,
            zorder=1,
        )
    )

    for run_index, (run_name, run_paths) in enumerate(all_paths.items()):
        line_style = RUN_STYLES[run_index % len(RUN_STYLES)]
        for drone_name, values in run_paths.items():
            xs, ys = zip(*values["points"])
            color = DRONES[drone_name]["color"]
            ax.plot(
                xs,
                ys,
                color=color,
                linestyle=line_style,
                linewidth=1.45,
                alpha=0.9,
                zorder=3,
            )
            ax.scatter(xs[0], ys[0], marker="o", s=26, color=color, zorder=4)
            ax.scatter(xs[-1], ys[-1], marker="*", s=55, color=color, zorder=4)

    drone_legend = [
        Line2D([0], [0], color=values["color"], linewidth=2.2, label=name)
        for name, values in DRONES.items()
    ]
    run_legend = [
        Line2D(
            [0],
            [0],
            color="#20252a",
            linestyle=RUN_STYLES[index % len(RUN_STYLES)],
            linewidth=1.6,
            label=f"Run {index + 1:02d}",
        )
        for index in range(len(all_paths))
    ]
    first_legend = ax.legend(
        handles=drone_legend,
        loc="upper left",
        title="Vehicle",
        framealpha=0.95,
    )
    ax.add_artist(first_legend)
    ax.legend(
        handles=run_legend,
        loc="upper right",
        title="Experiment",
        ncol=2,
        framealpha=0.95,
    )

    ax.set_title(
        f"{len(DRONES)}-UAV LiDAR MPPI — {len(all_paths)}-run trajectory overlay\n"
        f"{world_name}"
    )
    ax.set_xlabel("World X [m]")
    ax.set_ylabel("World Y [m]")
    ax.set_xlim(0, 150)
    ax.set_ylim(-15, 15)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.45, alpha=0.55)
    fig.tight_layout()
    fig.savefig(args.output, bbox_inches="tight")
    print(
        f"output={args.output} runs={len(all_paths)} "
        f"trajectories={len(all_paths) * len(DRONES)} obstacles={len(cylinders)}"
    )


if __name__ == "__main__":
    main()
