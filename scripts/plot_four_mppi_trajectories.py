#!/usr/bin/env python3
import argparse

import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle

from plot_cylinder_world_trajectory import load_cylinders, load_mission_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("bags", nargs=4)
    args = parser.parse_args()

    cylinders = load_cylinders(args.world)
    colors = ["#e63946", "#1d70b8", "#2a9d55", "#8e44ad"]

    fig, ax = plt.subplots(figsize=(16, 5.2), dpi=180)
    obstacles = [Circle((x, y), radius) for x, y, radius in cylinders]
    ax.add_collection(PatchCollection(
        obstacles,
        facecolor="#59636e",
        edgecolor="#20252a",
        linewidth=0.35,
        alpha=0.82,
        zorder=1,
    ))

    for index, (bag, color) in enumerate(zip(args.bags, colors), start=1):
        path, duration = load_mission_path(bag)
        xs, ys = zip(*path)
        ax.plot(
            xs,
            ys,
            color=color,
            linewidth=2.0,
            alpha=0.92,
            label=f"MPPI run {index} — {duration:.2f} s",
            zorder=3 + index,
        )

    ax.scatter([0.0], [0.0], marker="o", s=85, color="#17a866", edgecolor="white", linewidth=1.2, zorder=10)
    ax.scatter([140.0], [0.0], marker="*", s=160, color="#ef233c", edgecolor="white", linewidth=1.0, zorder=10)
    ax.annotate("Start", (0.0, 0.0), xytext=(6, 8), textcoords="offset points", fontsize=9)
    ax.annotate("Goal", (140.0, 0.0), xytext=(-32, 9), textcoords="offset points", fontsize=9)

    ax.set_title("LiDAR MPPI — Four Runs in Random Cylinder World")
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.set_xlim(-2, 152)
    ax.set_ylim(-16, 16)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#cfd5db", linewidth=0.45, alpha=0.65)
    ax.legend(loc="upper right", framealpha=0.96)
    fig.tight_layout()
    fig.savefig(args.output, bbox_inches="tight")
    print(args.output)


if __name__ == "__main__":
    main()
