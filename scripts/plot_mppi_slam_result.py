#!/usr/bin/env python3
import argparse
import json

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from nav_msgs.msg import Path
from rclpy.serialization import deserialize_message

from plot_cylinder_world_trajectory import load_mission_path


def load_planned_path(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    latest = []
    while reader.has_next():
        topic, raw, _timestamp = reader.read_next()
        if topic == "/drone1/planner/slam/path":
            msg = deserialize_message(raw, Path)
            if msg.poses:
                latest = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
                break
    return latest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--grid", required=True)
    parser.add_argument("--meta", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    grid = np.load(args.grid)
    with open(args.meta, "r", encoding="utf-8") as stream:
        meta = json.load(stream)
    resolution = float(meta["resolution"])
    extent = [
        float(meta["origin_x"]),
        float(meta["origin_x"]) + grid.shape[1] * resolution,
        float(meta["origin_y"]),
        float(meta["origin_y"]) + grid.shape[0] * resolution,
    ]
    actual, duration = load_mission_path(args.bag)
    planned = load_planned_path(args.bag)

    display = np.full(grid.shape, 0.72, dtype=np.float32)
    display[grid == 0] = 1.0
    display[grid >= 50] = 0.0

    fig, ax = plt.subplots(figsize=(16, 5.2), dpi=180)
    ax.imshow(display, origin="lower", extent=extent, cmap="gray", vmin=0.0, vmax=1.0, aspect="equal")
    if planned:
        px, py = zip(*planned)
        ax.plot(px, py, "--", color="#ff9f1c", linewidth=1.6, label="SLAM-map A* global path", zorder=3)
    ax_x, ax_y = zip(*actual)
    ax.plot(ax_x, ax_y, color="#1565c0", linewidth=2.0, label=f"MPPI actual trajectory — {duration:.2f} s", zorder=4)
    ax.scatter([ax_x[0]], [ax_y[0]], color="#1ca65b", s=80, edgecolor="white", zorder=5, label="Start")
    ax.scatter([ax_x[-1]], [ax_y[-1]], marker="*", color="#e63946", s=150, edgecolor="white", zorder=5, label="Goal")
    ax.set_xlim(-2, 152)
    ax.set_ylim(-16, 16)
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.set_title("MPPI-SLAM — Saved Map Global Path and Actual Flight")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", framealpha=0.96)
    fig.tight_layout()
    fig.savefig(args.output, bbox_inches="tight")
    print(args.output)


if __name__ == "__main__":
    main()
