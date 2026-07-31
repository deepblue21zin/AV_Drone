#!/usr/bin/env python3
import argparse
import math
import re
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool, String


def load_cylinders(world_path):
    root = ET.parse(world_path).getroot()
    cylinders = []
    for include in root.findall(".//include"):
        name = include.findtext("name", "")
        if not name.startswith("cylinder_"):
            continue
        pose = [float(v) for v in include.findtext("pose").split()]
        uri = include.findtext("uri", "")
        match = re.search(r"_r(\d+)", uri)
        radius = float(match.group(1)) / 10.0 if match else 0.5
        cylinders.append((pose[0], pose[1], radius))
    return cylinders


def load_mission_path(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    start_time = None
    goal_time = None
    poses = []
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if topic == "/drone1/mission/phase":
            phase = deserialize_message(raw, String).data
            if phase in {"MPPI_GO", "MAPPING_TO_GOAL", "FOLLOW_PLAN"} and start_time is None:
                start_time = timestamp
        elif topic == "/drone1/mission/goal_reached":
            if deserialize_message(raw, Bool).data and goal_time is None:
                goal_time = timestamp
        elif topic == "/mavros/local_position/pose":
            p = deserialize_message(raw, PoseStamped).pose.position
            poses.append((timestamp, p.x, p.y))
    if start_time is None or goal_time is None:
        raise RuntimeError("비행 시작 또는 goal_reached 시점을 rosbag에서 찾지 못했습니다.")
    path = [(x, y) for t, x, y in poses if start_time <= t <= goal_time]
    return path, (goal_time - start_time) / 1e9


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--planner", default="LiDAR MPPI")
    args = parser.parse_args()

    cylinders = load_cylinders(args.world)
    path, duration = load_mission_path(args.bag)
    distance = sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(path, path[1:]))

    fig, ax = plt.subplots(figsize=(16, 5.2), dpi=180)
    patches = [Circle((x, y), r) for x, y, r in cylinders]
    ax.add_collection(PatchCollection(patches, facecolor="#525b66", edgecolor="#20252a", linewidth=0.35, alpha=0.9))

    xs, ys = zip(*path)
    points = ax.scatter(xs, ys, c=[i * duration / max(len(path) - 1, 1) for i in range(len(path))],
                        cmap="turbo", s=5, linewidths=0, zorder=4)
    ax.plot(xs, ys, color="#132238", linewidth=0.7, alpha=0.65, zorder=3)
    ax.scatter([xs[0]], [ys[0]], marker="o", s=85, color="#20b26b", edgecolor="white", linewidth=1.2, zorder=5, label="Start")
    ax.scatter([xs[-1]], [ys[-1]], marker="*", s=150, color="#e83e3e", edgecolor="white", linewidth=1.0, zorder=5, label="Goal")

    cbar = fig.colorbar(points, ax=ax, pad=0.012)
    cbar.set_label("Mission time (s)")
    ax.set_title(f"{args.planner} — Random Cylinder World (100 obstacles)\nTravel time: {duration:.2f} s   |   Path length: {distance:.2f} m   |   Mean speed: {distance / duration:.2f} m/s")
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.set_xlim(-2, 152)
    ax.set_ylim(-16, 16)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#cfd5db", linewidth=0.45, alpha=0.65)
    ax.legend(loc="upper right", framealpha=0.95)
    fig.tight_layout()
    fig.savefig(args.output, bbox_inches="tight")
    print(f"output={args.output} duration={duration:.6f} distance={distance:.6f} samples={len(path)} obstacles={len(cylinders)}")


if __name__ == "__main__":
    main()
