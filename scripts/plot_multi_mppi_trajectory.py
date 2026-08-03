#!/usr/bin/env python3
import argparse
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool, String


DRONES = {
    "drone1": {"offset_y": -7.5, "color": "#1565c0"},
    "drone2": {"offset_y": 7.5, "color": "#ef6c00"},
    "drone3": {"offset_y": 0.0, "color": "#2e7d32"},
}


def load_cylinders(world_path):
    root = ET.parse(world_path).getroot()
    cylinders = []
    for include in root.findall(".//include"):
        name = include.findtext("name", "")
        if not name.startswith("cylinder_"):
            continue
        pose = [float(value) for value in include.findtext("pose").split()]
        uri = include.findtext("uri", "")
        match = re.search(r"_r(\d+)", uri)
        radius = float(match.group(1)) / 10.0 if match else 0.5
        cylinders.append((pose[0], pose[1], radius))
    return cylinders


def load_paths(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    data = {
        name: {"start": None, "goal": None, "poses": []}
        for name in DRONES
    }
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        for name, values in data.items():
            prefix = f"/{name}"
            if topic == f"{prefix}/mission/phase":
                phase = deserialize_message(raw, String).data
                if phase == "MPPI_GO" and values["start"] is None:
                    values["start"] = timestamp
            elif topic == f"{prefix}/mission/goal_reached":
                if deserialize_message(raw, Bool).data and values["goal"] is None:
                    values["goal"] = timestamp
            elif topic == f"{prefix}/mavros/local_position/pose":
                point = deserialize_message(raw, PoseStamped).pose.position
                # MAVROS pose is relative to the physical spawn at world x=3 m.
                values["poses"].append((timestamp, point.x + 3.0, point.y))

    paths = {}
    for name, values in data.items():
        if values["start"] is None or values["goal"] is None:
            raise RuntimeError(f"{name}: MPPI_GO 또는 goal_reached가 없습니다.")
        offset_y = DRONES[name]["offset_y"]
        points = [
            (x, y + offset_y)
            for timestamp, x, y in values["poses"]
            if values["start"] <= timestamp <= values["goal"]
        ]
        duration = (values["goal"] - values["start"]) / 1e9
        distance = sum(
            math.hypot(x1 - x0, y1 - y0)
            for (x0, y0), (x1, y1) in zip(points, points[1:])
        )
        paths[name] = {
            "points": points,
            "duration": duration,
            "distance": distance,
        }
    return paths


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--run-label", required=True)
    args = parser.parse_args()

    cylinders = load_cylinders(args.world)
    paths = load_paths(args.bag)
    fig, ax = plt.subplots(figsize=(16, 5.2), dpi=180)
    ax.add_collection(
        PatchCollection(
            [Circle((x, y), radius) for x, y, radius in cylinders],
            facecolor="#525b66",
            edgecolor="#20252a",
            linewidth=0.35,
            alpha=0.9,
        )
    )

    summary = []
    for name, values in paths.items():
        xs, ys = zip(*values["points"])
        color = DRONES[name]["color"]
        ax.plot(xs, ys, color=color, linewidth=1.7, label=name, zorder=4)
        ax.scatter(xs[0], ys[0], marker="o", s=70, color=color, zorder=5)
        ax.scatter(xs[-1], ys[-1], marker="*", s=140, color=color, zorder=5)
        summary.append(
            f"{name}: {values['duration']:.1f}s, {values['distance']:.1f}m"
        )

    world_name = Path(args.world).stem
    ax.set_title(
        f"3-UAV LiDAR MPPI — {world_name} — {args.run_label}\n"
        + " | ".join(summary)
    )
    ax.set_xlabel("World X [m]")
    ax.set_ylabel("World Y [m]")
    ax.set_xlim(0, 150)
    ax.set_ylim(-15, 15)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.45, alpha=0.6)
    ax.legend(loc="upper center", ncol=3)
    fig.tight_layout()
    fig.savefig(args.output, bbox_inches="tight")
    print(
        " ".join(
            f"{name}_duration={values['duration']:.3f} "
            f"{name}_distance={values['distance']:.3f}"
            for name, values in paths.items()
        )
    )


if __name__ == "__main__":
    main()
