#!/usr/bin/env python3
"""Create per-vehicle Gazebo Classic model variants with isolated ROS frames/topics."""

import argparse
import shutil
from pathlib import Path


def replace_tree(root: Path, replacements: dict[str, str]) -> None:
    for path in root.rglob("*"):
        if not path.is_file() or path.suffix not in {".sdf", ".config", ".jinja"}:
            continue
        text = path.read_text(encoding="utf-8")
        for old, new in replacements.items():
            text = text.replace(old, new)
        path.write_text(text, encoding="utf-8")


def copy_variant(source: Path, target: Path, replacements: dict[str, str]) -> None:
    if not source.is_dir():
        raise FileNotFoundError(f"missing source model: {source}")
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target)
    replace_tree(target, replacements)


def rename_vehicle_templates(target: Path, source_name: str, target_name: str) -> None:
    for suffix in (".sdf", ".sdf.jinja"):
        source = target / f"{source_name}{suffix}"
        if source.exists():
            source.rename(target / f"{target_name}{suffix}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--count", type=int, required=True)
    args = parser.parse_args()
    if args.count < 2 or args.count > 9:
        raise ValueError("vehicle count must be between 2 and 9")

    lidar_source = args.model_dir / "rplidar"
    vehicle_source = args.model_dir / "iris_rplidar"
    for ordinal in range(1, args.count + 1):
        drone = f"drone{ordinal}"
        lidar_name = f"rplidar_{drone}"
        vehicle_name = f"iris_rplidar_{drone}"
        copy_variant(
            lidar_source,
            args.model_dir / lidar_name,
            {
                "<model name=\"rplidar\">": f"<model name=\"{lidar_name}\">",
                "<name>rplidar</name>": f"<name>{lidar_name}</name>",
                "<namespace>/drone1</namespace>": f"<namespace>/{drone}</namespace>",
                "<frame_name>rplidar_link</frame_name>": (
                    f"<frame_name>{drone}/lidar_link</frame_name>"
                ),
            },
        )
        copy_variant(
            vehicle_source,
            args.model_dir / vehicle_name,
            {
                "model://rplidar": f"model://{lidar_name}",
                "<name>iris_rplidar</name>": f"<name>{vehicle_name}</name>",
                "<model name=\"iris_rplidar\">": (
                    f"<model name=\"{vehicle_name}\">"
                ),
            },
        )
        rename_vehicle_templates(
            args.model_dir / vehicle_name, "iris_rplidar", vehicle_name
        )
        print(f"[swarm model] {vehicle_name} -> /{drone}/scan")


if __name__ == "__main__":
    main()
