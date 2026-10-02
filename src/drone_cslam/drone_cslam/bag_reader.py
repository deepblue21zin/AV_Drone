"""Read the standard ROS 2 topics needed by the oracle experiment."""

from __future__ import annotations

from pathlib import Path
from typing import Dict

import numpy as np
import yaml

from .data import ScanRecord, TimedPose, VehicleBagData
from .se2 import quaternion_to_yaw


def _stamp_to_seconds(stamp, fallback_nanoseconds: int) -> float:
    seconds = float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
    if seconds <= 0.0:
        return 1.0e-9 * float(fallback_nanoseconds)
    return seconds


def _pose_from_message(pose) -> np.ndarray:
    orientation = pose.orientation
    return np.array(
        [
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        ],
        dtype=float,
    )


def _storage_identifier(bag_path: Path) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return "sqlite3"
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    information = metadata.get("rosbag2_bagfile_information") or {}
    return str(information.get("storage_identifier") or "sqlite3")


def load_bag(bag_path: str, config: dict, *, include_ground_truth=True) -> Dict[str, VehicleBagData]:
    """Load relevant standard messages through rosbag2_py.

    ROS imports remain local so pure math tests can run without sourcing ROS.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # pragma: no cover - depends on ROS environment
        raise RuntimeError(
            "rosbag reading requires a sourced ROS 2 environment with rosbag2_py"
        ) from exc

    bag_path = Path(bag_path).expanduser().resolve()
    if not bag_path.is_dir():
        raise FileNotFoundError(f"rosbag directory not found: {bag_path}")

    vehicles = {
        str(vehicle["name"]): VehicleBagData(name=str(vehicle["name"]))
        for vehicle in config["vehicles"]
    }
    topics = config.get("topics") or {}
    scan_suffix = str(topics.get("scan_suffix", "scan")).strip("/")
    odom_suffix = str(topics.get("odom_suffix", "odom")).strip("/")
    gt_suffix = str(topics.get("ground_truth_suffix", "ground_truth/odom")).strip("/")
    tf_topics = {
        str(topics.get("tf", "/tf")),
        str(topics.get("tf_static", "/tf_static")),
    }
    topic_routes = {}
    for name in vehicles:
        topic_routes[f"/{name}/{scan_suffix}"] = (name, "scan")
        topic_routes[f"/{name}/{odom_suffix}"] = (name, "odom")
        if include_ground_truth:
            topic_routes[f"/{name}/{gt_suffix}"] = (name, "ground_truth")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id=_storage_identifier(bag_path)
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    type_names = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in type_names.items()
        if topic in topic_routes or topic in tf_topics
    }

    while reader.has_next():
        topic, serialized, bag_timestamp = reader.read_next()
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(serialized, message_type)
        if topic in tf_topics:
            for transform in message.transforms:
                parent = str(transform.header.frame_id).lstrip("/")
                child = str(transform.child_frame_id).lstrip("/")
                for name, vehicle in vehicles.items():
                    if parent != f"{name}/map" or child != f"{name}/odom":
                        continue
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    vehicle.map_to_odom.append(
                        TimedPose(
                            timestamp=_stamp_to_seconds(
                                transform.header.stamp, bag_timestamp
                            ),
                            pose=np.array(
                                [
                                    float(translation.x),
                                    float(translation.y),
                                    quaternion_to_yaw(
                                        float(rotation.x),
                                        float(rotation.y),
                                        float(rotation.z),
                                        float(rotation.w),
                                    ),
                                ],
                                dtype=float,
                            ),
                        )
                    )
            continue

        name, route = topic_routes[topic]
        vehicle = vehicles[name]
        if route == "scan":
            vehicle.scans.append(
                ScanRecord(
                    timestamp=_stamp_to_seconds(message.header.stamp, bag_timestamp),
                    angle_min=float(message.angle_min),
                    angle_increment=float(message.angle_increment),
                    range_min=float(message.range_min),
                    range_max=float(message.range_max),
                    ranges=np.asarray(message.ranges, dtype=np.float32),
                )
            )
        else:
            record = TimedPose(
                timestamp=_stamp_to_seconds(message.header.stamp, bag_timestamp),
                pose=_pose_from_message(message.pose.pose),
            )
            if route == "odom":
                vehicle.odom.append(record)
            else:
                vehicle.ground_truth.append(record)

    missing = []
    for name, vehicle in vehicles.items():
        required = ["scans", "odom", "map_to_odom"]
        if include_ground_truth:
            required.append("ground_truth")
        for label in required:
            if not getattr(vehicle, label):
                missing.append(f"{name}:{label}")
    if missing:
        raise RuntimeError(
            "rosbag is missing phase-1 inputs: " + ", ".join(missing)
        )
    return vehicles
