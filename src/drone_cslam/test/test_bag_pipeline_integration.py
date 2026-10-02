import math
from pathlib import Path

import yaml

from nav_msgs.msg import Odometry
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from drone_cslam.oracle_eval import run_experiment


def _set_stamp(stamp, nanoseconds):
    stamp.sec = int(nanoseconds // 1_000_000_000)
    stamp.nanosec = int(nanoseconds % 1_000_000_000)


def _odom(timestamp, x, y, yaw=0.0):
    message = Odometry()
    _set_stamp(message.header.stamp, timestamp)
    message.header.frame_id = "world"
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.pose.orientation.w = math.cos(0.5 * yaw)
    return message


def _scan(timestamp):
    message = LaserScan()
    _set_stamp(message.header.stamp, timestamp)
    message.header.frame_id = "lidar"
    message.angle_min = -0.4
    message.angle_max = 0.4
    message.angle_increment = 0.2
    message.range_min = 0.1
    message.range_max = 5.0
    message.ranges = [3.0, 2.5, 2.0, 2.5, 3.0]
    return message


def _map_to_odom(timestamp, vehicle, drift_y, drift_yaw):
    transform = TransformStamped()
    _set_stamp(transform.header.stamp, timestamp)
    transform.header.frame_id = f"{vehicle}/map"
    transform.child_frame_id = f"{vehicle}/odom"
    transform.transform.translation.y = float(drift_y)
    transform.transform.rotation.z = math.sin(0.5 * drift_yaw)
    transform.transform.rotation.w = math.cos(0.5 * drift_yaw)
    return transform


def _write_synthetic_bag(path: Path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {
        "/tf": "tf2_msgs/msg/TFMessage",
        "/drone1/scan": "sensor_msgs/msg/LaserScan",
        "/drone1/odom": "nav_msgs/msg/Odometry",
        "/drone1/ground_truth/odom": "nav_msgs/msg/Odometry",
        "/drone2/scan": "sensor_msgs/msg/LaserScan",
        "/drone2/odom": "nav_msgs/msg/Odometry",
        "/drone2/ground_truth/odom": "nav_msgs/msg/Odometry",
    }
    for topic, type_name in topic_types.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic, type=type_name, serialization_format="cdr"
            )
        )
    for sample in range(21):
        timestamp = (sample + 1) * 500_000_000
        local_x = 0.5 * sample
        fraction = sample / 20.0
        transforms = []
        for vehicle, spawn_y, sign in (
            ("drone1", -2.0, 1.0),
            ("drone2", 2.0, -1.0),
        ):
            writer.write(
                f"/{vehicle}/scan", serialize_message(_scan(timestamp)), timestamp
            )
            writer.write(
                f"/{vehicle}/odom",
                serialize_message(_odom(timestamp, local_x, 0.0)),
                timestamp,
            )
            writer.write(
                f"/{vehicle}/ground_truth/odom",
                serialize_message(_odom(timestamp, local_x, spawn_y)),
                timestamp,
            )
            transforms.append(
                _map_to_odom(
                    timestamp,
                    vehicle,
                    sign * 0.8 * fraction,
                    sign * math.radians(3.0) * fraction,
                )
            )
        writer.write(
            "/tf", serialize_message(TFMessage(transforms=transforms)), timestamp
        )


def _config():
    return {
        "vehicles": [
            {"name": "drone1", "spawn": [0.0, -2.0, 0.0]},
            {"name": "drone2", "spawn": [0.0, 2.0, 0.0]},
        ],
        "topics": {
            "scan_suffix": "scan",
            "odom_suffix": "odom",
            "ground_truth_suffix": "ground_truth/odom",
            "tf": "/tf",
            "tf_static": "/tf_static",
        },
        "sampling": {
            "keyframe_translation_m": 1.0,
            "keyframe_yaw_deg": 5.0,
            "interpolation_max_gap_sec": 0.2,
            "map_scan_stride": 2,
            "map_beam_stride": 1,
        },
        "pose_graph": {
            "intra_translation_sigma_m": 0.2,
            "intra_yaw_sigma_deg": 2.0,
            "prior_translation_sigma_m": 0.02,
            "prior_yaw_sigma_deg": 0.2,
            "oracle_translation_sigma_m": 0.02,
            "oracle_yaw_sigma_deg": 0.2,
            "oracle_spacing_m": 2.0,
            "robust_loss": "huber",
            "robust_scale": 1.5,
            "max_function_evaluations": 200,
            "random_seed": 7,
        },
        "map": {
            "resolution_m": 0.25,
            "bounds": [-1.0, 15.0, -7.0, 7.0],
            "free_log_odds": -0.4,
            "occupied_log_odds": 0.85,
            "min_log_odds": -4.0,
            "max_log_odds": 4.0,
            "occupied_probability": 0.65,
        },
        "evaluation": {
            "rpe_distance_m": 2.0,
            "differential_sample_spacing_m": 1.0,
            "overlap_bin_m": 5.0,
            "overlap_voxel_m": 0.5,
            "phase1_gate": {
                "max_sync_drop_ratio": 0.01,
                "differential_endpoint_improvement_min": 0.3,
                "chamfer_improvement_min": 0.2,
                "occupied_f1_allowed_drop": 0.02,
            },
        },
    }


def test_rosbag_to_report_pipeline(tmp_path):
    bag_path = tmp_path / "synthetic_bag"
    config_path = tmp_path / "config.yaml"
    output_path = tmp_path / "output"
    _write_synthetic_bag(bag_path)
    config_path.write_text(yaml.safe_dump(_config()), encoding="utf-8")

    gate = run_experiment(
        str(bag_path), str(config_path), str(output_path), overwrite=False
    )

    assert isinstance(gate["pass"], bool)
    assert (output_path / "manifest.json").is_file()
    assert (output_path / "metrics.json").is_file()
    assert (output_path / "factors.jsonl").is_file()
    assert (output_path / "maps" / "O-periodic.npy").is_file()
    assert (output_path / "trajectories" / "O-periodic.csv").is_file()
    assert "Scientific gate" in (output_path / "report.md").read_text(
        encoding="utf-8"
    )
