#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _mavros_connection():
    try:
        px4_instance = int(os.environ.get("PX4_INSTANCE", "0"))
    except ValueError as exc:
        raise RuntimeError("PX4_INSTANCE must be an integer from 0 to 9") from exc
    if not 0 <= px4_instance <= 9:
        raise RuntimeError("PX4_INSTANCE must be an integer from 0 to 9")
    fcu_url = os.environ.get("MAVROS_FCU_URL") or (
        f"udp://:{14540 + px4_instance}@127.0.0.1:{14580 + px4_instance}"
    )
    target_system_id = int(
        os.environ.get("MAVROS_TARGET_SYSTEM_ID") or str(px4_instance + 1)
    )
    return fcu_url, target_system_id


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    config_file = os.path.join(
        bringup_share, "config", "drone1_astar_mppi_known_world.yaml"
    )
    mavros_config_yaml = os.path.join(bringup_share, "config", "mavros_config.yaml")
    mavros_pluginlists_yaml = os.path.join(
        bringup_share, "config", "mavros_pluginlists.yaml"
    )
    fcu_url, target_system_id = _mavros_connection()
    experiment_seed = int(os.environ.get("EXPERIMENT_SEED", "0"))
    experiment_stage = os.environ.get("EXPERIMENT_STAGE", "pilot")
    trial_index = int(os.environ.get("EXPERIMENT_TRIAL", "0"))

    return LaunchDescription([
        Node(
            package="mavros",
            executable="mavros_node",
            namespace="mavros",
            output="screen",
            parameters=[
                mavros_pluginlists_yaml,
                mavros_config_yaml,
                {"fcu_url": fcu_url},
                {"gcs_url": ""},
                {"target_system_id": target_system_id},
                {"target_component_id": 1},
            ],
        ),
        Node(
            package="drone_perception",
            executable="lidar_obstacle_node",
            name="lidar_obstacle",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="drone_planning",
            executable="astar_global_planner",
            name="astar_global_planner",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="mppi",
            executable="mppi_node",
            name="mppi",
            output="screen",
            parameters=[config_file, {"random_seed": experiment_seed}],
        ),
        Node(
            package="drone_safety",
            executable="safety_monitor",
            name="safety_monitor",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="drone_control",
            executable="autonomy_manager",
            name="autonomy_manager",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="drone_metrics",
            executable="metrics_logger",
            name="metrics_logger",
            output="screen",
            parameters=[config_file, {
                "experiment_seed": experiment_seed,
                "experiment_stage": experiment_stage,
                "trial_index": trial_index,
            }],
        ),
    ])
