#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mppi_slam")
    bringup = get_package_share_directory("drone_bringup")
    config = os.path.join(share, "config", "mppi_slam.yaml")
    return LaunchDescription([
        Node(
            package="mavros", executable="mavros_node", namespace="mavros", output="screen",
            parameters=[
                os.path.join(bringup, "config", "mavros_pluginlists.yaml"),
                os.path.join(bringup, "config", "mavros_config.yaml"),
                {"fcu_url": os.environ.get("MAVROS_FCU_URL", "udp://:14540@127.0.0.1:14580")},
                {"gcs_url": ""},
                {"target_system_id": int(os.environ.get("MAVROS_TARGET_SYSTEM_ID", "1"))},
                {"target_component_id": 1},
            ],
        ),
        Node(package="drone_perception", executable="lidar_obstacle_node", name="lidar_obstacle", parameters=[config], output="screen"),
        Node(package="mppi_slam", executable="slam_path_planner", name="slam_path_planner", parameters=[config], output="screen"),
        Node(package="mppi_slam", executable="mppi_slam_node", name="mppi", parameters=[config], output="screen"),
        Node(package="drone_safety", executable="safety_monitor", name="safety_monitor", parameters=[config], output="screen"),
        Node(package="drone_control", executable="autonomy_manager", name="autonomy_manager", parameters=[config], output="screen"),
        Node(package="drone_metrics", executable="metrics_logger", name="metrics_logger", parameters=[config], output="screen"),
    ])
