#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    config_file = os.path.join(bringup_share, "config", "drone1_mppi_known_world.yaml")

    return LaunchDescription([
        Node(
            package="mavros",
            executable="mavros_node",
            namespace="",
            output="screen",
            parameters=[
                {"fcu_url": "udp://:14540@127.0.0.1:14580"},
                {"gcs_url": ""},
                {"target_system_id": 1},
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
            package="mppi",
            executable="mppi_node",
            name="mppi",
            output="screen",
            parameters=[config_file],
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
            parameters=[config_file],
        ),
    ])