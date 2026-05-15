import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    mavros_share = get_package_share_directory("mavros")
    mavros_launch = os.path.join(mavros_share, "launch", "node.launch")

    pluginlists_yaml = os.path.join(bringup_share, "config", "mavros_pluginlists.yaml")
    mavros_config_yaml = os.path.join(bringup_share, "config", "mavros_config.yaml")
    slam_yaml = os.path.join(bringup_share, "config", "drone1_slam_dev.yaml")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch),
        launch_arguments={
            "pluginlists_yaml": pluginlists_yaml,
            "config_yaml": mavros_config_yaml,
            "fcu_url": "udp://:14540@127.0.0.1:14580",
            "gcs_url": "",
            "tgt_system": "1",
            "tgt_component": "1",
            "fcu_protocol": "v2.0",
            "respawn_mavros": "false",
            "namespace": "mavros",
        }.items(),
    )

    perception = Node(
        package="drone_perception",
        executable="lidar_obstacle_node",
        name="lidar_obstacle",
        output="screen",
        parameters=[slam_yaml],
    )

    slam = Node(
        package="drone_slam",
        executable="simple_2d_mapping_node",
        name="simple_2d_mapping",
        output="screen",
        parameters=[slam_yaml],
    )

    return LaunchDescription([mavros, perception, slam])
