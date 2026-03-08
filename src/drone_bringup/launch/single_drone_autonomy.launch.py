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
    autonomy_yaml = os.path.join(bringup_share, "config", "drone1_autonomy.yaml")

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
        parameters=[autonomy_yaml],
    )

    planner = Node(
        package="drone_planning",
        executable="local_planner_node",
        name="local_planner",
        output="screen",
        parameters=[autonomy_yaml],
    )

    safety = Node(
        package="drone_safety",
        executable="safety_monitor",
        name="safety_monitor",
        output="screen",
        parameters=[autonomy_yaml],
    )

    controller = Node(
        package="drone_control",
        executable="autonomy_manager",
        name="autonomy_manager",
        output="screen",
        parameters=[autonomy_yaml],
    )

    metrics = Node(
        package="drone_metrics",
        executable="metrics_logger",
        name="metrics_logger",
        output="screen",
        parameters=[autonomy_yaml],
    )

    return LaunchDescription([mavros, perception, planner, safety, controller, metrics])
