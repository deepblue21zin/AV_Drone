import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("a_star")
    bringup = get_package_share_directory("drone_bringup")
    mavros = get_package_share_directory("mavros")
    config = os.path.join(share, "config", "a_star_flight.yaml")
    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(mavros, "launch", "node.launch")),
        launch_arguments={
            "pluginlists_yaml": os.path.join(bringup, "config", "mavros_pluginlists.yaml"),
            "config_yaml": os.path.join(bringup, "config", "mavros_config.yaml"),
            "fcu_url": "udp://:14540@127.0.0.1:14580",
            "gcs_url": "", "tgt_system": "1", "tgt_component": "1",
            "fcu_protocol": "v2.0", "namespace": "mavros",
        }.items(),
    )
    nodes = [
        Node(package="drone_perception", executable="lidar_obstacle_node", name="lidar_obstacle", parameters=[config], output="screen"),
        Node(package="a_star", executable="a_star_node", name="a_star", parameters=[config], output="screen"),
        Node(package="drone_safety", executable="safety_monitor", name="safety_monitor", parameters=[config], output="screen"),
        Node(package="drone_control", executable="autonomy_manager", name="autonomy_manager", parameters=[config], output="screen"),
        Node(package="drone_metrics", executable="metrics_logger", name="metrics_logger", parameters=[config], output="screen"),
    ]
    return LaunchDescription([mavros_node] + nodes)
