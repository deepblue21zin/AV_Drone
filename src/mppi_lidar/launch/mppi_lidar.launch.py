from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory("mppi_lidar")
    param_file = os.path.join(pkg_dir, "config", "mppi_lidar_params.yaml")

    return LaunchDescription([
        Node(
            package="mppi_lidar",
            executable="mppi_lidar_node",
            name="mppi_lidar",
            output="screen",
            parameters=[param_file],
        )
    ])