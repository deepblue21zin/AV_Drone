import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mppi_share = get_package_share_directory("mppi")
    mppi_launch = os.path.join(mppi_share, "launch", "mppi.launch.py")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mppi_launch),
            )
        ]
    )
