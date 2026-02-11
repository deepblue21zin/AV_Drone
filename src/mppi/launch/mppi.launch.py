import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # MAVROS (PX4 SITL)
    mavros_share = get_package_share_directory("mavros")
    mavros_launch = os.path.join(mavros_share, "launch", "px4.launch")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch),
        launch_arguments={
            "fcu_url": "udp://:14540@127.0.0.1:14580",
        }.items(),
    )

    # MPPI node
    # UPDATED: slalom y=±2.2, r=1.5
    mppi_node = Node(
        package="mppi",
        executable="mppi_node",
        name="mppi",
        output="screen",
        parameters=[{
            "takeoff_z": 3.0,

            "goal_x": 24.0,
            "goal_y": 0.0,
            "goal_z": 3.0,
            "goal_yaw": 0.0,

            "obs_x": [
                # walls y=+5
                4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0,
                # walls y=-5
                4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0,
                # slalom
                6.0, 9.0, 12.0, 15.0, 18.0,
            ],
            "obs_y": [
                # walls y=+5
                5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                # walls y=-5
                -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
                # slalom (UPDATED: 2.0 -> 1.8)
                2.2, -2.2, 2.2, -2.2, 2.2,
            ],
            "obs_r": [
                # walls
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                # slalom
                1.5, 1.5, 1.5, 1.5, 1.5,
            ],
        }],
    )

    return LaunchDescription([
        mavros,
        mppi_node,
    ])
