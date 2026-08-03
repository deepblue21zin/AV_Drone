import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_two_drones(context: LaunchContext):
    package_dir = get_package_share_directory("mppi_lidar")
    bringup_dir = get_package_share_directory("drone_bringup")
    mavros_dir = get_package_share_directory("mavros")
    parameter_file = os.path.join(package_dir, "config", "mppi_lidar_params.yaml")
    mavros_launch = os.path.join(mavros_dir, "launch", "node.launch")
    pluginlists = os.path.join(bringup_dir, "config", "mavros_pluginlists.yaml")
    mavros_config = os.path.join(bringup_dir, "config", "mavros_config.yaml")
    goal_x = float(LaunchConfiguration("goal_x").perform(context))
    goal_z = float(LaunchConfiguration("goal_z").perform(context))

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import AnyLaunchDescriptionSource

    actions = []
    for index in range(2):
        drone_name = f"drone{index + 1}"
        mavros_namespace = f"/{drone_name}/mavros"
        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(mavros_launch),
                launch_arguments={
                    "pluginlists_yaml": pluginlists,
                    "config_yaml": mavros_config,
                    "fcu_url": (
                        f"udp://:{14540 + index}@127.0.0.1:{14580 + index}"
                    ),
                    "gcs_url": "",
                    "tgt_system": str(index + 1),
                    "tgt_component": "1",
                    "fcu_protocol": "v2.0",
                    "respawn_mavros": "false",
                    "namespace": mavros_namespace,
                }.items(),
            )
        )
        actions.append(
            Node(
                package="mppi_lidar",
                executable="mppi_lidar_node",
                namespace=drone_name,
                name="mppi_lidar",
                output="screen",
                parameters=[
                    parameter_file,
                    {
                        "drone_name": drone_name,
                        "mavros_namespace": mavros_namespace,
                        "scan_topic": f"/{drone_name}/scan",
                        "goal_x": goal_x,
                        "goal_y": 0.0,
                        "goal_z": goal_z,
                        "goal_tol_xy": 1.5,
                    },
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            # World start x=3.0 m, world goal x=147.0 m.
            # MAVROS local position starts at zero, so the local goal is 144 m.
            DeclareLaunchArgument("goal_x", default_value="144.0"),
            DeclareLaunchArgument("goal_z", default_value="3.0"),
            OpaqueFunction(function=_launch_two_drones),
        ]
    )
