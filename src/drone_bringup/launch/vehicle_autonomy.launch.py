import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def _launch_vehicle(context):
    drone_name = LaunchConfiguration("drone_name").perform(context).strip("/")
    fcu_url = LaunchConfiguration("fcu_url").perform(context)
    target_system = LaunchConfiguration("target_system").perform(context)
    goal_x = float(LaunchConfiguration("goal_x").perform(context))
    goal_y = float(LaunchConfiguration("goal_y").perform(context))
    goal_z = float(LaunchConfiguration("goal_z").perform(context))
    config_file = LaunchConfiguration("config_file").perform(context)
    enable_metrics = LaunchConfiguration("enable_metrics").perform(context).lower()
    enable_flight = LaunchConfiguration("enable_flight").perform(context).lower()

    if not drone_name:
        raise RuntimeError("drone_name must not be empty")
    if enable_metrics not in {"true", "false"}:
        raise RuntimeError("enable_metrics must be true or false")
    if enable_flight not in {"true", "false"}:
        raise RuntimeError("enable_flight must be true or false")

    bringup_share = get_package_share_directory("drone_bringup")
    mavros_share = get_package_share_directory("mavros")
    mavros_launch = os.path.join(mavros_share, "launch", "node.launch")
    pluginlists_yaml = os.path.join(bringup_share, "config", "mavros_pluginlists.yaml")
    mavros_config_yaml = os.path.join(bringup_share, "config", "mavros_config.yaml")

    vehicle_parameters = {
        "drone_name": drone_name,
        "frame_id": f"{drone_name}/map",
        "goal_x": goal_x,
        "goal_y": goal_y,
        "goal_z": goal_z,
        "home_goal_z": goal_z,
        "fcu_url": fcu_url,
    }

    actions = [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                "pluginlists_yaml": pluginlists_yaml,
                "config_yaml": mavros_config_yaml,
                "fcu_url": fcu_url,
                "gcs_url": "",
                "tgt_system": target_system,
                "tgt_component": "1",
                "fcu_protocol": "v2.0",
                "respawn_mavros": "false",
                "namespace": f"{drone_name}/mavros",
            }.items(),
        ),
        Node(
            package="drone_perception",
            executable="lidar_obstacle_node",
            namespace=drone_name,
            name="lidar_obstacle",
            output="screen",
            parameters=[config_file, vehicle_parameters],
        ),
        Node(
            package="drone_planning",
            executable="local_planner_node",
            namespace=drone_name,
            name="local_planner",
            output="screen",
            parameters=[config_file, vehicle_parameters],
        ),
        Node(
            package="drone_safety",
            executable="safety_monitor",
            namespace=drone_name,
            name="safety_monitor",
            output="screen",
            parameters=[config_file, vehicle_parameters],
        ),
    ]

    if enable_flight == "true":
        actions.append(
            Node(
                package="drone_control",
                executable="autonomy_manager",
                namespace=drone_name,
                name="autonomy_manager",
                output="screen",
                parameters=[config_file, vehicle_parameters],
            )
        )

    if enable_metrics == "true":
        actions.append(
            Node(
                package="drone_metrics",
                executable="metrics_logger",
                namespace=drone_name,
                name="metrics_logger",
                output="screen",
                parameters=[
                    config_file,
                    vehicle_parameters,
                    {
                        "autonomy_config_path": config_file,
                        "mavros_config_path": mavros_config_yaml,
                        "mavros_pluginlists_path": pluginlists_yaml,
                        "launch_file_path": __file__,
                    },
                ],
            )
        )

    return actions


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("drone_bringup"), "config", "multi_drone_autonomy.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_name", default_value="drone1"),
            DeclareLaunchArgument(
                "fcu_url",
                default_value="udp://:14540@127.0.0.1:14580",
            ),
            DeclareLaunchArgument("target_system", default_value="1"),
            DeclareLaunchArgument("goal_x", default_value="20.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="3.0"),
            DeclareLaunchArgument("enable_flight", default_value="false"),
            DeclareLaunchArgument("enable_metrics", default_value="true"),
            DeclareLaunchArgument("config_file", default_value=default_config),
            OpaqueFunction(function=_launch_vehicle),
        ]
    )
