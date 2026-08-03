from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_fleet(context):
    vehicle_count = int(LaunchConfiguration("vehicle_count").perform(context))
    goal_x = LaunchConfiguration("goal_x").perform(context)
    goal_z = LaunchConfiguration("goal_z").perform(context)
    lane_spacing = float(LaunchConfiguration("lane_spacing").perform(context))
    enable_flight = LaunchConfiguration("enable_flight").perform(context)
    enable_metrics = LaunchConfiguration("enable_metrics").perform(context)

    if vehicle_count < 1 or vehicle_count > 4:
        raise RuntimeError("vehicle_count must be between 1 and 4")

    vehicle_launch = PathJoinSubstitution(
        [FindPackageShare("drone_bringup"), "launch", "vehicle_autonomy.launch.py"]
    )
    center = (vehicle_count - 1) / 2.0
    actions = []

    for index in range(vehicle_count):
        instance = index
        drone_name = f"drone{index + 1}"
        goal_y = (index - center) * lane_spacing
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vehicle_launch),
                launch_arguments={
                    "drone_name": drone_name,
                    "fcu_url": (
                        f"udp://:{14540 + instance}@127.0.0.1:{14580 + instance}"
                    ),
                    "target_system": str(index + 1),
                    "goal_x": goal_x,
                    "goal_y": str(goal_y),
                    "goal_z": goal_z,
                    "enable_flight": enable_flight,
                    "enable_metrics": enable_metrics,
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_count", default_value="2"),
            DeclareLaunchArgument("goal_x", default_value="20.0"),
            DeclareLaunchArgument("goal_z", default_value="3.0"),
            DeclareLaunchArgument("lane_spacing", default_value="15.0"),
            DeclareLaunchArgument("enable_flight", default_value="false"),
            DeclareLaunchArgument("enable_metrics", default_value="true"),
            OpaqueFunction(function=_launch_fleet),
        ]
    )
