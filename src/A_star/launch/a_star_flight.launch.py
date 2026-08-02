import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def _mavros_connection():
    try:
        px4_instance = int(os.environ.get("PX4_INSTANCE", "0"))
    except ValueError as exc:
        raise RuntimeError("PX4_INSTANCE must be an integer from 0 to 9") from exc

    if not 0 <= px4_instance <= 9:
        raise RuntimeError("PX4_INSTANCE must be an integer from 0 to 9")

    fcu_url = os.environ.get("MAVROS_FCU_URL") or (
        f"udp://:{14540 + px4_instance}@127.0.0.1:{14580 + px4_instance}"
    )
    target_system_id = int(
        os.environ.get("MAVROS_TARGET_SYSTEM_ID") or str(px4_instance + 1)
    )
    return fcu_url, target_system_id


def generate_launch_description():
    share = get_package_share_directory("a_star")
    bringup = get_package_share_directory("drone_bringup")
    mavros = get_package_share_directory("mavros")
    config = os.path.join(share, "config", "a_star_flight.yaml")
    fcu_url, target_system_id = _mavros_connection()
    experiment_seed = int(os.environ.get("EXPERIMENT_SEED", "0"))
    experiment_stage = os.environ.get("EXPERIMENT_STAGE", "pilot")
    trial_index = int(os.environ.get("EXPERIMENT_TRIAL", "0"))
    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(mavros, "launch", "node.launch")),
        launch_arguments={
            "pluginlists_yaml": os.path.join(bringup, "config", "mavros_pluginlists.yaml"),
            "config_yaml": os.path.join(bringup, "config", "mavros_config.yaml"),
            "fcu_url": fcu_url,
            "gcs_url": "", "tgt_system": str(target_system_id), "tgt_component": "1",
            "fcu_protocol": "v2.0", "namespace": "mavros",
        }.items(),
    )
    nodes = [
        Node(package="drone_perception", executable="lidar_obstacle_node", name="lidar_obstacle", parameters=[config], output="screen"),
        Node(package="a_star", executable="a_star_node", name="a_star", parameters=[config], output="screen"),
        Node(package="drone_safety", executable="safety_monitor", name="safety_monitor", parameters=[config], output="screen"),
        Node(package="drone_control", executable="autonomy_manager", name="autonomy_manager", parameters=[config], output="screen"),
        Node(
            package="drone_metrics",
            executable="metrics_logger",
            name="metrics_logger",
            parameters=[config, {
                "experiment_seed": experiment_seed,
                "experiment_stage": experiment_stage,
                "trial_index": trial_index,
            }],
            output="screen",
        ),
    ]
    return LaunchDescription([mavros_node] + nodes)
