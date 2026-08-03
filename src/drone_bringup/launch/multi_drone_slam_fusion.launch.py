import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _load(context):
    bringup_share = get_package_share_directory("drone_bringup")
    slam_share = get_package_share_directory("drone_slam")
    mavros_share = get_package_share_directory("mavros")
    manifest_path = LaunchConfiguration("manifest").perform(context)
    with open(manifest_path, "r", encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)

    vehicles = manifest.get("vehicles", [])
    if len(vehicles) != int(manifest.get("vehicle_count", len(vehicles))):
        raise RuntimeError("vehicle_count does not match vehicles list")
    if not vehicles:
        raise RuntimeError("swarm manifest contains no vehicles")

    requested_source = LaunchConfiguration("fusion_source").perform(context).strip()
    fusion_source = requested_source or str(manifest.get("fusion_source", "known_pose"))
    if fusion_source not in {"known_pose", "slam"}:
        raise RuntimeError("fusion_source must be 'known_pose' or 'slam'")
    run_id = LaunchConfiguration("run_id").perform(context).strip()
    if not run_id:
        run_id = str(manifest.get("run_id", "")).strip()
    if not run_id:
        run_id = time.strftime("%Y-%m-%d_%H-%M-%S_swarm")

    shared_params = os.path.join(bringup_share, "config", "swarm_mapping.yaml")
    autonomy_params = os.path.join(bringup_share, "config", "drone1_autonomy.yaml")
    toolbox_params = os.path.join(slam_share, "config", "slam_toolbox_async.yaml")
    pluginlists = os.path.join(bringup_share, "config", "mavros_pluginlists.yaml")
    mavros_config = os.path.join(bringup_share, "config", "mavros_config.yaml")
    mavros_launch = os.path.join(mavros_share, "launch", "node.launch")
    actions = []

    source_topics = []
    source_frames = []
    source_x = []
    source_y = []
    source_yaw = []
    names = []

    for vehicle in vehicles:
        name = str(vehicle["name"])
        spawn = list(vehicle["spawn"])
        if len(spawn) != 4:
            raise RuntimeError(f"{name}.spawn must be [x, y, z, yaw]")
        goal = list(vehicle.get("goal_local", [38.0, 0.0, 3.0]))
        if len(goal) != 3:
            raise RuntimeError(f"{name}.goal_local must be [x, y, z]")
        map_frame = f"{name}/map"
        odom_frame = f"{name}/odom"
        base_frame = f"{name}/base_link"
        lidar_frame = f"{name}/lidar_link"
        selected_map = f"slam/map" if fusion_source == "slam" else "mapping/known_pose_map"
        vehicle_params = {
            "drone_name": name,
            "scenario_name": str(manifest["scenario_name"]),
            "mavros_namespace": "mavros",
            "scan_topic": "scan",
            "pose_topic": "mavros/local_position/pose",
            "state_topic": "mavros/state",
            "autonomy_cmd_topic": "autonomy/cmd_vel",
            "planner_cmd_topic": "autonomy/cmd_vel",
            "safe_cmd_topic": "safety/cmd_vel",
            "mission_phase_topic": "mission/phase",
            "home_pose_topic": "mission/home_pose",
            "active_goal_topic": "mission/active_goal",
            "nearest_obstacle_topic": "perception/nearest_obstacle_distance",
            "safety_event_topic": "safety/event",
            "goal_reached_topic": "mission/goal_reached",
            "slam_status_topic": "slam/status",
            "slam_input_ready_topic": "slam/input_ready",
            "slam_map_ready_topic": "slam/map_ready",
            "slam_localization_ok_topic": "slam/localization_ok",
            "slam_coverage_topic": "slam/coverage",
            "goal_x": float(goal[0]),
            "goal_y": float(goal[1]),
            "goal_z": float(goal[2]),
            "takeoff_z": 3.0,
            "return_home_enabled": True,
            "require_scan": True,
            "slam_mapping_mode_enabled": True,
            "map_source": fusion_source,
            "run_id": run_id,
            "world_name": str(manifest["world_name"]),
            "world_path": str(manifest.get("world_path", "")),
        }

        mavros = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                "pluginlists_yaml": pluginlists,
                "config_yaml": mavros_config,
                "fcu_url": str(vehicle["fcu_url"]),
                "gcs_url": "",
                "tgt_system": str(vehicle["system_id"]),
                "tgt_component": "1",
                "fcu_protocol": "v2.0",
                "respawn_mavros": "false",
                "namespace": "mavros",
            }.items(),
        )
        vehicle_nodes = [
            PushRosNamespace(name),
            mavros,
            Node(
                package="drone_slam",
                executable="pose_odom_tf_node",
                name="pose_odom_tf",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "pose_topic": "mavros/local_position/pose",
                    "odom_topic": "odom",
                    "odom_frame_id": odom_frame,
                    "base_frame_id": base_frame,
                }],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_lidar_tf",
                arguments=["0", "0", "0.12", "0", "0", "0", base_frame, lidar_frame],
            ),
            Node(
                package="drone_slam",
                executable="known_pose_mapper_node",
                name="known_pose_mapper",
                output="screen",
                parameters=[shared_params, {
                    "scan_topic": "scan",
                    "pose_topic": "mavros/local_position/pose",
                    "map_topic": "mapping/known_pose_map",
                    "map_frame_id": map_frame,
                    "publish_health": False,
                }],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                namespace="slam",
                name="slam_toolbox",
                output="screen",
                parameters=[toolbox_params, {
                    "use_sim_time": True,
                    "map_frame": map_frame,
                    "odom_frame": odom_frame,
                    "base_frame": base_frame,
                    "scan_topic": f"/{name}/scan",
                }],
                remappings=[("/map", f"/{name}/slam/map")],
            ),
            Node(
                package="drone_slam",
                executable="slam_health_node",
                name="slam_health",
                output="screen",
                parameters=[shared_params, {
                    "map_topic": selected_map,
                    "odom_topic": "odom",
                }],
            ),
            Node(
                package="drone_slam",
                executable="map_artifact_recorder",
                name="map_artifact_recorder",
                output="screen",
                parameters=[{
                    "map_topic": selected_map,
                    "mission_phase_topic": "mission/phase",
                    "run_id": run_id,
                    "vehicle_id": name,
                    "map_source": fusion_source,
                    "save_interval_sec": 30.0,
                }],
            ),
            Node(
                package="drone_perception",
                executable="lidar_obstacle_node",
                name="lidar_obstacle",
                output="screen",
                parameters=[autonomy_params, vehicle_params],
            ),
            Node(
                package="drone_planning",
                executable="local_planner_node",
                name="local_planner",
                output="screen",
                parameters=[autonomy_params, vehicle_params],
            ),
            Node(
                package="drone_safety",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
                parameters=[autonomy_params, vehicle_params],
            ),
            Node(
                package="drone_control",
                executable="autonomy_manager",
                name="autonomy_manager",
                output="screen",
                parameters=[autonomy_params, vehicle_params],
            ),
            Node(
                package="drone_metrics",
                executable="metrics_logger",
                name="metrics_logger",
                output="screen",
                parameters=[autonomy_params, vehicle_params, {
                    "baseline_name": "two_uav_known_pose_mapping",
                    "experiment_condition": "gate_c_known_pose_fusion",
                    "scenario_manifest_path": manifest_path,
                    "autonomy_config_path": autonomy_params,
                    "mavros_config_path": mavros_config,
                    "mavros_pluginlists_path": pluginlists,
                    "launch_file_path": __file__,
                }],
            ),
        ]
        actions.append(GroupAction(vehicle_nodes))
        names.append(name)
        source_topics.append(f"/{name}/{selected_map}")
        source_frames.append(map_frame)
        source_x.append(float(spawn[0]))
        source_y.append(float(spawn[1]))
        source_yaw.append(float(spawn[3]))

    fusion = manifest["fusion"]
    min_x, max_x, min_y, max_y = fusion["bounds"]
    actions.append(Node(
        package="drone_map_fusion",
        executable="map_fusion_node",
        name="map_fusion",
        output="screen",
        parameters=[{
            "source_names": names,
            "source_topics": source_topics,
            "source_frames": source_frames,
            "source_x": source_x,
            "source_y": source_y,
            "source_yaw": source_yaw,
            "source_confidence": [1.0] * len(names),
            "global_frame": str(fusion["frame_id"]),
            "resolution": float(fusion["resolution"]),
            "min_x": float(min_x),
            "max_x": float(max_x),
            "min_y": float(min_y),
            "max_y": float(max_y),
            "publish_hz": float(fusion["publish_hz"]),
            "map_timeout_sec": float(fusion["map_timeout_sec"]),
            "free_threshold": int(fusion["free_threshold"]),
            "occupied_threshold": int(fusion["occupied_threshold"]),
            "run_id": run_id,
        }],
    ))
    return actions


def generate_launch_description():
    default_manifest = os.path.join(
        get_package_share_directory("drone_bringup"), "config", "swarm_two_uav.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("manifest", default_value=default_manifest),
        DeclareLaunchArgument("fusion_source", default_value=""),
        DeclareLaunchArgument("run_id", default_value=""),
        OpaqueFunction(function=_load),
    ])
