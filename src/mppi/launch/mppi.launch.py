import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # MAVROS
    mavros_share = get_package_share_directory("mavros")
    mavros_launch = os.path.join(mavros_share, "launch", "px4.launch")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch),
        launch_arguments={
            "fcu_url": "udp://:14540@127.0.0.1:14580",
        }.items(),
    )

    # Takeoff node (offboard_control)
    takeoff_node = Node(
        package="offboard_control",
        executable="offboard_takeoff",
        name="offboard_takeoff",
        output="screen",
    )

    # MPPI orchestrator node
    mppi_node = Node(
        package="mppi",
        executable="mppi_node",
        name="mppi_node",
        output="screen",
        parameters=[{
            # Gate / orchestration
            "takeoff_z": 3.0,
            "z_tol": 0.15,
            "stable_sec": 3.0,
            "handover_sec": 3.0,
            "pose_timeout_sec": 0.5,
            "ctrl_rate_hz": 30.0,

            # Goal
            "goal_x": 22.0,
            "goal_y": 0.0,
            "goal_z": 3.0,
            "goal_yaw": float("nan"),

            # Obstacles (arrays)
            "obs_cx": [4.0, 8.0, 12.0, 16.0, 20.0, 4.0, 8.0, 12.0, 16.0, 20.0, 6.0, 9.0, 12.0, 15.0, 18.0],
            "obs_cy": [4.0, 4.0, 4.0, 4.0, 4.0, -4.0, -4.0, -4.0, -4.0, -4.0, 1.5, -1.5, 1.5, -1.5, 1.5],
            "obs_r":  [1.0, 1.0, 1.0, 1.0, 1.0,  1.0,  1.0,  1.0,  1.0,  1.0, 1.2,  1.2, 1.2,  1.2, 1.2],

            # MPPI tuning
            "safe_margin": 2.0,
            "w_goal": 1.0,
            "w_obs": 140.0,
            "obs_beta": 8.0,
            "w_collision": 1e6,

            "w_u": 0.02,
            "w_du": 0.10,
            "w_yaw": 0.3,

            "dt": 0.05,
            "H": 50,
            "num_samples": 600,
            "lambda_": 1.0,

            "max_v_xy": 2.0,
            "max_v_z": 1.0,
            "max_yaw_rate": 1.2,

            "sigma_vxy": 0.7,
            "sigma_vz": 0.3,
            "sigma_yaw_rate": 0.6,
        }],
    )

    return LaunchDescription([
        mavros,
        TimerAction(period=2.0, actions=[takeoff_node]),
        TimerAction(period=3.0, actions=[mppi_node]),
    ])
