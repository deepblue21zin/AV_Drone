#!/usr/bin/env bash
set -e

if [ -f /opt/ros/humble/setup.bash ]; then
  # gazebo_ros plugins require ROS 2 runtime environment in the sim container.
  source /opt/ros/humble/setup.bash
fi

cd /opt/PX4-Autopilot

CUSTOM_RPLIDAR_MODEL="/workspace/AV_Drone/sim_assets/models/rplidar/model.sdf"
PX4_RPLIDAR_MODEL="/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/rplidar/model.sdf"

if [ -f "${CUSTOM_RPLIDAR_MODEL}" ]; then
  cp "${CUSTOM_RPLIDAR_MODEL}" "${PX4_RPLIDAR_MODEL}"
fi

if [ -n "${PX4_GZ_WORLD:-}" ]; then
  export PX4_GZ_WORLD
fi

SIM_TARGET="${PX4_SIM_TARGET:-gazebo-classic_iris_rplidar}"

exec make px4_sitl "${SIM_TARGET}"
