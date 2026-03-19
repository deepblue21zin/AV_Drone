#!/usr/bin/env bash
set -e

if [ -f /opt/ros/humble/setup.bash ]; then
  # gazebo_ros plugins require ROS 2 runtime environment in the sim container.
  source /opt/ros/humble/setup.bash
fi

export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"

cd /opt/PX4-Autopilot

CUSTOM_RPLIDAR_MODEL="/workspace/AV_Drone/sim_assets/models/rplidar/model.sdf"
PX4_RPLIDAR_MODEL="/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/rplidar/model.sdf"
CUSTOM_WORLD_DIR="/workspace/AV_Drone/sim_assets/worlds"
PX4_WORLD_DIR="/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"

if [ -f "${CUSTOM_RPLIDAR_MODEL}" ]; then
  cp "${CUSTOM_RPLIDAR_MODEL}" "${PX4_RPLIDAR_MODEL}"
fi

if [ -d "${CUSTOM_WORLD_DIR}" ]; then
  find "${CUSTOM_WORLD_DIR}" -maxdepth 1 -type f -name "*.world" -exec cp {} "${PX4_WORLD_DIR}/" \;
fi

if [ -n "${PX4_SITL_WORLD:-}" ]; then
  export PX4_SITL_WORLD
fi

SIM_TARGET="${PX4_SIM_TARGET:-gazebo-classic_iris_rplidar}"

exec make px4_sitl "${SIM_TARGET}"
