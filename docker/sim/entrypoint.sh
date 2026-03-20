#!/usr/bin/env bash
set -e

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}" || true

cd /opt/PX4-Autopilot

# Reset cached Gazebo GUI layouts and force a known-good camera view.
rm -f /root/.gz/sim/8/gui.config /root/.gz/sim/7/gui.config

CUSTOM_GZ_MODEL_DIR="/workspace/AV_Drone/sim_assets/gz/models"
CUSTOM_GZ_WORLD_DIR="/workspace/AV_Drone/sim_assets/gz/worlds"
CUSTOM_GZ_GUI_CONFIG="/workspace/AV_Drone/sim_assets/gz/gui/gui.config"
PX4_GZ_MODEL_DIR="/opt/PX4-Autopilot/Tools/simulation/gz/models"
PX4_GZ_WORLD_DIR="/opt/PX4-Autopilot/Tools/simulation/gz/worlds"

if [ -d "${CUSTOM_GZ_MODEL_DIR}" ]; then
  rsync -a "${CUSTOM_GZ_MODEL_DIR}/" "${PX4_GZ_MODEL_DIR}/"
fi

if [ -d "${CUSTOM_GZ_WORLD_DIR}" ]; then
  find "${CUSTOM_GZ_WORLD_DIR}" -maxdepth 1 -type f -name "*.sdf" -exec cp {} "${PX4_GZ_WORLD_DIR}/" \;
fi

if [ -f "${CUSTOM_GZ_GUI_CONFIG}" ]; then
  mkdir -p /root/.gz/sim/8
  cp "${CUSTOM_GZ_GUI_CONFIG}" /root/.gz/sim/8/gui.config
fi

export PX4_GZ_WORLD="${PX4_GZ_WORLD:-obstacle_demo}"
export PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-drone1}"
export PX4_GZ_SIM_RENDER_ENGINE="${PX4_GZ_SIM_RENDER_ENGINE:-ogre2}"

HOST_SHARED_ROOT="${HOST_SHARED_ROOT:-/home/deepblue/AV_Drone}"
SHARED_GZ_CACHE_DIR="${HOST_SHARED_ROOT}/.cache/gz/models"
mkdir -p "${SHARED_GZ_CACHE_DIR}"

for model_name in x500 x500_base; do
  if [ -d "${PX4_GZ_MODEL_DIR}/${model_name}" ]; then
    rsync -a "${PX4_GZ_MODEL_DIR}/${model_name}/" "${SHARED_GZ_CACHE_DIR}/${model_name}/"
  fi
done

if [ -n "${HEADLESS:-}" ]; then
  unset DISPLAY
fi
export GZ_SIM_RESOURCE_PATH="${SHARED_GZ_CACHE_DIR}:${HOST_SHARED_ROOT}/sim_assets/gz/models:${HOST_SHARED_ROOT}/sim_assets/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

SIM_TARGET="${PX4_SIM_TARGET:-gz_x500}"

exec make px4_sitl "${SIM_TARGET}"
