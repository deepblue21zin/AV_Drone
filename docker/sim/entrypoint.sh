#!/usr/bin/env bash
set -euo pipefail

if [ -f /opt/ros/humble/setup.bash ]; then
  # Gazebo ROS plugins need the ROS 2 runtime available in the sim container.
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

cd /opt/PX4-Autopilot

PX4_CLASSIC_ROOT="/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
CUSTOM_MODEL_DIR="/workspace/AV_Drone/sim_assets/models"
CUSTOM_WORLD_DIR="/workspace/AV_Drone/sim_assets/worlds"
PX4_MODEL_DIR="${PX4_CLASSIC_ROOT}/models"
PX4_WORLD_DIR="${PX4_CLASSIC_ROOT}/worlds"

if [ -d "${CUSTOM_MODEL_DIR}" ]; then
  rsync -a "${CUSTOM_MODEL_DIR}/" "${PX4_MODEL_DIR}/"
fi

if [ -d "${CUSTOM_WORLD_DIR}" ]; then
  find "${CUSTOM_WORLD_DIR}" -maxdepth 1 -type f -name "*.world" -exec cp {} "${PX4_WORLD_DIR}/" \;
fi

export PX4_SITL_WORLD="${PX4_SITL_WORLD:-obstacle_demo}"
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
export PX4_GAZEBO_DISPLAY="${PX4_GAZEBO_DISPLAY:-${DISPLAY:-}}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-${PX4_SITL_WORLD}}"

case "${HEADLESS:-}" in
  1|true|TRUE|yes|YES)
    export HEADLESS=1
    ;;
  *)
    unset HEADLESS
    ;;
esac

# PX4's generated make->shell->sitl_run chain drops DISPLAY, but keeps PX4_GAZEBO_DISPLAY.
# Install a lightweight gzserver wrapper earlier in PATH so server-side rendering still works.
cat >/usr/local/bin/gzserver <<'EOF'
#!/usr/bin/env bash
set -e
if [ -z "${DISPLAY:-}" ] && [ -n "${PX4_GAZEBO_DISPLAY:-}" ]; then
  export DISPLAY="${PX4_GAZEBO_DISPLAY}"
fi
exec /usr/bin/gzserver "$@"
EOF
chmod +x /usr/local/bin/gzserver

SIM_TARGET="${PX4_SIM_TARGET:-gazebo-classic_iris_rplidar}"
exec make px4_sitl "${SIM_TARGET}"
