#!/usr/bin/env bash
set -euo pipefail

DISPLAY_VALUE="${DISPLAY:-}"
PX4_HOME_DEFAULT="/opt/PX4-Autopilot"
BUILD_DIR_DEFAULT="${PX4_HOME_DEFAULT}/build/px4_sitl_default"
GAZEBO_CLASSIC_DIR_DEFAULT="${PX4_HOME_DEFAULT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

fail() {
  echo "[FAIL] $1" >&2
  exit 1
}

info() {
  echo "[INFO] $1"
}

if [ -z "${DISPLAY_VALUE}" ]; then
  fail "DISPLAY is not set. Run 'echo $DISPLAY' first and allow X11 access with 'xhost +local:docker'."
fi

if ! docker compose ps --services --status running | grep -qx sim; then
  fail "docker compose service 'sim' is not running"
fi

info "waiting for gzserver to be ready"
for _ in $(seq 1 40); do
  if docker compose exec -T sim bash -lc "pgrep -x gzserver >/dev/null 2>&1"; then
    break
  fi
  sleep 1
done

if ! docker compose exec -T sim bash -lc "pgrep -x gzserver >/dev/null 2>&1"; then
  fail "gzserver is not running yet. Wait for 'Startup script returned successfully' in sim logs first."
fi

if docker compose exec -T sim bash -lc "pgrep -x gzclient >/dev/null 2>&1"; then
  info "restarting existing Gazebo Classic client"
  docker compose exec -T sim bash -lc "pkill -x gzclient || true"
  sleep 1
fi

info "resetting stale Gazebo GUI state"
docker compose exec -T sim bash -lc "rm -f /root/.gazebo/gui.ini && mkdir -p /root/.gazebo && printf '[geometry]
x=0
y=0
width=1600
height=900
' > /root/.gazebo/gui.ini"

info "starting Gazebo Classic client from the sim container with server-matching Gazebo paths"
exec docker compose exec sim bash -lc '
  export DISPLAY="'"${DISPLAY_VALUE}"'";
  export QT_X11_NO_MITSHM=1;
  export QT_QPA_PLATFORM=xcb;
  export GDK_BACKEND=x11;
  export XDG_SESSION_TYPE=x11;
  unset WAYLAND_DISPLAY;
  unset LIBGL_ALWAYS_SOFTWARE;
  unset GALLIUM_DRIVER;
  unset MESA_LOADER_DRIVER_OVERRIDE;
  export QT_OPENGL=desktop;
  unset QT_XCB_GL_INTEGRATION;
  unset MESA_GL_VERSION_OVERRIDE;
  unset LIBGL_ALWAYS_INDIRECT;
  unset __GLX_VENDOR_LIBRARY_NAME;
  unset LIBGL_DRI3_DISABLE;

  PX4_HOME="${PX4_HOME:-'"${PX4_HOME_DEFAULT}"'}";
  BUILD_DIR="${PX4_HOME}/build/px4_sitl_default";
  GAZEBO_CLASSIC_DIR="${PX4_HOME}/Tools/simulation/gazebo-classic/sitl_gazebo-classic";

  export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:${GAZEBO_CLASSIC_DIR}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}";
  export GAZEBO_PLUGIN_PATH="${BUILD_DIR}/build_gazebo-classic${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}";
  export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:${BUILD_DIR}/build_gazebo-classic${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}";
  export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}";
  export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}";
  export GAZEBO_MODEL_DATABASE_URI="";
  export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_CLASSIC_DIR}${GAZEBO_RESOURCE_PATH:+:${GAZEBO_RESOURCE_PATH}}";

  echo "[INFO] GAZEBO_IP=${GAZEBO_IP}";
  echo "[INFO] GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}";
  echo "[INFO] GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}";
  exec gzclient
'
