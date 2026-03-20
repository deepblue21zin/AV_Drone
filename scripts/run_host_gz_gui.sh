#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="${ROOT_DIR}/.cache/gz/models"
GZ_MODEL_EXPORT_DIR="/opt/PX4-Autopilot/Tools/simulation/gz/models"
RENDER_ENGINE="${PX4_GZ_GUI_RENDER_ENGINE:-ogre}"
PARTITION="${GZ_PARTITION:-av_drone}"

fail() {
  echo "[FAIL] $1" >&2
  exit 1
}

info() {
  echo "[INFO] $1"
}

if ! command -v gz >/dev/null 2>&1; then
  fail "host machine does not have 'gz' installed. Install Gazebo Sim Harmonic on the host first."
fi

if ! docker compose ps --services --status running | grep -qx sim; then
  fail "docker compose service 'sim' is not running"
fi

mkdir -p "${CACHE_DIR}"

for model_name in x500 x500_base; do
  if [ ! -d "${CACHE_DIR}/${model_name}" ]; then
    info "syncing ${model_name} model from sim container into host cache"
    docker compose exec -T sim bash -lc "tar -C '${GZ_MODEL_EXPORT_DIR}' -cf - ${model_name}" | tar -C "${CACHE_DIR}" -xf -
  fi
done

# Host Gazebo GUI is forced into CPU software rendering.
# This is slower, but it avoids the Wayland + AMD + render-node issues that
# were producing a blank gray window.
export QT_QPA_PLATFORM="xcb"
export GDK_BACKEND="x11"
export XDG_SESSION_TYPE="x11"
unset WAYLAND_DISPLAY
unset QT_QUICK_BACKEND
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER="llvmpipe"
export MESA_LOADER_DRIVER_OVERRIDE="swrast"

export GZ_PARTITION="${PARTITION}"
export IGN_PARTITION="${PARTITION}"
export GZ_SIM_RESOURCE_PATH="${CACHE_DIR}:${ROOT_DIR}/sim_assets/gz/models:${ROOT_DIR}/sim_assets/gz/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

info "starting host Gazebo GUI in CPU software-rendering mode"
info "GZ_PARTITION=${GZ_PARTITION}"
info "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
info "QT_QPA_PLATFORM=${QT_QPA_PLATFORM}"
info "GDK_BACKEND=${GDK_BACKEND}"
info "XDG_SESSION_TYPE=${XDG_SESSION_TYPE}"
info "PX4_GZ_GUI_RENDER_ENGINE=${RENDER_ENGINE}"
info "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"
info "GALLIUM_DRIVER=${GALLIUM_DRIVER}"
info "MESA_LOADER_DRIVER_OVERRIDE=${MESA_LOADER_DRIVER_OVERRIDE}"

exec gz sim --render-engine "${RENDER_ENGINE}" -g --gui-config "${ROOT_DIR}/sim_assets/gz/gui/gui.config"
