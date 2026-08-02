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
PX4_INSTANCE="${PX4_INSTANCE:-0}"
VEHICLE_COUNT="${VEHICLE_COUNT:-1}"

if ! [[ "${PX4_INSTANCE}" =~ ^[0-9]$ ]]; then
  echo "[sim entrypoint] PX4_INSTANCE must be an integer from 0 to 9" >&2
  exit 2
fi
if ! [[ "${VEHICLE_COUNT}" =~ ^[1-9]$ ]]; then
  echo "[sim entrypoint] VEHICLE_COUNT must be an integer from 1 to 9" >&2
  exit 2
fi

export PX4_INSTANCE
MAVLINK_TCP_PORT=$((4560 + PX4_INSTANCE))
MAVLINK_UDP_PORT=$((14560 + PX4_INSTANCE))
MAVLINK_SDK_PORT=$((14540 + PX4_INSTANCE))

if [ -d "${CUSTOM_MODEL_DIR}" ]; then
  rsync -a "${CUSTOM_MODEL_DIR}/" "${PX4_MODEL_DIR}/"
fi

if [ -d "${CUSTOM_WORLD_DIR}" ]; then
  find "${CUSTOM_WORLD_DIR}" -maxdepth 1 -type f -name "*.world" -exec cp {} "${PX4_WORLD_DIR}/" \;
fi

export PX4_SITL_WORLD="${PX4_SITL_WORLD:-obstacle_demo}"

PX4_CLASSIC_TARGETS_CMAKE="/opt/PX4-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
if [ -f "${PX4_CLASSIC_TARGETS_CMAKE}" ] && ! grep -q "^[[:space:]]*${PX4_SITL_WORLD}[[:space:]]*$" "${PX4_CLASSIC_TARGETS_CMAKE}"; then
  sed -i "/^[[:space:]]*empty[[:space:]]*$/a\	${PX4_SITL_WORLD}" "${PX4_CLASSIC_TARGETS_CMAKE}"
  echo "[sim entrypoint] added world target: ${PX4_SITL_WORLD}"
fi

export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
export PX4_GAZEBO_DISPLAY="${PX4_GAZEBO_DISPLAY:-${DISPLAY:-}}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-${PX4_SITL_WORLD}}"

if [ "${VEHICLE_COUNT}" -gt 1 ]; then
  MULTI_RUN="/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh"
  if [ ! -f "${MULTI_RUN}" ]; then
    echo "[sim entrypoint] missing PX4 multi-vehicle launcher: ${MULTI_RUN}" >&2
    exit 3
  fi
  DONT_RUN=1 make px4_sitl gazebo-classic
  python3 /workspace/AV_Drone/docker/sim/prepare_swarm_models.py \
    --model-dir "${PX4_MODEL_DIR}" --count "${VEHICLE_COUNT}"
  MULTI_RUN_COPY="/tmp/av_drone_sitl_multiple_run.sh"
  cp "${MULTI_RUN}" "${MULTI_RUN_COPY}"
  sed -i 's/spawn_model ${target_vehicle}${LABEL} $(($n + 1)) $target_x $target_y/spawn_model "${target_vehicle}${LABEL}_drone$(($n + 1))" "$(($n + 1))" "$target_x" "$target_y"/' "${MULTI_RUN_COPY}"
  for ordinal in $(seq 1 "${VEHICLE_COUNT}"); do
    sed -i "s/SUPPORTED_MODELS=(/SUPPORTED_MODELS=(\"iris_rplidar_drone${ordinal}\" /" "${MULTI_RUN_COPY}"
  done
  if ! grep -q 'vehicle_model}_drone' "${MULTI_RUN_COPY}"; then
    if ! grep -q 'target_vehicle}${LABEL}_drone' "${MULTI_RUN_COPY}"; then
      echo "[sim entrypoint] PX4 multi-run script layout changed; refusing unnamespaced LiDAR launch" >&2
      exit 4
    fi
  fi
  echo "[sim entrypoint] launching ${VEHICLE_COUNT} isolated iris_rplidar vehicles"
  SWARM_SPAWN_SCRIPT="${SWARM_SPAWN_SCRIPT:-iris:1:0:-3,iris:1:0:3}"
  exec bash "${MULTI_RUN_COPY}" -s "${SWARM_SPAWN_SCRIPT}" -l rplidar -w "${PX4_SITL_WORLD}"
fi

# In an isolated shared network namespace there is no external GCS packet to
# teach PX4 the offboard peer address, so make the loopback target explicit.
PX4_MAVLINK_RC="/opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
if [ -f "${PX4_MAVLINK_RC}" ]; then
  sed -i -E '/udp_offboard_port_local.*-m onboard/ { /-t 127\.0\.0\.1/! s/$/ -t 127.0.0.1/; }' "${PX4_MAVLINK_RC}"
fi

# Optional isolated SITL instance. Each non-zero instance receives its own
# simulator and MAVLink ports instead of the instance-0 defaults.
if [ "${PX4_INSTANCE}" -ne 0 ]; then
  IRIS_SDF="${PX4_MODEL_DIR}/iris/iris.sdf"
  IRIS_JINJA="${PX4_MODEL_DIR}/iris/iris.sdf.jinja"
  JINJA_GEN="${PX4_CLASSIC_ROOT}/scripts/jinja_gen.py"
  if [ -f "${JINJA_GEN}" ]; then
    sed -i -E "s/(--mavlink_tcp_port', default=)[0-9]+/\1${MAVLINK_TCP_PORT}/" "${JINJA_GEN}"
    sed -i -E "s/(--mavlink_udp_port', default=)[0-9]+/\1${MAVLINK_UDP_PORT}/" "${JINJA_GEN}"
  fi
  if [ -f "${IRIS_JINJA}" ]; then
    sed -i -E "s#<sdk_udp_port>[0-9]+</sdk_udp_port>#<sdk_udp_port>${MAVLINK_SDK_PORT}</sdk_udp_port>#" "${IRIS_JINJA}"
  fi
  if [ -f "${IRIS_SDF}" ]; then
    sed -i -E "s#<mavlink_tcp_port>[0-9]+</mavlink_tcp_port>#<mavlink_tcp_port>${MAVLINK_TCP_PORT}</mavlink_tcp_port>#" "${IRIS_SDF}"
    sed -i -E "s#<mavlink_udp_port>[0-9]+</mavlink_udp_port>#<mavlink_udp_port>${MAVLINK_UDP_PORT}</mavlink_udp_port>#" "${IRIS_SDF}"
    sed -i -E "s#<sdk_udp_port>[0-9]+</sdk_udp_port>#<sdk_udp_port>${MAVLINK_SDK_PORT}</sdk_udp_port>#" "${IRIS_SDF}"
  fi
  SITL_RUN="/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh"
  if [ -f "${SITL_RUN}" ]; then
    sed -i -E "s#^sitl_command=.*#sitl_command=\"\\\"\$sitl_bin\\\" -i ${PX4_INSTANCE} \$no_pxh \\\"\$build_path\\\"/etc\"#" "${SITL_RUN}"
  fi
else
  IRIS_SDF="${PX4_MODEL_DIR}/iris/iris.sdf"
  IRIS_JINJA="${PX4_MODEL_DIR}/iris/iris.sdf.jinja"
  JINJA_GEN="${PX4_CLASSIC_ROOT}/scripts/jinja_gen.py"
  SITL_RUN="/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh"
  [ ! -f "${JINJA_GEN}" ] || sed -i -E "s/(--mavlink_tcp_port', default=)[0-9]+/\14560/; s/(--mavlink_udp_port', default=)[0-9]+/\114560/" "${JINJA_GEN}"
  [ ! -f "${IRIS_JINJA}" ] || sed -i -E "s#<sdk_udp_port>[0-9]+</sdk_udp_port>#<sdk_udp_port>14540</sdk_udp_port>#" "${IRIS_JINJA}"
  [ ! -f "${IRIS_SDF}" ] || sed -i -E "s#<mavlink_tcp_port>[0-9]+</mavlink_tcp_port>#<mavlink_tcp_port>4560</mavlink_tcp_port>#; s#<mavlink_udp_port>[0-9]+</mavlink_udp_port>#<mavlink_udp_port>14560</mavlink_udp_port>#; s#<sdk_udp_port>[0-9]+</sdk_udp_port>#<sdk_udp_port>14540</sdk_udp_port>#" "${IRIS_SDF}"
  [ ! -f "${SITL_RUN}" ] || sed -i -E 's#^sitl_command=.*#sitl_command="\\"$sitl_bin\\" $no_pxh \\"$build_path\\"/etc"#' "${SITL_RUN}"
fi

case "${HEADLESS:-}" in
  1|true|TRUE|yes|YES)
    export HEADLESS=1
    ;;
  *)
    unset HEADLESS
    ;;
esac

if [ -z "${HEADLESS:-}" ]; then
  # Reset stale Gazebo Classic GUI state so the world camera pose controls the first view.
  rm -f /root/.gazebo/gui.ini
fi

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

if [[ "${SIM_TARGET}" == *"__"* ]]; then
  MAKE_TARGET="${SIM_TARGET}"
else
  MAKE_TARGET="${SIM_TARGET}__${PX4_SITL_WORLD}"
fi

echo "[sim entrypoint] PX4_SITL_WORLD=${PX4_SITL_WORLD}"
echo "[sim entrypoint] PX4 make target=${MAKE_TARGET}"
echo "[sim entrypoint] PX4_INSTANCE=${PX4_INSTANCE} (Gazebo TCP ${MAVLINK_TCP_PORT}, UDP ${MAVLINK_UDP_PORT}, SDK ${MAVLINK_SDK_PORT})"

exec make px4_sitl "${MAKE_TARGET}"
