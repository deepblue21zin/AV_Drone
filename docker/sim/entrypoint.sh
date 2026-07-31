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

PX4_CLASSIC_TARGETS_CMAKE="/opt/PX4-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
if [ -f "${PX4_CLASSIC_TARGETS_CMAKE}" ] && ! grep -q "^[[:space:]]*${PX4_SITL_WORLD}[[:space:]]*$" "${PX4_CLASSIC_TARGETS_CMAKE}"; then
  sed -i "/^[[:space:]]*empty[[:space:]]*$/a\	${PX4_SITL_WORLD}" "${PX4_CLASSIC_TARGETS_CMAKE}"
  echo "[sim entrypoint] added world target: ${PX4_SITL_WORLD}"
fi

export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
export PX4_GAZEBO_DISPLAY="${PX4_GAZEBO_DISPLAY:-${DISPLAY:-}}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-${PX4_SITL_WORLD}}"

# In an isolated shared network namespace there is no external GCS packet to
# teach PX4 the offboard peer address, so make the loopback target explicit.
PX4_MAVLINK_RC="/opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
if [ -f "${PX4_MAVLINK_RC}" ]; then
  sed -i -E '/udp_offboard_port_local.*-m onboard/ { /-t 127\.0\.0\.1/! s/$/ -t 127.0.0.1/; }' "${PX4_MAVLINK_RC}"
fi

# Optional isolated SITL instance. Instance 1 uses simulator/MAVLink ports
# 4561, 14561, 14581 and 14541 instead of the instance-0 defaults.
PX4_INSTANCE="${PX4_INSTANCE:-0}"
if [ "${PX4_INSTANCE}" -ne 0 ]; then
  IRIS_SDF="${PX4_MODEL_DIR}/iris/iris.sdf"
  IRIS_JINJA="${PX4_MODEL_DIR}/iris/iris.sdf.jinja"
  JINJA_GEN="${PX4_CLASSIC_ROOT}/scripts/jinja_gen.py"
  if [ -f "${JINJA_GEN}" ]; then
    sed -i -E "s/(--mavlink_tcp_port', default=)[0-9]+/\1$((4560 + PX4_INSTANCE))/" "${JINJA_GEN}"
    sed -i -E "s/(--mavlink_udp_port', default=)[0-9]+/\1$((14560 + PX4_INSTANCE))/" "${JINJA_GEN}"
  fi
  if [ -f "${IRIS_JINJA}" ]; then
    sed -i -E "s#<sdk_udp_port>[0-9]+</sdk_udp_port>#<sdk_udp_port>$((14540 + PX4_INSTANCE))</sdk_udp_port>#" "${IRIS_JINJA}"
  fi
  if [ -f "${IRIS_SDF}" ]; then
    sed -i -E "s#<mavlink_tcp_port>[0-9]+</mavlink_tcp_port>#<mavlink_tcp_port>$((4560 + PX4_INSTANCE))</mavlink_tcp_port>#" "${IRIS_SDF}"
    sed -i -E "s#<mavlink_udp_port>[0-9]+</mavlink_udp_port>#<mavlink_udp_port>$((14560 + PX4_INSTANCE))</mavlink_udp_port>#" "${IRIS_SDF}"
    sed -i -E "s#<sdk_udp_port>[0-9]+</sdk_udp_port>#<sdk_udp_port>$((14540 + PX4_INSTANCE))</sdk_udp_port>#" "${IRIS_SDF}"
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
echo "[sim entrypoint] PX4 instance=${PX4_INSTANCE}"

exec make px4_sitl "${MAKE_TARGET}"
