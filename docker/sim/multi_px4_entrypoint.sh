#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u
cd /opt/PX4-Autopilot

PX4_ROOT=/opt/PX4-Autopilot
CLASSIC_ROOT="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
BUILD_ROOT="${PX4_ROOT}/build/px4_sitl_default"
RUNTIME_MODELS=/tmp/av_drone_multi_models
WORLD_NAME="${PX4_SITL_WORLD:-random_cylinders_double}"
WORLD_SOURCE="/workspace/AV_Drone/sim_assets/worlds/${WORLD_NAME}.world"
SPAWN_X="${SWARM_SPAWN_X:-3.0}"
SPAWN_Y_DRONE1="${SWARM_SPAWN_Y_DRONE1:--7.5}"
SPAWN_Y_DRONE2="${SWARM_SPAWN_Y_DRONE2:-7.5}"
PX4_INSTANCE_BASE="${PX4_INSTANCE_BASE:-2}"

if [ ! -f "${WORLD_SOURCE}" ]; then
  echo "[multi sim] world not found: ${WORLD_SOURCE}" >&2
  exit 1
fi

cleanup() {
  pkill -x px4 2>/dev/null || true
  pkill -x gzclient 2>/dev/null || true
  pkill -x gzserver 2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup

DONT_RUN=1 make px4_sitl gazebo-classic_iris_rplidar
rsync -a /workspace/AV_Drone/sim_assets/models/ "${CLASSIC_ROOT}/models/"
mkdir -p "${RUNTIME_MODELS}"
set +u
source "${PX4_ROOT}/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "${PX4_ROOT}" "${BUILD_ROOT}"
set -u
export GAZEBO_MODEL_PATH="${RUNTIME_MODELS}:${GAZEBO_MODEL_PATH}"
export PX4_SIM_MODEL=gazebo-classic_iris
export PX4_SIM_WORLD="${WORLD_NAME}"

python3 - "${CLASSIC_ROOT}" "${RUNTIME_MODELS}" <<'PY'
import shutil
import sys
from pathlib import Path

classic_root = Path(sys.argv[1])
runtime = Path(sys.argv[2])
rplidar_source = classic_root / "models" / "rplidar" / "model.sdf"

if runtime.exists():
    shutil.rmtree(runtime)
runtime.mkdir(parents=True)

for index in range(2):
    drone = f"drone{index + 1}"
    iris_dir = runtime / f"iris_{index}"
    lidar_dir = runtime / f"rplidar_{index}"
    wrapper_dir = runtime / f"iris_rplidar_{index}"
    for directory in (iris_dir, lidar_dir, wrapper_dir):
        directory.mkdir(parents=True)

    (iris_dir / "model.config").write_text(
        f"<?xml version='1.0'?><model><name>iris_{index}</name>"
        "<version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>"
    )
    lidar_sdf = rplidar_source.read_text()
    lidar_sdf = lidar_sdf.replace(
        "<namespace>/drone1</namespace>", f"<namespace>/{drone}</namespace>"
    )
    lidar_sdf = lidar_sdf.replace(
        "<frame_name>rplidar_link</frame_name>",
        f"<frame_name>{drone}/lidar_link</frame_name>",
    )
    (lidar_dir / "model.sdf").write_text(lidar_sdf)
    (lidar_dir / "model.config").write_text(
        f"<?xml version='1.0'?><model><name>rplidar_{index}</name>"
        "<version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>"
    )
    (wrapper_dir / "model.sdf").write_text(
        f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="iris_rplidar_{index}">
    <include><uri>model://iris_{index}</uri></include>
    <include><uri>model://rplidar_{index}</uri><pose>0 0 0.1 0 0 0</pose></include>
    <joint name="rplidar_joint" type="fixed">
      <child>rplidar::link</child><parent>iris::base_link</parent>
    </joint>
  </model>
</sdf>
'''
    )
    (wrapper_dir / "model.config").write_text(
        f"<?xml version='1.0'?><model><name>iris_rplidar_{index}</name>"
        "<version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>"
    )
PY

for index in 0 1; do
  instance="$((PX4_INSTANCE_BASE + index))"
  python3 "${CLASSIC_ROOT}/scripts/jinja_gen.py" \
    "${CLASSIC_ROOT}/models/iris/iris.sdf.jinja" \
    "${CLASSIC_ROOT}" \
    --mavlink_tcp_port "$((4560 + instance))" \
    --mavlink_udp_port "$((14560 + instance))" \
    --mavlink_id "$((1 + instance))" \
    --gst_udp_port "$((5600 + instance))" \
    --video_uri "$((5600 + instance))" \
    --mavlink_cam_udp_port "$((14530 + instance))" \
    --output-file "${RUNTIME_MODELS}/iris_${index}/model.sdf"
done

gzserver --verbose "${WORLD_SOURCE}" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
server_pid=$!
sleep 6
kill -0 "${server_pid}"

spawn_y=("${SPAWN_Y_DRONE1}" "${SPAWN_Y_DRONE2}")
for index in 0 1; do
  instance="$((PX4_INSTANCE_BASE + index))"
  working_dir="${BUILD_ROOT}/rootfs/${instance}"
  mkdir -p "${working_dir}"
  (
    cd "${working_dir}"
    "${BUILD_ROOT}/bin/px4" -i "${instance}" -d "${BUILD_ROOT}/etc" \
      >px4_stdout.log 2>px4_stderr.log
  ) &
  gz model --spawn-file="${RUNTIME_MODELS}/iris_rplidar_${index}/model.sdf" \
    --model-name="iris_rplidar_${index}" \
    -x "${SPAWN_X}" -y "${spawn_y[$index]}" -z 0.83
done

echo "[multi sim] two PX4 vehicles in ${WORLD_NAME}: instances=${PX4_INSTANCE_BASE}/$((PX4_INSTANCE_BASE + 1)), x=${SPAWN_X}, y=${SPAWN_Y_DRONE1}/${SPAWN_Y_DRONE2}"
wait "${server_pid}"
