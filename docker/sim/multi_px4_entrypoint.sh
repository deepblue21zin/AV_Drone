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
WORLD_NAME="${PX4_SITL_WORLD:-random_cylinders_double_42}"
WORLD_SOURCE="/workspace/AV_Drone/sim_assets/worlds/${WORLD_NAME}.world"

if [ ! -f "${WORLD_SOURCE}" ]; then
  echo "world not found: ${WORLD_SOURCE}" >&2
  exit 1
fi

cleanup() {
  pkill -x px4 2>/dev/null || true
  pkill -x gzclient 2>/dev/null || true
  pkill -x gzserver 2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup

# Compose image에는 PX4 바이너리가 있지만 Gazebo 플러그인은 컨테이너 시작
# 시 빌드된다. 시뮬레이션은 실행하지 않고 필요한 플러그인만 먼저 준비한다.
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
iris_source = classic_root / "models" / "iris"
rplidar_source = classic_root / "models" / "rplidar" / "model.sdf"

for index in range(3):
    iris_dir = runtime / f"iris_{index}"
    lidar_dir = runtime / f"rplidar_{index}"
    wrapper_dir = runtime / f"iris_rplidar_{index}"
    for directory in (iris_dir, lidar_dir, wrapper_dir):
        directory.mkdir(parents=True, exist_ok=True)

    (iris_dir / "model.config").write_text(
        f"<?xml version='1.0'?><model><name>iris_{index}</name>"
        "<version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>"
    )
    lidar_sdf = rplidar_source.read_text().replace(
        "<namespace>/drone1</namespace>",
        f"<namespace>/drone{index + 1}</namespace>",
    )
    (lidar_dir / "model.sdf").write_text(lidar_sdf)
    (lidar_dir / "model.config").write_text(
        f"<?xml version='1.0'?><model><name>rplidar_{index}</name>"
        "<version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>"
    )
    wrapper_sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="iris_rplidar_{index}">
    <include><uri>model://iris_{index}</uri></include>
    <include><uri>model://rplidar_{index}</uri><pose>0 0 0.1 0 0 0</pose></include>
    <joint name="rplidar_joint" type="fixed">
      <child>rplidar::link</child>
      <parent>iris::base_link</parent>
    </joint>
  </model>
</sdf>
"""
    (wrapper_dir / "model.sdf").write_text(wrapper_sdf)
    (wrapper_dir / "model.config").write_text(
        f"<?xml version='1.0'?><model><name>iris_rplidar_{index}</name>"
        "<version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>"
    )
PY

for index in 0 1 2; do
  python3 "${CLASSIC_ROOT}/scripts/jinja_gen.py" \
    "${CLASSIC_ROOT}/models/iris/iris.sdf.jinja" \
    "${CLASSIC_ROOT}" \
    --mavlink_tcp_port "$((4560 + index))" \
    --mavlink_udp_port "$((14560 + index))" \
    --mavlink_id "$((1 + index))" \
    --gst_udp_port "$((5600 + index))" \
    --video_uri "$((5600 + index))" \
    --mavlink_cam_udp_port "$((14530 + index))" \
    --output-file "${RUNTIME_MODELS}/iris_${index}/model.sdf"
done

gzserver --verbose "${WORLD_SOURCE}" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
server_pid=$!

sleep 6
kill -0 "${server_pid}"

spawn_y=(-7.5 7.5 0.0)
for index in 0 1 2; do
  working_dir="${BUILD_ROOT}/rootfs/${index}"
  mkdir -p "${working_dir}"
  (
    cd "${working_dir}"
    "${BUILD_ROOT}/bin/px4" -i "${index}" -d "${BUILD_ROOT}/etc" \
      >px4_stdout.log 2>px4_stderr.log
  ) &
  gz model --spawn-file="${RUNTIME_MODELS}/iris_rplidar_${index}/model.sdf" \
    --model-name="iris_rplidar_${index}" \
    -x 3.0 -y "${spawn_y[$index]}" -z 0.83
done

echo "three PX4 vehicles spawned in ${WORLD_NAME}: x=3.0, y=-7.5/+7.5/0.0"
wait "${server_pid}"
