#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${ROOT_DIR}/.logs"
SIM_TIMEOUT_SEC=600
ROS_TIMEOUT_SEC=180
SKIP_BUILD=0
NO_GUI=0

while (($#)); do
  case "$1" in
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --no-gui)
      NO_GUI=1
      shift
      ;;
    *)
      echo "[FAIL] unknown argument: $1" >&2
      echo "usage: ./start.sh [--skip-build] [--no-gui]" >&2
      exit 1
      ;;
  esac
done

cd "${ROOT_DIR}"
mkdir -p "${LOG_DIR}"

info() {
  echo "[INFO] $1"
}

pass() {
  echo "[PASS] $1"
}

fail() {
  echo "[FAIL] $1" >&2
  exit 1
}

ros_exec() {
  docker compose exec -T ros bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && if [ -f install/setup.bash ]; then source install/setup.bash; fi && $*"
}

wait_for_sim_ready() {
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if docker compose logs --tail=200 sim 2>/dev/null | grep -q "Startup script returned successfully"; then
      pass "sim runtime is ready"
      return 0
    fi
    if (( $(date +%s) - start_ts >= SIM_TIMEOUT_SEC )); then
      fail "sim did not reach runtime readiness within ${SIM_TIMEOUT_SEC}s"
    fi
    sleep 2
  done
}

wait_for_ros_node() {
  local node_name="$1"
  if ros_exec "python3 scripts/wait_for_ros_node.py '${node_name}' --timeout-sec ${ROS_TIMEOUT_SEC}"; then
    pass "ROS node is running: ${node_name}"
    return 0
  fi
  fail "ROS node '${node_name}' was not found within ${ROS_TIMEOUT_SEC}s"
}

wait_for_scan_sample() {
  if ros_exec "python3 scripts/wait_for_scan_sample.py --timeout-sec ${ROS_TIMEOUT_SEC}"; then
    pass "/drone1/scan is publishing"
    return 0
  fi
  fail "/drone1/scan did not publish within ${ROS_TIMEOUT_SEC}s"
}

if (( NO_GUI == 0 )); then
  info "granting local X11 access for Gazebo Classic GUI"
  xhost +SI:localuser:root >/dev/null 2>&1 || true
  xhost +local:docker >/dev/null 2>&1 || true
else
  info "running without Gazebo GUI (--no-gui)"
fi

info "starting sim and ros containers (Gazebo Classic)"
if (( NO_GUI == 0 )); then
  docker compose up -d --force-recreate sim ros >/dev/null
  info "Gazebo Classic GUI will auto-start from the sim container"
else
  HEADLESS=1 docker compose up -d --force-recreate sim ros >/dev/null
  info "running headless (--no-gui)"
fi
wait_for_sim_ready

info "stopping stale ROS launch if present"
docker compose exec -T ros bash -lc "pkill -f 'ros2 launch drone_bringup single_drone_autonomy.launch.py' >/dev/null 2>&1 || true" >/dev/null 2>&1 || true

if (( SKIP_BUILD == 0 )); then
  info "building ROS workspace packages"
  docker compose exec -T ros bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && colcon build --packages-select drone_control drone_bringup drone_perception drone_planning drone_safety drone_metrics ros_states --symlink-install" | tee "${LOG_DIR}/colcon_build.log"
else
  info "skipping colcon build (--skip-build)"
fi

info "starting autonomy launch in ros container"
docker compose exec -T ros bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && nohup ros2 launch drone_bringup single_drone_autonomy.launch.py > /workspace/AV_Drone/.logs/ros_launch.log 2>&1 &"

wait_for_ros_node "/autonomy_manager"
wait_for_ros_node "/metrics_logger"
wait_for_scan_sample

cat <<EOF2

[PASS] Gazebo Classic startup sequence completed

Logs:
  sim logs:         docker compose logs -f sim
  ros launch log:   tail -f .logs/ros_launch.log
  build log:        tail -f .logs/colcon_build.log

Quick checks:
  docker compose exec ros bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /drone1/mission/phase --once'
  docker compose exec ros bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /drone1/scan --once'
  docker compose exec ros bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /mavros/local_position/pose --once'
  ./scripts/smoke_test_single_drone.sh --scenario single_drone_obstacle_demo --notes "start.sh validation"
EOF2
