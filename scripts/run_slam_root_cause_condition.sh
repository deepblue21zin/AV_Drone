#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "usage: $0 {baseline_r1|baseline_r2|lane_swap|scan_matching_off|angle_penalty|odom_penalty|odom_penalty_full|sequential_only_full|no_loop_medium|hard_odom_medium|hard_odom_full|guarded_sequential_full|stable_fusion_full} [max_runtime_sec]" >&2
  exit 2
fi

CONDITION="$1"
MAX_RUNTIME_SEC="${2:-180}"
case "$CONDITION" in
  baseline_r1|baseline_r2|scan_matching_off|angle_penalty|odom_penalty|odom_penalty_full|sequential_only_full|no_loop_medium|hard_odom_medium|hard_odom_full|guarded_sequential_full|stable_fusion_full)
    SPAWN_Y_DRONE1=-7.5
    SPAWN_Y_DRONE2=7.5
    ;;
  lane_swap)
    SPAWN_Y_DRONE1=7.5
    SPAWN_Y_DRONE2=-7.5
    ;;
  *)
    echo "unknown condition: $CONDITION" >&2
    exit 2
    ;;
esac

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="$ROOT_DIR/experiments/slam_root_cause/manifests/${CONDITION}.yaml"
CONTAINER_MANIFEST="/workspace/AV_Drone/experiments/slam_root_cause/manifests/${CONDITION}.yaml"
if [ ! -f "$MANIFEST" ]; then
  echo "manifest not found: $MANIFEST" >&2
  exit 2
fi

RUN_ID="${RUN_ID:-$(date '+%Y-%m-%d_%H-%M-%S')_slam_diag_${CONDITION}}"
ROS_DOMAIN_ID="${SLAM_DIAG_ROS_DOMAIN_ID:-164}"
GAZEBO_MASTER_URI="${SLAM_DIAG_GAZEBO_MASTER_URI:-http://127.0.0.1:11485}"
PX4_INSTANCE_BASE="${SLAM_DIAG_PX4_INSTANCE_BASE:-8}"
SIM_CONTAINER="av-drone-slam-diag-sim-${RUN_ID}"
ROS_CONTAINER="av-drone-slam-diag-ros-${RUN_ID}"
ARTIFACT_DIR="$ROOT_DIR/artifacts/$RUN_ID"

for container in "$SIM_CONTAINER" "$ROS_CONTAINER"; do
  if docker inspect "$container" >/dev/null 2>&1; then
    echo "refusing to reuse existing container: $container" >&2
    exit 3
  fi
done

mkdir -p "$ARTIFACT_DIR"

cleanup() {
  set +e
  if docker inspect "$ROS_CONTAINER" >/dev/null 2>&1; then
    docker logs "$ROS_CONTAINER" >"$ARTIFACT_DIR/runtime_ros.log" 2>&1
    docker stop --time 20 "$ROS_CONTAINER" >/dev/null 2>&1 || true
    docker rm "$ROS_CONTAINER" >/dev/null 2>&1 || true
  fi
  if docker inspect "$SIM_CONTAINER" >/dev/null 2>&1; then
    mkdir -p "$ARTIFACT_DIR/px4"
    for instance in "$PX4_INSTANCE_BASE" "$((PX4_INSTANCE_BASE + 1))"; do
      docker cp \
        "$SIM_CONTAINER:/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/$instance/px4_stdout.log" \
        "$ARTIFACT_DIR/px4/instance_${instance}_stdout.log" \
        >/dev/null 2>&1 || true
      docker cp \
        "$SIM_CONTAINER:/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/$instance/px4_stderr.log" \
        "$ARTIFACT_DIR/px4/instance_${instance}_stderr.log" \
        >/dev/null 2>&1 || true
    done
    docker logs "$SIM_CONTAINER" >"$ARTIFACT_DIR/runtime_sim.log" 2>&1
    docker stop --time 20 "$SIM_CONTAINER" >/dev/null 2>&1 || true
    docker rm "$SIM_CONTAINER" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "[slam diag] run_id=$RUN_ID condition=$CONDITION"
echo "[slam diag] ROS_DOMAIN_ID=$ROS_DOMAIN_ID GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"

docker run -d \
  --name "$SIM_CONTAINER" \
  --network host \
  --ipc host \
  -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
  -e "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI" \
  -e VEHICLE_COUNT=2 \
  -e "PX4_INSTANCE_BASE=$PX4_INSTANCE_BASE" \
  -e PX4_SITL_WORLD=random_cylinders_double \
  -e HEADLESS=1 \
  -e SWARM_SPAWN_X=3.0 \
  -e "SWARM_SPAWN_Y_DRONE1=$SPAWN_Y_DRONE1" \
  -e "SWARM_SPAWN_Y_DRONE2=$SPAWN_Y_DRONE2" \
  -v "$ROOT_DIR:/workspace/AV_Drone" \
  av-drone-sim:latest \
  /workspace/AV_Drone/docker/sim/entrypoint.sh >/dev/null

sim_ready=0
for _ in $(seq 1 90); do
  if ! docker inspect -f '{{.State.Running}}' "$SIM_CONTAINER" 2>/dev/null | grep -q true; then
    echo "sim container exited before readiness" >&2
    docker logs "$SIM_CONTAINER" >&2 || true
    exit 4
  fi
  if docker logs "$SIM_CONTAINER" 2>&1 | grep -q '\[multi sim\] two PX4 vehicles'; then
    sim_ready=1
    break
  fi
  sleep 2
done
if [ "$sim_ready" -ne 1 ]; then
  echo "sim readiness timed out" >&2
  exit 4
fi

docker run -d \
  --name "$ROS_CONTAINER" \
  --network host \
  --ipc host \
  -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
  -e "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI" \
  -v "$ROOT_DIR:/workspace/AV_Drone" \
  -w /workspace/AV_Drone \
  av-drone-ros:latest \
  bash -lc "source /opt/ros/humble/setup.bash && source /workspace/AV_Drone/install/setup.bash && exec ros2 launch drone_bringup multi_drone_slam_fusion.launch.py manifest:=$CONTAINER_MANIFEST run_id:=$RUN_ID" >/dev/null

# Do not spend the experiment time budget on a run where one PX4 never
# connected. This also makes an infrastructure failure explicit instead of
# incorrectly attributing an empty map to the SLAM parameters under test.
fcu_ready=0
for _ in $(seq 1 90); do
  connected=0
  for vehicle in drone1 drone2; do
    metrics="$ARTIFACT_DIR/$vehicle/metrics.csv"
    if [ -f "$metrics" ]; then
      value=$(awk -F, 'END {gsub(/\r/, "", $2); print $2}' "$metrics")
      if [ "$value" = "True" ]; then
        connected=$((connected + 1))
      fi
    fi
  done
  if [ "$connected" -eq 2 ]; then
    fcu_ready=1
    break
  fi
  if ! docker inspect -f '{{.State.Running}}' "$ROS_CONTAINER" 2>/dev/null | grep -q true; then
    break
  fi
  sleep 2
done
if [ "$fcu_ready" -ne 1 ]; then
  echo "both FCUs did not become ready; marking run invalid" >&2
  exit 6
fi

start_epoch=$(date +%s)
while true; do
  if ! docker inspect -f '{{.State.Running}}' "$ROS_CONTAINER" 2>/dev/null | grep -q true; then
    echo "ROS container exited before the stop condition" >&2
    docker logs "$ROS_CONTAINER" >&2 || true
    exit 5
  fi
  elapsed=$(( $(date +%s) - start_epoch ))
  goals=0
  for vehicle in drone1 drone2; do
    trajectory="$ARTIFACT_DIR/$vehicle/trajectory.csv"
    if [ -f "$trajectory" ]; then
      reached=$(awk -F, 'END {gsub(/\r/, "", $9); print $9}' "$trajectory")
      if [ "$reached" = "True" ]; then
        goals=$((goals + 1))
      fi
    fi
  done
  printf '[slam diag] elapsed=%ss goals=%s/2\n' "$elapsed" "$goals"
  if [ "$goals" -eq 2 ]; then
    echo "[slam diag] both vehicles reached the diagnostic goal"
    break
  fi
  if [ "$elapsed" -ge "$MAX_RUNTIME_SEC" ]; then
    echo "[slam diag] reached max runtime ${MAX_RUNTIME_SEC}s"
    break
  fi
  sleep 5
done

# Stop ROS first so recorders can flush while the simulator is still alive.
docker stop --time 20 "$ROS_CONTAINER" >/dev/null
docker logs "$ROS_CONTAINER" >"$ARTIFACT_DIR/runtime_ros.log" 2>&1
docker rm "$ROS_CONTAINER" >/dev/null
docker stop --time 20 "$SIM_CONTAINER" >/dev/null
docker logs "$SIM_CONTAINER" >"$ARTIFACT_DIR/runtime_sim.log" 2>&1
mkdir -p "$ARTIFACT_DIR/px4"
for instance in "$PX4_INSTANCE_BASE" "$((PX4_INSTANCE_BASE + 1))"; do
  docker cp \
    "$SIM_CONTAINER:/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/$instance/px4_stdout.log" \
    "$ARTIFACT_DIR/px4/instance_${instance}_stdout.log" \
    >/dev/null 2>&1 || true
  docker cp \
    "$SIM_CONTAINER:/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/$instance/px4_stderr.log" \
    "$ARTIFACT_DIR/px4/instance_${instance}_stderr.log" \
    >/dev/null 2>&1 || true
done
docker rm "$SIM_CONTAINER" >/dev/null
trap - EXIT INT TERM

echo "$RUN_ID"
