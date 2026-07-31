#!/usr/bin/env bash
set -uo pipefail

ROOT=/home3/quddnr/workspace/AV_Drone
COMPOSE_OVERRIDE=/tmp/quddnr-mppi-lidar.compose.yml
BATCH_DIR="$ROOT/experiments/2026-07-18_mppi_slam_10x"
mkdir -p "$BATCH_DIR"

for run in $(seq -w 1 10); do
  exp="$BATCH_DIR/run_${run}"
  echo "[batch] run=${run} state=recreate"
  DISPLAY=:13 ROS_DOMAIN_ID=31 PX4_INSTANCE=0 PX4_SITL_WORLD=random_cylinders_double \
    docker compose -p quddnr -f "$ROOT/docker-compose.yml" -f "$COMPOSE_OVERRIDE" \
    up -d --force-recreate sim ros

  echo "[batch] run=${run} state=flight"
  docker exec -d quddnr-ros bash -lc "source /opt/ros/humble/setup.bash; source /workspace/AV_Drone/install/setup.bash
EXP=/workspace/AV_Drone/experiments/2026-07-18_mppi_slam_10x/run_${run}; rm -rf \"\$EXP\"; mkdir -p \"\$EXP\"
ros2 bag record -a -x '.*(camera|image|video).*' -o \"\$EXP/rosbag\" >\"\$EXP/rosbag.log\" 2>&1 & BAG_PID=\$!
MAVROS_FCU_URL='udp://:14540@127.0.0.1:14580' ros2 launch mppi_slam mppi_slam.launch.py >\"\$EXP/launch.log\" 2>&1 & LAUNCH_PID=\$!
python3 /workspace/AV_Drone/scripts/wait_flight_success.py --timeout 240 >\"\$EXP/result.json\"; rc=\$?
kill -INT \$BAG_PID \$LAUNCH_PID 2>/dev/null || true
sleep 5
kill -TERM \$BAG_PID \$LAUNCH_PID 2>/dev/null || true
exit \$rc"

  rc=124
  for _ in $(seq 1 270); do
    if [[ -s "$exp/result.json" ]]; then
      if grep -q '"success": true' "$exp/result.json"; then rc=0; else rc=1; fi
      break
    fi
    sleep 1
  done
  # Allow rosbag's SIGINT handler to finish metadata before the next recreate.
  for _ in $(seq 1 15); do
    [[ -f "$exp/rosbag/metadata.yaml" ]] && break
    sleep 1
  done

  if [[ $rc -ne 0 ]]; then
    echo "[batch] run=${run} state=failed rc=${rc}"
    docker exec quddnr-ros rm -rf "/workspace/AV_Drone/experiments/2026-07-18_mppi_slam_10x/run_${run}/rosbag" 2>/dev/null || true
  else
    echo "[batch] run=${run} state=success"
  fi
  if [[ -f "$exp/result.json" ]]; then
    tr -d '\n' < "$exp/result.json"
    echo
  fi
done

echo "[batch] state=complete"
