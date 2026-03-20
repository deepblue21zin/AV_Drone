#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

pkill -f 'gz sim -g' >/dev/null 2>&1 || true
pkill -f 'run_gz_scan_pipe_bridge.sh' >/dev/null 2>&1 || true
docker compose exec -T ros bash -lc "pkill -f 'ros2 launch drone_bringup single_drone_autonomy.launch.py' >/dev/null 2>&1 || true" >/dev/null 2>&1 || true
docker compose exec -T ros bash -lc "pkill -f 'gz_scan_stdin_bridge.py' >/dev/null 2>&1 || true" >/dev/null 2>&1 || true
docker compose down

echo "[PASS] stopped host Gazebo GUI, scan bridge, ROS launch, and Docker services"
