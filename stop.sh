#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

pkill -f 'scripts/run_host_gz_gui.sh' >/dev/null 2>&1 || true
pkill -f 'docker compose exec sim bash -lc.*gzclient' >/dev/null 2>&1 || true
docker compose exec -T ros bash -lc "pkill -f 'ros2 launch drone_bringup single_drone_autonomy.launch.py' >/dev/null 2>&1 || true" >/dev/null 2>&1 || true
docker compose down

echo "[PASS] stopped Gazebo Classic / PX4 containers and ROS launch"
