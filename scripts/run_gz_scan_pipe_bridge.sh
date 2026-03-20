#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

docker compose exec -T sim bash -lc 'gz topic -e -t /scan' \
  | docker compose exec -T ros bash -lc '
      source /opt/ros/humble/setup.bash
      cd /workspace/AV_Drone
      source install/setup.bash
      python3 scripts/gz_scan_stdin_bridge.py
    '
