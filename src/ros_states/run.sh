#!/bin/bash
# ROS2 State Observer launcher tuned for system ROS environments.
# Usage: ./run.sh [--port PORT] [--update-interval MS] [--drone-name drone1] [...]

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

if [ -z "$ROS_DISTRO" ]; then
  for dist in humble jazzy iron rolling; do
    if [ -f "/opt/ros/$dist/setup.bash" ]; then
      # shellcheck disable=SC1090
      source "/opt/ros/$dist/setup.bash"
      break
    fi
  done
fi

echo "Starting ROS2 State Observer..."
echo "Workspace: $SCRIPT_DIR"
echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
echo ""

python3 app.py "$@"
