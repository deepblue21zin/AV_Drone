#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /workspace/AV_Drone/install/setup.bash ]; then
  source /workspace/AV_Drone/install/setup.bash
fi

if [ "$#" -eq 0 ]; then
  exec bash
fi

exec "$@"

