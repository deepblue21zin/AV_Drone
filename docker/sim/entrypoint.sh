#!/usr/bin/env bash
set -e

cd /opt/PX4-Autopilot

if [ -n "${PX4_GZ_WORLD:-}" ]; then
  export PX4_GZ_WORLD
fi

if [ -n "${PX4_SIM_MODEL:-}" ]; then
  export PX4_SIM_MODEL
fi

exec make px4_sitl gazebo-classic
