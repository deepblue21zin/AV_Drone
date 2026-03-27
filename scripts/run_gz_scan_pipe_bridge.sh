#!/usr/bin/env bash
set -euo pipefail

cat <<'MSG'
[INFO] Gazebo Classic baseline does not use the legacy stdin scan bridge.
[INFO] The LiDAR scan is published directly to ROS 2 as /drone1/scan.
[INFO] Verify it with:
  docker compose exec ros bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /drone1/scan --once'
MSG
