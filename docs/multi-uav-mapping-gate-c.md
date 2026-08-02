# Two-UAV mapping and known-pose fusion

This implementation covers Gate A through Gate C of
`multi_uav_slam_map_stitching_expansion_plan.md` for two PX4 v1.14.3 SITL
vehicles. It runs the pose-assisted mapper and `slam_toolbox` side-by-side;
the fusion input is selected by the manifest and defaults to the deterministic
`known_pose` baseline.

## Runtime contract

The source of truth is `src/drone_bringup/config/swarm_two_uav.yaml`.

| Vehicle | PX4 instance | SYSID | MAVROS URL | Spawn |
|---|---:|---:|---|---|
| drone1 | 0 | 2 | `udp://:14541@127.0.0.1:14581` | `(0, -3, 0.2, 0)` |
| drone2 | 1 | 3 | `udp://:14542@127.0.0.1:14582` | `(0, 3, 0.2, 0)` |

PX4 v1.14.3's Gazebo Classic multi-run script starts instances at 1, so its
first two MAVLink system IDs are 2 and 3. The sim entrypoint also generates a
separate `iris_rplidar_droneN` model for every vehicle. This is required because
the upstream LiDAR model has one fixed ROS namespace and frame.

The TF ownership is:

```text
map_fusion:  swarm_map -> droneN/map        (static known pose)
slam_toolbox: droneN/map -> droneN/odom     (dynamic)
pose_odom_tf: droneN/odom -> droneN/base_link (dynamic)
bringup:      droneN/base_link -> droneN/lidar_link (static)
```

No second publisher is allowed for any one of these transforms.

## Build and run

Start the two-vehicle simulation in the dedicated deterministic world:

```bash
export RUN_ID="$(date +%Y-%m-%d_%H-%M-%S)_swarm"
VEHICLE_COUNT=2 PX4_SITL_WORLD=swarm_two_lane RUN_ID="$RUN_ID" \
  docker compose up --build sim ros
```

In a second terminal, build the affected packages and start both local stacks
plus the central fusion node:

```bash
docker compose exec ros bash -lc '
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-up-to drone_bringup
  source install/setup.bash
  ros2 launch drone_bringup multi_drone_slam_fusion.launch.py \
    run_id:="${RUN_ID}" fusion_source:=known_pose
'
```

For the scan-matching comparison, change only
`fusion_source:=slam`. Both mappers run in either mode, so this does not alter
the sensor or vehicle trajectory baseline.

## Gate checks

Gate A (SLAM/map production):

```bash
ros2 topic hz /drone1/mapping/known_pose_map
ros2 topic hz /drone1/slam/map
ros2 topic echo /drone1/slam/coverage --once
ros2 run tf2_ros tf2_echo drone1/map drone1/base_link
```

Gate B (namespace and control isolation):

```bash
ros2 topic list | sort | grep -E '^/(drone1|drone2)/'
ros2 topic info /drone1/scan --verbose
ros2 topic info /drone2/scan --verbose
ros2 run tf2_tools view_frames
```

Each scan topic must have exactly one Gazebo publisher, MAVROS topics must live
below their vehicle namespace, and `view_frames` must show no duplicate TF
authority. The two autonomy managers use local goals `(38, 0, 3)`, which map to
the two physical lanes because MAVROS local coordinates start at each spawn.

Gate C (known-pose fusion and fallback):

```bash
ros2 topic hz /swarm/global_map
ros2 topic echo /swarm/map_version --once
ros2 topic echo /swarm/fusion_status --once
```

Stop one local mapper and wait longer than two seconds. Fusion status must move
from `HEALTHY` to `LOCAL_ONLY_FALLBACK`, while `/swarm/global_map` continues at
1 Hz from the remaining source. The run output is:

```text
artifacts/<run_id>/
├── fusion_metrics.csv
├── swarm_summary.json
├── drone1/maps/
└── drone2/maps/
```

`swarm_summary.json` contains fusion p95 latency, source state counts, and the
latest conflict ratio. Per-vehicle metrics use the same run ID under their own
directories.

## Current boundary

This is the known-pose Gate C baseline. Global A* consumption of
`/swarm/global_map`, MPPI global-path tracking, unknown-pose registration, peer
trajectory collision costs, and four-vehicle scaling remain later gates.
