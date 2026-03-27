# AV_Drone Architecture

이 문서는 현재 저장소의 `active path`를 기준으로 설명한다.  
기준 런타임은 `PX4 SITL + Gazebo Classic 11 + ROS 2 Humble + MAVROS` 기반의 단일 드론 baseline이다.

## 1. 아키텍처 요약

```text
Host Ubuntu 22.04
└─ Docker Compose
   ├─ sim
   │  ├─ PX4 SITL
   │  ├─ Gazebo Classic 11
   │  ├─ iris_rplidar
   │  └─ obstacle_demo.world
   └─ ros
      ├─ ROS 2 Humble
      ├─ MAVROS
      ├─ drone_bringup
      ├─ drone_control
      ├─ drone_perception
      ├─ drone_planning
      ├─ drone_safety
      ├─ drone_metrics
      └─ ros_states
```

현재 baseline에서 중요한 점은 아래다.

- `sim`은 시뮬레이션 전담이다.
- `ros`는 autonomy pipeline 전담이다.
- LiDAR는 Gazebo Classic 쪽 plugin을 통해 ROS 2 topic으로 직접 들어온다.
- 현재 active path에는 `ros_gz_bridge`가 없다.

## 2. 런타임 데이터 흐름

```text
Gazebo Classic LiDAR
  -> /drone1/scan
  -> drone_perception/lidar_obstacle_node
  -> /drone1/perception/nearest_obstacle_distance
  -> drone_planning/local_planner_node
  -> /drone1/autonomy/cmd_vel
  -> drone_safety/safety_monitor
  -> /drone1/safety/cmd_vel
  -> drone_control/autonomy_manager
  -> /mavros/setpoint_velocity/cmd_vel

PX4 SITL
  -> MAVROS
  -> /mavros/state
  -> /mavros/local_position/pose
  -> control / metrics / ros_states

All runtime streams
  -> drone_metrics/metrics_logger
  -> artifacts/<timestamp>_drone1/
```

## 3. 컨테이너별 책임

### `sim`

관련 파일:

- [docker/sim/Dockerfile](/home/deepblue/AV_Drone/docker/sim/Dockerfile)
- [docker/sim/entrypoint.sh](/home/deepblue/AV_Drone/docker/sim/entrypoint.sh)
- [sim_assets/models/rplidar/model.sdf](/home/deepblue/AV_Drone/sim_assets/models/rplidar/model.sdf)
- [sim_assets/worlds/obstacle_demo.world](/home/deepblue/AV_Drone/sim_assets/worlds/obstacle_demo.world)

역할:

- PX4 SITL 실행
- Gazebo Classic world/model 준비
- `make px4_sitl gazebo-classic_iris_rplidar` 실행
- GUI가 가능하면 Gazebo Classic client도 같이 사용

### `ros`

관련 파일:

- [docker/ros/Dockerfile](/home/deepblue/AV_Drone/docker/ros/Dockerfile)
- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)

역할:

- ROS 2 Humble workspace 빌드
- MAVROS 실행
- perception / planning / safety / control / metrics 실행
- artifact 저장
- `ros_states` 대시보드 제공

## 4. Active ROS packages

### `drone_bringup`

- [single_drone_autonomy.launch.py](/home/deepblue/AV_Drone/src/drone_bringup/launch/single_drone_autonomy.launch.py)
- [drone1_autonomy.yaml](/home/deepblue/AV_Drone/src/drone_bringup/config/drone1_autonomy.yaml)

역할:

- 현재 baseline의 단일 진입점
- MAVROS와 autonomy pipeline을 함께 launch
- topic 이름, threshold, goal, artifact root 정의

### `drone_perception`

- [lidar_obstacle_node.py](/home/deepblue/AV_Drone/src/drone_perception/drone_perception/lidar_obstacle_node.py)

역할:

- `/drone1/scan` 구독
- finite range만 추려 nearest obstacle distance 생성

### `drone_planning`

- [local_planner_node.py](/home/deepblue/AV_Drone/src/drone_planning/drone_planning/local_planner_node.py)

역할:

- 현재 pose와 goal을 보고 velocity command 생성
- 현재는 LaserScan 기반 `local reactive avoidance` baseline이며, global replanning은 아직 없다

### `drone_safety`

- [safety_monitor_node.py](/home/deepblue/AV_Drone/src/drone_safety/drone_safety/safety_monitor_node.py)

역할:

- `pose_timeout`
- `scan_timeout`
- `planner_cmd_timeout`
- `emergency_stop_obstacle`

를 감시하고 zero-velocity fail-safe를 출력한다.

### `drone_control`

- [autonomy_manager_node.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/autonomy_manager_node.py)
- [vehicle_interface.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/vehicle_interface.py)

역할:

- `WAIT_STREAM -> OFFBOARD_ARM -> TAKEOFF -> HOVER_AFTER_TAKEOFF -> FOLLOW_PLAN -> HOVER_AT_GOAL`
- phase 전환 관리
- safe command를 MAVROS setpoint로 전달

### `drone_metrics`

- [metrics_logger_node.py](/home/deepblue/AV_Drone/src/drone_metrics/drone_metrics/metrics_logger_node.py)

역할:

- `metadata.json`
- `metrics.csv`
- `summary.json`
- `events.log`

를 실행마다 저장한다.

### `ros_states`

- [src/ros_states](/home/deepblue/AV_Drone/src/ros_states)

역할:

- topic / phase / artifact를 웹으로 보여주는 운영 대시보드
- 초심자도 현재 상태를 한눈에 보게 해주는 인수인계 도구

## 5. 현재 검증 상태

현재 기준으로 확인된 것:

- `/drone1/scan` 수신
- `/mavros/local_position/pose` 수신
- `HOVER_AT_GOAL` 도달
- `goal_reached = true`
- artifact 저장

즉 현재 아키텍처는 `single-UAV Gazebo Classic baseline`으로는 살아 있다.  
다만 다중 드론, task reallocation, MPPI 복귀, failure-aware continuation은 아직 상위 연구 단계다.
