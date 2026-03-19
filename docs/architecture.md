# AV_Drone Architecture

이 문서는 현재 저장소의 `active path`를 기준으로 코드 구조와 런타임 구조를 설명한다.  
기준은 `single_drone_autonomy.launch.py` 기반의 단일 드론 baseline이며, 이후 이를 `failure-aware mission continuation` 연구 플랫폼으로 확장하는 방향을 전제로 한다.

## 1. 현재 아키텍처 요약

```text
Host Ubuntu 22.04
└─ Docker Compose
   ├─ sim
   │  ├─ PX4 SITL
   │  ├─ Gazebo Classic
   │  ├─ iris_rplidar model
   │  └─ obstacle_demo.world
   └─ ros
      ├─ ROS 2 Humble
      ├─ MAVROS
      ├─ drone_bringup
      ├─ drone_control
      ├─ drone_perception
      ├─ drone_planning
      ├─ drone_safety
      └─ drone_metrics
```

현재는 `single-UAV baseline`이 active path다.  
`mppi` 패키지는 삭제하지 않았지만, 현재 주 실행 경로가 아니라 레거시 baseline / 비교용 코드로 둔다.

## 2. 런타임 데이터 흐름

```text
Gazebo Classic + PX4 SITL
  -> MAVROS
  -> /mavros/local_position/pose
  -> drone_control/autonomy_manager

Gazebo LiDAR
  -> /drone1/scan
  -> drone_perception/lidar_obstacle_node
  -> /drone1/perception/nearest_obstacle_distance
  -> drone_planning/local_planner_node
  -> /drone1/autonomy/cmd_vel
  -> drone_safety/safety_monitor
  -> /drone1/safety/cmd_vel
  -> drone_control/autonomy_manager
  -> /mavros/setpoint_velocity/cmd_vel

All state / scan / phase / event streams
  -> drone_metrics/metrics_logger
  -> artifacts/<timestamp>_drone1/
  -> experiments/index.csv, scenario_table.csv, ledger.csv
  -> experiments/plots/<run_id>/
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
- Gazebo Classic 실행
- custom LiDAR model 반영
- custom world 반영

메모:

- 컨테이너가 `Up`이어도 내부에서 PX4/Gazebo build가 진행 중일 수 있다.
- smoke test는 `Startup script returned successfully`가 나올 때까지 기다리도록 설계돼 있다.

### `ros`

관련 파일:

- [docker/ros/Dockerfile](/home/deepblue/AV_Drone/docker/ros/Dockerfile)
- [docker/ros/entrypoint.sh](/home/deepblue/AV_Drone/docker/ros/entrypoint.sh)
- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)

역할:

- ROS 2 Humble workspace 빌드
- MAVROS 실행
- autonomy pipeline 실행
- artifact 저장

## 4. active ROS packages

### `drone_bringup`

관련 파일:

- [single_drone_autonomy.launch.py](/home/deepblue/AV_Drone/src/drone_bringup/launch/single_drone_autonomy.launch.py)
- [drone1_autonomy.yaml](/home/deepblue/AV_Drone/src/drone_bringup/config/drone1_autonomy.yaml)

역할:

- 현재 baseline의 단일 진입점
- MAVROS와 autonomy pipeline 노드를 함께 launch
- 주요 topic / threshold / goal / artifact root를 YAML로 정의

### `drone_control`

관련 파일:

- [autonomy_manager_node.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/autonomy_manager_node.py)
- [vehicle_interface.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/vehicle_interface.py)

역할:

- mission phase 관리
- OFFBOARD / arm / takeoff / hover / follow-plan 상태 전환
- safe command를 MAVROS velocity setpoint로 전달

핵심 phase:

- `WAIT_STREAM`
- `OFFBOARD_ARM`
- `TAKEOFF`
- `HOVER_AFTER_TAKEOFF`
- `FOLLOW_PLAN`
- `HOVER_AT_GOAL`

### `drone_perception`

관련 파일:

- [lidar_obstacle_node.py](/home/deepblue/AV_Drone/src/drone_perception/drone_perception/lidar_obstacle_node.py)

역할:

- `/drone1/scan` 구독
- finite range만 추려 nearest obstacle distance 추출
- `/drone1/perception/nearest_obstacle_distance` 발행

### `drone_planning`

관련 파일:

- [local_planner_node.py](/home/deepblue/AV_Drone/src/drone_planning/drone_planning/local_planner_node.py)

역할:

- current pose와 goal을 이용해 local velocity command 생성
- `goal_reached` 판정
- 현재는 latch 방식으로 goal 판정을 안정화함

### `drone_safety`

관련 파일:

- [safety_monitor_node.py](/home/deepblue/AV_Drone/src/drone_safety/drone_safety/safety_monitor_node.py)

역할:

- `pose_timeout`
- `scan_timeout`
- `planner_cmd_timeout`
- `emergency_stop_obstacle`

을 감시하고, 필요 시 zero velocity fail-safe를 출력한다.

### `drone_metrics`

관련 파일:

- [metrics_logger_node.py](/home/deepblue/AV_Drone/src/drone_metrics/drone_metrics/metrics_logger_node.py)

역할:

- `metadata.json`
- `metrics.csv`
- `summary.json`
- `events.log`

를 실행마다 저장한다.

현재 baseline에서 중요한 필드:

- `mission_phase`
- `goal_reached`
- `pose_count`
- `scan_count`
- `closest_obstacle_m`
- `pose_period_p99_s`
- `scan_period_p99_s`

## 5. 주요 topic

상태 / 제어:

- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/setpoint_velocity/cmd_vel`
- `/drone1/mission/phase`
- `/drone1/mission/goal_reached`

센서 / planner:

- `/drone1/scan`
- `/drone1/perception/nearest_obstacle_distance`
- `/drone1/autonomy/cmd_vel`
- `/drone1/safety/cmd_vel`

계측:

- artifact directory under [artifacts](/home/deepblue/AV_Drone/artifacts)

## 6. 현재 기준 검증 상태

대표 예시 artifact:

- [baseline_summary_example.json](/home/deepblue/AV_Drone/docs/examples/baseline_summary_example.json)

확인된 값:

- `mission_phase = HOVER_AT_GOAL`
- `goal_reached = true`
- `pose_count = 49171`
- `scan_count = 8231`
- `closest_obstacle_m = 0.2117559313774109`

즉 현재 아키텍처는 “single-UAV baseline이 살아 있고, 측정 가능한 상태”까지는 도달했다.

## 7. 레거시 코드의 위치

관련 파일:

- [src/mppi/launch/mppi.launch.py](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)
- [src/mppi/mppi/mppi_node.py](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)

의미:

- 기존 단일 드론 MPPI 데모
- 현재 autonomy manager 상태 머신 설계의 참고 소스
- 이후 논문용 baseline 비교 대상으로 유지

중요:

- `mppi`는 현재 주 실행 경로가 아니다.
- 현재 문서와 팀 인수인계는 `single_drone_autonomy.launch.py` 기준으로 본다.

## 8. 현재 논문 방향에서 아직 없는 것

- multi-UAV namespace / spawn 구조
- failure injection
- landing bubble / descent corridor hazard model
- orphan task extraction / reallocation
- 반복 실험 통계 검증

즉, 현재 아키텍처는 연구 주제의 “시작 플랫폼”이고, 논문 본체는 아직 위에 쌓아야 한다.
