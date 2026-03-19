# AV_Drone

`AV_Drone`는 `PX4 SITL + Gazebo Classic + ROS 2 Humble + MAVROS` 기반의 드론 시뮬레이션 저장소다.  
현재 저장소의 기준 목표는 단순 데모가 아니라, 이후 `failure-aware mission continuation` 연구로 확장 가능한 `single-UAV baseline`을 안정적으로 제공하는 것이다.

현재까지 검증된 baseline은 아래다.

- Docker 기반 `sim` / `ros` 실행 절차 확정
- `iris_rplidar` 모델 기반 단일 드론 이륙 및 목표점 도달
- `/drone1/scan` 실제 `LaserScan` 수신
- obstacle world에서 `closest_obstacle_m` finite 기록
- `goal_reached` 안정화
- artifact 자동 저장
- 팀원용 smoke test 통과

대표 예시 artifact:

- [baseline_summary_example.json](/home/deepblue/AV_Drone/docs/examples/baseline_summary_example.json)

## 연구 목표

이 저장소가 최종적으로 향하는 주제는 다음이다.

- 다중 드론 환경에서
- 일부 UAV의 고장 또는 임무 이탈이 발생했을 때
- 남은 UAV가 위험영역과 잔여 작업을 고려해
- 임무를 안전하게 계속 수행하는 `failure-aware mission continuation`

즉, 지금 저장소의 역할은 다음 두 단계로 나뉜다.

1. 현재 단계: `single-UAV baseline` 구축
2. 다음 단계: `multi-UAV fault-aware research platform`으로 확장

## 현재 상태

현재는 아래 수준까지 완료됐다.

- `sim` 컨테이너에서 `PX4 SITL + Gazebo Classic` 실행
- `ros` 컨테이너에서 `MAVROS + autonomy pipeline` 실행
- `/drone1/scan`, `/mavros/local_position/pose`, `/drone1/mission/phase` 확인
- `mission_phase = HOVER_AT_GOAL`
- `goal_reached = true`
- `closest_obstacle_m` 기록
- smoke test 자동 통과

아직 안 된 것은 아래다.

- 2대/4대 멀티드론 namespace 구조
- failure injection
- landing bubble / descent corridor hazard model
- orphan task extraction / reallocation
- 반복 실험 기반 통계 검증

## 현재 활성 아키텍처

```text
Host Ubuntu 22.04
└─ Docker Compose
   ├─ sim
   │  ├─ PX4 SITL
   │  ├─ Gazebo Classic
   │  └─ iris_rplidar + obstacle_demo.world
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

데이터 흐름은 다음과 같다.

```text
Gazebo/PX4
  -> MAVROS
  -> /mavros/local_position/pose
  -> drone_control

Gazebo LiDAR
  -> /drone1/scan
  -> drone_perception
  -> /drone1/perception/nearest_obstacle_distance
  -> drone_planning
  -> /drone1/autonomy/cmd_vel
  -> drone_safety
  -> /drone1/safety/cmd_vel
  -> drone_control
  -> /mavros/setpoint_velocity/cmd_vel

All runtime topics/events
  -> drone_metrics
  -> artifacts/<timestamp>_drone1/
```

## 현재 기준 저장소 구조

```text
AV_Drone/
├─ docker/
│  ├─ ros/
│  │  ├─ Dockerfile
│  │  └─ entrypoint.sh
│  └─ sim/
│     ├─ Dockerfile
│     └─ entrypoint.sh
├─ sim_assets/
│  ├─ models/rplidar/model.sdf
│  └─ worlds/obstacle_demo.world
├─ scripts/
│  └─ smoke_test_single_drone.sh
├─ src/
│  ├─ drone_bringup/
│  ├─ drone_control/
│  ├─ drone_perception/
│  ├─ drone_planning/
│  ├─ drone_safety/
│  ├─ drone_metrics/
│  └─ mppi/                 # legacy baseline reference
├─ artifacts/
├─ docs/
└─ docker-compose.yml
```

핵심 경로:

- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)
- [docker/sim/Dockerfile](/home/deepblue/AV_Drone/docker/sim/Dockerfile)
- [docker/sim/entrypoint.sh](/home/deepblue/AV_Drone/docker/sim/entrypoint.sh)
- [sim_assets/worlds/obstacle_demo.world](/home/deepblue/AV_Drone/sim_assets/worlds/obstacle_demo.world)
- [src/drone_bringup/launch/single_drone_autonomy.launch.py](/home/deepblue/AV_Drone/src/drone_bringup/launch/single_drone_autonomy.launch.py)
- [src/drone_bringup/config/drone1_autonomy.yaml](/home/deepblue/AV_Drone/src/drone_bringup/config/drone1_autonomy.yaml)
- [src/drone_control/drone_control/autonomy_manager_node.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/autonomy_manager_node.py)
- [src/drone_perception/drone_perception/lidar_obstacle_node.py](/home/deepblue/AV_Drone/src/drone_perception/drone_perception/lidar_obstacle_node.py)
- [src/drone_planning/drone_planning/local_planner_node.py](/home/deepblue/AV_Drone/src/drone_planning/drone_planning/local_planner_node.py)
- [src/drone_safety/drone_safety/safety_monitor_node.py](/home/deepblue/AV_Drone/src/drone_safety/drone_safety/safety_monitor_node.py)
- [src/drone_metrics/drone_metrics/metrics_logger_node.py](/home/deepblue/AV_Drone/src/drone_metrics/drone_metrics/metrics_logger_node.py)

## 패키지 역할

- [drone_bringup](/home/deepblue/AV_Drone/src/drone_bringup): launch와 YAML 설정
- [drone_control](/home/deepblue/AV_Drone/src/drone_control): mission phase와 MAVROS 인터페이스
- [drone_perception](/home/deepblue/AV_Drone/src/drone_perception): `LaserScan`에서 nearest obstacle 추출
- [drone_planning](/home/deepblue/AV_Drone/src/drone_planning): goal 방향 local velocity 생성. 현재 baseline은 `회피`가 아니라 `직진 + obstacle threshold 이하면 정지`에 가깝다.
- [drone_safety](/home/deepblue/AV_Drone/src/drone_safety): timeout / obstacle emergency stop 처리
- [drone_metrics](/home/deepblue/AV_Drone/src/drone_metrics): `summary.json`, `metrics.csv`, `events.log` 저장
- [mppi](/home/deepblue/AV_Drone/src/mppi): 이전 단일 드론 MPPI baseline. 현재는 연구용 비교/참고 경로로 유지

## 팀 인수인계 실행 절차

### 1. 호스트 준비

프로젝트 루트:

```bash
cd /home/deepblue/AV_Drone
```

Gazebo GUI를 보려면:

```bash
echo $DISPLAY
xhost +local:docker
```

GUI가 안 떠도 headless baseline 검증은 가능하다.

장애물 world를 기본으로 쓰는 현재 설정은 `PX4_SITL_WORLD=obstacle_demo` 기준이다.
Gazebo Classic에서는 `PX4_GZ_WORLD`가 아니라 `PX4_SITL_WORLD`가 실제 world 선택에 사용된다.
기본 설정은 LiDAR 데이터는 유지하되 Gazebo 안의 파란 scan fan 시각화는 끈 상태다.
`sim` 컨테이너에는 `/dev/dri`를 전달하므로 AMD/Intel 내장그래픽도 하드웨어 가속에 활용할 수 있다.

### 2. 이미지 빌드

```bash
docker compose build sim
docker compose build ros
```

### 3. 컨테이너 실행

```bash
docker compose up -d sim ros
docker compose ps
```

### 4. ROS 워크스페이스 빌드와 launch

```bash
docker compose exec ros bash
```

ROS 컨테이너 안에서:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics
source install/setup.bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```

### 5. smoke test

호스트에서:

```bash
cd /home/deepblue/AV_Drone
./scripts/smoke_test_single_drone.sh
```

이 스크립트는 아래를 자동으로 기다리고 확인한다.

- `sim` runtime readiness
- `/autonomy_manager`, `/metrics_logger` 실행 여부
- `/drone1/scan` 수신
- `/mavros/local_position/pose` 수신
- `/drone1/mission/phase = HOVER_AT_GOAL`
- `/drone1/mission/goal_reached = true`
- 최신 artifact의 `pose_count`, `scan_count`, `closest_obstacle_m`

## 직접 확인 명령

ROS 컨테이너 안에서:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
source install/setup.bash
```

LiDAR:

```bash
ros2 topic echo /drone1/scan --once
ros2 topic hz /drone1/scan
```

mission / pose:

```bash
ros2 topic echo /drone1/mission/phase --once
ros2 topic echo /drone1/mission/goal_reached --once
ros2 topic echo /mavros/local_position/pose --once
```

## Artifact 구조

실행 결과는 아래에 저장된다.

- [artifacts](/home/deepblue/AV_Drone/artifacts)

예:

```text
artifacts/2026-03-19_16-16-24_drone1/
├─ metadata.json
├─ metrics.csv
├─ summary.json
└─ events.log
```

메모:

- `artifacts/`는 runtime 출력이므로 `.gitignore`에 포함했다.
- 공유용 예시는 [baseline_summary_example.json](/home/deepblue/AV_Drone/docs/examples/baseline_summary_example.json)에 따로 남긴다.

## 실험 추적 자동화

자동 실험 장부는 아래에 누적된다.

- [experiments/README.md](/home/deepblue/AV_Drone/experiments/README.md)
- `experiments/index.csv`
- `experiments/scenario_table.csv`
- `experiments/ledger.csv`

메모:

- 위 CSV/MD/SVG 출력은 실행 후 생성되는 generated output이므로 `.gitignore`에 포함했다.
- 저장소에는 [experiments/README.md](/home/deepblue/AV_Drone/experiments/README.md)만 문서로 유지한다.

smoke test를 실행하면 자동으로:

1. 최신 artifact를 검증한다.
2. `experiments/plots/<run_id>/` 아래에 SVG 그래프를 생성한다.
3. `experiments/index.csv`에 실행 결과를 누적한다.
4. `experiments/scenario_table.csv`를 시나리오별로 다시 집계한다.
5. `experiments/ledger.csv`에 `문제 -> 수정 -> 재실행 -> 결과`를 한 줄로 남긴다.

예:

```bash
./scripts/smoke_test_single_drone.sh \
  --scenario single_drone_obstacle_demo \
  --issue "world variable mismatch" \
  --fix "use PX4_SITL_WORLD for gazebo-classic" \
  --notes "revalidated obstacle world and LiDAR stream"
```

기존 artifact를 기준으로 장부를 다시 만들려면:

```bash
python3 scripts/update_experiment_registry.py --scan-artifacts artifacts
```

현재 baseline에서 중요하게 보는 값:

- `mission_phase`
- `goal_reached`
- `pose_count`
- `scan_count`
- `closest_obstacle_m`
- `pose_period_p99_s`
- `scan_period_p99_s`
- `safety_reason_counts`

## 현재 baseline에서 확인된 기준값

[baseline_summary_example.json](/home/deepblue/AV_Drone/docs/examples/baseline_summary_example.json) 기준:

- `mission_phase = HOVER_AT_GOAL`
- `goal_reached = true`
- `pose_count = 49171`
- `scan_count = 8231`
- `closest_obstacle_m = 0.2117559313774109`
- `pose_period_p99_s = 0.0380094051361084`
- `scan_period_p99_s = 0.10906672477722168`

## 현재 기준 필요 없는 경로 정리

현재 논문 목표와 baseline에 직접 쓰지 않는 단순 오프보드 예제 패키지 `offboard_control`는 저장소에서 제거했다.  
이제 active path는 `single_drone_autonomy.launch.py` 기준으로만 설명한다.

## 레거시 코드에 대한 입장

[mppi](/home/deepblue/AV_Drone/src/mppi)는 삭제하지 않았다.

이유:

- 기존 단일 드론 MPPI baseline으로 이미 검증된 경로다.
- 현재 `autonomy_manager` 상태 머신을 만들 때 중요한 참고 소스였다.
- 이후 논문에서 local planner 고도화나 baseline 비교 대상으로 재사용할 수 있다.

즉 `mppi`는 현재 주 실행 경로는 아니지만, 연구 개발용 레거시 baseline으로 유지한다.

## 다음 개발 우선순위

1. obstacle world를 더 연구용에 맞게 고도화
2. 2대/4대 namespace, spawn, MAVROS instance 구조 확장
3. failure injection 시나리오 구현
4. landing bubble / descent corridor hazard model 추가
5. orphan task extraction / reallocation
6. 반복 실험과 KPI 자동 리포트

## 문서 안내

- 발표/교수님 공유: [project_overview.html](/home/deepblue/AV_Drone/docs/project_overview.html)
- 코드/런타임 구조: [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
- Docker 명세: [docker-environment-spec.md](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)
- 실행 명령 모음: [command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
- 연구 확장 로드맵: [multi-drone-development-roadmap.md](/home/deepblue/AV_Drone/docs/multi-drone-development-roadmap.md)
- 현재 문제/제약: [problem.md](/home/deepblue/AV_Drone/docs/problem.md)
- 변경 이력: [change.md](/home/deepblue/AV_Drone/docs/change.md)
- 날짜별 업데이트: [updates/README.md](/home/deepblue/AV_Drone/docs/updates/README.md)
- 실험 추적 장부: [experiments/README.md](/home/deepblue/AV_Drone/experiments/README.md)
