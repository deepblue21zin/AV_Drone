# AV_Drone

`AV_Drone`는 현재 `PX4 SITL + Gazebo Sim Harmonic + ROS 2 Humble + MAVROS` 기반의 단일 드론 baseline 저장소다.
지금 목표는 단순 비행 데모가 아니라, 이후 `failure-aware mission continuation` 연구로 확장할 수 있는 재현 가능한 기준 환경을 유지하는 것이다.

이 README는 아래 내용을 한 번에 이해할 수 있게 정리했다.

- 현재 환경이 어떻게 구성되어 있는가
- 왜 그런 구조로 나눴는가
- 지금 코드가 실제로 어떻게 움직이는가
- 어떻게 실행하고 검증하는가
- 실행 결과가 어디에 남는가

## 1. 현재 상태

현재 확인된 것:

- `sim` / `ros` 분리 Docker 환경
- Gazebo Sim Harmonic 기반 단일 드론 baseline
- `/drone1/scan` `LaserScan` 수신
- `/mavros/local_position/pose` 수신
- OFFBOARD / arm / takeoff / goal follow 동작
- runtime artifact 자동 저장
- smoke test 자동 검증

아직 안 된 것:

- 2대 이상 멀티드론 구조
- failure injection
- landing hazard bubble / corridor
- orphan task reallocation
- 논문 본체인 multi-UAV continuation logic

즉 지금 상태는:

- `단일 드론 baseline`: 됨
- `LiDAR sensing`: 됨
- `실험 기록/재현`: 됨
- `failure-aware multi-UAV 연구 기능`: 아직 개발 전

대표 개요 문서:

- [project_overview.html](/home/deepblue/AV_Drone/docs/project_overview.html)
- [current_state_guide.html](/home/deepblue/AV_Drone/docs/current_state_guide.html)
- [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)

## 2. 현재 환경 구성

현재 런타임은 3개로 나뉜다.

1. `sim` 컨테이너
- PX4 SITL
- Gazebo Sim Harmonic 서버
- obstacle world 로드

2. `ros` 컨테이너
- ROS 2 Humble
- MAVROS
- perception / planning / safety / control / metrics 노드

3. 호스트 GUI
- Gazebo GUI만 호스트에서 실행
- 현재는 GPU 대신 CPU software rendering 사용

핵심 파일:

- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)
- [docker/sim/Dockerfile](/home/deepblue/AV_Drone/docker/sim/Dockerfile)
- [docker/ros/Dockerfile](/home/deepblue/AV_Drone/docker/ros/Dockerfile)
- [docker/sim/entrypoint.sh](/home/deepblue/AV_Drone/docker/sim/entrypoint.sh)
- [scripts/run_host_gz_gui.sh](/home/deepblue/AV_Drone/scripts/run_host_gz_gui.sh)

## 3. 왜 이렇게 구성했나

### `sim`과 `ros`를 분리한 이유

- PX4 / Gazebo 문제와 ROS 2 노드 문제를 분리해서 볼 수 있다.
- 팀원이 디버깅할 때 `시뮬레이터가 죽었는지`, `ROS 쪽이 죽었는지`를 바로 구분할 수 있다.
- 연구 확장 시 `sim`은 그대로 두고 `ros` 쪽 알고리즘만 바꾸기 쉽다.

### Gazebo GUI를 호스트에서 띄우는 이유

- Docker 안 GUI는 AMD + Wayland 환경에서 회색 빈 창 문제가 반복됐다.
- 그래서 현재는 시뮬레이터 서버는 컨테이너 안에서 돌리고, GUI만 호스트에서 붙인다.
- 이 방식이 지금 환경에서는 가장 안정적이다.

### GPU를 안 쓰고 CPU software rendering을 쓰는 이유

- 현재 호스트의 AMD GPU 경로가 Gazebo GUI에서 불안정했다.
- 그래서 지금은 `llvmpipe / swrast`로 강제해서 우선 “확실히 보이게” 맞췄다.
- 성능은 떨어지지만, 현재 목표는 고성능보다 재현성과 안정성이다.

## 4. 전체 구조

```text
Host Ubuntu 22.04
├─ Gazebo Sim GUI (host)
│  └─ scripts/run_host_gz_gui.sh
└─ Docker Compose
   ├─ sim
   │  ├─ PX4 SITL
   │  ├─ Gazebo Sim Harmonic server
   │  ├─ world: obstacle_demo
   │  └─ model cache sync
   └─ ros
      ├─ ROS 2 Humble
      ├─ MAVROS
      ├─ drone_perception
      ├─ drone_planning
      ├─ drone_safety
      ├─ drone_control
      └─ drone_metrics
```

## 5. 코드가 실제로 움직이는 방식

데이터 흐름은 아래와 같다.

```text
Gazebo Sim / PX4
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

All runtime state
  -> drone_metrics
  -> artifacts/<timestamp>_drone1/
```

핵심 노드:

- [single_drone_autonomy.launch.py](/home/deepblue/AV_Drone/src/drone_bringup/launch/single_drone_autonomy.launch.py)
  - MAVROS + perception + planner + safety + control + metrics를 같이 띄운다.
- [lidar_obstacle_node.py](/home/deepblue/AV_Drone/src/drone_perception/drone_perception/lidar_obstacle_node.py)
  - `/drone1/scan`에서 가장 가까운 장애물 거리만 뽑는다.
- [local_planner_node.py](/home/deepblue/AV_Drone/src/drone_planning/drone_planning/local_planner_node.py)
  - 현재 위치와 목표점을 보고 직선 방향 속도를 만든다.
  - 아직 좌/우 우회 planner는 아니다.
- [safety_monitor_node.py](/home/deepblue/AV_Drone/src/drone_safety/drone_safety/safety_monitor_node.py)
  - pose timeout, scan timeout, planner command timeout, emergency stop 거리를 검사한다.
- [autonomy_manager_node.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/autonomy_manager_node.py)
  - OFFBOARD 요청, arm, takeoff, hover, follow plan 상태 머신을 수행한다.
- [metrics_logger_node.py](/home/deepblue/AV_Drone/src/drone_metrics/drone_metrics/metrics_logger_node.py)
  - artifact를 자동으로 저장한다.

## 6. 현재 기본 비행 동작

기본 파라미터는 여기 있다.

- [drone1_autonomy.yaml](/home/deepblue/AV_Drone/src/drone_bringup/config/drone1_autonomy.yaml)

현재 기본값:

- `takeoff_z: 3.0`
- `goal_x: 8.0`
- `goal_y: 0.0`
- `goal_z: 3.0`
- `cruise_speed: 1.0`
- `hover_sec_after_takeoff: 2.0`

즉 현재 기본 동작은 아래 순서다.

1. zero setpoint pre-stream
2. OFFBOARD 요청
3. arm 요청
4. 약 `3m`까지 상승
5. 짧게 hover
6. `x=8.0` 쪽으로 이동
7. 목표 근처에서 `HOVER_AT_GOAL`

### 드론이 Z축으로만 계속 올라가는 것처럼 보이는 이유

현재 데모는 `z=3.0`까지 상승한 뒤 `x=8.0` 방향으로 이동한다.
지금은 host GUI가 CPU software rendering이라 화면 반응이 부드럽지 않고, 카메라도 넓게 잡혀 있어서 상승이 더 눈에 띌 수 있다.

정상이라면:

- 약 `z=3m` 부근에서 상승이 멈춰야 한다.
- 그 다음 `FOLLOW_PLAN`으로 넘어가면서 수평 이동을 한다.

즉 **계속 무한히 올라가는 것은 정상 동작이 아니다.**
현재 기준으로는 상승 후 수평 이동이 이어져야 정상이며, 정말 계속 Z축으로만 올라간다면 `mission phase`와 `pose.position.z`를 먼저 확인해야 한다.

## 7. 환경 세팅 방식

두 가지 방식이 있다.

### 방법 A. 로컬에서 직접 build

장점:

- 가장 단순하다.
- 로컬 코드와 이미지가 항상 맞는다.

### 방법 B. Docker Hub 이미지 pull

장점:

- PX4 / Gazebo 이미지를 오래 빌드하지 않아도 된다.

주의:

- 이 저장소는 bind mount 구조라서 Docker 이미지 만으로는 부족하다.
- 반드시 git 저장소도 같이 clone 해야 한다.

## 8. 최초 1회 실행 방법

### 1. 저장소 받기

```bash
git clone https://github.com/deepblue21zin/AV_Drone.git
cd AV_Drone
```

### 2. GUI 권한 열기

```bash
echo $DISPLAY
xhost +local:docker
```

### 3. 이미지 build

```bash
docker compose build sim
docker compose build ros
```

## 9. 매번 실행 방법

터미널 1:

```bash
cd /home/deepblue/AV_Drone
docker compose up -d --force-recreate sim
docker compose logs -f sim
```

여기서 아래가 보이면 sim 서버는 정상이다.

- `px4 starting`
- `starting gazebo with world: ... obstacle_demo.sdf`
- `Startup script returned successfully`

그 다음 기존 Gazebo 창이 남아 있으면 닫고, 같은 터미널에서 host GUI를 띄운다.

```bash
pkill -f 'gz sim -g' || true
./scripts/run_host_gz_gui.sh
```

터미널 2:

```bash
cd /home/deepblue/AV_Drone
docker compose up -d ros
docker compose exec ros bash
```

프롬프트가 `root@...:/workspace/AV_Drone#` 로 바뀌면 그때부터 컨테이너 안이다. 그 다음 실행:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics
source install/setup.bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```

## 10. Docker Hub에서 pull하는 방법

예시 태그:

- `deepblue2121/av-drone-sim:2026-03-20`
- `deepblue2121/av-drone-ros:2026-03-20`

```bash
git clone https://github.com/deepblue21zin/AV_Drone.git
cd AV_Drone

docker pull deepblue2121/av-drone-sim:2026-03-20
docker pull deepblue2121/av-drone-ros:2026-03-20

docker tag deepblue2121/av-drone-sim:2026-03-20 av-drone-sim:latest
docker tag deepblue2121/av-drone-ros:2026-03-20 av-drone-ros:latest
```

그 다음 실행은 위 `매번 실행 방법`과 같다.

## 11. 실행 후 확인 명령

ROS 컨테이너 안에서:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
source install/setup.bash
```

센서 확인:

```bash
ros2 topic echo /drone1/scan --once
ros2 topic hz /drone1/scan
```

상태 확인:

```bash
ros2 topic echo /drone1/mission/phase --once
ros2 topic echo /drone1/mission/goal_reached --once
ros2 topic echo /mavros/local_position/pose --once
```

정상이라면 보통 아래를 보게 된다.

- `/drone1/scan` 수신
- `/mavros/local_position/pose` 수신
- `mission_phase = HOVER_AT_GOAL`
- `goal_reached = true`

## 12. 자동 검증

호스트에서:

```bash
./scripts/smoke_test_single_drone.sh --scenario single_drone_obstacle_demo --notes "manual validation"
```

이 스크립트는 아래를 확인한다.

- `sim` / `ros` 컨테이너 실행 여부
- sim startup 완료 여부
- autonomy 관련 노드 실행 여부
- `/drone1/scan` 샘플 수신
- `/mavros/local_position/pose` 샘플 수신
- `mission_phase = HOVER_AT_GOAL`
- `goal_reached = true`
- 최신 artifact 유효성

스크립트 위치:

- [smoke_test_single_drone.sh](/home/deepblue/AV_Drone/scripts/smoke_test_single_drone.sh)

## 13. 실행 결과 저장 위치

artifact는 아래에 저장된다.

- [artifacts](/home/deepblue/AV_Drone/artifacts)

예:

```text
artifacts/2026-03-20_09-17-49_drone1/
├─ metadata.json
├─ metrics.csv
├─ summary.json
└─ events.log
```

주요 파일:

- `metadata.json`
  - run id, git commit, scenario name, topic 정보
- `metrics.csv`
  - 시간별 상태 변화
- `summary.json`
  - 최종 요약 수치
- `events.log`
  - phase 전환, safety event

## 14. 실험 장부와 그래프

실행 후 후처리 결과는 아래에 모인다.

- [experiments](/home/deepblue/AV_Drone/experiments)

여기에는:

- `index.csv`
- `scenario_table.csv`
- `ledger.csv`
- `plots/<run_id>/`

가 저장된다.

즉 현재 저장소는 단순 실행 환경이 아니라, 실험 결과를 누적해서 논문/포트폴리오에 쓸 수 있게 정리하는 구조다.

## 15. 자주 헷갈리는 점

### `docker compose exec ros bash`가 실패했는데 뒤 명령을 계속 친 경우

이 경우 뒤 명령은 컨테이너 안이 아니라 호스트에서 실행된다.
그래서 아래 에러가 연달아 난다.

- `/workspace/AV_Drone` 없음
- `ros2: command not found`
- `service "ros" is not running`

항상 먼저 확인할 것:

```bash
docker compose up -d ros
docker compose exec ros bash
```

그리고 프롬프트가 `root@...` 로 바뀌는지 확인한 뒤에만 다음 명령을 친다.

### Gazebo GUI가 회색 빈 창일 때

현재 기본 경로는 host GUI + CPU software rendering이다.
즉 아래 스크립트를 써야 한다.

- [run_host_gz_gui.sh](/home/deepblue/AV_Drone/scripts/run_host_gz_gui.sh)

이 스크립트는 아래 환경을 강제한다.

- `QT_QPA_PLATFORM=xcb`
- `GDK_BACKEND=x11`
- `XDG_SESSION_TYPE=x11`
- `LIBGL_ALWAYS_SOFTWARE=1`
- `GALLIUM_DRIVER=llvmpipe`
- `MESA_LOADER_DRIVER_OVERRIDE=swrast`

## 16. 현재 핵심 파일 읽는 순서

처음 보는 사람이 읽기 좋은 순서는 아래다.

1. [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)
2. [docker/sim/entrypoint.sh](/home/deepblue/AV_Drone/docker/sim/entrypoint.sh)
3. [scripts/run_host_gz_gui.sh](/home/deepblue/AV_Drone/scripts/run_host_gz_gui.sh)
4. [single_drone_autonomy.launch.py](/home/deepblue/AV_Drone/src/drone_bringup/launch/single_drone_autonomy.launch.py)
5. [drone1_autonomy.yaml](/home/deepblue/AV_Drone/src/drone_bringup/config/drone1_autonomy.yaml)
6. [autonomy_manager_node.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/autonomy_manager_node.py)
7. [lidar_obstacle_node.py](/home/deepblue/AV_Drone/src/drone_perception/drone_perception/lidar_obstacle_node.py)
8. [local_planner_node.py](/home/deepblue/AV_Drone/src/drone_planning/drone_planning/local_planner_node.py)
9. [safety_monitor_node.py](/home/deepblue/AV_Drone/src/drone_safety/drone_safety/safety_monitor_node.py)
10. [metrics_logger_node.py](/home/deepblue/AV_Drone/src/drone_metrics/drone_metrics/metrics_logger_node.py)

## 17. 관련 문서

- [project_overview.html](/home/deepblue/AV_Drone/docs/project_overview.html)
- [current_state_guide.html](/home/deepblue/AV_Drone/docs/current_state_guide.html)
- [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
- [command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
- [problem.md](/home/deepblue/AV_Drone/docs/problem.md)
- [updates/README.md](/home/deepblue/AV_Drone/docs/updates/README.md)
