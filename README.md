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

## 이 README로 보는 순서

이 README는 현재 저장소를 처음 보는 사람도 아래 순서로 이해할 수 있게 작성했다.

1. 이 저장소가 무엇을 목표로 하는지 이해한다.
2. 현재 무엇이 구현됐고 무엇이 아직 안 됐는지 본다.
3. `sim` / `ros` Docker 구조와 활성 패키지 구성을 본다.
4. 로컬 빌드 방식 또는 Docker Hub pull 방식 중 하나로 환경을 맞춘다.
5. `single_drone_autonomy.launch.py`를 실행하고 smoke test로 baseline을 검증한다.
6. 논문/교수님 발표용 설명은 [project_overview.html](/home/deepblue/AV_Drone/docs/project_overview.html)에서 본다.

문서 역할은 다음처럼 나뉜다.

- 실행/배포/재현: 이 README
- 발표/논문 방향 설명: [project_overview.html](/home/deepblue/AV_Drone/docs/project_overview.html)
- 상세 코드/런타임 구조: [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
- 명령어 치트시트: [command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
- 현재 문제/제약: [problem.md](/home/deepblue/AV_Drone/docs/problem.md)

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

## Docker 이미지에 포함되는 것

현재 Docker 구조는 `sim` 이미지, `ros` 이미지, 그리고 실행 시 bind mount 되는 저장소 코드로 나뉜다.

- `sim` 이미지:
  - `Ubuntu 22.04`
  - `PX4-Autopilot`
  - `Gazebo Classic`
  - `ROS 2 runtime for gazebo_ros plugins`
  - PX4/Gazebo 빌드 의존성
- `ros` 이미지:
  - `Ubuntu 22.04`
  - `ROS 2 Humble Desktop`
  - `MAVROS`
  - `colcon`, `rosdep` 등 ROS 개발 도구
- 저장소에서 bind mount 되는 것:
  - `src/` 패키지
  - `sim_assets/` 아래 custom model/world
  - `docs/`, `scripts/`

중요:

- Docker 이미지는 현재 기준으로 `환경 세팅 묶음`이다.
- 하지만 이 저장소는 source bind mount 구조이므로, 이미지만 받아서는 부족하고 git repo도 같이 있어야 한다.
- 즉 팀 배포 기준은 `git repo + Docker 이미지` 조합이다.

## 배포/재현 방법 선택

현재 팀원이 환경을 맞추는 방법은 두 가지다.

### 방법 A. 저장소 기준 로컬 빌드

가장 단순하고 안전한 방식이다.

- 저장소 clone
- 로컬에서 `docker compose build`
- 실행

장점:

- 항상 현재 저장소 코드와 이미지 정의가 일치한다.
- Docker Hub 계정/권한 없이도 된다.

### 방법 B. Docker Hub 이미지 pull + 저장소 코드 사용

빌드 시간을 줄이고 싶을 때 쓰는 방식이다.

- 저장소 clone
- 미리 올려둔 `sim`, `ros` 이미지를 pull
- 현재 compose가 기대하는 로컬 이름으로 tag
- 실행

장점:

- 팀원이 PX4/Gazebo 이미지를 오래 빌드하지 않아도 된다.

주의:

- 이 프로젝트는 bind mount 구조라서, Docker Hub 이미지만으로는 실행되지 않는다.
- 반드시 git repo를 같이 받아야 한다.

## Docker Hub에서 이미지 받아서 실행하는 방법

아래 예시는 Docker Hub 사용자명이 `<dockerhub-user>`이고, 환경 태그가 `2026-03-20`인 경우다.

### 1. 저장소 clone

```bash
git clone <your-fork-or-repo-url>
cd AV_Drone
```

### 2. Docker Hub 로그인과 이미지 pull

```bash
docker login
docker pull <dockerhub-user>/av-drone-sim:2026-03-20
docker pull <dockerhub-user>/av-drone-ros:2026-03-20
```

### 3. 현재 compose가 기대하는 로컬 이름으로 tag

현재 [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)은 기본적으로 아래 로컬 이미지 이름을 사용한다.

- `av-drone-sim:latest`
- `av-drone-ros:latest`

그래서 pull한 이미지를 아래처럼 다시 tag 하면 compose 수정 없이 바로 쓸 수 있다.

```bash
docker tag <dockerhub-user>/av-drone-sim:2026-03-20 av-drone-sim:latest
docker tag <dockerhub-user>/av-drone-ros:2026-03-20 av-drone-ros:latest
```

### 4. GUI 권한 열기

```bash
echo $DISPLAY
xhost +local:docker
```

### 5. 컨테이너 실행

```bash
docker compose up -d sim ros
docker compose ps
```

### 6. ROS 워크스페이스 빌드와 실행

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics
source install/setup.bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```

### 7. smoke test

호스트에서:

```bash
./scripts/smoke_test_single_drone.sh
```

## 특정 git commit 기준으로 환경을 그대로 재현하는 방법

논문, 발표, 포트폴리오용으로는 `코드 상태`와 `이미지 환경`을 같이 고정하는 편이 맞다.

권장 조합:

- git commit hash: 코드 버전 고정
- Docker image tag: 환경 버전 고정

예:

- 코드: `git checkout <commit-hash>`
- 이미지: `2026-03-20`

### 방법 1. 해당 commit에서 로컬 빌드

가장 재현성이 높다.

```bash
git clone <your-fork-or-repo-url>
cd AV_Drone
git checkout <commit-hash>
docker compose build sim ros
docker compose up -d sim ros
```

이 방식은 해당 commit에 들어있는 Dockerfile과 compose 정의를 그대로 빌드하므로 가장 정확하다.

### 방법 2. 해당 commit + 고정 tag 이미지 사용

빌드 시간을 줄이고 싶을 때 쓴다.

```bash
git clone <your-fork-or-repo-url>
cd AV_Drone
git checkout <commit-hash>
docker pull <dockerhub-user>/av-drone-sim:2026-03-20
docker pull <dockerhub-user>/av-drone-ros:2026-03-20
docker tag <dockerhub-user>/av-drone-sim:2026-03-20 av-drone-sim:latest
docker tag <dockerhub-user>/av-drone-ros:2026-03-20 av-drone-ros:latest
docker compose up -d sim ros
```

메모:

- 이 방식은 `그 commit이 기대하는 이미지 환경`과 `pull한 tag`가 서로 맞아야 한다.
- Dockerfile이나 entrypoint가 바뀐 시점이라면, 가능하면 같은 시점 tag를 써야 한다.

## 팀에 공유할 때 권장하는 버전 표기

`latest`만 쓰지 말고, 아래 두 개를 같이 운영하는 게 좋다.

- `latest`: 가장 최근 팀 개발용
- `2026-03-20` 같은 고정 tag: 발표/논문/재현성용

예:

```bash
docker tag av-drone-sim:latest <dockerhub-user>/av-drone-sim:2026-03-20
docker tag av-drone-sim:latest <dockerhub-user>/av-drone-sim:latest
docker push <dockerhub-user>/av-drone-sim:2026-03-20
docker push <dockerhub-user>/av-drone-sim:latest
```

이미지 환경이 의미 있게 바뀌면 새 날짜 tag를 추가하면 된다.

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
