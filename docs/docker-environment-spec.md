# Docker Environment Specification

이 문서는 현재 저장소의 Docker 실행 기준을 간단하고 현재 상태에 맞게 정리한 명세다.  
기준은 `single-UAV baseline`이며, 이후 multi-UAV 연구 확장을 고려해 두 컨테이너 구조를 유지한다.

## 1. 목표

현재 Docker 환경의 목적은 아래다.

- 호스트에 PX4, Gazebo, ROS 2를 직접 뒤섞어 설치하지 않는다.
- `sim`과 `ros` 역할을 분리한다.
- 팀원이 같은 절차로 baseline을 재현할 수 있게 한다.
- 이후 multi-UAV / fault-aware 실험으로 확장 가능한 기반을 유지한다.

## 2. 서비스 구성

### `sim`

역할:

- PX4 SITL
- Gazebo Classic
- `iris_rplidar` 센서 모델
- `obstacle_demo.world`

관련 파일:

- [docker/sim/Dockerfile](/home/deepblue/AV_Drone/docker/sim/Dockerfile)
- [docker/sim/entrypoint.sh](/home/deepblue/AV_Drone/docker/sim/entrypoint.sh)
- [sim_assets/models/rplidar/model.sdf](/home/deepblue/AV_Drone/sim_assets/models/rplidar/model.sdf)
- [sim_assets/worlds/obstacle_demo.world](/home/deepblue/AV_Drone/sim_assets/worlds/obstacle_demo.world)

### `ros`

역할:

- ROS 2 Humble runtime
- MAVROS
- autonomy pipeline 실행
- `colcon build`

관련 파일:

- [docker/ros/Dockerfile](/home/deepblue/AV_Drone/docker/ros/Dockerfile)
- [docker/ros/entrypoint.sh](/home/deepblue/AV_Drone/docker/ros/entrypoint.sh)
- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)

## 3. 기준 버전

- Host OS: Ubuntu 22.04
- Docker Engine / Compose Plugin
- Container base: Ubuntu 22.04
- ROS 2: Humble
- PX4: v1.15.4 기반
- Gazebo Classic: 11
- Python: 3.10

## 4. 네트워크 / IPC 정책

현재 기준:

- `network_mode: host`
- `ipc: host`
- `/dev/shm` 공유

이유:

- PX4 SITL과 MAVROS의 UDP 연결 단순화
- ROS 2 DDS 전달 안정화
- 컨테이너 간 topic discovery와 payload 전달 단순화

## 5. GUI 정책

Gazebo GUI가 필요하면 호스트에서 X11 권한이 필요하다.

대표 명령:

```bash
echo $DISPLAY
xhost +local:docker
```

메모:

- GUI가 안 떠도 headless baseline 자체는 가능하다.
- 첫 `sim` 기동 시에는 내부 PX4/Gazebo build 때문에 GUI가 늦게 뜰 수 있다.

## 6. 볼륨 정책

프로젝트 루트는 양쪽 컨테이너에 마운트된다.

- Host: `/home/deepblue/AV_Drone`
- Container: `/workspace/AV_Drone`

의미:

- 코드 수정 즉시 컨테이너에서 반영 가능
- `sim`이 custom world / custom model을 읽을 수 있음
- `ros`가 artifact를 host와 공유 가능

## 7. 실행 기준

호스트에서:

```bash
cd /home/deepblue/AV_Drone
docker compose build sim
docker compose build ros
docker compose up -d sim ros
```

ROS 컨테이너 안에서:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics
source install/setup.bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```

## 8. 현재 baseline 검증 기준

아래가 충족되면 팀 인수인계용 baseline으로 본다.

- `/drone1/scan` 실수신
- `/mavros/local_position/pose` 실수신
- `mission_phase = HOVER_AT_GOAL`
- `goal_reached = true`
- 최신 artifact에 `pose_count > 0`
- 최신 artifact에 `scan_count > 0`
- 최신 artifact에 `closest_obstacle_m` finite

대표 예시 artifact:

- [baseline_summary_example.json](/home/deepblue/AV_Drone/docs/examples/baseline_summary_example.json)

## 9. 운영상 주의

- `sim` 컨테이너가 `Up`이어도 내부 PX4/Gazebo build가 더 진행될 수 있다.
- smoke test는 이 readiness를 자동으로 기다리도록 구현돼 있다.
- 현재 Docker 환경은 single-UAV baseline까지 검증 완료 상태다.
- multi-UAV, failure injection, task reallocation은 아직 상위 연구 단계다.

## 10. 관련 문서

- 개요: [README.md](/home/deepblue/AV_Drone/README.md)
- 구조: [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
- 명령어: [command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
- 문제 기록: [problem.md](/home/deepblue/AV_Drone/docs/problem.md)
- 업데이트 로그: [updates/README.md](/home/deepblue/AV_Drone/docs/updates/README.md)
