# Docker Environment Specification

이 문서는 현재 저장소의 Docker 실행 기준을 `Gazebo Classic active path`에 맞춰 정리한 명세다.

## 1. 목표

- PX4, Gazebo, ROS 2를 호스트에 뒤섞어 설치하지 않는다.
- `sim`과 `ros` 역할을 분리한다.
- 팀원이 같은 절차로 단일 드론 baseline을 재현할 수 있게 한다.
- 이후 multi-UAV / failure-aware 실험으로 확장 가능한 기반을 유지한다.

## 2. 서비스 구성

### `sim`

역할:

- PX4 SITL
- Gazebo Classic 11
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
- `ros_states` 실행

관련 파일:

- [docker/ros/Dockerfile](/home/deepblue/AV_Drone/docker/ros/Dockerfile)
- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)

## 3. 기준 버전

- Host OS: Ubuntu 22.04
- Container base: Ubuntu 22.04
- ROS 2: Humble
- PX4: v1.15.4
- Gazebo Classic: 11
- Python: 3.10

## 4. 네트워크 정책

현재 기준:

- `network_mode: host`

이유:

- PX4 SITL과 MAVROS의 UDP 연결 단순화
- ROS 2 discovery 안정화
- 팀 온보딩 시 네트워크 디버깅 비용 감소

## 5. GUI 정책

대표 명령:

```bash
echo $DISPLAY
xhost +SI:localuser:root
xhost +local:docker
```

메모:

- `DISPLAY`가 있으면 Gazebo Classic GUI는 기본적으로 자동 실행된다.
- `./start.sh`는 위 X11 권한까지 같이 처리하는 기본 실행 경로다.
- GUI가 안 떠도 headless baseline 자체는 가능하다.
- 창을 수동으로 다시 띄울 때만 [run_host_gz_gui.sh](/home/deepblue/AV_Drone/scripts/run_host_gz_gui.sh)를 쓴다.

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
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics ros_states --symlink-install
source install/setup.bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```

## 8. 현재 baseline 검증 기준

아래가 충족되면 현재 baseline이 살아 있다고 본다.

- `/drone1/scan` 실수신
- `/mavros/local_position/pose` 실수신
- `mission_phase = HOVER_AT_GOAL`
- `goal_reached = true`
- 최신 artifact에 `pose_count > 0`
- 최신 artifact에 `scan_count > 0`
- 최신 artifact에 `closest_obstacle_m` finite

## 9. 운영상 주의

- `sim` 컨테이너가 `Up`이어도 내부 PX4/Gazebo 준비가 더 진행될 수 있다.
- readiness는 `Startup script returned successfully`로 확인한다.
- 코드 변경과 Docker 이미지 변경은 다른 작업이다.
- 현재 active runtime은 Gazebo Classic이며, 예전 `gz sim` helper는 운영 필수 경로가 아니다.
