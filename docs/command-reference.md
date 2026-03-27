# AV_Drone Command Reference

이 문서는 현재 저장소의 `active path` 기준 명령만 남긴 치트시트다.  
기준은 `Gazebo Classic + single_drone_autonomy.launch.py` 기반 단일 드론 baseline이다.

## 1. 호스트 기본 경로

```bash
cd /home/deepblue/AV_Drone
```

## 2. Docker 기본 명령

상태 확인:

```bash
docker compose ps
docker compose logs -f sim
docker compose logs -f ros
```

이미지 빌드:

```bash
docker compose build sim
docker compose build ros
```

서비스 기동 / 종료:

```bash
docker compose up -d sim ros
docker compose down
```

headless 기동:

```bash
HEADLESS=1 DISPLAY= docker compose up -d sim ros
```

컨테이너 진입:

```bash
docker compose exec ros bash
docker compose exec sim bash
```

## 3. Gazebo Classic GUI

X11 허용:

```bash
echo $DISPLAY
xhost +SI:localuser:root
xhost +local:docker
```

기본 원칙:

- `DISPLAY`가 있으면 Gazebo Classic GUI는 기본적으로 자동 실행된다.
- 가장 안정적인 실행 경로는 `./start.sh`다.
- 창이 닫혔거나 자동 실행에 실패했을 때만 아래 helper를 실행한다.

```bash
./scripts/run_host_gz_gui.sh
```

## 4. ROS 컨테이너 진입 후 기본 환경

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
source install/setup.bash
```

패키지 빌드:

```bash
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics ros_states --symlink-install
source install/setup.bash
```

전체 빌드:

```bash
colcon build --symlink-install
source install/setup.bash
```

## 5. 현재 주 실행 명령

단일 드론 baseline 실행:

```bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```

`ros_states` 실행:

```bash
ros2 launch ros_states ros_states.launch.py \
  drone_name:=drone1 \
  mavros_namespace:=/mavros \
  artifacts_root:=/workspace/AV_Drone/artifacts \
  port:=5050 \
  open_browser:=false
```

legacy MPPI baseline:

```bash
ros2 launch mppi mppi.launch.py
```

## 6. 핵심 topic 확인

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

planner / perception:

```bash
ros2 topic echo /drone1/perception/nearest_obstacle_distance --once
ros2 topic echo /drone1/safety/cmd_vel --once
```

MAVROS:

```bash
ros2 topic echo /mavros/state --once
ros2 node list | grep mavros
```

## 7. Low-level sim 확인

`sim` 컨테이너 안에서 Gazebo transport topic 확인:

```bash
gz topic -l | grep -E 'scan|laser|lidar'
```

Classic baseline에서는 ROS 쪽 `/drone1/scan`이 더 중요한 운영 기준이다.

## 8. smoke test

호스트에서:

```bash
./scripts/smoke_test_single_drone.sh
```

메모 포함:

```bash
./scripts/smoke_test_single_drone.sh \
  --scenario single_drone_obstacle_demo \
  --issue "planner did not react to obstacle" \
  --fix "updated classic runtime and planner logic" \
  --notes "baseline revalidated"
```

## 9. artifact 확인

최신 artifact:

```bash
ls -1dt artifacts/*_drone1 | head -n1
```

summary 확인:

```bash
LATEST="$(ls -1dt artifacts/*_drone1 | head -n1)"
cat "${LATEST}/summary.json"
```

그래프 생성:

```bash
LATEST="$(ls -1dt artifacts/*_drone1 | head -n1)"
python3 scripts/generate_artifact_plots.py --artifact "${LATEST}"
```

## 10. 자주 보는 정상 로그

- `Startup script returned successfully`
- `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`
- `PHASE => OFFBOARD_ARM`
- `PHASE => TAKEOFF`
- `PHASE => HOVER_AFTER_TAKEOFF`
- `PHASE => FOLLOW_PLAN`
- `PHASE => HOVER_AT_GOAL`

## 11. 문서

- 개요: [README.md](/home/deepblue/AV_Drone/README.md)
- 구조: [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
- Docker 명세: [docker-environment-spec.md](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)
- HTML 대시보드: [project_command_center.html](/home/deepblue/AV_Drone/docs/project_command_center.html)
