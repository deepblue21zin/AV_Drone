# AV_Drone Command Reference

이 문서는 현재 저장소의 `active path` 기준 명령만 남긴 실행 치트시트다.  
기준은 `single_drone_autonomy.launch.py` 기반 단일 드론 baseline이다.

## 1. 호스트 기본 경로

```bash
cd /home/deepblue/AV_Drone
```

## 2. Docker 기본 명령

전체 상태:

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

컨테이너 진입:

```bash
docker compose exec ros bash
docker compose exec sim bash
```

## 3. Gazebo GUI 관련

호스트에서 GUI 허용:

```bash
echo $DISPLAY
xhost +local:docker
```

주의:

- GUI가 안 떠도 headless 실행 자체는 가능하다.
- `sim` 컨테이너는 `Up` 상태라도 내부 PX4/Gazebo build가 더 진행될 수 있다.

## 4. ROS 컨테이너 진입 후 기본 환경

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
source install/setup.bash
```

패키지 빌드:

```bash
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics
source install/setup.bash
```

전체 빌드:

```bash
colcon build
source install/setup.bash
```

## 5. 현재 주 실행 명령

단일 드론 baseline 실행:

```bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```

legacy MPPI baseline:

```bash
ros2 launch mppi mppi.launch.py
```

현재 팀 인수인계와 baseline 검증은 `drone_bringup` 경로 기준으로 본다.

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

## 7. smoke test

호스트에서:

```bash
./scripts/smoke_test_single_drone.sh
```

문제/수정/메모까지 같이 기록:

```bash
./scripts/smoke_test_single_drone.sh \
  --scenario single_drone_obstacle_demo \
  --issue "planner did not react to obstacle" \
  --fix "updated world loading and planner logic" \
  --notes "baseline revalidated"
```

이 스크립트는 아래를 자동 검증한다.

- `sim`, `ros` 컨테이너 실행
- `sim` runtime readiness
- `/autonomy_manager`, `/metrics_logger` 실행
- `/drone1/scan` 수신
- `/mavros/local_position/pose` 수신
- `HOVER_AT_GOAL`
- `goal_reached=true`
- 최신 artifact 요약 필드
- `experiments/index.csv` 누적
- `experiments/scenario_table.csv` 집계
- `experiments/ledger.csv` 기록
- `experiments/plots/<run_id>/` 그래프 생성

메모:

- 위 `experiments/` 산출물은 실행 후 생성되는 generated output이며 `.gitignore`에 포함했다.

## 8. artifact 확인

최신 artifact 찾기:

```bash
ls -1dt artifacts/*_drone1 | head -n1
```

summary 확인:

```bash
LATEST="$(ls -1dt artifacts/*_drone1 | head -n1)"
cat "${LATEST}/summary.json"
```

찾아볼 핵심 값:

- `mission_phase`
- `goal_reached`
- `pose_count`
- `scan_count`
- `closest_obstacle_m`

문서용 고정 예시:

```bash
cat docs/examples/baseline_summary_example.json
```

실험 장부 재생성:

```bash
python3 scripts/update_experiment_registry.py --scan-artifacts artifacts
```

최신 artifact 그래프 생성:

```bash
LATEST="$(ls -1dt artifacts/*_drone1 | head -n1)"
python3 scripts/generate_artifact_plots.py --artifact "${LATEST}"
```

## 9. sim 내부 센서 확인

`sim` 컨테이너 안에서:

```bash
gz topic -l | grep -E 'scan|laser|lidar'
```

현재 기대하는 Gazebo topic 예:

- `/gazebo/default/iris_rplidar/rplidar/link/laser/scan`

## 10. 자주 보는 정상 로그

- `Startup script returned successfully`
- `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`
- `PHASE => OFFBOARD_ARM`
- `PHASE => TAKEOFF`
- `PHASE => HOVER_AFTER_TAKEOFF`
- `PHASE => FOLLOW_PLAN`
- `PHASE => HOVER_AT_GOAL`

## 11. 문서

- 개요/온보딩: [README.md](/home/deepblue/AV_Drone/README.md)
- 구조: [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
- Docker 명세: [docker-environment-spec.md](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)
- 로드맵: [multi-drone-development-roadmap.md](/home/deepblue/AV_Drone/docs/multi-drone-development-roadmap.md)
- 문제 기록: [problem.md](/home/deepblue/AV_Drone/docs/problem.md)
