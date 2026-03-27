# AV_Drone

`AV_Drone`는 현재 `PX4 SITL + Gazebo Classic 11 + ROS 2 Humble + MAVROS` 기준으로 정리된 단일 드론 baseline 저장소입니다.
지금 active path의 핵심은 `LiDAR sensing -> reactive obstacle avoidance -> artifact/logging -> 이후 multi-UAV / MPPI 연구 확장`으로 이어지는 재현 가능한 시작점을 유지하는 것입니다.

## 1. 현재 기준 스택

- `sim` 컨테이너: PX4 SITL + Gazebo Classic 11
- `ros` 컨테이너: ROS 2 Humble + MAVROS + autonomy nodes + `ros_states`
- 센서 입력: Gazebo Classic LiDAR가 `/drone1/scan`으로 직접 publish
- 주 실행 경로: `single_drone_autonomy.launch.py`
- 상태 대시보드: `ros_states`

중요:

- 현재 active runtime은 `Gazebo Classic only`입니다.
- `sim`과 `ros`는 `host network + host ipc + UDP-only Fast DDS`로 맞춰져 있습니다.
- 이 설정이 맞아야 `/drone1/scan`이 `sim` 컨테이너에서 `ros` 컨테이너로 정상 전달됩니다.

## 2. 기본 실행 순서

### 2-1. Gazebo Classic + PX4 시작

이제 `sim` 컨테이너는 기본적으로 headless로 먼저 올라오고, 검증된 helper 경로로만 GUI를 붙입니다.
즉 순서는 `sim ready 확인 -> GUI helper 실행`입니다.

```bash
cd /home/deepblue/AV_Drone
xhost +SI:localuser:root
xhost +local:docker
docker compose up -d --force-recreate sim ros
docker compose logs -f sim
```

아래 문구가 보이면 시뮬레이터 준비 완료입니다.

```text
Startup script returned successfully
```

그 다음에만 Gazebo GUI를 붙입니다.

```bash
xhost +SI:localuser:root
xhost +local:docker
./scripts/run_host_gz_gui.sh
```

### 2-2. autonomy launch 실행

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics ros_states --symlink-install
source install/setup.bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```

### 2-3. `ros_states` 실행

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select ros_states --symlink-install
source install/setup.bash
ros2 launch ros_states ros_states.launch.py \
  drone_name:=drone1 \
  mavros_namespace:=/mavros \
  artifacts_root:=/workspace/AV_Drone/artifacts \
  port:=5050 \
  open_browser:=false
```

`ros_states` 기본 URL:

```text
http://localhost:5050
```

같은 네트워크 다른 장치에서 볼 때는 호스트 IP 기준 `http://<host-ip>:5050`를 사용하면 됩니다.

### 2-4. `ros_states` 디버깅 기록 저장

이제 `ros_states`에서 보고 있는 디버깅 상태를 그대로 저장할 수 있습니다.

중요:

- 이건 브라우저 픽셀 화면을 캡처하는 기능이 아닙니다.
- `ros_states` 백엔드가 브라우저에 보여주던 상태를 JSON으로 저장하는 기능입니다.
- 따라서 아무 버튼도 누르지 않으면 계속 자동 저장되지는 않습니다.

버튼 의미는 아래와 같습니다.

- `Save Snapshot` : 현재 화면 기준 상태를 한 번 JSON으로 저장하고, 바로 HTML report도 갱신
- `Start Recording` : 일정 간격으로 디버깅 타임라인을 계속 기록 시작
- `Stop Recording` : 기록 세션 종료 후 마지막 스냅샷 저장
- `Generate Report` : 저장된 JSON/timeline을 읽어서 사람이 보기 쉬운 요약 + 시각화 + 그래프로 다시 생성
- `Open Report` : 가장 최근 생성된 report를 브라우저 새 탭에서 열기

저장 위치는 기본적으로 아래입니다.

```text
/workspace/AV_Drone/artifacts/_ros_states_debug/
```

세션 폴더 안에는 아래 파일이 생깁니다.

- `session_manifest.json` : 이 세션이 언제 시작/종료됐는지, interval이 몇 초인지
- `timeline.jsonl` : 주기적으로 쌓이는 디버깅 타임라인
- `snapshots/` : 수동 저장 또는 시작/종료 시점 전체 스냅샷
- `report.html` : 저장된 세션을 사람이 읽기 쉬운 보고서 형태로 변환한 결과
- `report_summary.json` : report용 핵심 요약값

report를 직접 여는 URL은 아래입니다.

```text
http://localhost:5050/debug/report/current
```

한 번 코드 변경 후에는 `ros_states`만 다시 빌드하고 재실행하면 됩니다.

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select ros_states --symlink-install
source install/setup.bash
ros2 launch ros_states ros_states.launch.py \
  drone_name:=drone1 \
  mavros_namespace:=/mavros \
  artifacts_root:=/workspace/AV_Drone/artifacts \
  port:=5050 \
  open_browser:=false
```

## 3. Gazebo GUI

시뮬레이터 서버는 `docker compose up -d --force-recreate sim ros` 시점에 headless로 올라갑니다.
보이는 창은 `sim` 준비 완료 뒤에 helper로만 붙이는 것이 현재 가장 안정적인 경로입니다.

```bash
./scripts/run_host_gz_gui.sh
```

중요:

- 검은 자동 Gazebo 창이나 완전히 빈 회색 Gazebo 창이 보이면, 먼저 그 창을 닫고 helper로 다시 띄우는 쪽이 맞습니다.
- 이번 blank-window 원인은 `sim` 컨테이너가 이미지 안의 오래된 `/opt/PX4-Autopilot/docker/sim/entrypoint.sh`를 계속 써서, 최신 Gazebo fix가 반영되지 않던 것이었습니다. 지금은 `docker-compose.yml` 기준으로 워크스페이스의 `/workspace/AV_Drone/docker/sim/entrypoint.sh`를 직접 사용하도록 맞췄습니다.
- helper로 붙은 정상 client log에는 `Connected to gazebo master @ http://127.0.0.1:11345`와 `Publicized address: 127.0.0.1`가 보입니다.
- 정상 server log에는 더 이상 `Can't open display`, `Rendering will be disabled`, `Unable to create CameraSensor`가 나오지 않아야 합니다.

창이 안 뜨면 먼저 X11 권한을 다시 열어주세요.

```bash
xhost +SI:localuser:root
xhost +local:docker
```

## 4. 지금 obstacle demo 기준 값

현재 demo는 아래 기준으로 맞춰져 있습니다.

- start pose: `(0.0, 0.0, 0.0)`
- goal pose: `(31.0, 0.0, 3.0)`
- `goal_tol_xy`: `0.35 m`
- obstacle world: `obstacle_demo`
- obstacle layout: side-wall corridor + thick poles
- planner: `local_planner_lidar_reactive`

즉 현재 코스는 양쪽 벽으로 폭을 제한한 corridor 안에 두꺼운 기둥을 번갈아 배치해, 드론이 옆으로 크게 도망가지 않고 장애물 사이를 통과하며 더 오래 회피하도록 설계되어 있습니다.

## 5. 멈췄을 때 해석하는 법

### 5-1. 정상 종료

`ros_states`에서 아래처럼 보이면 센싱 실패가 아니라 미션 완료입니다.

- `Mission Phase = HOVER_AT_GOAL`
- `Goal reached = true`

이 경우 planner/safety가 `0,0,0`을 내도 정상입니다. 이미 목표에 도달해서 hover 중인 상태입니다.

### 5-2. 센싱/통신 문제

아래면 런타임 문제를 먼저 의심해야 합니다.

- `LiDAR Scan = stale`
- `Nearest Obstacle = no metric yet`
- `/drone1/scan` 샘플이 안 옴
- `/mavros/state`에서 `connected: false`

빠른 확인:

```bash
docker compose exec ros bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /drone1/scan --once'
```

```bash
docker compose exec ros bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /drone1/perception/nearest_obstacle_distance --once'
```

```bash
docker compose exec ros bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /mavros/state --once'
```

## 6. 다시 처음부터 실행할 때

```bash
cd /home/deepblue/AV_Drone
./stop.sh
xhost +SI:localuser:root
xhost +local:docker
docker compose up -d --force-recreate sim ros
docker compose logs -f sim
```

이후 `Startup script returned successfully`가 뜨면 autonomy와 `ros_states`를 다시 띄우면 됩니다.

## 7. `source`, `colcon build`, `docker compose build` 의미

`source /opt/ros/humble/setup.bash`

- 현재 셸에 ROS 2 환경변수를 올립니다.
- 새 터미널을 열면 다시 해줘야 합니다.

`source install/setup.bash`

- 이 저장소에서 빌드한 패키지 경로를 현재 셸에 올립니다.
- `ros2 launch`가 내 패키지를 찾을 수 있게 해줍니다.

`colcon build`

- ROS 워크스페이스 빌드입니다.
- 코드가 안 바뀌었으면 매번 할 필요는 없습니다.
- 패키지 소스, launch, config가 바뀌면 다시 빌드하는 것이 안전합니다.

`docker compose build`

- 컨테이너 이미지 자체를 다시 만드는 단계입니다.
- Dockerfile이나 apt 의존성이 바뀌었을 때 주로 다시 합니다.

## 8. 문서

- 구조 설명: [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
- 명령어 치트시트: [command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
- Docker 명세: [docker-environment-spec.md](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)
- HTML 대시보드: [project_command_center.html](/home/deepblue/AV_Drone/docs/project_command_center.html)
- 실험 기록 규칙: [experiment_recording_policy.md](/home/deepblue/AV_Drone/docs/experiment_recording_policy.md)
- 변경 로그: [docs/change/README.md](/home/deepblue/AV_Drone/docs/change/README.md)
- 장애 보고서: [docs/error/index.html](/home/deepblue/AV_Drone/docs/error/index.html)
