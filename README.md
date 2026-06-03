# AV_Drone

`AV_Drone`는 현재 `PX4 SITL + Gazebo Classic 11 + ROS 2 Humble + MAVROS` 기준으로 정리된 단일 드론 baseline 저장소입니다.
지금 active path의 핵심은 `LiDAR sensing -> reactive obstacle avoidance -> artifact/logging -> 이후 multi-UAV / MPPI 연구 확장`으로 이어지는 재현 가능한 시작점을 유지하는 것입니다.
이제 저장소 안에는 팀 분리 개발을 위한 `회피`, `SLAM`, `MPPI`, `MPPI LiDAR` 관련 패키지와 개별 실행 프로필도 같이 들어 있습니다.

## 1. 현재 기준 스택

- `sim` 컨테이너: PX4 SITL + Gazebo Classic 11
- `ros` 컨테이너: ROS 2 Humble + MAVROS + autonomy nodes + `ros_states`
- 센서 입력: Gazebo Classic LiDAR가 `/drone1/scan`으로 직접 publish
- 주 실행 경로: `single_drone_autonomy.launch.py`
- 개발 실행 경로: `single_drone_avoidance_dev.launch.py`, `single_drone_slam_dev.launch.py`, `single_drone_mppi_dev.launch.py`
- 상태 대시보드: `ros_states`

중요:

- 현재 active runtime은 `Gazebo Classic only`입니다.
- `sim`과 `ros`는 `host network + host ipc + UDP-only Fast DDS`로 맞춰져 있습니다.
- 이 설정이 맞아야 `/drone1/scan`이 `sim` 컨테이너에서 `ros` 컨테이너로 정상 전달됩니다.

## 2. 분리 개발 프로필

현재 저장소는 팀원별로 아래 세 가지 프로필을 따로 실행하도록 세팅되어 있습니다.

- 회피 개발: `drone_bringup/single_drone_avoidance_dev.launch.py`
- SLAM 개발: `drone_bringup/single_drone_slam_dev.launch.py`
- MPPI 개발: `drone_bringup/single_drone_mppi_dev.launch.py`

중요:

- 같은 `drone1` 네임스페이스 기준으로는 한 번에 **하나의 프로필만** 띄우는 것을 원칙으로 합니다.
- 회피와 MPPI는 지금 단계에서 섞어 돌리지 않습니다.
- SLAM 프로필은 `drone_slam/slam_scaffold_node`를 기본으로 올립니다. 이 노드는 실제 SLAM 알고리즘이 아니라, 팀원이 나중에 실제 구현으로 교체할 수 있게 만든 인터페이스 예약용 scaffold입니다.
- 현재 MPPI는 아직 공통 planner selector에 연결된 형태가 아니라, 기존 팀원이 만든 독립 미션형 노드를 wrapper launch로 감싼 상태입니다.

### 2-1. 공통 빌드

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics ros_states drone_slam mppi mppi_lidar --symlink-install
source install/setup.bash
```

### 2-2. 회피 개발 프로필

```bash
ros2 launch drone_bringup single_drone_avoidance_dev.launch.py
```

이 프로필은 아래 경로를 사용합니다.

- 회피 planner 출력: `/drone1/planner/avoid/cmd_vel`
- safety 입력 planner command: `/drone1/planner/avoid/cmd_vel`
- safety 출력: `/drone1/safety/cmd_vel`
- mission active goal: `/drone1/mission/active_goal`
- mission home pose: `/drone1/mission/home_pose`
- controller는 기존처럼 safe command를 받아 비행합니다.

현재 이 프로필은 `Baseline A` 논문 실험용 왕복 baseline입니다.

- outbound: `MAPPING_TO_GOAL`
- goal hover: `HOVER_AT_GOAL`
- return: `RETURN_HOME_AVOID`
- home hover: `HOVER_AT_HOME`

즉 회피 알고리즘 담당자는 같은 회피 planner로 목표점까지 이동한 뒤, 이륙 후 저장한 `home_pose`를 `active_goal`로 다시 받아 출발 지점 근처까지 복귀하는지 확인하면 됩니다.

### 2-3. SLAM 개발 프로필

```bash
ros2 launch drone_bringup single_drone_slam_dev.launch.py
```

이 프로필은 MAVROS + LiDAR perception + SLAM scaffold만 띄웁니다.

예약된 SLAM 상태 토픽은 아래입니다.

- `/drone1/slam/status`
- `/drone1/slam/input_ready`
- `/drone1/slam/map_ready`
- `/drone1/slam/localization_ok`
- `/drone1/slam/coverage`

즉 SLAM 담당자는 scan/pose 입력 경로를 따로 확인하면서, 나중에 실제 SLAM 패키지 구현으로 `drone_slam` 패키지 안 scaffold를 교체해 나가면 됩니다.

### 2-4. MPPI 개발 프로필

```bash
ros2 launch drone_bringup single_drone_mppi_dev.launch.py
```

이 프로필은 현재 `mppi/mppi.launch.py`를 wrapper로 감싼 것입니다.

중요:

- 현재 MPPI 노드는 planner 단독 노드가 아니라 `takeoff -> mppi -> hover/land` 성격의 독립 미션 노드입니다.
- 따라서 지금 단계에서는 회피 baseline과 동시에 실행하지 않고, MPPI 성능 확인용으로만 따로 실행하는 것이 맞습니다.
- 나중에 통합 단계에서 planner core와 mission wrapper를 분리하는 것이 권장됩니다.

참고:

- LiDAR를 직접 읽는 별도 MPPI 구현은 이제 `src/mppi_lidar` 패키지로 정리되어 있습니다.
- 이 패키지는 자체 launch를 포함하므로 필요하면 아래처럼 독립 실행할 수 있습니다.

```bash
ros2 launch mppi_lidar mppi_lidar.launch.py
```

### 2-5. 향후 통합을 위한 토픽 계약

지금 바로 다 쓰지는 않지만, 나중 통합을 위해 아래 토픽 이름을 먼저 잡아 두었습니다.

- 회피 planner 출력: `/drone1/planner/avoid/cmd_vel`
- MPPI planner 출력 예약: `/drone1/planner/mppi/cmd_vel`
- 최종 selector 출력 예약: `/drone1/planner/selected/cmd_vel`
- safety 입력 planner command는 통합 시 최종적으로 selector 출력에 맞추는 방향입니다.

즉 개발 단계에서는 분리 실행, 통합 단계에서는 planner selector를 추가해 hard switch 방식으로 붙이는 전략을 기본으로 합니다.

### 2-6. Git 협업 권장 범위

merge 충돌을 줄이기 위해, 각 담당자는 아래 범위를 우선 소유하는 식으로 작업하는 것을 권장합니다.

- 회피 담당: `src/drone_planning`, `src/drone_bringup/launch/single_drone_avoidance_dev.launch.py`, `src/drone_bringup/config/drone1_avoidance_dev.yaml`
- SLAM 담당: `src/drone_slam`, `src/drone_bringup/launch/single_drone_slam_dev.launch.py`, `src/drone_bringup/config/drone1_slam_dev.yaml`
- MPPI 담당: `src/mppi`, `src/mppi_lidar`, `src/drone_bringup/launch/single_drone_mppi_dev.launch.py`
- 통합 담당: `src/drone_bringup`, 이후 planner selector / behavior manager 추가 영역

즉 알고리즘 구현은 각 패키지에서 분리하고, 공통 launch와 topic 계약만 최소한으로 맞추는 방식이 가장 안전합니다.

## 3. 기본 실행 순서

### 3-1. Gazebo Classic + PX4 시작

이제 기본 경로는 예전처럼 `sim` 컨테이너가 올라갈 때 Gazebo Classic GUI가 같이 뜨는 방식입니다.
helper 스크립트는 GUI가 꼬였을 때만 쓰는 보조 경로입니다.

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

기본 경로에서는 이 시점에 Gazebo Classic GUI가 자동으로 떠야 합니다.
helper 스크립트는 자동 GUI가 꼬였을 때만 보조로 사용합니다.

### 3-2. autonomy launch 실행

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics ros_states --symlink-install
source install/setup.bash
ros2 launch drone_bringup single_drone_autonomy.launch.py
```
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone

colcon build --packages-up-to drone_bringup --symlink-install
source install/setup.bash

ros2 launch drone_bringup single_drone_mppi_known_world.launch.py

### 3-3. `ros_states` 실행

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

rosbag 녹화
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
source install/setup.bash
ros2 bag record \
  -o rosbags/gap_vs_mppi_$(date +%Y%m%d_%H%M%S) \
  /mavros/local_position/pose \
  /mavros/state \
  /drone1/scan \
  /drone1/mission/phase \
  /drone1/mission/goal_reached \
  /drone1/mission/active_goal \
  /drone1/mission/home_pose \
  /drone1/planner/avoid/cmd_vel \
  /drone1/planner/mppi/cmd_vel \
  /drone1/safety/cmd_vel \
  /drone1/safety/state \
  /drone1/slam/map_ready \
  /drone1/slam/localization_ok \
  /drone1/slam/coverage

같은 네트워크 다른 장치에서 볼 때는 호스트 IP 기준 `http://<host-ip>:5050`를 사용하면 됩니다.

### 3-4. `ros_states` 디버깅 기록 저장

이제 `ros_states`에서 보고 있는 디버깅 상태를 그대로 저장할 수 있습니다.

중요:

- 이건 브라우저 픽셀 화면을 캡처하는 기능이 아닙니다.
- `ros_states` 백엔드가 브라우저에 보여주던 상태를 JSON으로 저장하는 기능입니다.
- 따라서 아무 버튼도 누르지 않으면 계속 자동 저장되지는 않습니다.

버튼 의미는 아래와 같습니다.

- `Save Snapshot` : 현재 화면 기준 상태를 한 번 JSON으로 저장하고, 바로 HTML report도 갱신
- `Start Recording` : 일정 간격으로 디버깅 타임라인을 계속 기록 시작
- `Stop Recording` : 기록 세션 종료 후 마지막 스냅샷 저장, 그리고 report를 바로 자동 생성
- `Generate Report` : 저장된 JSON/timeline을 읽어서 사람이 보기 쉬운 요약 + 시각화 + 그래프로 다시 생성
- `Open Report` : 가장 최근 생성된 report를 브라우저 새 탭에서 열기

즉 일반적인 사용 흐름에서는 `Generate Report`를 꼭 따로 누를 필요가 없습니다.

- `Start Recording`
- 실험 진행
- `Stop Recording`

이렇게만 해도 마지막 시점에 report가 자동으로 생성됩니다. `Generate Report` 버튼은 기존 세션을 다시 변환하거나, 수동 snapshot만 저장한 뒤 다시 HTML을 만들고 싶을 때 쓰는 재생성 용도입니다.

세션별로 report를 나눠서 여는 방식도 이제 지원합니다.

- 최신 report 별칭: `http://localhost:5050/debug/report/current`
- 세션별 고정 URL: `http://localhost:5050/debug/report/<session_name>`
- 세션 목록 API: `http://localhost:5050/api/debug/reports`

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

특정 세션을 고정해서 열려면 session 폴더 이름을 그대로 붙이면 됩니다.

예:

```text
http://localhost:5050/debug/report/20260331_084137_drone1_recording
```

중요:

- `artifacts/_ros_states_debug/.../report.html` 파일을 로컬 경로로 직접 열면 브라우저나 IDE preview에서 막힐 수 있습니다.
- 가장 안정적인 방법은 항상 `http://localhost:5050/debug/report/current` 또는 세션별 `http://localhost:5050/debug/report/<session_name>` 로 여는 것입니다.
- 생성된 `report.html` 자체는 정적 HTML이라서 공유용 결과물로 쓸 수 있습니다. 다만 로컬 파일 직접 열기는 환경에 따라 막힐 수 있어서, 로컬에서는 Flask URL로 여는 쪽이 더 안정적입니다.

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

## 4. Gazebo GUI

기본 경로에서는 `docker compose up -d --force-recreate sim ros`만으로 Gazebo Classic GUI가 자동으로 떠야 합니다.
아래 helper는 정말로 GUI가 꼬였을 때만 수동 복구용으로 남겨둡니다.

```bash
./scripts/run_host_gz_gui.sh
```

중요:

- 이번 blank-window 원인은 `sim` 컨테이너가 이미지 안의 오래된 `/opt/PX4-Autopilot/docker/sim/entrypoint.sh`를 계속 써서, 최신 Gazebo fix가 반영되지 않던 것이었습니다. 지금은 `docker-compose.yml` 기준으로 워크스페이스의 `/workspace/AV_Drone/docker/sim/entrypoint.sh`를 직접 사용하도록 맞췄습니다.
- 기본 실행에서는 helper를 따로 치지 않아도 GUI가 떠야 정상입니다.
- 현재 obstacle demo는 world 안에 기본 GUI camera pose를 명시해 두었고, PX4 follow camera plugin은 기본 비활성화(`PX4_NO_FOLLOW_MODE=1`) 상태입니다. 그래서 처음부터 코스가 보이는 시점으로 뜨는 것이 정상입니다.
- 정상 server log에는 더 이상 `Can't open display`, `Rendering will be disabled`, `Unable to create CameraSensor`가 나오지 않아야 합니다.

창이 안 뜨면 먼저 X11 권한을 다시 열어주세요.

```bash
xhost +SI:localuser:root
xhost +local:docker
```

## 5. 지금 obstacle demo 기준 값

현재 demo는 아래 기준으로 맞춰져 있습니다.

- start pose: `(0.0, 0.0, 0.0)`
- goal pose: `(31.0, 0.0, 3.0)`
- `goal_tol_xy`: `0.35 m`
- obstacle world: `obstacle_demo`
- obstacle layout: side-wall corridor + thick poles
- planner: `local_planner_follow_the_gap`

즉 현재 코스는 양쪽 벽으로 폭을 제한한 corridor 안에 두꺼운 기둥을 번갈아 배치해, 드론이 옆으로 크게 도망가지 않고 장애물 사이를 통과하며 더 오래 회피하도록 설계되어 있습니다.

## 6. 멈췄을 때 해석하는 법

### 6-1. 정상 종료

`ros_states`에서 아래처럼 보이면 센싱 실패가 아니라 미션 완료입니다.

- `Mission Phase = HOVER_AT_GOAL`
- `Goal reached = true`

이 경우 planner/safety가 `0,0,0`을 내도 정상입니다. 이미 목표에 도달해서 hover 중인 상태입니다.

### 6-2. 센싱/통신 문제

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

## 7. 다시 처음부터 실행할 때

```bash
cd /home/deepblue/AV_Drone
./stop.sh
xhost +SI:localuser:root
xhost +local:docker
docker compose up -d --force-recreate sim ros
docker compose logs -f sim
```

이후 `Startup script returned successfully`가 뜨면 autonomy와 `ros_states`를 다시 띄우면 됩니다.

## 8. `source`, `colcon build`, `docker compose build` 의미

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

## 9. 정량 실험 대시보드

Gap return baseline과 SLAM-MPPI return 비교 실험은 `artifacts/`와 `experiments/`에 누적됩니다.
Streamlit 대시보드는 이 파일들을 읽어서 condition, scenario, run 단위로 비교하는 read-only 조회 도구입니다.

처음 한 번만 의존성을 설치합니다.

```bash
python3 -m pip install -r requirements-dashboard.txt
```

대시보드 실행:

```bash
./scripts/run_quant_dashboard.sh
```

브라우저에서 아래 주소를 엽니다.

```text
http://localhost:8501
```

Streamlit 없이 데이터 스캔만 검증할 때:

```bash
python3 scripts/quant_dashboard.py --check-data --repo-root .
```

주의:

- 대시보드는 실험 데이터를 수정하지 않습니다.
- 논문에 들어갈 최종 숫자는 `paper_metrics.json`, `experiments/paper_outputs/summary_table.csv`, `experiments/paper_outputs/figure_manifest.csv`를 기준으로 관리합니다.
- 자세한 실험 설계는 [gap_vs_mppi_quantification_plan.md](/home/deepblue/AV_Drone/docs/gap_vs_mppi_quantification_plan.md)를 봅니다.

## 10. 문서

- 구조 설명: [architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
- 명령어 치트시트: [command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
- Docker 명세: [docker-environment-spec.md](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)
- HTML 대시보드: [project_command_center.html](/home/deepblue/AV_Drone/docs/project_command_center.html)
- Gap vs MPPI 정량화 계획: [gap_vs_mppi_quantification_plan.md](/home/deepblue/AV_Drone/docs/gap_vs_mppi_quantification_plan.md)
- 실험 기록 규칙: [experiment_recording_policy.md](/home/deepblue/AV_Drone/docs/experiment_recording_policy.md)
- 변경 로그: [docs/change/README.md](/home/deepblue/AV_Drone/docs/change/README.md)
- 장애 보고서: [docs/error/index.html](/home/deepblue/AV_Drone/docs/error/index.html)
