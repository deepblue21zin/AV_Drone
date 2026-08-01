# 콜드 부팅 → 테스트 가능 상태 체크리스트

작성일: 2026-07-18
목적: 컴퓨터를 완전히 껐다 켠 상태에서, RDP(가제보 GUI) + VSCode(SSH, 터미널 5개)를 다시 세팅해서
Gap 회피 + SLAM 매핑 + 지도 저장 테스트를 바로 시작할 수 있는 상태까지 가는 절차.

## 터미널 구성

| 위치 | 터미널 | 용도 |
|---|---|---|
| RDP | 1 | 가제보 GUI 실행 (`docker compose up`) |
| VSCode | 1 | 드론 현재 위치 실시간 출력 |
| VSCode | 2 | 출발(home) 위치 실시간 출력 |
| VSCode | 3 | 목표(goal) 위치 실시간 출력 |
| VSCode | 4 | 빌드 + `ros2 launch` 실행 |
| VSCode | 5 | Claude CLI |

## 핵심 원칙 (왜 이렇게 나누는지)

- 가제보 GUI가 뜨는 화면은 `docker compose up`을 **어느 터미널에서 실행했는지**로 결정된다
  (`docker-compose.yml`이 그 순간의 `$DISPLAY`를 컨테이너에 넘김) → 그래서 최초 기동은 RDP에서.
- `ros2 topic echo`, `ros2 launch` 같은 텍스트 명령은 이미 떠 있는 컨테이너에 새 셸을 붙이는 것뿐이라
  DISPLAY와 무관 → VSCode SSH 터미널에서 자유롭게 실행 가능.
- 경로 주의: **호스트(RDP/VSCode, 컨테이너 밖)**에서는 `/home3/sjee/Workspace/Drone_0716`,
  **컨테이너 안**(`docker compose exec ... bash` 이후)에서는 항상 `/workspace/AV_Drone`.

---

## 0. RDP 재접속

1. RDP 클라이언트로 서버에 접속
2. 터미널 앱 하나 실행 → 이게 "RDP 터미널 1"
3. 확인:
   ```bash
   echo $DISPLAY
   ```
   `:10`처럼 값이 나와야 함. 비어있으면 RDP 세션 자체를 다시 확인.

## 1. [RDP 터미널 1] 가제보/PX4 SITL 기동

```bash
cd /home3/sjee/Workspace/Drone_0716
docker ps --format '{{.Names}}\t{{.Status}}' | grep -E "sim|ros"
```
→ 이 시점에 `sjee-`로 시작하는 것 말고 다른 팀원 컨테이너(`quddnr-*`, `av_drone-*` 등)가 떠서
돌고 있으면 포트/ROS_DOMAIN 충돌 위험이 있으니, 먼저 확인하고 필요하면 조율한다.
(컴퓨터를 완전히 껐다 켠 직후라면 보통 sjee 쪽은 전부 꺼진 상태일 것.)

```bash
xhost +SI:localuser:root
xhost +local:docker
docker compose up -d --force-recreate sim ros
docker compose logs -f sim
```

로그에 아래 문구가 뜰 때까지 기다린다 (처음 뜨는 거라 gazebo-classic 플러그인 재컴파일이 돌아서
몇 분 걸릴 수 있음):

```text
Startup script returned successfully
```

뜨면 `Ctrl+C`로 로그 팔로우만 종료 (컨테이너는 계속 실행 중). 이 시점에 가제보 GUI 창이 RDP
화면에 자동으로 떠야 한다.

### 1-1. GUI가 안 뜨거나 다른 화면에 뜬 경우에만 (같은 RDP 터미널)

```bash
./scripts/run_host_gz_gui.sh
```

---

## 2. VSCode 재접속 + 터미널 5개 준비

1. VSCode에서 Remote-SSH로 서버에 재접속
2. 터미널 5개를 연다 (탭 또는 Split Terminal)

아래 4번(빌드+실행)을 먼저 끝내야 1/2/3번 좌표 토픽이 실제로 값을 뱉기 시작한다. 순서상
**4번을 먼저 실행**하고, 그다음 1/2/3번을 켜는 걸 권장 (1/2/3을 먼저 켜둬도 에러는 안 나고
그냥 "토픽 없음" 경고만 뜨다가 4번 이후 값이 흐르기 시작함).

### [VSCode 터미널 4] 빌드 + 실행

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select drone_bringup drone_control drone_perception drone_planning drone_safety drone_metrics drone_slam --symlink-install
source install/setup.bash
ros2 launch drone_bringup single_drone_avoidance_dev.launch.py
```

이 launch가 띄우는 것: `mavros + lidar_obstacle + local_planner(Gap) + safety_monitor +
autonomy_manager + metrics_logger + simple_2d_mapping(SLAM) + map_saver + foxglove_bridge`.

미션 진행 순서:
```text
WAIT_STREAM → OFFBOARD_ARM → TAKEOFF → HOVER_AFTER_TAKEOFF
  → MAPPING_TO_GOAL (Gap으로 목표까지 이동하며 SLAM 매핑)
  → LAND_AT_GOAL (AUTO.LAND 모드로 착륙)
  → LANDED (착륙 완료 시점에 지도 파일 저장, 이후 정지)
```

이 터미널은 launch가 계속 실행 중인 상태로 그대로 둔다 (여기서 다른 명령 치지 않음).

### [VSCode 터미널 1] 드론 현재 위치

```bash
docker compose exec ros bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /mavros/local_position/pose --field pose.position"
```

### [VSCode 터미널 2] 출발(home) 위치

```bash
docker compose exec ros bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /drone1/mission/home_pose --field pose.position"
```
참고: 드론이 실제 이륙 처리에 들어가기 전(`TAKEOFF` 단계)까지는 메시지가 안 온다 — 처음엔 안 나와도 정상.

### [VSCode 터미널 3] 목표(goal) 위치

```bash
docker compose exec ros bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && ros2 topic echo /drone1/mission/active_goal --field pose.position"
```

### [VSCode 터미널 5] Claude CLI

```bash
claude
```

---

## 3. (선택) Foxglove로 지도 시각화

로컬 노트북의 Foxglove Studio에서:
- **Open connection → Foxglove WebSocket**
- 주소: `ws://<서버 IP>:8765`

접속되면 `/map`, `/drone1/scan`, `/mavros/local_position/pose` 등이 보인다. Map 패널을 추가하고
토픽을 `/map`으로 잡으면 비행하면서 지도가 채워지는 걸 실시간으로 볼 수 있다.

---

## 4. 착륙 완료 후 확인할 것

`LANDED` phase(터미널 4 로그 또는 `/drone1/mission/phase`)가 뜨면:

```bash
docker compose exec ros bash -lc "ls -la /workspace/AV_Drone/maps/single_drone_obstacle_demo/"
```

`{map_file_basename}_grid.npy`, `{map_file_basename}_meta.json` 두 파일이 방금 시각으로 갱신됐는지
확인 (파일명은 `drone1_avoidance_dev.yaml`의 `map_file_basename` 값 기준, 지금은
`obstacle_demo_v3_*`). 자세한 읽는 법은 `maps/single_drone_obstacle_demo/README.md` 참고.

---

## 5. Gazebo world 버전을 바꿔서 테스트하고 싶을 때 (예: v3 → v4)

새 world 파일로 바꿔서 테스트하려면 **딱 3곳**만 수정하면 된다.

| # | 파일 | 항목 | 역할 |
|---|---|---|---|
| 1 | `sim_assets/worlds/obstacle_demo_vN.world` (신규 생성) | 파일 자체 | 실제 world 내용 (다른 팀원 브랜치에서 가져오거나, 이전 버전을 복사해서 수정) |
| 2 | `docker-compose.override.yml` | `services.sim.environment.PX4_SITL_WORLD` | 가제보가 **어느 world 파일**을 로드할지 (sjee 개인 설정, git에 안 올라감) |
| 3 | `src/drone_bringup/config/drone1_avoidance_dev.yaml` | `map_file_basename` | SLAM 결과 **저장 파일명** 접두어 (버전별로 지도 파일이 안 덮어써지게 구분) |

`scenario_name`(`maps/<scenario_name>/` 폴더명)은 그대로 둬도 된다 — 폴더는 고정, `map_file_basename`만
버전별로 바뀌면서 `maps/single_drone_obstacle_demo/` 안에 `obstacle_demo_v2_grid.npy`,
`obstacle_demo_v3_grid.npy`처럼 파일명으로 구분되어 쌓인다.

체크리스트:
1. `sim_assets/worlds/obstacle_demo_vN.world` 준비 (내용 확보)
2. `docker-compose.override.yml`의 `PX4_SITL_WORLD`를 `"obstacle_demo_vN"`으로 변경
3. `drone1_avoidance_dev.yaml`의 `map_file_basename`을 `"obstacle_demo_vN"`으로 변경
4. **world의 실제 레이아웃(길이/폭)이 이전 버전과 다르면** 같은 yaml의 `goal_x/goal_y/goal_z`,
   `map_min_x/max_x/min_y/max_y`도 새 world 크기에 맞게 같이 조정 (world 파일 안 벽/바닥
   `<pose>`, `<size>` 값을 보고 실제 코스 범위를 확인한 뒤 정한다)
5. `docker compose up -d --force-recreate sim ros`로 재기동 (world 이름이 바뀌면 PX4 빌드
   타겟에 새로 등록되는 과정이 있어서 플러그인 재컴파일이 한 번 더 돎, RDP 터미널에서 실행)
6. `drone_bringup` 재빌드 (VSCode 터미널 4에서 `colcon build --packages-select drone_bringup`)

---

## 6. 종료할 때

`stop.sh`는 `docker compose down`을 실행하므로 **호스트(컨테이너 밖)**에서 실행해야 한다.
RDP 터미널이든 VSCode 터미널이든 컨테이너 안에 안 들어가 있는 셸이면 된다 (VSCode 터미널
4는 `ros2 launch`가 실행 중이라 먼저 `Ctrl+C`로 중단하거나 새 터미널을 써야 함).

```bash
cd /home3/sjee/Workspace/Drone_0716
./stop.sh
```

---

## 요약 순서표 (콜드 부팅 기준)

| 순서 | 위치 | 명령 |
|---|---|---|
| 0 | RDP | 재접속, `echo $DISPLAY` 확인 |
| 1 | RDP 터미널 1 | `docker ps`로 다른 팀원 충돌 확인 → `xhost` 2줄 → `docker compose up -d --force-recreate sim ros` → 로그에서 `Startup script returned successfully` 확인 |
| 1-1 (필요시) | RDP 터미널 1 | `./scripts/run_host_gz_gui.sh` |
| 2 | VSCode 접속 | Remote-SSH 재접속, 터미널 5개 열기 |
| 3 | VSCode 터미널 4 | `docker compose exec ros bash` → source → `colcon build` → source install → `ros2 launch drone_bringup single_drone_avoidance_dev.launch.py` (계속 실행 상태 유지) |
| 4 | VSCode 터미널 1/2/3 | 각각 현재/출발/목표 위치 `ros2 topic echo` |
| 5 | VSCode 터미널 5 | `claude` |
| 6 (선택) | 로컬 노트북 | Foxglove Studio → `ws://<서버 IP>:8765` |
| - (필요시) | world/설정 파일 3곳 + RDP/VSCode 터미널 4 | world 버전 전환 (5절 참고) |
| 7 | 아무 호스트 셸 | 종료 시 `./stop.sh` |
