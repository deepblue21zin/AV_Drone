# AV_Drone Command Reference

이 문서는 `/home/deepblue/AV_Drone` 프로젝트를 실행, 빌드, 디버깅할 때 자주 사용하는 명령어를 정리한 실무용 레퍼런스다.  
기준 환경은 `Docker Compose + ROS 2 Humble + PX4 SITL + Gazebo Classic + MAVROS`다.

중요 원칙:

- 호스트에서 치는 명령과 컨테이너 안에서 치는 명령을 구분한다.
- `sim`은 PX4/Gazebo 컨테이너다.
- `ros`는 ROS 2 작업 컨테이너다.
- ROS 관련 명령은 기본적으로 `ros` 컨테이너 안에서 실행한다.

## 1. 자주 쓰는 기본 경로

호스트 프로젝트 경로:

```bash
cd /home/deepblue/AV_Drone
```

ROS 컨테이너 내부 프로젝트 경로:

```bash
cd /workspace/AV_Drone
```

## 2. 호스트에서 먼저 확인할 것

Docker 동작 확인:

```bash
docker --version
docker compose version
docker run hello-world
```

X11 확인:

```bash
echo $DISPLAY
xhost +local:docker
```

## 3. Docker Compose 기본 명령

전체 이미지 빌드:

```bash
docker compose build
```

ROS 이미지만 다시 빌드:

```bash
docker compose build ros
```

SIM 이미지만 다시 빌드:

```bash
docker compose build sim
```

캐시 없이 전체 다시 빌드:

```bash
docker compose build --no-cache
```

서비스 기동:

```bash
docker compose up -d sim
docker compose up -d ros
```

서비스 종료:

```bash
docker compose down
```

서비스 상태 확인:

```bash
docker compose ps
```

## 4. 로그 확인 명령

SIM 로그 보기:

```bash
docker compose logs -f sim
```

ROS 로그 보기:

```bash
docker compose logs -f ros
```

최근 로그만 보기:

```bash
docker compose logs --tail=100 sim
docker compose logs --tail=100 ros
```

## 5. 컨테이너 접속

ROS 컨테이너 진입:

```bash
docker compose exec ros bash
```

SIM 컨테이너 진입:

```bash
docker compose exec sim bash
```

현재 실행 중인 컨테이너 확인:

```bash
docker ps
```

특정 컨테이너 내부 진입:

```bash
docker exec -it <container_name> bash
```

## 6. ROS 컨테이너에 들어간 뒤 가장 먼저 할 것

기본 환경 로드:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
```

빌드가 이미 끝난 경우 install 환경까지 로드:

```bash
source install/setup.bash
```

한 번에 정리하면:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
source install/setup.bash
```

## 7. 의존성 설치와 워크스페이스 빌드

`rosdep` DB 갱신:

```bash
rosdep update
```

의존성 설치:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

전체 빌드:

```bash
colcon build
```

특정 패키지만 빌드:

```bash
colcon build --packages-select mppi
colcon build --packages-select offboard_control
```

심볼릭 설치로 빌드:

```bash
colcon build --symlink-install
```

빌드 후 환경 반영:

```bash
source install/setup.bash
```

## 8. 주요 실행 명령

MPPI 실행:

```bash
ros2 launch mppi mppi.launch.py
```

단순 오프보드 이륙 테스트:

```bash
ros2 launch offboard_control offboard_control.launch.py
```

## 9. ROS 2 기본 점검 명령

토픽 목록:

```bash
ros2 topic list
```

MAVROS 관련 토픽만 보기:

```bash
ros2 topic list | grep mavros
```

노드 목록:

```bash
ros2 node list
```

서비스 목록:

```bash
ros2 service list
```

패키지 목록:

```bash
ros2 pkg list | grep -E 'mppi|offboard_control|mavros'
```

## 10. 자주 확인하는 토픽

비행 상태:

```bash
ros2 topic echo /mavros/state
```

로컬 위치:

```bash
ros2 topic echo /mavros/local_position/pose
```

로컬 속도:

```bash
ros2 topic echo /mavros/local_position/velocity_local
```

속도 셋포인트:

```bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

배터리 상태:

```bash
ros2 topic echo /mavros/battery
```

## 11. 토픽/서비스 타입 확인

토픽 정보:

```bash
ros2 topic info /mavros/state
ros2 topic info /mavros/local_position/pose
```

서비스 정보:

```bash
ros2 service type /mavros/set_mode
ros2 service type /mavros/cmd/arming
```

메시지 정의 확인:

```bash
ros2 interface show mavros_msgs/msg/State
ros2 interface show geometry_msgs/msg/PoseStamped
```

서비스 정의 확인:

```bash
ros2 interface show mavros_msgs/srv/SetMode
ros2 interface show mavros_msgs/srv/CommandBool
```

## 12. MAVROS 연결 확인용 핵심 명령

MAVROS 토픽이 살아있는지 보기:

```bash
ros2 topic list | grep mavros
```

PX4 heartbeat 연결 확인:

```bash
ros2 topic echo /mavros/state
```

확인 포인트:

- `connected: true`
- `armed: true/false`
- `mode: OFFBOARD` 또는 그 이전 모드

관련 서비스 확인:

```bash
ros2 service list | grep mavros
```

주요 서비스:

- `/mavros/set_mode`
- `/mavros/cmd/arming`

## 13. 노드와 프로세스 확인

현재 노드 목록:

```bash
ros2 node list
```

특정 노드 정보:

```bash
ros2 node info /mavros/mavros
ros2 node info /mppi
```

컨테이너 안 프로세스 확인:

```bash
ps -ef
```

## 14. 패키지 실행 파일 확인

패키지 실행 파일 목록:

```bash
ros2 pkg executables mppi
ros2 pkg executables offboard_control
```

예상 결과:

- `mppi mppi_node`
- `offboard_control offboard_takeoff`

## 15. 빌드 산출물 정리

ROS workspace 빌드 산출물:

- `build/`
- `install/`
- `log/`

정리:

```bash
rm -rf build install log
```

주의:

- 이 명령은 빌드 결과를 지운다.
- 소스코드는 지우지 않는다.

정리 후 다시 빌드:

```bash
colcon build
source install/setup.bash
```

## 16. 자주 쓰는 디버깅 명령

MAVROS 로그가 보고 싶을 때:

```bash
ros2 launch mppi mppi.launch.py
```

그 상태에서 다른 터미널에서 ROS 컨테이너 진입:

```bash
docker compose exec ros bash
```

노드가 살아있는지 확인:

```bash
ros2 node list
```

토픽 주기 확인:

```bash
ros2 topic hz /mavros/local_position/pose
ros2 topic hz /mavros/setpoint_velocity/cmd_vel
```

토픽 한 번만 보기:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
```

## 17. 자주 보는 성공 로그

MAVROS 연결 성공 신호:

- `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`

MPPI 미션 진행 신호:

- `PHASE => OFFBOARD_ARM`
- `PHASE => TAKEOFF`
- `PHASE => HOVER_AFTER_TAKEOFF`
- `PHASE => MPPI_GO`
- `PHASE => HOVER_AT_GOAL`
- `PHASE => LAND`
- `PHASE => WAIT_LANDED`
- `PHASE => DONE`

## 18. 자주 보는 경고와 해석

`Time jump detected`:

- Docker 환경이나 시뮬레이션 부하에서 발생할 수 있다.
- 무조건 치명적 오류는 아니다.

`RTT too high for timesync`:

- SITL과 컨테이너 타이밍 차이 때문에 생길 수 있다.

`PositionTargetGlobal failed because no origin`:

- 글로벌 원점이 없을 때 일부 MAVROS 플러그인에서 발생한다.
- 현재 프로젝트는 로컬 좌표계 기반 비행이라 직접 치명적이지 않을 수 있다.

## 19. 자주 쓰는 Git 명령

현재 변경 사항 확인:

```bash
git status
```

변경 파일만 간단히 보기:

```bash
git status --short
```

변경 내용 보기:

```bash
git diff
```

특정 파일만 보기:

```bash
git diff README.md
git diff src/mppi/mppi/mppi_node.py
```

## 20. 추천 작업 루틴

### 루틴 1. 새 환경에서 처음 시작

```bash
cd /home/deepblue/AV_Drone
docker compose build
docker compose up -d sim
docker compose up -d ros
docker compose exec ros bash
```

컨테이너 안:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch mppi mppi.launch.py
```

### 루틴 2. 코드만 수정한 뒤 다시 실행

호스트:

```bash
docker compose up -d sim
docker compose up -d ros
docker compose exec ros bash
```

컨테이너 안:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build --packages-select mppi
source install/setup.bash
ros2 launch mppi mppi.launch.py
```

### 루틴 3. 실행 상태 확인만 할 때

호스트:

```bash
docker compose ps
docker compose logs --tail=100 sim
docker compose logs --tail=100 ros
```

컨테이너 안:

```bash
ros2 node list
ros2 topic list | grep mavros
ros2 topic echo /mavros/state --once
```

## 21. 관련 문서

발표/개요 문서:

- [README.md](/home/deepblue/AV_Drone/README.md)

Docker 환경 명세:

- [docs/docker-environment-spec.md](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)

이 문서:

- [docs/command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
