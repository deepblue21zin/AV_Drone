# AV_Drone Architecture

이 문서는 `/home/deepblue/AV_Drone` 프로젝트의 코드 구조와 런타임 구조를 README보다 더 깊게 설명하는 아키텍처 문서다.  
목표는 처음 보는 사람이 아래를 이해할 수 있게 하는 것이다.

- 어떤 프로세스가 어디서 실행되는가
- 패키지와 파일은 어떤 역할을 하는가
- 노드 간 데이터는 어떻게 흐르는가
- MPPI 노드는 어떤 상태 머신으로 움직이는가
- 어떤 파일을 수정하면 어떤 동작이 달라지는가

최근 구조 변경 배경은 [change.md](/home/deepblue/AV_Drone/docs/change.md)에 정리돼 있다.

## 1. 아키텍처 요약

이 프로젝트는 크게 두 층으로 나뉜다.

### 1. 시뮬레이션 층

구성:

- PX4 SITL
- Gazebo Classic

역할:

- 드론 비행 제어기와 물리 환경을 가상으로 재현

### 2. 제어 소프트웨어 층

구성:

- ROS 2 Humble
- MAVROS
- `mppi` 패키지
- `offboard_control` 패키지
- `drone_bringup`
- `drone_control`
- `drone_perception`
- `drone_planning`
- `drone_safety`
- `drone_metrics`

역할:

- PX4 상태를 읽고
- 오프보드 명령을 만들고
- 목표 비행 미션을 수행

## 2. 런타임 구성

Docker Compose 기준으로 두 개의 컨테이너가 실행된다.

```text
compose
├─ sim
│  ├─ PX4 SITL
│  └─ Gazebo Classic
└─ ros
   ├─ ROS 2 runtime
   ├─ MAVROS
   ├─ mppi node
   ├─ offboard_control node
   ├─ perception/planning/control pipeline
   └─ safety/metrics pipeline
```

런타임 시작 순서를 시퀀스 관점에서 보면 아래와 같다.

```text
Host/User         Docker Compose         sim container         ros container         PX4 SITL           Gazebo           MAVROS           MPPI Node
   |                    |                     |                     |                   |                 |                |                 |
   | docker up sim      |                     |                     |                   |                 |                |                 |
   |------------------->|                     |                     |                   |                 |                |                 |
   |                    | create/start sim    |                     |                   |                 |                |                 |
   |                    |-------------------->|                     |                   |                 |                |                 |
   |                    |                     | start PX4+Gazebo    |                   |                 |                |                 |
   |                    |                     |-------------------> |                   |                 |                |                 |
   |                    |                     |                     |                   | boot            | launch world    |                 |
   |                    |                     |                     |                   |---------------> |--------------->|                 |
   |                    |                     |                     |                   | open UDP        |                |                 |
   |                    |                     |                     |                   |                 |                |                 |
   | docker up ros      |                     |                     |                   |                 |                |                 |
   |------------------->|                     |                     |                   |                 |                |                 |
   |                    | create/start ros    |                     |                   |                 |                |                 |
   |                    |------------------------------------------>|                   |                 |                |                 |
   |                    |                     |                     | sleep/infinity     |                 |                |                 |
   |                    |                     |                     |                   |                 |                |                 |
   | exec ros bash      |                     |                     |                   |                 |                |                 |
   |------------------->|------------------------------------------>|                   |                 |                |                 |
   | colcon build       |                     |                     | build packages     |                 |                |                 |
   |-------------------------------------------------------------->|                   |                 |                |                 |
   | ros2 launch mppi   |                     |                     | launch             |                 |                |                 |
   |-------------------------------------------------------------->|                   |                 |                |                 |
   |                    |                     |                     | start MAVROS       |                 |                |                 |
   |                    |                     |                     |----------------------------------------------->|                 |
   |                    |                     |                     | start MPPI node    |                 |                |---------------->|
   |                    |                     |                     |                   |                 |                | connect UDP     |
   |                    |                     |                     |                   |<---------------------------------------------------|
   |                    |                     |                     |                   | HEARTBEAT/state |                | publish topics  |
   |                    |                     |                     |                   |------------------------------->|---------------->|
```

### `sim` 컨테이너

관련 파일:

- [docker/sim/Dockerfile](/home/deepblue/AV_Drone/docker/sim/Dockerfile)
- [docker/sim/entrypoint.sh](/home/deepblue/AV_Drone/docker/sim/entrypoint.sh)

책임:

- PX4 소스 보유
- `make px4_sitl gazebo-classic` 실행
- Gazebo world와 PX4 SITL을 함께 기동

현재 메모:

- 기존에는 PX4 제공 `focal` 개발 이미지를 사용했었다.
- 현재는 라이다의 ROS 2 직접 연동을 위해 `Ubuntu 22.04 + ROS 2 Humble + Gazebo ROS` 기준으로 전환 중이다.
- 기본 실행 타깃은 `gazebo-classic_iris_rplidar`로 맞춰져 있다.

### `ros` 컨테이너

관련 파일:

- [docker/ros/Dockerfile](/home/deepblue/AV_Drone/docker/ros/Dockerfile)
- [docker/ros/entrypoint.sh](/home/deepblue/AV_Drone/docker/ros/entrypoint.sh)
- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)

책임:

- ROS 2 Humble 환경 제공
- MAVROS 제공
- 이 저장소 ROS 2 패키지 빌드와 실행

## 3. 네트워크와 연결 방식

현재 구성은 `host network`를 사용한다.

이유:

- PX4 SITL과 MAVROS의 UDP 연결을 단순하게 유지할 수 있다.
- `udp://:14540@127.0.0.1:14580`를 그대로 사용할 수 있다.
- Docker 브리지 네트워크보다 디버깅이 쉽다.

PX4와 MAVROS 연결 기준:

- PX4 송수신 대상: UDP
- MAVROS FCU URL: `udp://:14540@127.0.0.1:14580`

관련 파일:

- [src/mppi/launch/mppi.launch.py](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)

## 4. 코드 패키지 구조

이 저장소에는 현재 두 종류의 패키지가 공존한다.

- 기존 단일 드론 MPPI 데모 패키지
- 새 센서 기반 단일 드론 자율주행 뼈대 패키지

### 4.1 `mppi`

경로:

- [src/mppi](/home/deepblue/AV_Drone/src/mppi)

역할:

- MAVROS launch 포함
- 장애물 파라미터 전달
- MPPI 제어 알고리즘 수행
- 전체 비행 시나리오 관리

주요 파일:

- [src/mppi/launch/mppi.launch.py](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)
- [src/mppi/mppi/mppi_node.py](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)
- [src/mppi/config/mavros_config.yaml](/home/deepblue/AV_Drone/src/mppi/config/mavros_config.yaml)
- [src/mppi/config/mavros_pluginlists.yaml](/home/deepblue/AV_Drone/src/mppi/config/mavros_pluginlists.yaml)
- [src/mppi/package.xml](/home/deepblue/AV_Drone/src/mppi/package.xml)
- [src/mppi/setup.py](/home/deepblue/AV_Drone/src/mppi/setup.py)

### 4.2 `offboard_control`

경로:

- [src/offboard_control](/home/deepblue/AV_Drone/src/offboard_control)

역할:

- 최소 오프보드 이륙 예제
- MAVROS와 PX4 연결 sanity check

주요 파일:

- [src/offboard_control/launch/offboard_control.launch.py](/home/deepblue/AV_Drone/src/offboard_control/launch/offboard_control.launch.py)
- [src/offboard_control/offboard_control/offboard_takeoff_node.py](/home/deepblue/AV_Drone/src/offboard_control/offboard_control/offboard_takeoff_node.py)

### 4.3 `drone_bringup`

경로:

- [src/drone_bringup](/home/deepblue/AV_Drone/src/drone_bringup)

역할:

- 센서 기반 단일 드론 자율주행 launch 구성
- 노드 파라미터 YAML 관리

주요 파일:

- [src/drone_bringup/launch/single_drone_autonomy.launch.py](/home/deepblue/AV_Drone/src/drone_bringup/launch/single_drone_autonomy.launch.py)
- [src/drone_bringup/config/drone1_autonomy.yaml](/home/deepblue/AV_Drone/src/drone_bringup/config/drone1_autonomy.yaml)

### 4.4 `drone_control`

- [src/drone_control/drone_control/autonomy_manager_node.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/autonomy_manager_node.py)
- [src/drone_control/drone_control/vehicle_interface.py](/home/deepblue/AV_Drone/src/drone_control/drone_control/vehicle_interface.py)

역할:

- safe command만 PX4 offboard velocity setpoint로 전달
- MAVROS 서비스와 publisher/subscriber 래핑

### 4.5 `drone_perception`

- [src/drone_perception/drone_perception/lidar_obstacle_node.py](/home/deepblue/AV_Drone/src/drone_perception/drone_perception/lidar_obstacle_node.py)

역할:

- `LaserScan`에서 nearest obstacle distance 추출

### 4.6 `drone_planning`

- [src/drone_planning/drone_planning/local_planner_node.py](/home/deepblue/AV_Drone/src/drone_planning/drone_planning/local_planner_node.py)

역할:

- perception 결과를 기반으로 자율주행용 velocity command 생성

### 4.7 `drone_safety`

- [src/drone_safety/drone_safety/safety_monitor_node.py](/home/deepblue/AV_Drone/src/drone_safety/drone_safety/safety_monitor_node.py)

역할:

- pose timeout
- scan timeout
- planner command timeout
- obstacle emergency stop

상황을 감시하고, 이상 시 zero velocity fail-safe를 발행

### 4.8 `drone_metrics`

- [src/drone_metrics/drone_metrics/metrics_logger_node.py](/home/deepblue/AV_Drone/src/drone_metrics/drone_metrics/metrics_logger_node.py)

역할:

- runtime metrics를 `artifacts/`에 저장
- `metrics.csv`, `events.log`, `summary.json`, `metadata.json` 생성

## 5. Launch 구조

## 현재 우선 작업

현재 아키텍처 관점에서 가장 중요한 작업은 아래 순서다.

1. `sim` 컨테이너에서 ROS 2 호환 라이다 토픽 `/drone1/scan`을 실제로 발행하게 만들기
2. `single_drone_autonomy.launch.py`의 perception/safety/control 경로가 그 토픽을 소비하는지 확인하기
3. `autonomy_manager`에 takeoff/offboard 미션 상태 머신을 추가해 실제 비행 구조로 올리기
4. obstacle world를 넣고 센서 기반 회피를 검증하기

### `mppi.launch.py`

관련 파일:

- [src/mppi/launch/mppi.launch.py](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)

이 launch는 두 개의 실행 단위를 함께 띄운다.

1. MAVROS
2. MPPI node

즉, MPPI 노드는 단독으로 PX4와 붙지 않는다.  
반드시 MAVROS가 같이 떠서 ROS 2 토픽/서비스 레이어를 형성해야 한다.

`mppi.launch.py`가 하는 일:

- MAVROS 기본 launch 포함
- 저장소 내부 커스텀 MAVROS YAML 적용
- FCU URL 전달
- MPPI 노드 파라미터 전달

### `offboard_control.launch.py`

관련 파일:

- [src/offboard_control/launch/offboard_control.launch.py](/home/deepblue/AV_Drone/src/offboard_control/launch/offboard_control.launch.py)

이 launch는 매우 단순하다.

- `offboard_takeoff` 실행 파일 하나만 띄운다.

### `single_drone_autonomy.launch.py`

관련 파일:

- [src/drone_bringup/launch/single_drone_autonomy.launch.py](/home/deepblue/AV_Drone/src/drone_bringup/launch/single_drone_autonomy.launch.py)

이 launch는 현재 센서 기반 단일 드론 자율주행의 뼈대다.

포함 노드:

1. MAVROS
2. `lidar_obstacle_node`
3. `local_planner_node`
4. `safety_monitor_node`
5. `autonomy_manager_node`
6. `metrics_logger_node`

## 6. `mppi_node.py` 상세 구조

관련 파일:

- [src/mppi/mppi/mppi_node.py](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)

이 파일은 아래 순서로 읽으면 이해가 쉽다.

### 6.1 유틸 함수 영역

포함 함수:

- `clamp()`
- `wrap_pi()`
- `quat_to_yaw()`

역할:

- 제어 계산에 필요한 보조 연산

### 6.2 데이터 구조 영역

포함 구조:

- `Obstacle2D`
- `MPPIConfig`

역할:

- 장애물 정보 표현
- MPPI 하이퍼파라미터 정의

주요 파라미터 예:

- `dt`
- `horizon`
- `num_samples`
- `v_max`
- `yaw_rate_max`
- `w_goal`
- `w_obst`
- `safety_margin`

### 6.3 MPPI 알고리즘 영역

클래스:

- `MPPIController`

이 클래스의 핵심 절차:

1. nominal control sequence 준비
2. 노이즈를 더한 후보 제어 입력 샘플링
3. 각 후보에 대해 rollout 수행
4. 비용 계산
5. soft-min 가중 평균으로 nominal sequence 업데이트
6. 첫 번째 제어 입력 반환

비용 함수 구성:

- 목표점 평균 거리 비용
- 최종 목표 거리 비용
- 최종 yaw 정렬 비용
- 장애물 근접/관통 비용
- 제어 입력 크기 비용
- 제어 변화율 비용

즉, 이 제어기는 "목표로 가되, 장애물에 가까이 가지 않고, 제어 입력이 너무 크거나 급격히 변하지 않도록" 설계되어 있다.

### 6.4 ROS 2 노드 영역

클래스:

- `MPPIOffboardNode`

이 노드는 다음 책임을 모두 가진다.

- 파라미터 선언 및 로드
- 상태 수신
- 제어 명령 발행
- 오프보드 모드 요청
- arm/disarm 요청
- 상태 머신 운영

## 7. MPPI 노드의 입출력 인터페이스

### 입력

구독 토픽:

- `/mavros/state`
- `/mavros/local_position/pose`

입력 의미:

- 연결 여부
- arm 여부
- 현재 모드
- 현재 위치
- 현재 yaw

### 출력

발행 토픽:

- `/mavros/setpoint_velocity/cmd_vel`

출력 의미:

- `linear.x = vx`
- `linear.y = vy`
- `linear.z = vz`
- `angular.z = yaw_rate`

### 서비스 호출

- `/mavros/set_mode`
- `/mavros/cmd/arming`

## 8. 미션 상태 머신

`MPPIOffboardNode`는 phase 기반 상태 머신으로 움직인다.

```text
WAIT_STREAM
   ↓
OFFBOARD_ARM
   ↓
TAKEOFF
   ↓
HOVER_AFTER_TAKEOFF
   ↓
MPPI_GO
   ↓
HOVER_AT_GOAL
   ↓
LAND
   ↓
WAIT_LANDED
   ↓
DONE
```

상태 머신과 주요 인터페이스 호출을 시퀀스로 보면 아래와 같다.

```text
MPPI Node                     MAVROS                        PX4 SITL                     Gazebo
   |                            |                              |                           |
   | WAIT_STREAM                |                              |                           |
   | publish zero cmd_vel       |                              |                           |
   |--------------------------->| setpoint_velocity            |                           |
   |                            |----------------------------->| receive setpoint          |
   |                            |                              | keep offboard stream      |
   |                            |                              |                           |
   | OFFBOARD_ARM               |                              |                           |
   | set_mode(OFFBOARD)         |                              |                           |
   |--------------------------->| /mavros/set_mode             |                           |
   |                            |----------------------------->| switch mode               |
   | arm(true)                  |                              |                           |
   |--------------------------->| /mavros/cmd/arming           |                           |
   |                            |----------------------------->| arm motors                |
   |                            |                              |                           |
   | TAKEOFF                    |                              |                           |
   | publish vz > 0             |                              |                           |
   |--------------------------->| cmd_vel                      |                           |
   |                            |----------------------------->| ascend                    |
   |                            |                              |-------------------------->|
   |                            |                              | local pose updates        |
   |<---------------------------| /mavros/local_position/pose  |<--------------------------|
   |                            |                              |                           |
   | HOVER_AFTER_TAKEOFF        |                              |                           |
   | publish hold cmd           |                              |                           |
   |--------------------------->| cmd_vel                      |                           |
   |                            |----------------------------->| hover                     |
   |                            |                              |                           |
   | MPPI_GO                    |                              |                           |
   | compute vx, vy, yaw_rate   |                              |                           |
   | publish cmd_vel            |                              |                           |
   |--------------------------->| setpoint_velocity            |                           |
   |                            |----------------------------->| move to goal              |
   |                            |                              |-------------------------->|
   |<---------------------------| pose/state feedback          |<--------------------------|
   |                            |                              |                           |
   | HOVER_AT_GOAL              |                              |                           |
   | publish hold cmd           |                              |                           |
   |--------------------------->| cmd_vel                      |                           |
   |                            |----------------------------->| hover                     |
   |                            |                              |                           |
   | LAND                       |                              |                           |
   | set_mode(AUTO.LAND)        |                              |                           |
   |--------------------------->| /mavros/set_mode             |                           |
   |                            |----------------------------->| auto land                 |
   |                            |                              | descend                   |
   |                            |                              |-------------------------->|
   |<---------------------------| pose feedback                |<--------------------------|
   |                            |                              |                           |
   | WAIT_LANDED                |                              |                           |
   | arm(false) if needed       |                              |                           |
   |--------------------------->| /mavros/cmd/arming           |                           |
   |                            |----------------------------->| disarm                    |
   |                            |                              |                           |
   | DONE                       |                              |                           |
```

### `WAIT_STREAM`

의미:

- PX4가 OFFBOARD 모드에 들어갈 수 있도록 setpoint를 선발행

### `OFFBOARD_ARM`

의미:

- OFFBOARD 모드 요청
- ARM 요청

### `TAKEOFF`

의미:

- 목표 고도까지 상승

특징:

- MPPI가 아니라 Z축 P 제어 사용

### `HOVER_AFTER_TAKEOFF`

의미:

- 이륙 직후 자세와 높이를 잠시 안정화

### `MPPI_GO`

의미:

- 본격적인 MPPI 이동 구간

특징:

- `(x, y, yaw)` -> `(goal_x, goal_y, goal_yaw)` 기준으로 속도 명령 계산
- 장애물 정보를 비용 함수에 반영

### `HOVER_AT_GOAL`

의미:

- 목표점 도달 후 잠시 정지

### `LAND`

의미:

- PX4에 `AUTO.LAND` 요청

### `WAIT_LANDED`

의미:

- 착륙 감시 후 필요 시 disarm

### `DONE`

의미:

- 전체 미션 종료

## 9. 장애물 정보 처리

장애물 정보는 launch에서 파라미터로 전달된다.

관련 파일:

- [src/mppi/launch/mppi.launch.py](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)

파라미터:

- `obs_x`
- `obs_y`
- `obs_r`

노드 내부에서는 이 값이 `Obstacle2D` 리스트로 변환된다.

주의:

- Gazebo world의 장애물 배치와 launch 파라미터는 반드시 일치해야 한다.

이 부분이 어긋나면 다음 문제가 생긴다.

- 시뮬레이터 상 장애물을 안 피함
- 실제로는 없는 장애물을 피하려 함
- 경로가 비정상적으로 휘어짐

## 10. `offboard_control` 노드의 의미

관련 파일:

- [src/offboard_control/offboard_control/offboard_takeoff_node.py](/home/deepblue/AV_Drone/src/offboard_control/offboard_control/offboard_takeoff_node.py)

이 노드는 가장 단순한 오프보드 예제다.

절차:

1. `/mavros/setpoint_position/local`에 위치 목표 발행
2. 일정 시간 선발행
3. arm 요청
4. OFFBOARD 요청

이 노드는 아래를 빠르게 점검할 때 유용하다.

- MAVROS 설치가 정상인가
- PX4와 통신이 붙는가
- OFFBOARD 진입이 되는가

즉, 복잡한 MPPI 미션 이전의 최소 기능 테스트다.

## 11. Docker 파일과 코드의 관계

### Compose

관련 파일:

- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)

역할:

- 어떤 컨테이너를 띄울지 정의
- 환경 변수 전달
- X11 소켓 마운트
- 프로젝트 디렉터리 bind mount

### ROS Dockerfile

관련 파일:

- [docker/ros/Dockerfile](/home/deepblue/AV_Drone/docker/ros/Dockerfile)

역할:

- ROS 2 Humble 설치
- MAVROS 설치
- 빌드 유틸 설치

### SIM Dockerfile

관련 파일:

- [docker/sim/Dockerfile](/home/deepblue/AV_Drone/docker/sim/Dockerfile)

역할:

- PX4 개발 이미지 기반 환경
- PX4 소스 준비
- Gazebo 실행 기반 제공

## 12. 실행 시퀀스 전체 요약

```text
1. docker compose up -d sim
   -> PX4 SITL + Gazebo 실행

2. docker compose up -d ros
   -> ROS 2 작업 컨테이너 실행

3. docker compose exec ros bash
   -> ROS 컨테이너 진입

4. colcon build
   -> mppi / offboard_control 빌드

5. ros2 launch mppi mppi.launch.py
   -> MAVROS 실행
   -> MPPI node 실행
   -> PX4와 연결
   -> 상태 머신 진행
   -> takeoff -> move -> land
```

실행 명령 기준의 요약 시퀀스는 아래처럼 볼 수도 있다.

```text
[Host]
  docker compose build
      ↓
  docker compose up -d sim
      ↓
  docker compose up -d ros
      ↓
  docker compose exec ros bash
      ↓
[ros container]
  source /opt/ros/humble/setup.bash
      ↓
  colcon build
      ↓
  source install/setup.bash
      ↓
  ros2 launch mppi mppi.launch.py
      ↓
  MAVROS starts
      ↓
  MPPI node starts
      ↓
  PX4 heartbeat connected
      ↓
  OFFBOARD_ARM -> TAKEOFF -> MPPI_GO -> LAND -> DONE
```

## 13. 수정 포인트 가이드

처음 코드를 수정하려는 사람 입장에서 어떤 파일을 건드리면 무엇이 바뀌는지 정리하면 다음과 같다.

### 비행 목표를 바꾸고 싶을 때

수정 파일:

- [src/mppi/launch/mppi.launch.py](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)

수정 대상:

- `goal_x`
- `goal_y`
- `goal_z`
- `goal_yaw`

### 장애물 배치를 바꾸고 싶을 때

수정 파일:

- [src/mppi/launch/mppi.launch.py](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)

수정 대상:

- `obs_x`
- `obs_y`
- `obs_r`

주의:

- Gazebo world와 반드시 같이 맞춰야 한다.

### MPPI 튜닝을 바꾸고 싶을 때

수정 파일:

- [src/mppi/mppi/mppi_node.py](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)

주요 대상:

- `dt`
- `horizon`
- `num_samples`
- `v_max`
- `yaw_rate_max`
- `w_obst`
- `safety_margin`
- `near_buffer`

### MAVROS 동작을 조정하고 싶을 때

수정 파일:

- [src/mppi/config/mavros_config.yaml](/home/deepblue/AV_Drone/src/mppi/config/mavros_config.yaml)
- [src/mppi/config/mavros_pluginlists.yaml](/home/deepblue/AV_Drone/src/mppi/config/mavros_pluginlists.yaml)

### Docker 동작을 조정하고 싶을 때

수정 파일:

- [docker-compose.yml](/home/deepblue/AV_Drone/docker-compose.yml)
- [docker/ros/Dockerfile](/home/deepblue/AV_Drone/docker/ros/Dockerfile)
- [docker/sim/Dockerfile](/home/deepblue/AV_Drone/docker/sim/Dockerfile)

## 14. 관련 문서

- 메인 문서: [README.md](/home/deepblue/AV_Drone/README.md)
- 환경 명세: [docs/docker-environment-spec.md](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)
- 명령어 레퍼런스: [docs/command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
