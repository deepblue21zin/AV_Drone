# AV_Drone

PX4 SITL, ROS 2 Humble, Gazebo Classic, MAVROS, MPPI를 결합한 드론 시뮬레이션 프로젝트다.  
이 저장소는 Docker 기반으로 실행 환경을 고정하고, MAVROS를 통해 PX4와 ROS 2 노드를 연결한다.

이 README는 단순 실행법만 적는 문서가 아니라, 이 프로젝트를 처음 보는 사람이 아래 내용을 이해할 수 있도록 작성했다.

- 프로젝트가 어떤 구조로 이루어져 있는지
- 각 파일이 어떤 역할을 하는지
- 실행하면 내부적으로 어떤 순서로 동작하는지
- 어떤 프로세스가 어떤 토픽과 서비스로 연결되는지
- Docker 환경에서 어떻게 재현 가능하게 운영하는지

## 프로젝트 개요

이 프로젝트의 목표는 다음과 같다.

- PX4 SITL 상에서 드론을 시뮬레이션한다.
- ROS 2 노드에서 오프보드 제어를 수행한다.
- MPPI 기반 경로 추종 및 장애물 회피를 검증한다.
- 로컬 개발 환경 차이를 줄이기 위해 Docker Compose로 실행 환경을 표준화한다.

핵심 구성 요소는 다음과 같다.

- `PX4 SITL`: 비행 제어기 시뮬레이션
- `Gazebo Classic`: 물리 시뮬레이터 및 시각화
- `ROS 2 Humble`: 제어 노드 실행 환경
- `MAVROS`: PX4와 ROS 2 간 브리지
- `MPPI`: 목표점 이동 및 장애물 회피 제어

한 줄로 요약하면 다음과 같다.

`Gazebo` 안에서 드론을 띄우고, `PX4 SITL`이 비행 제어기를 흉내 내며, `MAVROS`가 PX4와 ROS 2를 연결하고, 이 저장소의 `MPPI 노드`가 속도 명령을 만들어 드론을 목표 지점까지 이동시키는 구조다.

## 전체 아키텍처

이 프로젝트의 런타임 아키텍처는 아래처럼 이해하면 된다.

```text
┌─────────────────────────────────────────────────────────────────┐
│ Host: Ubuntu 22.04                                             │
│ ├─ Docker Compose                                              │
│ │  ├─ sim container                                            │
│ │  │  ├─ PX4 SITL                                              │
│ │  │  └─ Gazebo Classic                                        │
│ │  └─ ros container                                            │
│ │     ├─ ROS 2 Humble                                          │
│ │     ├─ MAVROS                                                │
│ │     ├─ mppi package                                          │
│ │     └─ offboard_control package                              │
│ └─ X11                                                         │
│    └─ Gazebo GUI rendering                                     │
└─────────────────────────────────────────────────────────────────┘
```

데이터 흐름은 아래와 같다.

```text
┌─────────────────┐      MAVLink/UDP       ┌─────────────────┐
│   Gazebo        │ <--------------------> │    PX4 SITL     │
│   Classic       │   sensor/physics loop  │  flight logic   │
└────────┬────────┘                         └────────┬────────┘
         │                                           │
         │                                           │
         │                                 MAVLink bridge
         │                                           │
         │                                           v
         │                                ┌─────────────────┐
         │                                │     MAVROS      │
         │                                │ ROS2 <-> MAVLink│
         │                                └────────┬────────┘
         │                                         │
         │                                         │ ROS 2 topics/services
         │                                         │
         │                                         v
         │                                ┌─────────────────┐
         └──────────────────────────────> │    MPPI Node    │
                  state feedback          │ mission + ctrl  │
                                          └─────────────────┘
```

좀 더 구체적으로 보면 각 구성 요소의 입출력은 다음과 같다.

```text
Gazebo Classic
  - 드론 모델, 물리, world, 충돌 처리
  - 결과적으로 드론의 위치/자세 변화를 만든다

PX4 SITL
  - Gazebo 상태를 기반으로 비행 제어 로직 수행
  - MAVLink로 상태 송신
  - MAVLink로 명령 수신

MAVROS
  - PX4 상태를 ROS 2 topic/service로 변환
  - ROS 2 명령을 PX4용 MAVLink로 변환

MPPI Node
  - /mavros/state 구독
  - /mavros/local_position/pose 구독
  - /mavros/setpoint_velocity/cmd_vel 발행
  - /mavros/set_mode, /mavros/cmd/arming 서비스 호출
```

즉, 제어 루프의 핵심은 다음이다.

1. PX4가 현재 비행 상태를 계산한다.
2. MAVROS가 이를 ROS 2 토픽으로 전달한다.
3. MPPI 노드가 현재 위치와 목표 위치를 바탕으로 속도 명령을 계산한다.
4. MAVROS가 그 속도 명령을 PX4에 다시 전달한다.
5. PX4가 Gazebo 상의 드론을 움직인다.

## 저장소 구조

주요 패키지는 아래와 같다.

- [`src/mppi`](/home/deepblue/AV_Drone/src/mppi)
- [`src/offboard_control`](/home/deepblue/AV_Drone/src/offboard_control)

주요 실행 파일:

- [`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)
- [`src/offboard_control/offboard_control/offboard_takeoff_node.py`](/home/deepblue/AV_Drone/src/offboard_control/offboard_control/offboard_takeoff_node.py)
- [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)

MAVROS 기준 주요 토픽과 서비스:

- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/setpoint_velocity/cmd_vel`
- `/mavros/set_mode`
- `/mavros/cmd/arming`

즉, 이 저장소는 `px4_ros_com` 기반이 아니라 `MAVROS` 기반 오프보드 제어 구조다.

### 파일 구조 설명

처음 보는 사람이 보면 가장 헷갈리는 부분은 "어디가 실행 파일이고, 어디가 설정 파일인지"다.  
이 저장소는 아래처럼 이해하면 된다.

```text
AV_Drone/
├─ src/
│  ├─ mppi/
│  │  ├─ mppi/
│  │  │  └─ mppi_node.py
│  │  ├─ launch/
│  │  │  └─ mppi.launch.py
│  │  ├─ config/
│  │  │  ├─ mavros_config.yaml
│  │  │  └─ mavros_pluginlists.yaml
│  │  ├─ package.xml
│  │  └─ setup.py
│  └─ offboard_control/
│     ├─ offboard_control/
│     │  └─ offboard_takeoff_node.py
│     ├─ launch/
│     │  └─ offboard_control.launch.py
│     ├─ package.xml
│     └─ setup.py
├─ docker/
│  ├─ ros/
│  │  ├─ Dockerfile
│  │  └─ entrypoint.sh
│  └─ sim/
│     ├─ Dockerfile
│     └─ entrypoint.sh
├─ docs/
│  ├─ docker-environment-spec.md
│  ├─ command-reference.md
│  └─ architecture.md
├─ docker-compose.yml
└─ README.md
```

각 경로의 역할은 다음과 같다.

- [`src/mppi`](/home/deepblue/AV_Drone/src/mppi): 실제 MPPI 제어 로직이 들어 있는 ROS 2 패키지
- [`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py): 이 프로젝트의 핵심 실행 코드
- [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py): MAVROS와 MPPI 노드를 함께 띄우는 launch 파일
- [`src/mppi/config/mavros_config.yaml`](/home/deepblue/AV_Drone/src/mppi/config/mavros_config.yaml): MAVROS 동작 파라미터
- [`src/mppi/config/mavros_pluginlists.yaml`](/home/deepblue/AV_Drone/src/mppi/config/mavros_pluginlists.yaml): 사용할 MAVROS plugin 목록 제어
- [`src/offboard_control`](/home/deepblue/AV_Drone/src/offboard_control): 단순 오프보드 이륙 테스트용 패키지
- [`docker/ros`](/home/deepblue/AV_Drone/docker/ros): ROS 2 작업 컨테이너 정의
- [`docker/sim`](/home/deepblue/AV_Drone/docker/sim): PX4/Gazebo 컨테이너 정의
- [`docker-compose.yml`](/home/deepblue/AV_Drone/docker-compose.yml): 전체 실행 구성

### 어떤 파일부터 보면 좋은가

처음 코드 리딩을 할 때는 아래 순서를 권장한다.

1. [`docker-compose.yml`](/home/deepblue/AV_Drone/docker-compose.yml)
2. [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)
3. [`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)
4. [`src/offboard_control/offboard_control/offboard_takeoff_node.py`](/home/deepblue/AV_Drone/src/offboard_control/offboard_control/offboard_takeoff_node.py)

이 순서로 보면:

- 전체 실행 단위가 무엇인지
- 어떤 노드가 같이 떠야 하는지
- 실제 제어 알고리즘이 어디에 있는지
- 가장 단순한 오프보드 제어 예제가 무엇인지

를 빠르게 이해할 수 있다.

## 코드 관점의 구성 설명

이 프로젝트는 크게 두 개의 ROS 2 패키지로 나뉜다.

### 1. `mppi` 패키지

[`src/mppi`](/home/deepblue/AV_Drone/src/mppi)는 실제 데모용 핵심 패키지다.

포함된 역할:

- MAVROS launch 포함
- 장애물 파라미터 정의
- MPPI 제어기 구현
- 미션 상태 머신 구현
- takeoff -> hover -> MPPI 이동 -> land 전체 시나리오 수행

중요 파일:

- [`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)
- [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)

### 2. `offboard_control` 패키지

[`src/offboard_control`](/home/deepblue/AV_Drone/src/offboard_control)는 단순한 오프보드 이륙 테스트 패키지다.

역할:

- 위치 setpoint를 주기적으로 발행
- 일정 시간 선발행 후
- ARM 요청
- OFFBOARD 모드 요청

중요 파일:

- [`src/offboard_control/offboard_control/offboard_takeoff_node.py`](/home/deepblue/AV_Drone/src/offboard_control/offboard_control/offboard_takeoff_node.py)

이 패키지는 MPPI 전체 미션을 보기 전에, "MAVROS와 PX4가 기본적으로 붙는가"를 확인하는 간단한 테스트 용도로 이해하면 된다.

## `mppi_node.py`는 내부적으로 어떻게 구성되어 있는가

[`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)는 사실상 이 프로젝트의 핵심이다.  
이 파일은 내부적으로 크게 네 덩어리로 나뉜다.

### 1. 유틸 함수

역할:

- 값 clamp
- yaw angle wrapping
- quaternion -> yaw 변환

이 부분은 제어 계산에 필요한 수학 보조 함수다.

### 2. 데이터 구조

포함 내용:

- `Obstacle2D`
- `MPPIConfig`

역할:

- 장애물 위치와 반경 정의
- MPPI 샘플링/예측/비용 가중치 파라미터 정의

즉, 이 부분은 "제어기 튜닝 값과 환경 파라미터를 담는 구조체"라고 보면 된다.

### 3. MPPI 제어기

클래스:

- `MPPIController`

핵심 역할:

- 후보 제어 입력 샘플링
- 각 후보 입력에 대해 trajectory rollout 수행
- 목표점 비용, 장애물 비용, 제어 비용 계산
- soft-min 방식으로 nominal control update
- 최종적으로 현재 시점에 적용할 첫 번째 입력 반환

출력 제어 입력은 아래 형식이다.

- `vx`
- `vy`
- `yaw_rate`

여기서 중요한 점은 이 제어기가 추력이나 자세를 직접 내보내는 것이 아니라, `속도 기반 오프보드 제어`를 수행한다는 점이다.

### 4. ROS 2 노드와 상태 머신

클래스:

- `MPPIOffboardNode`

이 클래스는 다음을 모두 담당한다.

- ROS 파라미터 로드
- MAVROS 토픽 구독
- setpoint 퍼블리시
- arm/mode 서비스 호출
- MPPI 제어기 호출
- 미션 phase 전환

즉, 제어 알고리즘과 실행 시나리오가 한 파일 안에 통합되어 있는 구조다.

## 실행하면 실제로 어떤 순서로 동작하는가

이 프로젝트를 실행하면, 내부 동작은 아래 순서로 이해하면 된다.

### 1. `sim` 컨테이너 시작

실행 대상:

- PX4 SITL
- Gazebo Classic

결과:

- Gazebo world가 뜬다.
- 가상 드론이 spawn 된다.
- PX4가 MAVLink UDP 포트를 연다.

### 2. `ros` 컨테이너에서 launch 실행

실행 명령:

```bash
ros2 launch mppi mppi.launch.py
```

이 launch는 내부적으로 두 가지를 시작한다.

1. MAVROS 노드
2. MPPI 노드

즉, `mppi.launch.py`는 단순히 MPPI만 띄우는 파일이 아니라, PX4와 붙기 위한 `MAVROS`도 같이 띄운다.

### 3. MAVROS가 PX4와 연결

MAVROS는 다음 URL로 PX4 SITL에 연결한다.

- `udp://:14540@127.0.0.1:14580`

연결이 성공하면 ROS 2에 아래 토픽들이 생긴다.

- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/setpoint_velocity/cmd_vel`
- 기타 MAVROS 관련 토픽

### 4. MPPI 노드가 초기화

MPPI 노드는 실행 직후 다음을 수행한다.

- 목표 위치 파라미터 로드
- 장애물 배열 로드
- MPPIConfig 생성
- MPPIController 생성
- `/mavros/state`와 `/mavros/local_position/pose` 구독 시작
- `/mavros/setpoint_velocity/cmd_vel` 퍼블리셔 생성
- `/mavros/set_mode`, `/mavros/cmd/arming` 서비스 클라이언트 생성

### 5. 상태 머신 시작

MPPI 노드는 내부적으로 phase 기반 상태 머신을 가진다.

순서는 다음과 같다.

1. `WAIT_STREAM`
2. `OFFBOARD_ARM`
3. `TAKEOFF`
4. `HOVER_AFTER_TAKEOFF`
5. `MPPI_GO`
6. `HOVER_AT_GOAL`
7. `LAND`
8. `WAIT_LANDED`
9. `DONE`

각 phase의 의미는 다음과 같다.

#### `WAIT_STREAM`

역할:

- PX4가 OFFBOARD 모드로 들어가기 전에 필요한 setpoint를 먼저 일정 시간 발행

이유:

- PX4는 오프보드 모드 진입 전에 setpoint 스트림이 들어와야 한다.

#### `OFFBOARD_ARM`

역할:

- PX4를 `OFFBOARD` 모드로 전환 요청
- 드론 arm 요청

이 단계가 끝나야 실제 자율 제어가 가능해진다.

#### `TAKEOFF`

역할:

- 목표 고도(`takeoff_z`)까지 상승

구현 방식:

- MPPI를 쓰지 않고 단순 Z축 P 제어로 상승

#### `HOVER_AFTER_TAKEOFF`

역할:

- 이륙 직후 바로 움직이지 않고 잠시 안정화

#### `MPPI_GO`

역할:

- 현재 위치에서 목표점으로 이동
- 장애물을 회피하면서 속도 명령 생성

핵심:

- 이 단계에서만 MPPIController가 본격적으로 사용된다.
- MPPI는 현재 `(x, y, yaw)` 상태와 목표 `(gx, gy, gyaw)`를 이용해 `vx`, `vy`, `yaw_rate`를 계산한다.

#### `HOVER_AT_GOAL`

역할:

- 목표점 도달 후 잠시 정지

#### `LAND`

역할:

- PX4에 `AUTO.LAND` 모드 요청

#### `WAIT_LANDED`

역할:

- 착륙 여부 확인
- 필요 시 disarm

#### `DONE`

역할:

- 전체 미션 종료

## 토픽과 서비스는 어떻게 연결되는가

초심자 입장에서 가장 중요한 것은 "무슨 데이터를 어디서 받고 어디로 보내는가"다.

### 구독하는 데이터

`mppi_node`는 아래를 구독한다.

- `/mavros/state`
  - PX4 연결 여부
  - 현재 모드
  - arm 상태

- `/mavros/local_position/pose`
  - 현재 위치 `(x, y, z)`
  - 현재 자세 quaternion
  - yaw 계산의 입력

### 발행하는 데이터

`mppi_node`는 아래를 발행한다.

- `/mavros/setpoint_velocity/cmd_vel`
  - `vx`
  - `vy`
  - `vz`
  - `yaw_rate`

즉, 이 프로젝트는 위치 setpoint 기반이 아니라 `속도 setpoint 기반 제어`를 수행한다.

### 호출하는 서비스

`mppi_node`는 아래 서비스를 호출한다.

- `/mavros/set_mode`
  - `OFFBOARD`
  - `AUTO.LAND`

- `/mavros/cmd/arming`
  - arm / disarm

## 장애물 정보는 어디서 오는가

장애물 정보는 현재 launch 파일의 파라미터로 전달된다.

관련 파일:

- [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)

주요 파라미터:

- `obs_x`
- `obs_y`
- `obs_r`

이 값들은 MPPI 노드 내부에서 `Obstacle2D` 리스트로 변환된다.

중요한 전제:

- Gazebo world에 배치된 장애물 위치와
- `mppi.launch.py` 안에 적힌 장애물 파라미터가
- 반드시 일치해야 한다

이 둘이 다르면 제어기는 "없는 장애물"을 피하거나 "있는 장애물"을 못 피하게 된다.

## `offboard_control` 패키지는 왜 있는가

[`src/offboard_control/offboard_control/offboard_takeoff_node.py`](/home/deepblue/AV_Drone/src/offboard_control/offboard_control/offboard_takeoff_node.py)는 구조가 단순하다.

동작 순서:

1. `/mavros/setpoint_position/local`로 위치 setpoint 발행
2. 약 2초 동안 선발행
3. arm 요청
4. OFFBOARD 모드 요청

이 패키지는 MPPI 미션 전체와는 별개로, 아래를 빠르게 점검하는 용도다.

- MAVROS가 살아 있는가
- PX4와 통신이 되는가
- 오프보드 진입이 되는가

즉, 복잡한 MPPI 전에 가장 작은 단위의 sanity check 역할을 한다.

## 기술 스택과 표준 환경

현재 표준 실행 환경은 다음과 같다.

- Host OS: `Ubuntu 22.04`
- Container base: `Ubuntu 22.04`
- ROS 2: `Humble`
- PX4: `PX4 SITL`
- Simulator: `Gazebo Classic`
- Runtime: `Docker Compose`

이 조합을 기준으로 문서와 Docker 설정을 맞춰두었다.  
관련 명세는 [`docs/docker-environment-spec.md`](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)에 따로 정리되어 있다.

## Docker 기반 아키텍처

이 프로젝트는 두 개의 컨테이너로 나누어 실행한다.

### `sim`

역할:

- PX4 SITL 실행
- Gazebo Classic 실행

### `ros`

역할:

- ROS 2 workspace 제공
- `colcon build` 수행
- MAVROS 및 제어 노드 실행

운영 원칙:

- `sim`과 `ros`는 모두 `host network`를 사용한다.
- Gazebo GUI를 위해 X11을 사용한다.
- 소스코드는 호스트에서 컨테이너로 bind mount 한다.
- `ros` 컨테이너는 상시 살아있는 작업 공간으로 두고 `exec`로 진입한다.

## 환경 세팅

### 1. Docker 설치

호스트에서는 Docker Engine과 Compose Plugin이 필요하다.

예시:

```bash
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
newgrp docker
```

동작 확인:

```bash
docker --version
docker compose version
docker run hello-world
```

### 2. Gazebo GUI를 위한 X11 준비

```bash
echo $DISPLAY
xhost +local:docker
```

확인 기준:

- `DISPLAY`가 비어 있지 않아야 한다.
- `xhost +local:docker`가 성공해야 한다.

## 실행 절차

### 1. 이미지 빌드

프로젝트 루트에서:

```bash
cd /home/deepblue/AV_Drone
docker compose build
```

첫 빌드는 오래 걸릴 수 있다.  
특히 `sim` 이미지는 PX4 소스와 Gazebo 관련 빌드 때문에 시간이 더 걸린다.

### 2. 시뮬레이터 실행

```bash
docker compose up -d sim
```

정상 기준:

- PX4가 빌드 또는 실행된다.
- Gazebo 창이 뜬다.
- 기체가 world에 spawn 된다.

로그 확인:

```bash
docker compose logs -f sim
```

### 3. ROS 작업 컨테이너 실행

```bash
docker compose up -d ros
docker compose exec ros bash
```

이제부터 ROS 관련 명령은 컨테이너 내부에서 실행한다.

### 4. ROS 환경 로드 및 빌드

컨테이너 내부:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 5. MPPI 시뮬레이션 실행

```bash
ros2 launch mppi mppi.launch.py
```

단순 이륙 테스트만 원하면:

```bash
ros2 launch offboard_control offboard_control.launch.py
```

## 정상 동작 확인

MAVROS 연결이 정상이라면 다음이 확인된다.

```bash
ros2 topic list | grep mavros
ros2 topic echo /mavros/state
ros2 topic echo /mavros/local_position/pose
```

또는 launch 로그에서 아래 메시지가 보인다.

- `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`

MPPI 미션이 정상 동작하면 상태 로그가 아래 순서로 진행된다.

- `PHASE => OFFBOARD_ARM`
- `PHASE => TAKEOFF`
- `PHASE => HOVER_AFTER_TAKEOFF`
- `PHASE => MPPI_GO`
- `PHASE => HOVER_AT_GOAL`
- `PHASE => LAND`
- `PHASE => WAIT_LANDED`
- `PHASE => DONE`

실제로는 아래 흐름이 보이면 거의 정상이라고 판단할 수 있다.

1. Gazebo 창이 뜬다.
2. `/mavros/state`가 보인다.
3. 로그에 `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`가 뜬다.
4. phase 로그가 `OFFBOARD_ARM -> TAKEOFF -> MPPI_GO -> LAND -> DONE` 순서로 진행된다.

## 경고를 줄이기 위해 반영한 사항

이 저장소에서는 실행 로그를 정리하기 위해 아래 조정을 반영했다.

### 1. ROS 파라미터 선언 정리

[`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)에서 `obs_x`, `obs_y`, `obs_r`는 `DOUBLE_ARRAY` 타입으로 명시적으로 선언했다.  
이 변경으로 ROS 2의 deprecated parameter declaration 경고를 줄였다.

### 2. MAVROS 커스텀 설정 적용

다음 파일을 추가해 MAVROS 설정을 저장소 안에서 관리한다.

- [`src/mppi/config/mavros_pluginlists.yaml`](/home/deepblue/AV_Drone/src/mppi/config/mavros_pluginlists.yaml)
- [`src/mppi/config/mavros_config.yaml`](/home/deepblue/AV_Drone/src/mppi/config/mavros_config.yaml)

적용 목적:

- `guided_target` 플러그인 비활성화
- Docker SITL 환경에서 과도한 timesync 로그 감소
- PX4 로컬 오프보드 비행에 필요한 설정만 명시

### 3. Compose 실행 방식 정리

[`docker-compose.yml`](/home/deepblue/AV_Drone/docker-compose.yml)에서 `ros`를 매번 새로 만드는 `run --rm` 중심 구조 대신, 상시 유지되는 서비스 컨테이너로 사용하도록 정리했다.

이 방식의 장점:

- 컨테이너가 계속 유지되어 디버깅이 쉽다.
- 토픽 확인과 로그 확인이 덜 헷갈린다.
- 발표나 데모 시 실행 절차가 단순해진다.

## 재현성 정리

이 프로젝트에서 재현성은 다음을 의미한다.

- 같은 OS 계열에서
- 같은 Dockerfile과 Compose 설정을 사용하고
- 같은 PX4/ROS 2 조합을 유지하면
- 동일한 시뮬레이션과 제어 흐름을 반복 실행할 수 있는 것

재현성을 높이는 요소:

- Docker 기반 실행 환경
- `Ubuntu 22.04 + ROS 2 Humble` 고정
- Compose 기반 서비스 분리
- PX4 버전 인자 고정
- 저장소 내부에서 MAVROS 설정 관리
- 문서화된 실행 순서 유지

재현성에 영향을 주는 요소:

- GPU 및 그래픽 드라이버 차이
- X11 설정 차이
- 시뮬레이션 중 CPU 부하
- 첫 PX4 빌드 시간 차이

운영 원칙:

- 호스트는 가능하면 `Ubuntu 22.04`를 유지한다.
- ROS 2는 `Humble` 기준으로 유지한다.
- PX4 버전은 Docker 설정에 명시된 값을 기준으로 유지한다.
- 설정을 바꿀 때는 Docker 파일과 문서를 함께 수정한다.

## 처음 보는 사람을 위한 추천 이해 순서

이 프로젝트를 처음 보는 사람이면 아래 순서로 이해하는 것이 가장 빠르다.

1. 이 README에서 `전체 아키텍처`, `파일 구조`, `실행 순서`를 먼저 읽는다.
2. [`docker-compose.yml`](/home/deepblue/AV_Drone/docker-compose.yml)로 실행 단위를 본다.
3. [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)에서 어떤 노드가 같이 뜨는지 본다.
4. [`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)에서 상태 머신과 제어 로직을 본다.
5. [`src/offboard_control/offboard_control/offboard_takeoff_node.py`](/home/deepblue/AV_Drone/src/offboard_control/offboard_control/offboard_takeoff_node.py)로 단순 오프보드 예제를 비교해 본다.
6. 실제로 `sim`과 `ros` 컨테이너를 띄우고 토픽을 확인한다.

이 순서로 보면 "문서 -> 구성 -> 실행 -> 코드" 흐름으로 이해할 수 있어서 가장 덜 헷갈린다.

## 종료 방법

작업 종료:

```bash
docker compose down
```

이미지까지 다시 빌드하려면:

```bash
docker compose build --no-cache
```

## 현재 확인된 상태

현재 저장소는 아래 흐름이 실제로 확인되었다.

- Docker 이미지 빌드 성공
- `sim` 컨테이너에서 PX4 SITL + Gazebo 실행 성공
- Gazebo에서 기체 spawn 확인
- `ros2 launch mppi mppi.launch.py` 실행 성공
- MAVROS가 PX4 heartbeat 수신
- MPPI 상태 머신이 `DONE`까지 진행

즉, 현재 구성은 Docker 기반으로 재현 가능한 실행 환경을 갖춘 상태다.

## 관련 문서

- 환경 명세: [docs/docker-environment-spec.md](/home/deepblue/AV_Drone/docs/docker-environment-spec.md)
- 명령어 레퍼런스: [docs/command-reference.md](/home/deepblue/AV_Drone/docs/command-reference.md)
- 아키텍처 심화 문서: [docs/architecture.md](/home/deepblue/AV_Drone/docs/architecture.md)
