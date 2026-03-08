# Multi-Drone Development Roadmap

이 문서는 현재 `/home/deepblue/AV_Drone`의 단일 드론 MPPI 시뮬레이션 구조를, 최종 목표인 `4대 드론 + 센서 기반 장애물 인지 + 자율주행` 구조로 확장하기 위한 설계 방향과 개발 로드맵을 정리한 문서다.

대상 독자:

- 현재 코드를 처음 인수받은 사람
- 단일 드론 구조를 멀티드론 구조로 바꾸려는 개발자
- Gazebo 센서를 붙여 인지 파이프라인을 추가하려는 사람
- 구현 순서를 정해야 하는 팀

## 현재 구현 상태

현재 저장소에는 로드맵 전체가 구현된 것은 아니지만, 다음 기초공사가 반영되어 있다.

- `drone_bringup`: 단일 드론 자율주행 launch/YAML
- `drone_control`: MAVROS vehicle interface와 autonomy manager
- `drone_perception`: LiDAR nearest obstacle scaffold
- `drone_planning`: obstacle-aware local planner scaffold
- `drone_safety`: timeout/emergency-stop fail-safe scaffold
- `drone_metrics`: artifacts 기반 metrics logger scaffold

즉, 현재 상태는 `단일 드론 + 센서 자율주행 파이프라인의 뼈대`까지는 들어간 상태고, 실제 Gazebo 센서 모델 연결과 planner 고도화가 다음 단계다.

추가 메모:

- 라이다 연동 과정에서 `sim` 컨테이너를 `focal`에서 `jammy` 기준으로 옮기는 작업을 진행 중이다.
- 변경 배경과 세부 내용은 [change.md](/home/deepblue/AV_Drone/docs/change.md)에 따로 정리했다.

## 지금 바로 다음에 할 일

로드맵 관점에서 현재 가장 먼저 끝내야 하는 일은 아래다.

1. `/drone1/scan` 발행 확인
2. perception/safety 파이프라인 정상 동작 확인
3. `single_drone_autonomy`에 takeoff/offboard 로직 추가
4. 단순 obstacle world 추가
5. 센서 기반 회피 성능을 metrics로 기록

## 1. 최종 목표 정의

최종 목표는 다음과 같다.

- Gazebo 상에서 드론 4대를 동시에 띄운다.
- 각 드론은 독립적인 상태, 제어, 센서 입력을 가진다.
- 각 드론은 센서를 통해 장애물을 인식한다.
- 인식된 정보를 바탕으로 경로 계획 및 회피를 수행한다.
- 드론별 자율주행 노드는 서로 간섭하지 않고 독립적으로 동작한다.
- 필요하면 드론 간 협업 또는 충돌 회피까지 확장 가능해야 한다.

즉, 현재의 "단일 기체 + 하드코딩 장애물 파라미터" 구조에서 다음 단계로 가야 한다.

- `단일 드론` -> `멀티드론`
- `정적 장애물 파라미터 기반` -> `센서 기반 장애물 인지`
- `단일 노드 통합 구조` -> `역할별 패키지 분리 구조`

## 2. 현재 구조의 한계

현재 구조는 단일 기체 데모에는 충분하지만, 4대 멀티드론 구조로 가기에는 아래 한계가 있다.

### 2.1 노드와 미션이 강하게 결합되어 있음

현재 [`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py)는 다음 역할을 한 파일 안에서 모두 처리한다.

- 미션 상태 머신
- MAVROS 입출력
- MPPI 알고리즘
- 장애물 파라미터 처리

문제:

- 드론 수가 늘어나면 노드 재사용성이 떨어진다.
- 센서 인지 노드를 별도로 붙이기 어렵다.
- 제어 로직과 인지 로직이 분리되어 있지 않다.

### 2.2 단일 토픽 네임스페이스 기준

현재는 `/mavros/...` 토픽을 직접 사용한다.

문제:

- 4대 드론이 동시에 뜨면 토픽 충돌이 발생한다.
- 드론별 네임스페이스 구분이 필요하다.

예를 들어 최종적으로는 아래처럼 가야 한다.

- `/drone1/mavros/state`
- `/drone2/mavros/state`
- `/drone3/mavros/state`
- `/drone4/mavros/state`

### 2.3 장애물 정보가 센서가 아니라 launch 파라미터에 고정됨

현재 장애물은 [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)에 `obs_x`, `obs_y`, `obs_r`로 직접 넣는다.

문제:

- Gazebo world가 바뀌면 코드도 같이 수정해야 한다.
- 센서 인지 기반 자율주행으로 확장할 수 없다.
- 동적 장애물이나 미지 환경 대응이 불가능하다.

### 2.4 단일 드론 launch 구조

현재 launch는 한 개의 MAVROS와 한 개의 MPPI 노드를 띄운다.

문제:

- 4대 드론을 띄우려면 launch 구조 자체가 다중 인스턴스형으로 바뀌어야 한다.

## 3. 목표 구조 제안

최종적으로는 아래 구조를 권장한다.

```text
src/
├─ common_interfaces/
│  ├─ msg/
│  ├─ srv/
│  └─ package.xml
├─ drone_bringup/
│  ├─ launch/
│  ├─ config/
│  └─ package.xml
├─ drone_control/
│  ├─ drone_control/
│  │  ├─ offboard_manager.py
│  │  ├─ mission_manager.py
│  │  └─ vehicle_interface.py
│  └─ package.xml
├─ drone_planning/
│  ├─ drone_planning/
│  │  ├─ mppi_controller.py
│  │  ├─ local_planner_node.py
│  │  └─ planner_utils.py
│  └─ package.xml
├─ drone_perception/
│  ├─ drone_perception/
│  │  ├─ lidar_processor.py
│  │  ├─ depth_processor.py
│  │  ├─ obstacle_tracker.py
│  │  └─ perception_node.py
│  └─ package.xml
├─ drone_sim/
│  ├─ worlds/
│  ├─ models/
│  ├─ sensors/
│  └─ package.xml
└─ offboard_control/
   └─ ...
```

핵심 아이디어는 역할 분리다.

- `drone_bringup`: 멀티드론 launch와 파라미터 묶음
- `drone_control`: PX4/MAVROS 인터페이스와 오프보드 상태 관리
- `drone_planning`: MPPI와 local planner
- `drone_perception`: 센서 데이터 처리와 장애물 추출
- `drone_sim`: Gazebo world, 모델, 센서 정의
- `common_interfaces`: 노드 간 메시지 타입 정의

## 4. 현재 파일 구조를 어떻게 바꿔야 하는가

현재에서 바로 크게 갈아엎기보다, 아래 순서로 점진적으로 바꾸는 편이 안전하다.

### 4.1 1단계: `mppi_node.py` 분리

현재:

- [`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py) 하나에 모든 기능이 들어 있음

권장 분리:

```text
src/mppi/mppi/
├─ mppi_node.py
├─ mppi_controller.py
├─ mission_state_machine.py
├─ vehicle_interface.py
└─ obstacle_provider.py
```

각 파일 역할:

- `mppi_controller.py`
  - MPPI 알고리즘만 담당
- `mission_state_machine.py`
  - WAIT_STREAM, OFFBOARD_ARM 등 phase 관리
- `vehicle_interface.py`
  - `/mavros/...` 토픽/서비스 래핑
- `obstacle_provider.py`
  - 현재는 launch 파라미터 기반
  - 이후 센서 기반으로 대체 가능
- `mppi_node.py`
  - 위 모듈들을 묶는 orchestration 역할

이 단계의 목적:

- 코드 읽기 쉬움
- 센서 기반 인지 추가 준비
- 멀티 인스턴스화 준비

### 4.2 2단계: 네임스페이스 기반 구조로 전환

현재는 `/mavros/state`처럼 절대 토픽 이름을 직접 사용한다.

멀티드론 구조에서는 반드시 네임스페이스를 써야 한다.

예시:

- `/drone1/mavros/state`
- `/drone2/mavros/state`
- `/drone3/mavros/state`
- `/drone4/mavros/state`

즉, 코드에서 아래를 바꿔야 한다.

- 하드코딩된 `/mavros/...` 토픽 이름
- launch에서 단일 `namespace="mavros"`로 고정한 부분

권장 방식:

- 드론별 namespace를 launch argument로 받는다.
- `vehicle_interface.py`에서 namespace 기반으로 토픽을 조합한다.

예시 개념:

```text
drone_ns = "/drone1"
state_topic = f"{drone_ns}/mavros/state"
pose_topic = f"{drone_ns}/mavros/local_position/pose"
```

### 4.3 3단계: launch를 멀티 인스턴스형으로 재구성

현재 [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py)는 단일 MAVROS + 단일 MPPI 노드 구조다.

최종적으로는 아래 형태가 필요하다.

```text
multi_drone.launch.py
├─ drone1 mavros
├─ drone1 controller
├─ drone1 perception
├─ drone2 mavros
├─ drone2 controller
├─ drone2 perception
├─ drone3 mavros
├─ drone3 controller
├─ drone3 perception
├─ drone4 mavros
├─ drone4 controller
└─ drone4 perception
```

권장 파일 구조:

```text
src/drone_bringup/launch/
├─ single_drone.launch.py
├─ multi_drone.launch.py
├─ mavros_instance.launch.py
└─ perception_instance.launch.py
```

## 5. Gazebo에서 센서를 쓰려면 어떻게 해야 하는가

센서 기반 자율주행을 하려면 Gazebo world에 센서가 붙은 드론 모델이 필요하다.

핵심은 "드론 모델 SDF/URDF/Xacro 안에 센서 plugin을 추가하고, 그 센서 토픽을 ROS 2 쪽에서 읽는 것"이다.

### 5.1 사용할 수 있는 대표 센서

멀티드론 자율주행 목적이라면 보통 아래 센서 중 하나 또는 조합을 쓴다.

- 2D LiDAR
- 3D LiDAR
- Depth camera
- RGB camera
- IMU
- Optical flow

현재 목표가 "장애물 인지 + 회피"라면 가장 현실적인 시작점은 아래 둘이다.

- `2D/3D LiDAR`
- `Depth camera`

이유:

- 장애물까지의 거리 정보를 직접 얻기 쉽다.
- 점군 또는 depth map 기반 장애물 추출이 가능하다.

### 5.2 Gazebo에서 센서를 붙이는 방법

센서는 일반적으로 드론 모델의 SDF 안에 붙인다.

개념적으로는 아래와 같은 구조다.

```text
model.sdf
└─ link
   └─ sensor
      ├─ lidar or camera
      └─ gazebo plugin
```

예를 들어 depth camera나 lidar를 붙이면 Gazebo가 다음 종류의 데이터를 발행하게 된다.

- laser scan
- point cloud
- image
- depth image

### 5.3 ROS 2에서 센서 데이터를 받는 경로

센서를 붙였다고 자동으로 MAVROS가 읽어주는 것은 아니다.

경로는 보통 다음 둘 중 하나다.

#### 방법 A. Gazebo -> ROS 2 토픽 브리지

Gazebo plugin 또는 bridge를 통해 ROS 2 토픽으로 직접 전달

예:

- `/drone1/scan`
- `/drone1/depth/image_raw`
- `/drone1/points`

장점:

- 인지 노드가 바로 ROS 토픽을 읽으면 됨

#### 방법 B. Gazebo sensor -> PX4 -> MAVROS

일부 센서는 PX4 SITL 경유로 들어갈 수도 있지만, 장애물 인지용 고수준 센서는 보통 ROS 2 perception 노드가 직접 받는 구조가 낫다.

권장:

- 고수준 인지 센서는 ROS 2 perception 노드가 직접 처리
- MAVROS는 비행 상태와 제어용 브리지 역할에 집중

### 5.4 센서 적용 권장 구조

드론 4대 구조에서 센서는 아래처럼 가는 것이 좋다.

```text
Gazebo sensor topic
   -> drone_perception node
   -> obstacle list / occupancy / local map
   -> drone_planning node
   -> velocity setpoint
   -> MAVROS
   -> PX4
```

즉, 장애물 인지 결과를 planner가 사용할 수 있는 형태로 바꿔주는 perception 계층이 필요하다.

## 6. 멀티드론에서 필요한 핵심 설계 포인트

### 6.1 네임스페이스 분리

반드시 필요하다.

예시:

- `/drone1/mavros/state`
- `/drone1/scan`
- `/drone1/planner/obstacles`
- `/drone1/cmd/velocity`

드론 2, 3, 4도 같은 패턴으로 분리한다.

### 6.2 파라미터 파일 분리

드론별로 아래 파라미터가 달라질 수 있다.

- spawn 위치
- goal 위치
- MAVROS UDP 포트
- 센서 토픽 이름
- planner 파라미터

따라서 드론별 YAML이 필요하다.

예시:

```text
config/
├─ drone1.yaml
├─ drone2.yaml
├─ drone3.yaml
└─ drone4.yaml
```

### 6.3 포트 분리

PX4 SITL과 MAVROS를 4대 동시에 띄우려면 UDP 포트도 분리되어야 한다.

즉, 드론별로 아래가 달라져야 한다.

- FCU URL
- MAVLink 포트
- SITL instance 번호

### 6.4 spawn 위치 분리

4대 드론을 같은 좌표에 spawn 하면 시뮬레이션 시작부터 충돌한다.

따라서 launch 단계에서 드론별 초기 위치를 분리해야 한다.

예시:

- drone1: `(0, 0, 0)`
- drone2: `(2, 0, 0)`
- drone3: `(0, 2, 0)`
- drone4: `(2, 2, 0)`

## 7. 권장 최종 패키지 구조

아래 구조가 장기적으로 가장 관리하기 쉽다.

```text
src/
├─ common_interfaces/
│  ├─ msg/
│  │  ├─ Obstacle.msg
│  │  ├─ ObstacleArray.msg
│  │  └─ LocalPlan.msg
│  ├─ srv/
│  └─ package.xml
├─ drone_bringup/
│  ├─ launch/
│  │  ├─ single_drone.launch.py
│  │  ├─ multi_drone.launch.py
│  │  └─ drone_instance.launch.py
│  ├─ config/
│  │  ├─ drone1.yaml
│  │  ├─ drone2.yaml
│  │  ├─ drone3.yaml
│  │  └─ drone4.yaml
│  └─ package.xml
├─ drone_control/
│  ├─ drone_control/
│  │  ├─ vehicle_interface.py
│  │  ├─ offboard_manager.py
│  │  ├─ mission_manager.py
│  │  └─ safety_manager.py
│  └─ package.xml
├─ drone_perception/
│  ├─ drone_perception/
│  │  ├─ lidar_processor.py
│  │  ├─ depth_processor.py
│  │  ├─ obstacle_fusion.py
│  │  └─ perception_node.py
│  └─ package.xml
├─ drone_planning/
│  ├─ drone_planning/
│  │  ├─ mppi_controller.py
│  │  ├─ local_planner_node.py
│  │  ├─ global_goal_manager.py
│  │  └─ costmap_builder.py
│  └─ package.xml
├─ drone_sim/
│  ├─ worlds/
│  ├─ models/
│  ├─ sensors/
│  └─ package.xml
└─ offboard_control/
```

## 8. 개발 로드맵

구현은 한 번에 하지 말고 아래 단계로 나누는 것이 좋다.

### Phase 0. 현재 단일 드론 구조 안정화

목표:

- 현재 Docker 실행 흐름 완전 고정
- MPPI 단일 기체 미션 안정성 확인
- 장애물 회피 데모 재현 가능 상태 확보

완료 기준:

- 문서대로 항상 재현 가능
- `OFFBOARD_ARM -> DONE`까지 안정 실행

### Phase 1. 코드 구조 분리

목표:

- `mppi_node.py`를 역할별 모듈로 분리

해야 할 일:

- MPPI 알고리즘 분리
- vehicle interface 분리
- state machine 분리
- obstacle provider 분리

완료 기준:

- 기능 변화 없이 코드 구조만 정리
- 단일 드론 실행 동일하게 유지

### Phase 2. 멀티드론 네임스페이스 구조 도입

목표:

- 드론별 namespace와 포트 분리

해야 할 일:

- launch argument로 `drone_name`, `namespace`, `fcu_url` 받기
- 토픽 하드코딩 제거
- 단일 드론 launch를 multi-instance 가능하게 수정

완료 기준:

- 드론 2대까지 동시에 launch 성공
- 각 드론의 `/droneX/mavros/state` 분리 확인

### Phase 3. 4대 드론 동시 기동

목표:

- Gazebo에서 4대 드론 spawn
- 각 드론 PX4/MAVROS 독립 연결

해야 할 일:

- 드론별 spawn 위치 설정
- 드론별 SITL instance 분리
- 드론별 MAVROS instance 분리

완료 기준:

- 4대 모두 arm/offboard 가능
- 각 드론 위치 토픽 독립 수신 가능

### Phase 4. 센서 모델 추가

목표:

- 각 드론 모델에 장애물 인지용 센서 탑재

권장 시작 센서:

- 2D LiDAR 또는 depth camera

해야 할 일:

- Gazebo model/SDF에 센서 추가
- 센서 토픽 확인
- ROS 2에서 센서 데이터 수신 확인

완료 기준:

- `/droneX/scan` 또는 `/droneX/depth/...` 토픽 확인

### Phase 5. Perception 패키지 도입

목표:

- 센서 raw data를 obstacle representation으로 변환

해야 할 일:

- lidar/depth processor 작성
- obstacle extraction 작성
- obstacle topic/message 정의

완료 기준:

- planner가 사용할 obstacle array 출력

### Phase 6. Planner와 Perception 연결

목표:

- 하드코딩 obstacle 대신 perception 결과 사용

해야 할 일:

- `obs_x/obs_y/obs_r` 제거 또는 fallback화
- planner가 perception topic 구독
- local map / obstacle list 기반 회피 구현

완료 기준:

- Gazebo 장애물을 센서로 인식하고 회피

### Phase 7. 멀티드론 충돌 회피

목표:

- 드론끼리도 서로 장애물처럼 고려

해야 할 일:

- 타 드론 위치를 obstacle로 취급
- inter-drone safety margin 정의
- 분산형 또는 중앙형 정책 결정

완료 기준:

- 4대 동시 비행 중 상호 충돌 없이 이동

### Phase 8. 고급 기능

확장 후보:

- 전역 경로 계획기 추가
- 동적 장애물 추적
- 임무 할당
- 군집 비행
- 중앙 관제 노드

## 9. 구현 우선순위 제안

가장 현실적인 우선순위는 아래다.

1. 단일 드론 코드 분리
2. 네임스페이스/포트 기반 멀티드론 구조
3. 2대 동시 실행
4. 4대 동시 실행
5. 센서 부착
6. perception node 작성
7. planner와 perception 연동
8. 드론 간 충돌 회피

이 순서가 좋은 이유:

- 구조 문제를 먼저 해결하지 않으면 센서와 멀티드론이 같이 꼬인다.
- perception을 넣기 전에 namespace와 launch 구조를 먼저 고쳐야 한다.
- 4대를 한 번에 하기보다 2대에서 먼저 구조를 검증하는 편이 안전하다.

## 10. 지금 당장 바꾸면 좋은 것

현재 코드 기준으로 가장 먼저 손봐야 할 지점은 아래다.

### 필수

- [`src/mppi/mppi/mppi_node.py`](/home/deepblue/AV_Drone/src/mppi/mppi/mppi_node.py) 분리
- `/mavros/...` 하드코딩 제거
- [`src/mppi/launch/mppi.launch.py`](/home/deepblue/AV_Drone/src/mppi/launch/mppi.launch.py) 단일 인스턴스 구조 개선

### 다음 단계

- 드론별 config YAML 도입
- perception 패키지 초안 생성
- `drone_bringup` 패키지 생성

## 11. 문서 결론

현재 저장소는 "단일 드론 MPPI 데모"로는 충분히 동작한다.  
하지만 최종 목표가 `4대 멀티드론 + 센서 기반 자율주행`이라면, 단순 파라미터 추가 수준이 아니라 구조를 역할별로 재편해야 한다.

핵심 방향은 세 가지다.

- 단일 노드 통합 구조를 분리 구조로 바꾼다.
- 단일 `/mavros/...` 토픽 구조를 드론별 namespace 구조로 바꾼다.
- 하드코딩 장애물 정보를 센서 기반 인지 파이프라인으로 교체한다.

이 문서를 기준으로 구현을 시작하면, 현재 코드에서 어떤 부분을 유지하고 어떤 부분을 갈아엎어야 하는지 방향을 잡기 쉽다.
