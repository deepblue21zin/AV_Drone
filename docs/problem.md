# Current Problems and Resolution Notes

## 1. 목적

이 문서는 현재 프로젝트에서 실제로 막히고 있는 문제를 기록하는 작업 로그다.

목표는 다음과 같다.

- 지금 무엇이 안 되는지 명확히 남긴다.
- 어떤 로그와 artifact가 그 문제를 보여주는지 근거를 남긴다.
- 다음에 무엇을 수정해야 하는지 결정 근거를 남긴다.
- 이후 새 문제가 생기면 같은 형식으로 계속 누적 업데이트한다.

즉, 이 문서는 단순 메모가 아니라 `문제 정의 -> 증거 -> 해석 -> 해결 방향`을 정리하는 운영 문서다.

## 2. 현재 핵심 문제 요약

2026-03-20 기준으로 기존 핵심 문제 두 가지는 baseline 수준에서 해결됐다.

1. `/drone1/scan` 실수신과 obstacle world 연동이 확인됐다.
2. `goal_reached` 판정은 latch 방식으로 안정화됐고, 최신 artifact에서 `mission_phase=HOVER_AT_GOAL`, `goal_reached=true`가 확인됐다.

현재 남은 핵심 문제는 아래와 같다.

1. obstacle world는 동작하지만 아직 연구용 시나리오로는 단순하다.
2. Gazebo GUI는 호스트 X11 권한 상태에 따라 headless로 실행될 수 있다.
3. multi-UAV, failure injection, task reallocation은 아직 미구현이다.

즉, 현재 상태는 아래처럼 정리할 수 있다.

- Gazebo 센서 모델: 붙음
- ROS 2 토픽 이름: 보임
- 실제 scan payload 수신: 확인 완료
- MAVROS 연결: 살아 있음
- 실제 takeoff mission: 확인 완료
- 최신 baseline artifact: 보관 완료

## 3. 문제 1: `/drone1/scan`은 실제로 수신되지만 obstacle world가 아직 없음

### 3.1 증거

ROS 컨테이너에서 확인된 사항:

- `/drone1/scan` 토픽이 `ros2 topic list`에 나타남
- `/drone1/scan`의 `Publisher count: 1`

Gazebo 컨테이너에서 확인된 사항:

- `/gazebo/default/iris_rplidar/rplidar/link/laser/scan`
- `/gazebo/default/iris_rplidar/lidar/link/laser/scan`

즉 Gazebo 내부 라이다 센서는 실제로 존재한다.

모델 파일 확인:

- `libgazebo_ros_ray_sensor.so` 플러그인 선언 존재
- `<namespace>/drone1</namespace>`
- `<remapping>~/out:=scan</remapping>`

즉 커스텀 센서 모델도 실제로 반영되었다.

추가 로그:

- `gazebo_ros_node`가 뜨는 로그가 있었기 때문에 `gazebo_ros` 계열 플러그인 자체는 로드되었다고 볼 수 있다.

### 3.2 현재 확인된 정상 동작

다음 명령으로 실제 scan payload 수신을 확인했다.

```bash
ros2 topic echo /drone1/scan --once
ros2 topic hz /drone1/scan
```

확인 결과:

- `LaserScan` 메시지가 실제 출력됨
- 약 `10Hz`로 안정적으로 수신됨
- `frame_id: rplidar_link`

즉 `토픽 등록 성공`뿐 아니라 `데이터 수신 성공`도 확인됐다.

### 3.3 해석

현재 해석은 아래와 같다.

1. Docker transport와 QoS 보강이 실제 sample delivery 안정화에 도움이 된 것으로 보인다.
2. 현재 scan이 대부분 `inf`인 것은 장애물이 가까이 없거나, 라이다 범위 내에 물체가 없는 환경이기 때문이다.

즉 이제 scan 자체는 막힌 문제가 아니고, 다음 단계는 `장애물이 실제로 보이는 world`를 구성해 perception/planner 반응을 검증하는 것이다.

## 4. 문제 2: Gazebo에서 드론이 실제로 움직이지 않음

이 문제는 baseline 수준에서 해결됐다.

### 4.1 해결 전 증거

실제 관찰:

- Gazebo 창은 정상적으로 뜸
- MAVROS와 `autonomy_manager`는 살아 있음
- PX4 쪽에서는 arm 시도가 반복됨
- 하지만 드론은 이륙하지 않음

PX4 로그에서 반복적으로 보인 경고:

- `Arming denied: Resolve system health failures first`

artifact에서도 관련 근거가 보인다.

참고 파일:

- 로컬 artifact 예: `artifacts/2026-03-08_10-46-13_drone1/summary.json`

확인된 값:

- `"connected": true`
- `"armed": false`
- `"mode": "OFFBOARD"`
- `"pose_count": 0`
- `"closest_obstacle_m": Infinity`

### 4.2 해결 후 상태

최신 baseline 실행에서는 아래가 확인됐다.

- `/drone1/mission/phase = HOVER_AT_GOAL`
- `/drone1/mission/goal_reached = true`
- `/mavros/local_position/pose` 정상 수신
- 최신 artifact:
  - [baseline_summary_example.json](/home/deepblue/AV_Drone/docs/examples/baseline_summary_example.json)

확인된 값:

- `"connected": true`
- `"armed": true`
- `"mode": "OFFBOARD"`
- `"mission_phase": "HOVER_AT_GOAL"`
- `"goal_reached": true`
- `"pose_count": 22916`
- `"scan_count": 3855`
- `"closest_obstacle_m": 0.2117559313774109`

즉 현재는 “드론이 움직이지 않음”이 아니라, “단일 드론 baseline은 됐고 다음은 obstacle course 고도화와 멀티드론 확장” 단계다.

## 5. 현재까지의 근거 정리

### 정상인 것

- `sim` 컨테이너는 `iris_rplidar` 모델로 올라감
- Gazebo 내부 라이다 토픽 존재
- ROS 2 그래프에 `/drone1/scan` 토픽 존재
- `lidar_obstacle`, `local_planner`, `safety_monitor`, `autonomy_manager`, `metrics_logger` 노드가 올라옴

### 아직 안 풀린 것

- 연구용 obstacle course 고도화
- 멀티드론 namespace/spawn 구조
- failure injection과 orphan task 재할당
- X11 권한 미설정 시 Gazebo GUI 미출력 가능성

## 6. 다음 해결 순서

현재는 아래 순서로 가는 것이 가장 합리적이다.

### 6.1 1단계: obstacle world 고도화

목표:

- 라이다가 더 다양한 장애물 형태를 읽음
- 회피 경로가 더 분명히 드러나는 world 구성
- `closest_obstacle_m`, 성공률, time-to-goal 비교

### 6.2 2단계: multi-UAV 구조 추가

목표:

- `drone1`, `drone2`, `drone3`, `drone4` namespace 분리
- 포트, spawn 위치, MAVROS instance 분리

### 6.3 3단계: failure-aware mission continuation 구현

이 단계부터는 취업용 정량 지표를 강화한다.

예:

- 성공률
- 최소 장애물 거리
- loop latency mean/p99/worst
- timeout 발생 횟수
- fail-safe trigger/recovery 기록

## 7. 지금 당장 확인해야 할 명령

### scan 수신 확인

```bash
ros2 topic info -v /drone1/scan
ros2 topic echo /drone1/scan --once
ros2 topic hz /drone1/scan
```

### pose 수신 확인

```bash
ros2 topic echo /mavros/local_position/pose --once
```

### artifact 확인

```bash
find artifacts -maxdepth 2 -type f | sort
```

최신 실행 결과:

- [baseline_summary_example.json](/home/deepblue/AV_Drone/docs/examples/baseline_summary_example.json)

## 8. 운영 원칙

앞으로 이 문서는 아래 원칙으로 계속 업데이트한다.

1. 새 문제가 생기면 문제 번호를 추가한다.
2. 반드시 근거 로그나 artifact를 같이 기록한다.
3. "추정"과 "확정"을 구분한다.
4. 해결 전에는 다음 액션을 명시한다.
5. 해결되면 해결 상태와 수정 파일을 함께 남긴다.

즉, 이후에도 이 문서는 계속 현재 상태를 반영하는 문제 추적 문서로 유지한다.
