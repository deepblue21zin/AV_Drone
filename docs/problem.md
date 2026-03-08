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

현재 가장 큰 문제는 아래 두 가지다.

1. 라이다 토픽은 ROS 2 그래프에 등록되지만 실제 `LaserScan` 샘플 수신이 안정적으로 확인되지 않는다.
2. `single_drone_autonomy.launch.py`는 아직 실제 이륙 가능한 오프보드 미션 상태 머신이 없어서 Gazebo에서 드론이 움직이지 않는다.

즉, 현재 상태는 아래처럼 정리할 수 있다.

- Gazebo 센서 모델: 붙음
- ROS 2 토픽 이름: 보임
- 실제 scan payload 수신: 미검증/실패 상태
- MAVROS 연결: 살아 있음
- 실제 takeoff mission: 아직 없음

## 3. 문제 1: `/drone1/scan`은 보이지만 실제 scan 데이터 확인이 안 됨

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

### 3.2 현재까지 관찰된 이상 징후

다음 명령은 기대처럼 동작하지 않았다.

```bash
ros2 topic echo /drone1/scan --once
ros2 topic hz /drone1/scan
```

의미:

- 토픽 discovery는 되는데
- 실제 sample delivery는 확인되지 않았다

즉 현재는 `토픽 등록 성공`과 `데이터 수신 성공`을 구분해야 한다.

### 3.3 해석

현재 가장 가능성 높은 원인은 아래 중 하나다.

1. `sim` 컨테이너에서 발행된 ROS 2 scan 데이터가 `ros` 컨테이너 subscriber까지 DDS transport로 정상 전달되지 않는다.
2. publisher는 등록되지만 QoS 또는 transport 문제로 실제 샘플 수신이 막힌다.
3. Gazebo sensor plugin이 토픽 endpoint는 만들지만 sample forwarding이 완전히 살아 있지 않다.

현재 우선순위상 가장 의심되는 것은 `Docker container 간 DDS/IPC/shared-memory 설정 문제`다.

## 4. 문제 2: Gazebo에서 드론이 실제로 움직이지 않음

### 4.1 증거

실제 관찰:

- Gazebo 창은 정상적으로 뜸
- MAVROS와 `autonomy_manager`는 살아 있음
- PX4 쪽에서는 arm 시도가 반복됨
- 하지만 드론은 이륙하지 않음

PX4 로그에서 반복적으로 보인 경고:

- `Arming denied: Resolve system health failures first`

artifact에서도 관련 근거가 보인다.

참고 파일:

- [summary.json](/home/deepblue/AV_Drone/artifacts/2026-03-08_10-46-13_drone1/summary.json)

확인된 값:

- `"connected": true`
- `"armed": false`
- `"mode": "OFFBOARD"`
- `"pose_count": 0`
- `"closest_obstacle_m": Infinity`

### 4.2 해석

현재 드론이 안 움직이는 이유는 단일 원인이 아니다.

#### 이유 A. `single_drone_autonomy.launch.py`는 아직 완전한 비행 launch가 아님

현재 `autonomy_manager`는 주로 아래만 수행한다.

- velocity setpoint 주기 발행
- `OFFBOARD` 요청
- `arm` 요청

하지만 기존 `mppi` 데모처럼 아래 상태 머신은 아직 없다.

- setpoint pre-stream
- arm 후 이륙 고도 상승
- hover
- planner 기반 수평 이동
- 착륙

즉 지금 구조는 `오프보드 bridge + safety gate`에 가깝고, `실제 비행 mission manager`는 아직 아니다.

#### 이유 B. pose와 scan이 실제 데이터로 안정적으로 들어오지 않음

artifact에 `pose_count: 0`가 나온다.

의미:

- `metrics_logger` 기준으로는 pose callback이 실제로 증가하지 않았다.
- 따라서 safety/control 입장에서는 정상 자율비행 조건이 충족되지 않는다.

#### 이유 C. safety 구조상 입력이 불완전하면 정지 명령으로 빠질 가능성이 높음

현재 `safety_monitor`는 아래 상황에서 zero velocity를 내보낸다.

- `pose_timeout`
- `scan_timeout`
- `planner_cmd_timeout`
- `emergency_stop_obstacle`

즉 센서나 위치 입력이 미완전하면 의도적으로 기체를 움직이지 않게 만든 구조다.

## 5. 현재까지의 근거 정리

### 정상인 것

- `sim` 컨테이너는 `iris_rplidar` 모델로 올라감
- Gazebo 내부 라이다 토픽 존재
- ROS 2 그래프에 `/drone1/scan` 토픽 존재
- `lidar_obstacle`, `local_planner`, `safety_monitor`, `autonomy_manager`, `metrics_logger` 노드가 올라옴

### 아직 안 풀린 것

- `/drone1/scan` 실제 메시지 수신 확인
- `/mavros/local_position/pose` 실제 데이터 카운트 증가
- takeoff 가능한 상태 머신 부재
- PX4 preflight/arming 안정화

## 6. 다음 해결 순서

현재는 아래 순서로 가는 것이 가장 합리적이다.

### 6.1 1단계: DDS/IPC 전달 문제 점검

목표:

- `/drone1/scan`에 실제 메시지가 들어오게 만들기

우선 검토할 수정:

- `docker-compose.yml`에 `ipc: host` 추가
- 필요 시 ROS 2 shared memory 관련 환경변수 조정
- 컨테이너 간 DDS transport 설정 명시

성공 기준:

```bash
ros2 topic echo /drone1/scan --once
ros2 topic hz /drone1/scan
```

이 두 명령이 정상적으로 데이터를 보여야 한다.

### 6.2 2단계: pose 수신 확인

목표:

- `/mavros/local_position/pose`가 실제로 들어오는지 확인
- `metrics_logger`의 `pose_count`가 증가하는지 확인

성공 기준:

- artifact summary에서 `pose_count > 0`
- `pose_period_mean_s`, `pose_period_p99_s`가 실제 값으로 채워짐

### 6.3 3단계: `autonomy_manager`를 실제 mission manager로 확장

현재는 arm/offboard 요청만 하는 구조다.

추가해야 할 것:

- setpoint pre-stream 단계
- arm 성공 확인
- takeoff 고도 목표
- hover
- 이후 planner command 적용

성공 기준:

- Gazebo에서 드론이 실제로 상승
- PX4 상태가 안정적으로 `OFFBOARD + armed`

### 6.4 4단계: obstacle world 추가

센서와 takeoff가 모두 정상화되면 다음은 장애물 world를 넣는다.

목표:

- 라이다가 실제 장애물을 읽음
- nearest obstacle distance가 바뀜
- planner가 회피 명령을 생성

### 6.5 5단계: metrics/failsafe를 정량 검증으로 강화

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

- [summary.json](/home/deepblue/AV_Drone/artifacts/2026-03-08_10-46-13_drone1/summary.json)

## 8. 운영 원칙

앞으로 이 문서는 아래 원칙으로 계속 업데이트한다.

1. 새 문제가 생기면 문제 번호를 추가한다.
2. 반드시 근거 로그나 artifact를 같이 기록한다.
3. "추정"과 "확정"을 구분한다.
4. 해결 전에는 다음 액션을 명시한다.
5. 해결되면 해결 상태와 수정 파일을 함께 남긴다.

즉, 이후에도 이 문서는 계속 현재 상태를 반영하는 문제 추적 문서로 유지한다.
