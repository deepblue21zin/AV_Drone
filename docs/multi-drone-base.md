# 멀티드론 베이스

## 현재 범위

현재 구현은 SLAM을 제외한 ROS 2 멀티드론 실행 기반이다.

- 동일한 autonomy pipeline을 `drone1`부터 `drone4`까지 재사용
- 드론별 MAVROS, perception, planner, safety, control, metrics namespace 분리
- 드론 수에 따라 MAVROS 포트와 target system ID 자동 할당
- 드론별 평행 목표 경로 자동 생성
- 기본 실행에서는 비행 controller를 띄우지 않는 안전 게이트 적용

Gazebo에 여러 기체를 생성하고 각 기체를 별도 PX4 SITL 인스턴스에 연결하는
작업은 다음 단계다. 현재 sim 컨테이너는 기본적으로 PX4 instance 0 한 대만
실행한다.

## namespace 구조

2대 실행 시 주요 토픽은 아래와 같이 분리된다.

```text
/drone1/mavros/state
/drone1/mavros/local_position/pose
/drone1/scan
/drone1/autonomy/cmd_vel
/drone1/safety/cmd_vel

/drone2/mavros/state
/drone2/mavros/local_position/pose
/drone2/scan
/drone2/autonomy/cmd_vel
/drone2/safety/cmd_vel
```

공통 설정 파일의 토픽은 상대 이름을 사용한다. 각 노드는 `drone1`,
`drone2` namespace 아래에서 실행되므로 코드와 YAML을 복사하지 않고
확장할 수 있다.

## 포트 규칙

| 드론 | PX4 instance | MAVROS bind | PX4 remote | target system |
|---|---:|---:|---:|---:|
| drone1 | 0 | 14540 | 14580 | 1 |
| drone2 | 1 | 14541 | 14581 | 2 |
| drone3 | 2 | 14542 | 14582 | 3 |
| drone4 | 3 | 14543 | 14583 | 4 |

이 규칙은 다음 Gazebo/PX4 멀티 인스턴스 구현에서도 동일하게 유지해야 한다.

## 빌드

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
colcon build \
  --packages-select \
  drone_bringup drone_control drone_perception \
  drone_planning drone_safety drone_metrics \
  --symlink-install
source install/setup.bash
```

## 안전한 구조 점검

기본값 `enable_flight:=false`에서는 autonomy manager를 실행하지 않으므로
arm, mode 변경, velocity setpoint 명령이 발생하지 않는다.

```bash
ros2 launch drone_bringup multi_drone_autonomy.launch.py \
  vehicle_count:=2 \
  enable_flight:=false \
  enable_metrics:=false
```

현재 단일 PX4 시뮬레이터가 실행 중이면 `drone1`만 연결되고 `drone2`는
연결 대기 상태가 되는 것이 정상이다.

## 실제 비행 활성화

다중 PX4와 다중 센서 연결이 검증되기 전에는 아래 옵션을 사용하지 않는다.

```bash
ros2 launch drone_bringup multi_drone_autonomy.launch.py \
  vehicle_count:=2 \
  lane_spacing:=15.0 \
  goal_x:=20.0 \
  goal_z:=3.0 \
  enable_flight:=true
```

`vehicle_count`는 1부터 4까지 지원한다. 목표 y 좌표는 `lane_spacing`을
기준으로 중앙에 대칭 배치된다. 2대와 간격 15m인 경우 drone1은
`y=-7.5m`, drone2는 `y=7.5m`를 목표로 한다. 실제 시작 y 좌표도
Gazebo/PX4 멀티 인스턴스 spawn 설정에서 각각 `-7.5m`, `7.5m`로
맞춰야 출발과 도착 차선이 일치한다.

## 3대 MPPI LiDAR 실험

`multi_mppi_lidar.launch.py`는 기본적으로 3대를 실행한다. 월드
`random_cylinders_double_178640653`에서 drone1, drone2, drone3의 시작점은
각각 `(3.0, -7.5)`, `(3.0, 7.5)`, `(3.0, 0.0)`이다. 각 MAVROS 로컬
좌표계의 목표는 `(144.0, 0.0)`이므로 월드 목표점은 각각
`(147.0, -7.5)`, `(147.0, 7.5)`, `(147.0, 0.0)`이 된다.

반복 실험은 저장소 루트에서 아래처럼 실행한다.

```bash
python3 scripts/run_multi_mppi_experiments.py --runs 4
```

주요 옵션은 다음과 같다.

- `--runs`: 반복 횟수
- `--timeout`: 회차별 비행 제한 시간(초)
- `--progress-interval`: 상태 출력 주기(초)
- `--output-dir`: 결과 폴더를 직접 지정할 때 사용

각 `run_XX` 폴더에는 `result.json`, `rosbag`, `launch.log`,
`rosbag.log`, `trajectory.png`, `trajectory_metrics.txt`가 생성된다. 모든
회차가 성공하면 최상위 결과 폴더에 `trajectory_overlay.png`도 생성된다.
한 회차라도 실패하면 오류와 로그 끝부분을 출력하고 이후 회차를 실행하지
않는다.

## 다음 구현 단계

1. 단일 gzserver에서 PX4 instance 0과 1 실행
2. instance별 MAVLink 포트가 적용된 기체 모델 생성
3. LiDAR plugin namespace를 `/drone1/scan`, `/drone2/scan`으로 분리
4. 선택적 arm과 순차 이륙 smoke test
5. 3대와 4대로 확장
6. 드론 간 최소거리 monitor 추가
