# Change Log and Rationale

## 1. 왜 `20.04`였는가

처음 `sim` 컨테이너는 아래 베이스 이미지를 사용하고 있었다.

- `px4io/px4-dev-simulation-focal:latest`

이 이미지는 이름 그대로 `Ubuntu 20.04 (focal)` 기반 PX4 개발 이미지다.

당시 이 구성을 쓴 이유는 다음과 같다.

- PX4 SITL + Gazebo Classic 조합이 이미 포함된 이미지라서 빠르게 시작할 수 있다.
- PX4 예제와 커뮤니티 사례 중 `focal` 기반이 많다.
- 초기 목표가 "단일 드론 MPPI 데모를 먼저 재현"하는 것이었고, 이 단계에서는 `MAVROS` 연결만 되면 충분했다.

즉, 초기에 `20.04`였던 것은 "ROS 2 Humble에 최적화된 선택"이라기보다, "PX4 SITL을 빨리 올리기 쉬운 이미지"를 먼저 가져온 결과였다.

## 2. 왜 `22.04`로 바꾸는가

이번 라이다 작업에서 문제가 드러났다.

문제 상황:

- PX4의 `iris_rplidar` 모델은 Gazebo에서 정상적으로 올라왔다.
- 하지만 ROS 2 쪽에서는 `LaserScan` 토픽이 보이지 않았다.
- 원인은 PX4 기본 `rplidar` 모델이 사용하던 센서 플러그인이 ROS 2 Humble 그래프에 바로 연결되지 않는 구조였기 때문이다.

라이다를 ROS 2 토픽으로 직접 받으려면 다음이 필요하다.

- Gazebo 모델 안에서 ROS 2용 `gazebo_ros` 플러그인 사용
- `sim` 컨테이너 내부에서 `ros-humble-gazebo-plugins`, `ros-humble-gazebo-ros-pkgs` 설치 가능

그런데 `focal` 기반 컨테이너에서는 `Humble` 패키지 설치가 불가능했다.

실제 문제:

- `ros-humble-gazebo-plugins`
- `ros-humble-gazebo-ros`
- `ros-humble-gazebo-ros-pkgs`

를 설치할 수 없었다.

그래서 `sim` 컨테이너도 `Ubuntu 22.04 + ROS 2 Humble` 기준으로 올려서,

- `ros` 컨테이너와 배포판을 맞추고
- Gazebo ROS 2 플러그인을 정식으로 사용하고
- 라이다 데이터를 ROS 2 토픽으로 직접 발행하는 구조

로 바꾸는 것이 맞다고 판단했다.

## 3. 이번에 정확히 무엇을 수정했는가

### 3.1 `docker/sim/Dockerfile`

이전 상태:

- `FROM px4io/px4-dev-simulation-focal:latest`

변경 후:

- `FROM ubuntu:22.04`

추가한 핵심 요소:

- locale 설정
- ROS 2 Humble apt source 등록
- Gazebo Classic 및 PX4 빌드 의존성 설치
- ROS 2 Gazebo 패키지 설치
  - `ros-humble-gazebo-plugins`
  - `ros-humble-gazebo-ros-pkgs`
- `pip3 install kconfiglib`
- PX4 소스를 `v1.15.4` 기준으로 직접 clone

의미:

- 이제 `sim` 컨테이너는 더 이상 예전 PX4 제공 이미지에 의존하지 않는다.
- 대신 `Ubuntu 22.04` 위에서 PX4와 ROS 2 Gazebo 플러그인을 같이 쓸 수 있게 설계되었다.

### 3.2 `docker/sim/entrypoint.sh`

핵심 변경:

- `/opt/ros/humble/setup.bash`가 있으면 source
- 저장소 안의 커스텀 라이다 모델 파일을 PX4 Gazebo 모델 경로에 덮어씀
- 실행 타깃을 `PX4_SIM_TARGET` 환경변수로 받도록 변경

주요 동작:

- `SIM_TARGET="${PX4_SIM_TARGET:-gazebo-classic_iris_rplidar}"`
- `make px4_sitl "${SIM_TARGET}"`

의미:

- 이제 단순 `gazebo-classic`가 아니라 `iris_rplidar` 타깃을 직접 실행한다.
- 커스텀 모델 수정사항을 컨테이너 재빌드 없이 덮어쓸 수 있다.

### 3.3 `docker-compose.yml`

`sim` 서비스 변경:

- `PX4_SIM_TARGET=${PX4_SIM_TARGET:-gazebo-classic_iris_rplidar}`
- 저장소 전체를 `sim` 컨테이너에도 마운트
  - `.:/workspace/AV_Drone`

의미:

- `sim` 컨테이너가 프로젝트 내부 센서 모델 오버라이드를 읽을 수 있다.
- 실행 시 기본 기체가 `iris_rplidar`가 된다.

### 3.4 `sim_assets/models/rplidar/model.sdf`

이 파일은 커스텀 라이다 센서 모델 오버라이드다.

핵심 변경 방향:

- ROS 2용 Gazebo 레이 센서 플러그인 사용
- namespace를 `/drone1`로 설정
- `LaserScan` 출력 사용

의도된 토픽 구조:

- `/drone1/scan`

의미:

- Gazebo 내부 센서를 ROS 2 토픽으로 직접 연결하기 위한 핵심 파일이다.

### 3.5 `src/drone_bringup/config/drone1_autonomy.yaml`

한 번 `/laser/scan`으로 바뀌었던 값을 다시 아래로 정리했다.

- `scan_topic: "/drone1/scan"`

이유:

- 커스텀 센서 플러그인 namespace 설계가 `/drone1` 기준이기 때문이다.
- perception / safety 노드가 같은 scan 토픽을 보도록 일관성을 맞췄다.

## 4. 현재 상태

현재 기준으로 확인된 사실:

- `iris_rplidar` 모델로 PX4 SITL 기체는 실제로 뜬다.
- Gazebo에서 라이다가 달린 모델이 로드되는 것까지는 확인했다.
- `sim` 이미지는 `22.04 + Humble + gazebo_ros` 구조로 마이그레이션 중이다.

아직 최종 확인이 남은 것:

- `docker compose build sim` 완료
- `docker compose up sim` 후 `ros2 topic info /drone1/scan`
- `ros2 topic echo /drone1/scan --once`

즉, 현재는 "구조 변경 완료, 최종 센서 발행 검증 진행 중" 상태다.

## 5. 왜 이 변경이 중요한가

이 변경은 단순히 Docker 베이스 이미지를 바꾼 것이 아니다.

실제 의미는 다음과 같다.

- PX4 SITL 실행 전용 환경에서
- ROS 2와 직접 연결되는 센서 시뮬레이션 환경으로
- `sim` 컨테이너의 역할을 확장한 것

이게 되면 다음 단계가 가능해진다.

- `/drone1/scan` 기반 perception
- 센서 기반 장애물 회피
- fail-safe와 metrics에 실제 센서 입력 연결
- 이후 2대, 4대 멀티드론 확장

## 6. 요약

요약하면 다음과 같다.

- 원래 `20.04`였던 이유는 PX4 제공 `focal` 개발 이미지를 그대로 사용했기 때문이다.
- 하지만 라이다를 ROS 2에서 직접 읽으려면 `Humble + gazebo_ros` 패키지가 필요했다.
- 그래서 `sim`도 `22.04` 기준으로 올려서 `ros` 컨테이너와 배포판을 맞추는 방향으로 변경했다.
- 관련 수정은 `docker/sim/Dockerfile`, `docker/sim/entrypoint.sh`, `docker-compose.yml`, `sim_assets/models/rplidar/model.sdf`, `src/drone_bringup/config/drone1_autonomy.yaml`에 반영되었다.
