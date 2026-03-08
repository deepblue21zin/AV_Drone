# Docker Environment Specification

## 1. 목적

이 문서는 `/home/deepblue/AV_Drone` 프로젝트를 `PX4 + ROS 2 + Gazebo + MAVROS + MPPI` 조합으로 안정적으로 실행하기 위한 Docker 기반 환경 구성을 정의한다.

목표는 다음과 같다.

- 로컬 OS에 직접 PX4, ROS 2, Gazebo를 혼합 설치하지 않는다.
- 개발 환경과 시뮬레이션 환경을 Docker Compose로 일관되게 관리한다.
- 현재 저장소의 ROS 2 패키지 구조와 MAVROS 인터페이스를 그대로 유지한다.
- 이후 `Dockerfile`, `docker-compose.yml`, `entrypoint.sh`, 실행 스크립트로 바로 구현 가능해야 한다.

## 2. 현재 코드 기준 전제

현재 저장소는 ROS 2 workspace 형태이며 다음 패키지를 포함한다.

- `src/mppi`
- `src/offboard_control`

코드상 제어 인터페이스는 `px4_ros_com`이 아니라 `MAVROS` 기준이다.

주요 사용 인터페이스:

- subscribe: `/mavros/state`
- subscribe: `/mavros/local_position/pose`
- publish: `/mavros/setpoint_velocity/cmd_vel`
- service: `/mavros/set_mode`
- service: `/mavros/cmd/arming`

따라서 환경 구성의 핵심은 다음 조합이다.

- PX4 SITL
- Gazebo Classic
- ROS 2 Humble
- MAVROS for ROS 2
- colcon build

## 3. 기준 버전

호환성과 구현 난이도를 고려해 아래 버전을 표준으로 채택한다.

- Host OS: Ubuntu 22.04
- Docker Engine: 24.x 이상
- Docker Compose Plugin: 2.x 이상
- Base distro in container: Ubuntu 22.04
- ROS 2: Humble
- PX4: `PX4-Autopilot` stable branch 계열
- Simulator: Gazebo Classic 11
- Python: 3.10

선정 이유:

- ROS 2 Humble는 Ubuntu 22.04와 가장 안정적으로 맞물린다.
- PX4 SITL + Gazebo Classic 조합이 문서와 사례가 가장 많다.
- 현재 코드가 `mavros` launch 및 `mavros_msgs`를 직접 사용하므로 Humble 계열 구성이 가장 단순하다.

## 4. 전체 아키텍처

Docker Compose 기준 2개 서비스로 구성한다.

### 4.1 `sim`

역할:

- PX4 SITL 실행
- Gazebo Classic 실행
- PX4 UDP 포트 제공

책임:

- world 로딩
- 기체 spawn
- SITL runtime

### 4.2 `ros`

역할:

- ROS 2 workspace 빌드
- MAVROS 실행
- `mppi` 또는 `offboard_control` 노드 실행

책임:

- `colcon build`
- `rosdep` 기반 의존성 설치
- `ros2 launch mppi mppi.launch.py` 또는 대체 launch 실행

## 5. 네트워크 정책

네트워크는 기본적으로 `host` 모드를 사용한다.

채택 이유:

- PX4 SITL과 MAVROS의 UDP 연결 구성이 단순해진다.
- 현재 코드의 MAVROS launch 인자 `udp://:14540@127.0.0.1:14580`를 거의 그대로 유지할 수 있다.
- Gazebo, MAVLink, ROS 노드 간 포트 디버깅이 쉬워진다.

적용 원칙:

- `sim` 서비스: `network_mode: host`
- `ros` 서비스: `network_mode: host`

제약:

- `host network`는 Linux host를 전제로 한다.
- macOS/Windows Docker Desktop에서는 동일 구성이 불안정하거나 추가 조정이 필요하다.

따라서 본 명세는 Linux 개발 환경을 표준으로 한다.

## 6. GUI 정책

Gazebo GUI 사용을 위해 X11 forwarding을 허용한다.

필수 설정:

- host의 `/tmp/.X11-unix` 마운트
- `DISPLAY` 환경변수 전달
- 필요 시 `QT_X11_NO_MITSHM=1`

보안/운영 방침:

- 로컬 개발 환경에서는 `xhost +local:docker` 수준의 제한적 허용 사용
- 장기적으로는 GUI 없는 headless 모드도 함께 지원 가능하게 설계

기본 정책:

- 1차 구현은 GUI 포함
- 2차 옵션으로 headless 실행 프로파일 추가 가능

## 7. 볼륨 마운트 정책

### 7.1 소스 마운트

- host: `/home/deepblue/AV_Drone`
- container: `/workspace/AV_Drone`

목적:

- 로컬 코드 수정 즉시 컨테이너에서 재빌드 가능
- 개발 환경과 실행 환경을 동일 workspace로 유지

### 7.2 PX4 소스 마운트

선택지는 두 가지다.

옵션 A. 이미지 내부 고정

- PX4를 Docker image build 단계에서 clone 및 setup
- 장점: 실행 재현성 우수
- 단점: 이미지 빌드 시간이 길고 branch 변경이 불편

옵션 B. 별도 workspace mount

- host의 PX4 source를 컨테이너에 마운트
- 장점: PX4 수정/업데이트 편함
- 단점: host에 PX4 소스가 따로 있어야 함

본 프로젝트의 1차 표준은 옵션 A로 한다.

이유:

- 현재 목적은 "이 저장소 실행 환경 표준화"이지 PX4 자체 개발이 아니다.
- 이미지 단일화가 온보딩에 유리하다.

## 8. 서비스별 설치 항목

### 8.1 `ros` 서비스 설치 항목

기반 이미지:

- `ubuntu:22.04`

설치 대상:

- ROS 2 Humble desktop
- `python3-colcon-common-extensions`
- `python3-rosdep`
- `python3-vcstool`
- `python3-numpy`
- `ros-humble-mavros`
- `ros-humble-mavros-extras`
- GeographicLib datasets
- 빌드 도구: `build-essential`, `cmake`, `git`
- 디버깅 유틸: `iputils-ping`, `net-tools`, `vim` 또는 최소 셸 유틸

workspace 빌드 대상:

- `src/mppi`
- `src/offboard_control`

### 8.2 `sim` 서비스 설치 항목

기반 이미지:

- `ubuntu:22.04`

설치 대상:

- PX4 build dependencies
- Gazebo Classic runtime dependencies
- `PX4-Autopilot` source
- `make`, `ninja`, `ccache`, `python3-pip`, `python3-jinja2`, `python3-empy`, `python3-toml`, `python3-numpy`
- OpenGL/X11 관련 패키지

실행 대상:

- `make px4_sitl gazebo-classic`

## 9. 컨테이너 시작 방식

### 9.1 `sim` 시작 순서

1. PX4 환경 변수 로드
2. Gazebo Classic world 준비
3. PX4 SITL 실행
4. UDP 포트 활성화 확인

### 9.2 `ros` 시작 순서

1. ROS 2 환경 source
2. workspace 의존성 확인
3. `colcon build`
4. install setup source
5. MAVROS launch
6. MPPI 또는 offboard launch 실행

## 10. 실행 모드

본 환경은 최소 3개 실행 모드를 지원해야 한다.

### 10.1 개발 모드

용도:

- 코드 수정
- 반복 빌드
- 로그 확인

특징:

- 소스 디렉터리 bind mount
- 컨테이너 재생성 없이 `colcon build` 가능

### 10.2 데모 모드

용도:

- PX4 + Gazebo + MAVROS + MPPI 일괄 실행

특징:

- Compose up 후 정해진 launch 실행
- GUI 활성화

### 10.3 테스트 모드

용도:

- ROS 패키지 빌드 검증
- linter 또는 smoke test

특징:

- Gazebo 없이 `colcon test` 중심

## 11. 환경 변수 명세

필수 환경 변수:

- `DISPLAY`
- `QT_X11_NO_MITSHM=1`
- `ROS_DOMAIN_ID` optional
- `PX4_SIM_MODEL` optional
- `PX4_GZ_WORLD` 또는 world 관련 변수 optional

ROS 컨테이너 공통:

- `RMW_IMPLEMENTATION`은 기본값 유지
- `ROS_LOCALHOST_ONLY`는 `0` 또는 unset 유지

## 12. 디렉터리 구조 명세

최종 파일 구성 목표:

- `docker/ros/Dockerfile`
- `docker/sim/Dockerfile`
- `docker/ros/entrypoint.sh`
- `docker/sim/entrypoint.sh`
- `docker-compose.yml`
- `.dockerignore`
- `docs/docker-environment-spec.md`

선택 추가 파일:

- `scripts/run_demo.sh`
- `scripts/build_workspace.sh`
- `scripts/enter_ros.sh`
- `scripts/enter_sim.sh`

## 13. 빌드 및 실행 정책

### 13.1 이미지 빌드

- `docker compose build`로 통합 빌드
- PX4 관련 레이어와 ROS 관련 레이어는 분리
- apt install과 source build는 최대한 캐시 친화적으로 배치

### 13.2 컨테이너 실행

- `sim`을 먼저 시작
- `ros`는 `sim` 준비 이후 시작
- 필요 시 healthcheck 또는 wait script 사용

### 13.3 종료

- Compose down으로 전체 정리
- 로그는 표준 출력 우선

## 14. 검증 기준

환경 구성이 완료되었다고 판단하는 기준은 다음과 같다.

### 14.1 빌드 검증

- `ros` 컨테이너에서 `colcon build` 성공
- `mppi`, `offboard_control` 패키지가 install space에 등록됨

### 14.2 연결 검증

- `/mavros/state` topic 수신 가능
- `/mavros/local_position/pose` topic 수신 가능
- `ros2 service list`에 `/mavros/set_mode`, `/mavros/cmd/arming` 존재

### 14.3 시뮬레이션 검증

- Gazebo에서 기체가 정상 spawn됨
- Offboard 전환 성공
- 이륙 성공
- `mppi` launch 시 목표점 이동 수행

## 15. 제약 및 리스크

### 15.1 Gazebo 버전 종속성

현재 명세는 Gazebo Classic 기준이다. Ignition/Garden/Harmonic 계열로 전환하면 PX4 실행 방식과 패키지 구성이 달라질 수 있다.

### 15.2 GUI 의존성

Docker에서 GUI를 사용할 경우 host X11 설정 문제로 초기 진입 장벽이 있다.

### 15.3 실시간성

컨테이너 내부 시뮬레이션은 host 성능에 영향을 받으며, 고부하 시 MPPI 주기 안정성이 낮아질 수 있다.

### 15.4 장애물 월드 정합성

`mppi_node.py` 내부 obstacle 파라미터와 Gazebo world 배치가 다르면 회피 결과가 비정상적일 수 있다. 환경 구현 단계에서 world 파일 관리 정책을 별도로 정해야 한다.

## 16. 구현 방침

이 명세를 기준으로 다음 순서로 구현한다.

1. `docker/ros/Dockerfile` 작성
2. `docker/sim/Dockerfile` 작성
3. 각 서비스 `entrypoint.sh` 작성
4. `docker-compose.yml` 작성
5. 실행 절차를 `README` 또는 별도 문서에 정리
6. smoke test 수행 후 포트/launch 인자 조정

## 17. 최종 결론

이 프로젝트의 표준 실행 환경은 다음으로 정의한다.

- Linux host(Ubuntu 22.04)
- Docker Compose 기반 2서비스 구조
- `sim`: PX4 SITL + Gazebo Classic
- `ros`: ROS 2 Humble + MAVROS + project workspace
- `network_mode: host`
- X11 기반 GUI 지원
- 소스는 bind mount, PX4는 이미지 내부 포함

이 구성이 현재 코드 구조와 가장 잘 맞고, 이후 실제 Docker 구현으로 옮기기 가장 단순하다.
