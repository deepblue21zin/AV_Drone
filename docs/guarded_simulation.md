# 시뮬레이션 용량 보호 및 자동 종료

적용일: 2026-09-11. 요청한 **1·3·4번**의 구현이며, **2번 PX4 기록 설정 변경은 보류**했다.
`SDLOG_MODE`, `SDLOG_PROFILE`, 기록 토픽·주기, 비행·SLAM 알고리즘은 변경하지 않았다.
이 작업에서는 기존 시뮬레이터를 시작하거나 컨테이너를 재생성하지 않았다. `sudo`도 사용하지 않았다.

## 1. 바뀐 내용

| 항목 | 적용 내용 |
|---|---|
| 1. 새 로그 저장 | PX4 인스턴스별 `log`와 stdout/stderr 경로를 프로젝트의 실행별 폴더로 연결. 관리 실행의 ROS 로그도 같은 실행 폴더에 저장 |
| 3. 종료 관리 | 모든 드론의 목표 상태 감지 → rosbag에 SIGINT·저장 대기 → ROS 종료 → PX4/Gazebo 및 시작한 컨테이너 종료 |
| 4. 보호 장치 | 시작 전 디스크 검사, 실행 중 용량·시간 검사, 초과 시 종료 요청. 과거 실험 데이터 자동 삭제 없음 |

기존 `log` 폴더는 첫 보호 실행 때 컨테이너 안에서 `log.pre_guard_<고유값>`으로 이름만 바꾸어 보존한다.
그 자리에 새 로그 목적지를 가리키는 링크를 만든다. 기존 로그를 `/home3`로 일괄 이사시키거나 삭제하는 작업은 아니다.
다음 실행에서는 새로운 실행 폴더로 연결하므로 이전 실험 로그를 덮어쓰지 않는다.

## 2. 저장 위치

서버의 `/home3/deepblue/work/AV_Drone`과 컨테이너의 `/workspace/AV_Drone`은 같은 bind mount다.

```text
/home3/deepblue/work/AV_Drone/
├── runtime/sim/<UTC시각_고유값>/
│   ├── px4/instance_2/log/.../*.ulg
│   ├── px4/instance_2/px4_stdout.log
│   ├── px4/instance_2/px4_stderr.log
│   ├── px4/instance_3/...
│   ├── ros/                 # ROS_LOG_DIR
│   ├── gazebo/              # GAZEBO_LOG_PATH
│   ├── ros_console.log      # 관리 실행의 launch 콘솔
│   ├── session.json         # 시작 시각·제한값
│   ├── mission.policy.json  # 드론별 종료 목표 상태
│   ├── stop.request.json    # 종료 요청 사유
│   ├── ros.result.json      # recorder/ROS 종료 결과
│   ├── sim.result.json      # sim 종료 결과
│   └── host.result.json     # 전체 관리 실행 결과
├── rosbags/<실행ID>_slam_debug/  # 기존 multi launch의 기록 위치 유지
└── artifacts/...                # 기존 지도·분석 결과 위치 유지
```

단일 기체 Classic SITL은 `rootfs/log`, 2대 실행은 `rootfs/2/log`, `rootfs/3/log`를 연결한다.
인스턴스가 8·9 등으로 설정된 실험도 해당 번호로 저장한다.
ROS 컨테이너의 별도 Docker writable layer와 sim의 `/root` volume 자체를 이전한 것은 아니다.
Gazebo 사용자 설정 등 모든 Docker 파일이 `/home3`로 옮겨지는 것은 아니다.

`runtime/`, `artifacts/`, `rosbags/`는 Docker 이미지 빌드 입력에서 제외했다.
따라서 복구용 대용량 백업을 새 이미지 빌드 컨텍스트로 보내지 않는다.

## 3. 앞으로 권장하는 실행 방법

### 현재 70m·드론 2대 설정: 먼저 검사만

서버 터미널에서 실행한다. 컨테이너 내부 명령이 아니다.

```bash
cd /home3/deepblue/work/AV_Drone
python3 scripts/run_guarded_experiment.py \
  --manifest src/drone_bringup/config/swarm_two_uav_phase2_70m.yaml \
  --check
```

`--check`는 디스크·컨테이너 mount·종료 상태·world·기체 수·PX4 인스턴스·spawn 좌표를 확인한다.
컨테이너를 시작하거나 파일을 생성하지 않는다. 모델 빌드, GPU, 실제 비행 성공까지 검증하는 것은 아니다.

### 실제 실험 시작

```bash
cd /home3/deepblue/work/AV_Drone
python3 scripts/run_guarded_experiment.py \
  --manifest src/drone_bringup/config/swarm_two_uav_phase2_70m.yaml
```

이 명령이 **정지 상태의 기존 sim/ros를 시작하고 ROS launch까지 실행**한다.
별도로 `docker compose up`을 먼저 실행하거나 두 번째 터미널에서 동일 launch를 다시 실행하지 않는다.
기존에 빌드된 ROS workspace와 현재 컨테이너 설정을 사용한다. 이미지/패키지를 자동 빌드하지 않는다.
기존 manifest의 `known_pose`/GT 기록 등 실험 설정도 그대로다. GT 없는 새 알고리즘 실험으로 바꾸는 작업이 아니다.

현재 실행기를 사용하려면 다음 조건이 맞아야 한다.

- 기존 `av_drone-sim-1`, `av_drone-ros-1`이 존재하고 정지 상태여야 한다.
- 두 컨테이너가 이 프로젝트를 `/workspace/AV_Drone`으로 writable bind mount해야 한다.
- sim의 시작 명령은 프로젝트의 `docker/sim/entrypoint.sh`, ros는 `sleep infinity`여야 한다.
- 이미 실행 중이면 자동으로 중단하거나 이어받지 않고 거부한다. 다른 작업 보호를 위한 동작이다.
- world·기체 수·인스턴스·spawn이 다르면 먼저 설정과 기존 데이터 보존 방식을 검토해야 한다. 자동 재생성하지 않는다.

중간에 중단하려면 실행 터미널에서 `Ctrl+C`를 누른다. 동일한 저장·종료 순서를 요청한다.
SSH 단절에 대비해 장시간 실험은 `tmux` 안에서 실행하는 것이 편하다. 호스트 실행기가 사라져도
sim/ROS 내부의 보호 프로세스는 시간·용량 검사를 계속하지만, 비어 있는 ROS 컨테이너까지 정리하는 것은 호스트 실행기의 역할이다.

### 단일 드론

```bash
python3 scripts/run_guarded_experiment.py --single --check
python3 scripts/run_guarded_experiment.py --single
```

기존 컨테이너가 단일 기체·인스턴스 0 설정이어야 한다. 현재 2대 설정에서는 의도적으로 거부한다.
종료 정책은 single launch가 사용하는 **설치된** `drone1_autonomy.yaml`에서 읽는다.
single launch에 원래 없던 rosbag 기록을 새로 추가하지는 않는다.

## 4. 자동 종료의 정확한 의미

1. 편도 드론은 `HOVER_AT_GOAL`, 복귀 드론은 `DONE`을 기다린다.
2. 모든 드론이 각각 요구된 상태를 5초간 유지하고 상태 heartbeat가 최신이어야 완료로 판단한다.
3. 현재 launch 아래의 `ros2 bag record`에 먼저 SIGINT를 보내고 기본 20초간 종료를 기다린다.
4. 이어서 ROS 프로세스 그룹을 SIGINT로 종료한다. 기본 20초 이후에도 남아 있으면 TERM/KILL로 단계적으로 정리한다.
5. sim은 관리 ROS 종료를 기다린 다음 PX4/Gazebo를 정리한다. 호스트 실행기는 자신이 시작한 정확한 컨테이너 ID만 정지시킨다.

`HOVER_AT_GOAL`/`DONE`은 **시뮬레이션 임무 상태이지 실제 착륙·disarm 완료 보증이 아니다.**
이 실행기는 실제 기체용 안전 착륙 기능이 아니다.

정상 완료 외에도 실행 명령 종료/실패, 상태 감시기 실패, Ctrl+C, 시간·용량 초과 시 정리한다.
상태 heartbeat가 처음부터 120초간 오지 않거나, 수신 후 10초 이상 끊겨도 감시기가 실패로 종료한다.
충돌이나 모든 종류의 노드 내부 오류를 별도로 판별하는 기능은 아니다.

`ros.result.json`에서 확인할 값:

- `reason: "mission_complete"`: 지정한 임무 상태 도달. `max_runtime`, `max_log_size`, `low_space:...`는 제한에 의한 중단.
- `clean_shutdown: true`: 검사한 프로세스가 강제 TERM/KILL 없이 정리됨. 발견한 recorder는 ROS 종료 전에 끝났고 metadata가 있음.
- `bags[].metadata_present`: 해당 bag의 `metadata.yaml` 존재 여부. 기록된 모든 센서 데이터의 완전성/품질 검증은 별도다.
- `forced: true` 또는 `remaining_pids`가 있으면 정상 종료로 취급하지 않는다.

관리 실행과 별도로 시작한 recorder, 다른 컨테이너/호스트의 recorder, 별도 세션으로 분리한 사용자 프로세스는 관리 대상이 아니다.
현재 launch의 recorder는 같은 프로세스 그룹에서 실행되는 것을 설치된 ROS 실행 라이브러리에서 확인했다.
SIGKILL, 서버 전원 차단, 극단적인 저장장치 장애에는 정상 저장을 보장할 수 없다.

## 5. 제한값 조정

설정 파일: [`docker/runtime/limits.json`](../docker/runtime/limits.json).

| 설정 | 기본값 | 동작 |
|---|---:|---|
| `root_min_free_gib` | 50GiB | `/` 여유 공간이 미만이면 시작 거부/중단 |
| `project_min_free_gib` | 100GiB | 프로젝트가 저장된 파일시스템 여유 공간이 미만이면 시작 거부/중단 |
| `max_log_gib` | 5GiB | 이번 `runtime/sim/<ID>` 전체 파일 크기 합계가 이상이면 중단 |
| `max_runtime_sec` | 1200초 | sim 시작부터 최대 20분. 빌드·준비 시간도 포함 |
| `poll_sec` | 2초 | 내부 보호 프로세스의 검사 간격 |
| `bag_grace_sec` | 20초 | recorder의 정상 종료 대기 |
| `ros_grace_sec` | 20초 | ROS의 정상 종료 대기 |
| `sim_grace_sec` | 15초 | sim의 정상 종료 대기 |
| `term_grace_sec` | 5초 | TERM 이후 KILL 전 대기 |

GiB는 1024³ byte다. 5GiB는 **드론 한 대당이 아니라 실행별 합계**이며 ROS 로그/콘솔 등도 포함한다.
rosbag은 기존 별도 폴더에 있으므로 이 5GiB 합계에 포함되지 않는다. rosbag 증가는 프로젝트 디스크 여유 및 최대 실행 시간으로 보호한다.
크기와 시간은 검사 및 정상 종료 대기 동안 초과할 수 있다. 디스크 quota나 정확한 하드 상한이 아니다.
값은 양수여야 하며 실험을 정지한 상태에서 수정하고 다음 실행에 적용한다.

20분이 필요한 실험 길이에 부족하면 먼저 예상 rosbag/ULog 용량을 확인한 뒤 늘린다.
2번인 PX4 기록 모드/프로필/토픽/주기 변경과 이 보호 한도 조정은 서로 다른 작업이다.

## 6. 기존 수동 실행과의 차이 / 적용 시점

- bind-mounted sim entrypoint를 수정했으므로 **기존 정지 컨테이너도 다음 시작부터** PX4 로그 경로·시간·용량 보호를 받는다. 재생성 불필요.
- **임무 완료 → rosbag → ROS → sim 전체 종료 연동은 위 관리 실행기를 사용해야 한다.** 예전처럼 launch를 별도로 수동 실행하면 해당 ROS/recorder가 자동 등록되지는 않는다.
- Compose에 수동 ROS의 `ROS_LOG_DIR`와 Docker 콘솔 로그 회전(`10m` × 3개)을 추가했다. 이 **컨테이너 생성 설정**은 기존 컨테이너에 소급 적용되지 않는다.
- 기존 컨테이너의 새 관리 실행은 ROS 로그 경로를 실행 시 직접 전달하므로 재생성 없이 `/home3`에 기록한다.
- Docker 로그 회전은 stdout/stderr 콘솔에만 적용된다. `.ulg` 크기 제한을 대신하지 않는다.
- 로그 회전을 적용하려고 지금 `--force-recreate`를 실행하지 않는다. 기존 컨테이너의 build/parameter/작은 로그 등 보존 확인이 먼저다.

서버 전체 Docker 저장 위치는 바꾸지 않았다. 다른 사용자/다른 컨테이너가 루트 디스크를 채우는 일까지 방지할 수는 없다.
이 구현은 이 실험의 새 로그 위치와 방치 실행을 관리하고, 서버 여유 공간 감소 시 이 실험을 중단하는 보호 장치다.

## 7. 검증과 한계

호스트 기본 검사:

```bash
python3 -m unittest discover -s test -p test_runtime_guard.py -v
python3 scripts/runtime_guard.py check
bash -n docker/sim/entrypoint.sh docker/sim/multi_px4_entrypoint.sh
docker compose config --quiet
```

단위 검사는 기존 파일 보존, 반복 실행, 용량/시간 한도, 중단 신호, 강제 종료 표시,
두 드론 상태 감지, 관련 없는/실행 중 컨테이너 거부를 확인한다.
ROS가 있는 환경에서는 실제 rosbag 종료 및 모의 상태 publisher를 이용한 목표 도달 통합 검사도 실행한다.
통합 검사는 네트워크를 차단한 별도 임시 ROS 컨테이너에서 수행하며 PX4/Gazebo 비행을 실행하지 않는다.

2026-09-11 검증 결과: 호스트 27개 통과·ROS 전용 2개 건너뜀, 격리 ROS 컨테이너에서 총 29개 모두 통과.
현재 70m manifest의 읽기 전용 사전 검사, Python/Shell 문법 검사, Compose 설정 검증도 통과했다.

**실제 PX4/Gazebo 비행으로 새 log 링크와 전체 종료 흐름을 확인하는 smoke run은 아직 하지 않았다.**
다음 실제 실험은 짧은 실행으로 로그가 `runtime/sim/<ID>/px4/...`에 생기는지 확인하고,
Ctrl+C 종료 후 rosbag metadata와 종료 결과 JSON을 확인한 다음 장거리 실험으로 진행한다.
