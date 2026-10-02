# Oracle 기반 Multi-UAV SLAM drift 보정 — 1차 개발 사용법

## 구현 목적

이 단계는 자동 원통 인식이나 장소 인식을 구현하기 전에 다음 질문을 검증한다.

> 신뢰 가능한 Inter-UAV 상대 pose constraint가 존재하면 현재 raw `slam_toolbox`의 차등 drift와 지도 왜곡을 줄일 수 있는가?

Gazebo ground truth는 Oracle factor 생성과 평가에만 사용한다. 최종 제안 알고리즘의 센서 입력으로 주장하지 않는다.

## 포함된 기능

- Gazebo `/gazebo/get_entity_state` 기반 10 Hz GT odometry 기록
- `/scan`, `/odom`, `/tf`, GT odometry를 포함한 ROS 2 bag 생성
- scan timestamp 기준 raw SLAM, known-pose, GT trajectory 동기화
- 2 m/5° keyframe과 frozen scan-to-keyframe 재투영
- `B0`, `B1`, `O-single`, `O-periodic` 비교
- SciPy Huber SE(2) pose graph optimization
- occupancy map 재생성, trajectory/map/overlap metric, Markdown report 생성

자동 원통 검출, descriptor, 실제 LiDAR inter-UAV registration, 실시간 corrected-map publish는 2차 이후 범위다.

## 최초 빌드

```bash
cd /home3/deepblue/work/AV_Drone
docker compose build ros

VEHICLE_COUNT=2 \
PX4_SITL_WORLD=random_cylinders_double \
docker compose up -d --force-recreate sim ros

docker compose exec ros bash -lc '
  source /opt/ros/humble/setup.bash
  cd /workspace/AV_Drone
  colcon build --packages-select drone_cslam drone_bringup --symlink-install
'
```

## 30 m smoke bag 기록

터미널 1에서 launch를 실행한다. `RUN_ID`는 bag과 artifact를 연결하는 식별자다.

```bash
RUN_ID="$(date +%Y-%m-%d_%H-%M-%S)_oracle_phase1_smoke"

docker compose exec ros bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/AV_Drone/install/setup.bash
  ros2 launch drone_bringup multi_drone_slam_fusion.launch.py \\
    manifest:=/workspace/AV_Drone/src/drone_bringup/config/swarm_two_uav_phase1_smoke.yaml \\
    run_id:=${RUN_ID}
"
```

두 UAV가 목표에 도달하고 artifact가 저장된 후 `Ctrl+C`로 launch를 종료한다. bag 경로는 다음과 같다.

```text
/home3/deepblue/work/AV_Drone/rosbags/<RUN_ID>_slam_debug
```

## Offline Oracle 분석

```bash
RUN_ID=<기록할 때 사용한 RUN_ID>

docker compose exec ros bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/AV_Drone/install/setup.bash
  ros2 run drone_cslam oracle_drift_eval \\
    --bag /workspace/AV_Drone/rosbags/${RUN_ID}_slam_debug \\
    --config /workspace/AV_Drone/src/drone_cslam/config/oracle_phase1.yaml \\
    --output /workspace/AV_Drone/artifacts/${RUN_ID}/oracle_correction
"
```

기존 출력 폴더에 명시된 파일을 다시 생성하려면 `--overwrite`를 추가한다. 이 옵션은 폴더 전체를 삭제하지 않는다.

주요 결과:

```text
oracle_correction/
├── manifest.json
├── keyframes.csv
├── factors.jsonl
├── trajectories/
├── maps/
├── metrics.json
├── metrics.csv
├── overlap_audit.csv
└── report.md
```

## 발표용 분석 그림 생성

host Python에는 `matplotlib`이 없을 수 있으므로 프로젝트 ROS 컨테이너에서 생성한다.

```bash
RUN_ID=<분석할_RUN_ID>

docker compose exec -T ros bash -lc "
  cd /workspace/AV_Drone
  python3 scripts/render_oracle_phase1_presentation.py \
    --artifact artifacts/${RUN_ID}/oracle_correction
"
```

생성 위치와 내용:

```text
artifacts/<RUN_ID>/oracle_correction/presentation/
├── 01_pipeline.png
├── 02_trajectories.png
├── 03_differential_error.png
├── 04_oracle_factors.png
├── 05_map_comparison.png
├── 06_map_error_overlay.png
├── 07_metric_summary.png
└── 08_gate_summary.png
```

현재 smoke 결과를 사용하는 Marp 발표 원고는
`docs/oracle_phase1_presentation.md`에 있다. 다른 run으로 교체할 때는
Markdown의 image path를 새 artifact 경로로 바꾸고 위 명령으로 그림을 다시 만든다.

Node.js 18 이상과 Chrome/Edge/Firefox 중 하나가 설치된 로컬 PC에서는 다음과
같이 PowerPoint 또는 PDF로 내보낼 수 있다. 이 저장소에서 만든 신뢰 가능한
Markdown에 한해서 local image 접근을 허용한다.

```bash
cd /path/to/AV_Drone

npx @marp-team/marp-cli@latest \
  docs/oracle_phase1_presentation.md \
  --pptx --allow-local-files \
  -o oracle_phase1_presentation.pptx

npx @marp-team/marp-cli@latest \
  docs/oracle_phase1_presentation.md \
  --pdf --allow-local-files \
  -o oracle_phase1_presentation.pdf
```

## 137 m 본 실험

smoke와 같은 명령에서 manifest만 다음 파일로 바꾼다.

```text
/workspace/AV_Drone/src/drone_bringup/config/swarm_two_uav_phase1_full.yaml
```

서로 독립적인 run ID로 3회 기록한 후 각 `report.md`의 `B0`, `O-single`, `O-periodic`을 비교한다. 단일 run 성공만으로 다음 단계로 넘어가지 않는다.

## 검증 명령

```bash
docker compose exec ros bash -lc '
  source /opt/ros/humble/setup.bash
  cd /workspace/AV_Drone
  source install/setup.bash
  colcon test --packages-select drone_cslam --event-handlers console_direct+
  colcon test-result --verbose
'
```

현재 시험은 SE(2), 보간/keyframe, synthetic drift optimization, map reprojection과 synthetic ROS 2 bag 전체 파이프라인을 포함한다.

## 판정 기준

- 모든 입력 stream의 공통 시간 구간 안에서 synchronized scan 누락률 1% 이하
- 두 Oracle optimizer 모두 정상 수렴
- `O-periodic` differential endpoint error가 `B0`보다 30% 이상 감소
- occupied Chamfer distance가 20% 이상 감소
- occupied F1 감소가 0.02 이하
- `O-periodic`의 후반 differential error가 `O-single`보다 작음

Gate가 실패하면 자동 장소 인식으로 넘어가지 않고, `metrics.json`과 `factors.jsonl`을 기준으로 common-mode drift, intra factor weight, Oracle factor 공간 분포를 먼저 분석한다.
