# 70m one-way Multi-UAV LiDAR 상대 pose 실험 보고서

## 1. 이번 실험의 질문

Phase 1에서는 GT로 만든 inter-UAV 상대 pose factor가 SLAM drift를 줄일 수
있음을 확인했다. 이번 Phase 2의 질문은 다음과 같다.

> GT가 상대 pose 값을 알려주지 않아도, 두 UAV의 LiDAR 서브맵 정합으로 만든
> 상대 pose factor가 70m one-way SLAM drift를 실제로 줄일 수 있는가?

이번 실험에서도 GT는 같은 진행 구간의 keyframe pair를 선택하고 결과를
채점하는 데 사용한다. LiDAR 등록기와 pose graph에 입력되는 상대 pose
measurement에는 GT transform을 전달하지 않는다. 따라서 이 결과는
`controlled registration` 결과이며 자동 장소 인식 결과는 아니다.

## 2. 시뮬레이션과 rosbag

- Run ID: `2026-08-27_lidar_phase2_70m_codex_v1`
- World: `random_cylinders_double` (x=0~150m)
- UAV: 2대, 시작 y=-7.5m / +7.5m
- Local goal: 두 UAV 모두 x=70m
- 비행: one-way, return/loop closure 없음
- 두 UAV 모두 최종 상태 `HOVER_AT_GOAL`, `goal_reached=true`
- 실제 outbound path: drone1 76.97m, drone2 81.94m
- outbound time: drone1 194.85초, drone2 211.80초
- 기록된 최소 LiDAR 거리: drone1 0.221m, drone2 0.218m
- rosbag: 258.39초, 45.0MiB, 총 48,024 messages
- LaserScan: UAV별 3,762개
- GT odometry: UAV별 2,571개
- offline 동기화 후 valid scan: UAV별 3,716개
- keyframe: drone1 39개, drone2 41개

두 UAV 모두 goal에는 도달했지만 최소 LiDAR 거리가 약 0.22m였으므로, 이
run을 navigation safety 성능 근거로 사용하면 안 된다. 이번 artifact의 용도는
one-way SLAM drift와 inter-UAV registration 분석이다.

실행 manifest는
`src/drone_bringup/config/swarm_two_uav_phase2_70m.yaml`이다. 런타임
`/swarm/global_map`은 기존 시스템을 안정적으로 실행하기 위해 known-pose
map을 사용했지만, B0와 Phase 2 분석은 rosbag의 raw `slam_toolbox` TF를
다시 읽어 계산했다. 즉 운영 map을 B0로 가장하지 않았다.

## 3. 실행 순서

### 3.1 Gazebo/PX4/ROS 시작

```bash
cd /home3/deepblue/work/AV_Drone
HEADLESS=1 VEHICLE_COUNT=2 PX4_SITL_WORLD=random_cylinders_double \
  docker compose up -d --force-recreate sim ros

docker compose exec -T ros bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /workspace/AV_Drone &&
  colcon build --packages-select \
    drone_bringup drone_control drone_perception drone_planning \
    drone_safety drone_metrics drone_slam drone_map_fusion \
    drone_cslam ros_states --symlink-install'
```

### 3.2 70m 비행과 기록

```bash
docker compose exec ros bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /workspace/AV_Drone &&
  source install/setup.bash &&
  ros2 launch drone_bringup multi_drone_slam_fusion.launch.py \
    manifest:=/workspace/AV_Drone/src/drone_bringup/config/swarm_two_uav_phase2_70m.yaml \
    run_id:=2026-08-27_lidar_phase2_70m_codex_v1'
```

두 UAV가 `HOVER_AT_GOAL`에 들어간 뒤 Ctrl-C로 launch를 정상 종료했다.
rosbag recorder가 cache를 기록하고 `Recording stopped`를 출력한 것을 확인했다.

### 3.3 Oracle 상한선

```bash
docker compose exec -T ros bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /workspace/AV_Drone &&
  source install/setup.bash &&
  ros2 run drone_cslam oracle_drift_eval \
    --bag /workspace/AV_Drone/rosbags/2026-08-27_lidar_phase2_70m_codex_v1_slam_debug \
    --config /workspace/AV_Drone/src/drone_cslam/config/oracle_phase1.yaml \
    --output /workspace/AV_Drone/artifacts/2026-08-27_lidar_phase2_70m_codex_v1/oracle_correction'
```

### 3.4 LiDAR 상대 pose 추정

```bash
docker compose exec -T ros bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /workspace/AV_Drone &&
  source install/setup.bash &&
  ros2 run drone_cslam lidar_relative_eval \
    --bag /workspace/AV_Drone/rosbags/2026-08-27_lidar_phase2_70m_codex_v1_slam_debug \
    --config /workspace/AV_Drone/src/drone_cslam/config/lidar_registration_phase2.yaml \
    --output /workspace/AV_Drone/artifacts/2026-08-27_lidar_phase2_70m_codex_v1/lidar_registration \
    --overwrite'
```

## 4. LiDAR 등록 방법

1. GT 진행 거리로 R-single 1쌍 또는 R-periodic 10m 간격 7쌍을 선택한다.
2. 각 keyframe 전후 raw-SLAM 진행 거리 5m의 scan을 모은다.
3. scan motion compensation은 B0 pose만 사용한다.
4. hit endpoint를 0.20m voxel로 downsample해 local submap을 만든다.
5. B0 상대 pose 주변 ±3m, ±25°를 0.5m/2.5° 간격으로 coarse search한다.
6. symmetric truncated Chamfer cost의 상위 가설을 Powell 방식으로 refine한다.
7. 0.75m/5° 안의 중복 수렴 결과는 같은 pose 가설로 합친다.
8. overlap, inlier RMSE, 두 번째 가설과의 score margin, 최대 보정량을 검사한다.
9. gate를 통과한 LiDAR factor만 robust SE(2) pose graph에 넣는다.
10. 같은 scan을 B0/B1/R-single/R-periodic pose로 재투영해 비교한다.

현재 gate는 다음과 같다.

- overlap ratio ≥ 0.18
- inlier RMSE ≤ 0.40m
- distinct-hypothesis score margin ≥ 0.02
- B0 기준 보정량 ≤ 3m, ≤ 25°

## 5. 결과

### 5.1 Oracle 상한선

| 조건 | 종단 상대 위치 오차 [m] | 종단 yaw 오차 [deg] | Chamfer [m] | F1 |
|---|---:|---:|---:|---:|
| B0 | 1.0669 | 0.2133 | 1.3184 | 0.1804 |
| B1 | 0.1787 | 3.3984 | 0.4410 | 0.5110 |
| O-single | 0.6102 | 1.2594 | 0.5875 | 0.5273 |
| O-periodic | 0.0888 | 0.2730 | 0.9350 | 0.3984 |

O-periodic은 10m 간격 Oracle factor 7개를 사용해 B0 종단 상대 위치 오차를
91.7% 줄였다. 따라서 이 70m 데이터에도 inter-UAV 보정 여지는 분명히 있다.

### 5.2 LiDAR factor 선택

| 진행 거리 [m] | overlap | score margin | B0 상대 pose 오차 [m] | LiDAR 오차 [m] | 결과 |
|---:|---:|---:|---:|---:|---|
| 10 | 0.061 | 0.000 | 0.361 | 2.526 | reject: low overlap, ambiguous |
| 20 | 0.173 | 0.057 | 0.633 | 0.752 | reject: low overlap |
| 30 | 0.371 | 0.177 | 0.779 | 0.440 | accept |
| 40 | 0.230 | 0.108 | 1.015 | 0.603 | accept |
| 50 | 0.172 | 0.061 | 1.136 | 0.700 | reject: low overlap |
| 60 | 0.000 | 0.000 | 1.140 | 2.556 | reject: no overlap, ambiguous |
| 70 | 0.006 | 0.001 | 1.013 | 3.131 | reject: no overlap, ambiguous |

R-periodic 7쌍 중 2쌍만 factor로 채택됐다. 채택 factor의 median 상대 위치
오차는 0.521m, median yaw 오차는 1.329°였다. 전체 pair 기준 상대 위치 오차
median은 B0 1.013m에서 LiDAR 0.752m로 감소했다.

### 5.3 PGO와 지도

| 조건 | 종단 상대 위치 오차 [m] | 종단 yaw 오차 [deg] | Chamfer [m] | F1 |
|---|---:|---:|---:|---:|
| B0 | 1.0669 | 0.2133 | 1.3184 | 0.1804 |
| B1 | 0.1787 | 3.3984 | 0.4410 | 0.5110 |
| R-single | 0.9067 | 3.0372 | 0.8527 | 0.3860 |
| R-periodic | 0.6347 | 2.5950 | 0.7743 | 0.4211 |

R-periodic은 B0 대비 다음 효과가 있었다.

- 종단 상대 위치 오차 40.5% 감소
- map Chamfer 41.3% 감소
- occupied F1 0.1804 → 0.4211
- drone1 ATE RMSE 0.3956m → 0.3346m
- drone2 ATE RMSE 1.2401m → 0.5352m

반면 종단 yaw 오차는 0.2133°에서 2.5950°로 증가했다. 따라서 이 결과를
모든 pose 성분이 개선된 최종 방법으로 주장하면 안 된다.

## 6. 해석

이번 결과는 다음 두 가지를 동시에 보여준다.

1. GT 상대 pose를 LiDAR measurement로 일부 대체해도 translation drift와 지도
   일치도를 줄일 수 있다.
2. 원통형 장애물, 작은 공통 관측 영역, 후반부 overlap 소멸 때문에 7개 중
   5개 factor는 안전하게 사용할 수 없었다.

특히 GT overlap audit은 0~20m 0.027, 20~40m 0.250, 40~60m 0.103,
60m 이후 0.0이었다. 30m/40m factor만 채택되고 60m/70m가 거절된 결과와
일치한다. 이 실험에서 “거절하고 B0를 유지”하는 것은 실패 처리가 아니라
false-positive factor가 지도를 망가뜨리는 것을 막는 설계 결과다.

또한 B1은 여전히 R-periodic보다 좋다. B1은 spawn 관계와 odometry를 사용한
강한 기준선이므로, 현재 LiDAR 등록법이 모든 baseline을 이겼다고 주장할 수
없다.

## 7. 다음 개발 우선순위

1. GT progress association을 scan-context/cylinder landmark descriptor 기반 후보
   검색으로 교체한다.
2. 원통 중심과 반지름을 직접 추출해 yaw가 약한 endpoint 정합을 보강한다.
3. registration covariance 또는 Hessian으로 translation/yaw factor weight를
   분리한다.
4. 50m처럼 overlap이 0.18 바로 아래인 구간은 temporal consistency와 인접
   factor 검증을 추가한 뒤 재평가한다.
5. 서로 다른 world seed와 반복 run으로 성공률, factor precision, drift 감소량의
   평균·표준편차를 보고한다.
6. 마지막에만 온라인 후보 교환과 통신량/latency 실험으로 확장한다.

## 8. 결과 확인

```bash
cd /home3/deepblue/work/AV_Drone
./scripts/run_quant_dashboard.sh
```

Streamlit의 `Oracle Drift Correction` 탭에서 상한선을, `LiDAR Registration`
탭에서 pair별 overlap, reject 이유, 정합 전후 그림, R-single/R-periodic
trajectory/map을 확인한다.

주요 원본 artifact:

```text
rosbags/2026-08-27_lidar_phase2_70m_codex_v1_slam_debug/
artifacts/2026-08-27_lidar_phase2_70m_codex_v1/oracle_correction/
artifacts/2026-08-27_lidar_phase2_70m_codex_v1/lidar_registration/
```
