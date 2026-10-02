# 전체 길이 · 공통 관측 2구간 실험

실행 ID: `20260911T122110Z_21f6a1d0f520`

## 목적과 변경 범위

두 UAV가 서로 다른 시간에 두 구역의 장애물을 공통 관측하도록 고정 경로를 설정하고,
같은 rosbag에서 제약 0개·앞 구간 1개·뒤 구간 1개·두 구간 2개를 비교한다.
실시간 재회, 통신 패킷 공유, 자율적인 재회 계획을 구현하는 실험은 아니다.

- Gazebo Classic + PX4 + MAVROS + 기존 개별 SLAM/자율비행 노드를 그대로 사용.
- 기존 150m × 30m 원통 100개 월드를 변경하지 않음. world x=3m에서 145m까지 비행.
- 첫 구간 world x≈60–85.5m 유지, 후반 x≈115–135m에 두 번째 공통 관측 기회 추가.
- 경유점은 비행 제어용 local 좌표. 추가 지점의 원통 표면까지 거리를 사전 검사했으나 경로 전체의 충돌 회피를 보증하지 않음.
- drone2 이륙 후 대기 20초. 동시에 같은 위치에 모으지 않음.
- SLAM scan matching ON, 단독 loop closing OFF. 온라인 fused known_pose 지도는 오프라인 N-double 결과가 아님.
- 기존 컨테이너를 사용하고 sudo·재생성·과거 데이터 삭제 없음. 저장은 프로젝트의 `/home3` 경로.

## 분석 규칙 — GT 평가 전에 고정

분석 설정: `src/drone_cslam/config/lidar_no_gt_two_regions.yaml`.
기존 `lidar_no_gt_full.yaml`의 검색·정합·gate·pose graph 가중치를 유지한다.

1. GT 토픽을 읽지 않고 LiDAR, odometry, TF로 scan/keyframe을 구성한다.
2. 형상 descriptor와 raw-SLAM 공간 prior(22m)로 후보를 검색한다.
3. 이동·회전 정합 후 overlap≥0.25, RMSE≤0.35m 등 기존 gate를 적용한다.
4. 양쪽 anchor가 각각 4–20m 떨어진 독립 이웃 후보끼리 보정 일관성을 검사한다.
5. 지지 관계의 연결 성분별로 overlap이 가장 높은 대표를 선택한다. 동률은 cost, pair 순서.
6. 대표를 raw-SLAM 진행 순서로 정렬하고, 양쪽 진행거리 각각 30m·공간거리 각각 25m 이상 떨어진 앞의 두 구간을 선택한다.
7. 경로의 의도 구간, GT pose, GT overlap audit는 1–6번에 입력하지 않는다.
8. 동일 raw pose에서 동일 prior·intra factor·가중치로 각 비교군을 최적화한다.
9. 추정 pose·제약·선택 기록·추정 지도를 저장한 뒤에만 GT를 읽어 평가한다.

| 조건 | 드론 간 제약 | 해석 |
|---|---:|---|
| B0 | 0 | 원래 개별 SLAM pose |
| B1 | 0 | 초기 정렬 + 외부 MAVROS/PX4 odometry 기준 |
| N-prior | 0 | 같은 pose graph에 알려진 시작 prior만 적용한 대조군 |
| N-single-A | 1 요청 | 앞쪽 검증 구간만 |
| N-single-B | 1 요청 | 뒤쪽 검증 구간만 |
| N-double | 2 요청 | A와 B를 함께 적용 |

`available`은 필요한 구간 수 확보 여부, `accepted_factor_count`는 채택 수,
`applied_factor_count`는 최적화 성공 후 실제 반영 수다. 실패하면 B0를 보존하며 성공으로 표시하지 않는다.
독립 구간이 하나뿐이면 두 번째 제약을 GT로 강제로 만들지 않는다.

N-prior를 추가한 이유: 기존 B0→N 비교에는 inter-UAV 제약뿐 아니라 알려진 시작 pose prior의 영향도 있다.
그 영향을 분리하고, 같은 prior를 사용하는 N-single-A/B와 N-double의 차이를 중심으로 해석한다.

## 실행과 저장

```bash
cd /home3/deepblue/work/AV_Drone
python3 scripts/run_guarded_experiment.py \
  --manifest src/drone_bringup/config/swarm_two_uav_full_overlap2.yaml --check
python3 scripts/run_guarded_experiment.py \
  --manifest src/drone_bringup/config/swarm_two_uav_full_overlap2.yaml
```

- `runtime/sim/<ID>/`: PX4·ROS 로그, 용량/시간 보호, 종료 검사.
- `rosbags/<ID>_slam_debug/`: 원본 기록.
- `artifacts/<ID>/drone1`, `drone2`, `swarm`: 비행 중 지도·상태.
- `artifacts/<ID>/lidar_registration/`: 오프라인 6조건, GT 평가, 그림, 정합 근거.
- Streamlit: `http://166.104.230.197:8501` → **전체맵 · LiDAR 2구간 비교**.

분석 명령은 ROS Humble 환경에서 실행한다. 기존 결과 폴더가 비어 있지 않으면 분석기는 덮어쓰기를 거부한다.

```bash
source /opt/ros/humble/setup.bash
cd /workspace/AV_Drone
export PYTHONPATH=/workspace/AV_Drone/src/drone_cslam:$PYTHONPATH
python3 -m drone_cslam.offline_lidar_eval \
  --bag rosbags/20260911T122110Z_21f6a1d0f520_slam_debug \
  --config src/drone_cslam/config/lidar_no_gt_two_regions.yaml \
  --output artifacts/20260911T122110Z_21f6a1d0f520/lidar_registration
```

## 결과

비행 완료: rosbag 533.078초, 192,981 messages, LiDAR 7,822/7,821장.
두 드론 모두 HOVER_AT_GOAL. 강제 종료 없이 recorder를 먼저 닫음.
외부 odometry 최종 누적거리 약 172.12/177.60m(이륙·대기 움직임 포함).

### 기본 검색의 실패와 추가 진단

기존 descriptor 상위 2개 검색은 43개 후보 중 첫 구간의 대표 1개만 채택했다.
후반 pair 21은 overlap 0.302·RMSE 0.235m로 형상 gate를 통과했으나 독립 이웃 지지가 없어 탈락했다.
GT를 읽지 않는 별도 진단에서 다음을 확인했다.

- drone1 raw x≈130.8/132.8/134.8/136.8m의 submap 점 수가 66/52/42/32개라 최소 80개 기준에 미달.
- raw x≈127.1m의 drone1 anchor에서는, raw x≈128.3m인 drone2 후보가 형상 순위 6위라 검색에서 제외됨.
- 이 후보의 raw 위치 거리는 11.7m로 공간 prior 22m 안이지만, 비슷한 원통 형상이 먼 후보를 앞에 놓을 수 있음.

GT 평가 결과를 근거로 임계값을 낮추지 않고, **형상 상위 2개 + 거리 상위 2개의 합집합**을 검사하는
별도 탐색적 분석을 추가했다. `lidar_no_gt_two_regions_expanded.yaml`을 사용하며 후보 총한도는 120개.
최소 점 수, 정합·일관성 gate, 구간 분리, 최적화 가중치는 모두 동일하다.
이는 같은 bag에서 검색 누락 원인을 점검한 개발 실험이며, 별도 데이터에서 검증한 일반화 성능은 아니다.
기본 검색의 실패 결과도 보존한다. 확장 검색은 64개 후보를 검사했지만 후반 구간의 독립 이웃 지지가
여전히 없어 대표 제약은 1개만 선택됐다. 두 검색의 6조건 추정 pose 배열은 정확히 동일하다.
따라서 **2구간 관측 기록은 확보했지만, 1제약 대 2제약 보정 효과 비교는 아직 성립하지 않았다.**

### 정량 결과 — 기본 검색, 동일 rosbag

| 조건 | 실제 드론 간 제약 | 종단 상대 위치 [m] ↓ | 상대 위치 RMSE [m] ↓ | 상대 yaw RMSE [°] ↓ | 지도 Chamfer [m] ↓ |
|---|---:|---:|---:|---:|---:|
| B0 | 0 | 4.055 | 2.201 | 1.817 | 1.285 |
| B1 | 0 | 0.932 | 0.214 | 3.199 | 0.357 |
| N-prior | 0 | 11.252 | 6.428 | 4.474 | 1.694 |
| N-single-A | 1 | 0.279 | 0.314 | 1.155 | 1.645 |
| N-single-B | 0 | — | — | — | — |
| N-double | **1 / 요청 2** | — | — | — | — |

N-single-B는 미성립하여 저장 배열은 B0와 같고, N-double은 A 제약만 사용해 N-single-A와 같다.
원본 CSV/Streamlit에는 fallback 수치와 상태를 함께 남겼지만, 위 표는 유효한 비교가 아니라서 `—`로 표시한다.

- B0→N-single-A의 종단 상대 위치 오차는 약 93.1% 감소했다.
- 그러나 지도 Chamfer는 약 28.0% 증가했고, 지도 F1도 0.226→0.116으로 악화했다.
- 종단 상대 yaw 오차는 0.360°→2.602°로 악화했다. 전체 상대 yaw RMSE 감소와 다른 결과다.
- 절대 위치 RMSE는 drone1 1.122→1.108m, drone2 1.152→1.143m로 변화가 작았다.
- 따라서 “지도 오차 보정 성공”, “2번 보정이 1번보다 좋다”는 주장을 할 수 없다.
- N-prior의 악화는 알려진 시작 pose를 강하게 고정하는 처리 자체도 결과에 영향을 준다는 진단이다.
  N-prior→N-single-A와 B0→N-single-A를 함께 봐야 한다. 시작 prior 및 초기 SLAM pose 안정화는 후속 점검 항목이다.

### 실제 공통 관측과 채택 제약

GT는 아래의 사후 평가에만 사용했다.

- 후반 world x=110–135m에서 각 5m bin의 공통 hit-voxel 비율은 약 0.19–0.28이었다.
- x=100–110m에서는 공통 hit voxel이 0이었다. 앞 구간과 떨어진 후반 공통 관측이 실제로 존재한다.
- 이 비율은 정합기의 submap overlap 점수와 다른 지표다. 공통 관측이 있다는 것만으로 신뢰할 만한 정합이 보장되지 않는다.
- 채택한 A는 pair 11: source keyframe 37, target keyframe 36. GT 평가상 관측 위치는
  각각 (67.82, −3.65)m, (69.74, 2.98)m, 관측 시각 차이는 10.676초였다.
- A의 정합 overlap 0.680, inlier RMSE 0.204m. GT 평가 상대 pose 오차 0.151m / 0.371°.
- 기본 검색의 후반 pair 21도 사후 GT 오차는 0.117m / 0.582°였으나, 이 사실을 보고 채택하지 않았다.
  추론 단계에서 독립 이웃 검증에 실패했으므로 계속 제외한다. GT 정답을 보고 gate를 우회하면 본 실험의 취지를 잃는다.

### 기록·시각화 검증

- SQLite `quick_check=ok`, metadata 메시지 수와 DB 메시지 수 일치.
- bag SHA-256: `66a79e48862e8e9bbb8b584ffca838abbabce5101e90dcc82ae5d1d44a56f704`.
- bag 약 110.31MiB. 원본과 이전 1구간 실험을 삭제하지 않음.
- 최소 LiDAR 거리 0.208/0.212m: 위험 근접이 있으므로 안전 항법 성공으로 주장하지 않음.
- host 종료 코드 0, rosbag 선종료, sim/ROS 컨테이너 정지. 일부 ROS recorder의 종료 시 `rcl_shutdown already called` 로그는 남음.
  `clean_shutdown=true`는 강제 종료가 없다는 의미이지 모든 노드가 오류 코드 0이라는 보증은 아님.
- cSLAM 회귀 22개, 대시보드 회귀 37개 통과. 실제 데이터 Streamlit 화면에서도 미성립 경고와 적용 제약 1개 표시를 확인.
- 기본 결과: `artifacts/<ID>/lidar_registration/`.
- 추가 검색 진단: `artifacts/<ID>/lidar_registration_retrieval_expanded/`. 기본 결과를 덮어쓰지 않는다.

## Streamlit에서 볼 순서

1. **전체맵 · LiDAR 2구간 비교** 선택.
2. `적용한 LiDAR 제약: 1개`, `2구간 비교 미성립` 경고 확인.
3. **공통 관측 위치** 그림: GT 궤적, 채택 A 연결선, 후반 공통 관측 bar 확인.
4. **절대 위치·yaw 오차**, **전체 지도 오차**에서 상대 오차 감소와 지도 악화를 구분.
5. **LiDAR 상세**에서 조건별 요청/채택/실제 적용 수와 Registration의 탈락 이유 확인.
6. **같은 rosbag의 검색 설정**에서 기본 검색 / 추가 진단을 전환해 43개·64개 후보 결과를 확인.

## 다음 실험에서 바꿀 것

검색 후보 추가만으로 해결되지 않았으므로 바로 3구간으로 늘리는 것은 권장하지 않는다.
먼저 **두 번째 구간의 관측 가능성과 독립 검증 기회**를 강화한다.

1. 후반 구간을 빈 공간이 아니라 여러 원통이 양쪽 LiDAR 8m 범위에 함께 들어오는 곳으로 옮긴다.
2. 단지 waypoint 1개가 가까운 것이 아니라, 두 드론이 15–20m 정도 같은 장애물 묶음을 계속 볼 수 있게 한다.
3. 비행 전 world geometry로 공통 가시성·예상 점 수를 검사하고, 비행 후에는 GT 없이 점 수·정합 성공·독립 anchor 수를 검사한다.
4. 하나의 후반 pair가 맞더라도 이웃 지지가 없다면 성공으로 세지 않는다. GT를 보고 임계값을 낮추지 않는다.
5. 두 독립 구간이 실제로 검증된 bag에서 동일 A / B / A+B 비교를 다시 수행한다.
6. 그 다음 여러 seed/반복 비행으로 일반화를 검증한다. 현재 수치는 단일 비행의 개발 실험이다.
