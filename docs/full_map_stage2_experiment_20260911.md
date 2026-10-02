# 전체 맵 길이를 사용한 2-UAV 2단계 실험 결과

실험일: 2026-09-11. 실행 ID: `20260911T091146Z_bed606a28e99`.

## 1. 결론

실제 Gazebo Classic + PX4 + ROS 2 시뮬레이션을 실행하고 rosbag을 기록했다.
짧은 smoke 검증 후, 기존 70m 대신 150m 길이 월드의 끝부분까지 두 드론을 비행시켰다.
기록된 LiDAR submap에서 GT 없이 후보를 찾고 상대 pose를 추정해, 비행 종료 후 제약 1개로 보정했다.

**기록·분석 파이프라인은 완료됐지만, 전체 지도 정확도 개선은 아직 달성하지 못했다.**
두 드론 사이의 종단 상대 위치 오차는 7.935m에서 1.245m로 약 84.3% 감소했다.
반면 GT 기준 지도의 점유셀 Chamfer 거리는 1.632m에서 2.110m로 약 29.3% 증가했다.
따라서 화면의 `종합 개선 판정: FAIL`은 분석 실행 오류가 아니라 지도 개선 기준 미달을 뜻한다.

## 2. 실제 비행과 기록

| 항목 | 내용 |
|---|---|
| 월드 | `random_cylinders_double`, 길이 150m × 폭 30m, 기존 원통 장애물 환경 |
| 출발 / 목표 | world x=3m → 145m, 전진 목표 거리 142m; 끝벽까지 5m 여유 |
| 경로 | 기체별 7개 waypoint, 중간에 가까운 구역을 관측하고 다시 분리 |
| 주 공통 관측 구간 | 사후 5m-bin 관측 감사에서 world x 약 55~90m |
| 시간차 | drone2는 이륙 후 20초 대기; 같은 장소를 동시에 지나야 하는 실험이 아님 |
| 실제 누적 비행거리 | 외부 odometry 궤적 기준 drone1 약 166.0m, drone2 약 174.5m |
| 목표 상태 | 두 기체 모두 `HOVER_AT_GOAL`, `goal_reached=true` |
| rosbag | 516.809초, 185,795개 메시지, 약 107MiB |
| LaserScan | drone1 7,494개 / drone2 7,485개 |
| 평가용 GT odometry | drone1 5,095개 / drone2 5,089개 |
| 종료 | `mission_complete`, recorder 정상 저장 후 ROS와 sim 종료, 강제 종료 없음 |

전체 길이 활용과 전체 공간 100% 관측은 다르다. GT pose로 동일 scan을 투영한 평가용 지도는
x=0.15~149.85m를 관측했고, 평가 격자 `[-1,151] × [-16,16]`의 약 85.58%가 관측됐다.
가려진 영역과 격자 외곽 여백이 남으므로 완전한 면적 커버리지를 주장하지 않는다.

또한 최소 기록 LiDAR 거리가 drone1 0.218m, drone2 0.212m까지 내려갔다.
목표 도달·기록 성공을 안전 항법 검증 성공과 동일하게 해석하면 안 된다.

### 준비 과정에서 보존한 실행

- `20260911T084642Z_48bae2d0ce0e`: GT recorder가 Gazebo entity 생성 전에 응답 필드에 접근해 실패한 smoke. 중단·보존했으며 성공 결과에 포함하지 않았다.
- `20260911T090605Z_c15f7a96ce08`: 응답 처리 수정 후 15m smoke 성공. 센서·GT 기록과 정상 종료를 확인했다.
- `20260911T091146Z_bed606a28e99`: 본 보고서의 전체 길이 실행. 실패한 smoke와 별도 데이터다.

## 3. B0, B1, N-single, GT의 의미

| 이름 | 실제 입력과 역할 |
|---|---|
| B0 | 초기 좌표 정렬 + raw SLAM pose. SLAM의 `map→odom`과 외부 odometry를 결합한 결과이며 드론 간 보정은 없음 |
| B1 | 초기 좌표 정렬 + MAVROS/PX4 외부 odometry. SLAM `map→odom` 보정은 적용하지 않은 비교군 |
| N-single | B0에 LiDAR 정합으로 추정한 드론 간 상대 pose 제약 1개를 추가해 pose graph 최적화 |
| GT / GT-reference | Gazebo 정답 pose와 그 pose로 동일 LiDAR scan을 투영한 평가용 지도. 보정 입력이 아님 |

이번 결과에는 `O-single`, `O-periodic`을 새로 계산하지 않았다.
이전 Oracle 실험은 GT에서 상대 pose 제약을 만들었지만, 이번 `N-single`은 LiDAR에서 만든다.
기존 70m의 GT-associated `R-single/R-periodic` 결과와도 구분해야 한다.

온라인 launch의 `fusion_source: known_pose`는 외부 odometry 기반 지도 융합이다.
그 화면의 지도와 이번 오프라인 `N-single` 지도를 혼동하지 않도록 별도 탭·파일로 보관했다.

## 4. 어떻게 GT 없이 보정했는가

1. rosbag에서 scan, odometry, TF만 먼저 읽는다. GT topic은 역직렬화하지 않는다.
2. raw SLAM 이동량을 기준으로 keyframe과 앞뒤 약 5m submap을 만든다. GT로 시간 유효성이나 keyframe을 고르지 않는다.
3. 점 사이 거리 분포로 LiDAR 형상 특징을 만들고, raw SLAM 위치 prior 22m 안에서 비교 후보를 찾는다.
4. 후보마다 평면 이동 ±6m, yaw ±30° 범위를 탐색한 후 정밀 정합한다.
5. 중첩 비율, 잔차, 최적해 구별 정도, 보정량을 검사하고 이웃의 독립 anchor pair가 비슷한 보정을 지지하는지 확인한다.
6. 통과 후보 중 중첩 비율·정합 비용으로 대표 1개를 선택하고, 기존 SLAM 연결과 함께 SE(2) pose graph를 최적화한다.
7. 추정 pose, 제약, 보정 지도를 파일로 확정한 다음에만 GT를 읽어 평가한다.

초기 기체 간 좌표 정렬은 알고 있다고 가정한다. 월드 전체에서 초기 위치·방향까지 모르는
완전한 전역 장소 인식을 구현한 것은 아니다. 고정 경로의 공통 구간 라벨이나 월드 장애물
좌표는 후보 검색에 넘기지 않는다. 실제 통신·실시간 재회 계획도 이번 범위 밖이다.

### 이번에 실제 채택된 제약

- 검사 후보 37개 → 형상 검사 통과 3개 → 이웃 일관성 통과 2개 → 최종 제약 1개.
- 최종 `pair=11`: drone1 keyframe 36과 drone2 keyframe 30.
- anchor 관측 시각: 시뮬레이션 시간 230.952초 / 204.772초. 약 26.18초 차이가 난다.
- 정합 overlap 42.68%, inlier RMSE 0.288m.
- **사후 GT 평가** 상대 pose 위치 오차: 초기 2.426m → 정합 후 0.776m.
- **사후 GT 평가** 상대 yaw 오차: 초기 3.305° → 정합 후 0.185°.
- 최적화는 316회 함수 평가 후 수렴했다. 후보 추정·최적화 약 67초, 지도 생성·평가 포함 약 320초.

여기서 이웃 후보는 신뢰도 검증에만 쓰고 추가 제약으로 넣지 않았다.
따라서 제약 1개이지, 센서 scan 1장만 사용했다는 뜻도, 통신 패킷 1번이라는 뜻도 아니다.
55~90m 외에도 약한 공통 관측이 일부 존재하므로 엄밀히 겹침이 정확히 한 번뿐이라고 주장하지 않는다.

## 5. 정량 결과와 해석

| 지표 | B0 | B1 | N-single |
|---|---:|---:|---:|
| 종단 상대 위치 오차 [m] ↓ | 7.935 | 0.323 | 1.245 |
| 상대 위치 RMSE [m] ↓ | 4.315 | 0.777 | 0.689 |
| 상대 yaw RMSE [°] ↓ | 3.343 | 0.290 | 0.729 |
| 종단 상대 yaw 오차 [°] ↓ | 1.686 | 0.291 | 1.963 |
| drone1 절대 위치 RMSE [m] ↓ | 2.331 | 0.228 | 2.335 |
| drone2 절대 위치 RMSE [m] ↓ | 2.174 | 0.233 | 1.999 |
| 지도 점유셀 Chamfer [m] ↓ | 1.632 | 0.255 | 2.110 |
| 지도 점유셀 F1 ↑ | 0.154 | 0.509 | 0.064 |

상대 위치 RMSE 약 84.0%, 상대 yaw RMSE 약 78.2% 감소.
하지만 종단 상대 yaw는 악화했고, drone1 절대 위치 RMSE는 사실상 개선되지 않았다.
B1보다 모든 지표가 좋아진 것도 아니다.

상대 제약은 두 추정 사이의 관계를 맞출 뿐, 모든 지점을 정답 좌표에 직접 고정하지 않는다.
이번 궤적에서 보정 후 두 기체가 함께 world +y 쪽으로 치우치는 현상이 보인다.
이처럼 서로의 상대 관계가 개선돼도 절대 지도 오차는 남거나 악화될 수 있다.
오차 배분이 비대칭인 원인이 정합 편향·graph 가중치·경로 구조 각각에 얼마나 있는지는 추가 분해 실험이 필요하다.

측정 정의:

- 종단 상대 오차: 각 기체의 마지막 GT 평가 가능 keyframe 두 개의 상대 변환을 비교한다. 두 표본이 같은 시각이라는 뜻은 아니다.
- 상대 오차 곡선/RMSE: 두 궤적의 공통 **raw-SLAM 누적거리**에서 표본화한다. 서로 다른 길이의 경로라 곡선 마지막 값과 종단 값이 다를 수 있다.
- yaw 오차: 각도 차이를 ±180°로 감싼 뒤 절댓값을 사용한다. signed yaw 자체가 아니다.
- 지도 기준: GT로 투영한 동일 관측 scan의 지도이지 월드 전체의 완벽한 CAD 지도는 아니다.
- 지도 F1은 0.1m 셀의 점유 일치를 비교하므로 작은 이동에도 민감하다. Chamfer와 그림을 함께 본다.

## 6. Streamlit에서 확인하는 순서

서버 주소: **http://166.104.230.197:8501**

1. 왼쪽 `실험 보기 → 실험 선택`에서 **`전체맵 · LiDAR 1구간 보정`**을 선택한다. 원본 ID는 `20260911T091146Z_bed606a28e99`이다.
2. `실험 요약`에서 종단 상대 오차, 조건별 표, 지도/궤적/절대 오차 그림을 확인한다.
3. `LiDAR 상세 → Registration`에서 `accepted=true`, `pair=11`을 확인한다. 이 쌍이 실제 보정에 들어간 제약이다.
4. `LiDAR 상세 → Drift & Trajectory`에서 B0와 N-single의 상대 오차 감소와 각 기체의 GT 대비 위치를 함께 본다.
5. `LiDAR 상세 → Maps`에서 B0 / B1 / N-single을 비교한다. 요약 지도 그림의 초록=GT에만 점유, 빨강=추정에만 점유, 검정=점유 일치, 회색=둘 다 미관측이다.
6. `보고서·파일`에서 해석·한계, CSV/그림 다운로드와 원본 ID·저장 경로를 확인한다.

실험 이름은 제목 아래 편집 영역에서 바꿀 수 있다. [목록·라벨 관리 사용법](dashboard_experiment_management.md) 참고.

비행 중 원본 지도 저장 이력은 `SLAM & Fusion Debug`에서 같은 run을 선택한다.
오프라인 보정은 비행 후 전체 궤적을 재계산하므로 N-single 결과를 비행 중 실시간 성능으로 해석하지 않는다.

현재 서비스가 꺼져 있다면 서버 터미널에서 다음 명령으로 다시 시작한다. 실행 중에는 중복 시작하지 않는다.

```bash
cd /home3/deepblue/work/AV_Drone
bash scripts/run_quant_dashboard.sh
```

직접 접속이 방화벽에 막히면 로컬 PC에서 SSH 터널을 열고 `http://localhost:8501`에 접속한다.

```bash
ssh -N -L 8501:127.0.0.1:8501 deepblue@166.104.230.197
```

## 7. 저장 위치와 재현

서버 기준 프로젝트: `/home3/deepblue/work/AV_Drone`.
컨테이너의 같은 위치: `/workspace/AV_Drone`.

```text
rosbags/20260911T091146Z_bed606a28e99_slam_debug/  # 원본 bag, metadata
runtime/sim/20260911T091146Z_bed606a28e99/        # PX4/ROS 로그, 종료 결과
artifacts/20260911T091146Z_bed606a28e99/
├── drone1/, drone2/, swarm/                    # 비행 중 지도·궤적·융합 이력
├── lidar_registration_v1_origin_dependent/     # 최초 분석, 비교용 보존
└── lidar_registration/                        # 최종 분석, Streamlit에서 표시
    ├── config.yaml, manifest.json, metrics.json, metrics.csv
    ├── inference_frozen.json, inference_poses.npz
    ├── registrations.csv, overlap_audit.csv
    ├── trajectories/, submaps/, debug_plots/, maps/
    └── overview_trajectories.png, overview_errors.png, overview_map_errors.png
```

원본 `.db3` SHA-256:
`f2762a711f376ce35c37769721827997bf89f44c7f555d20c5520763d8a15ee5`.
분석 설정 SHA-256:
`efed298588c512c777a70767c98c4e43c3c734022309a29f9cb7e44116230c4f`.
최종 `manifest.json`에는 주요 추론 소스의 SHA-256도 기록했다.

### 같은 경로로 새 시뮬레이션을 기록하려면

현재 패키지 빌드가 반영돼 있고 기존 sim/ros가 정지된 상태에서 사용한다.
새 run ID가 생성된다. **이번 작업에서는 이미 실행했으므로 결과 확인만 하려면 다시 돌릴 필요가 없다.**

```bash
cd /home3/deepblue/work/AV_Drone
python3 scripts/run_guarded_experiment.py \
  --manifest src/drone_bringup/config/swarm_two_uav_full_overlap1.yaml --check
python3 scripts/run_guarded_experiment.py \
  --manifest src/drone_bringup/config/swarm_two_uav_full_overlap1.yaml
```

기존 Gazebo/PX4와 multi-drone autonomy launch를 관리 실행기로 시작한 것이다.
가짜 궤적을 생성해 실제 비행처럼 표시한 것이 아니다.
`sudo`나 `docker compose up --force-recreate`는 사용하지 않았다.

### 저장된 bag만 다시 분석하려면

다음 예시는 시뮬레이터를 시작하지 않으며 결과를 별도 새 폴더에 쓴다.
이미 존재하는 비어 있지 않은 분석 폴더는 덮어쓰지 않고 거부한다.

```bash
cd /home3/deepblue/work/AV_Drone
docker run --rm --network none --cpus 4 --memory 8g --user 1027:1027 \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -e MPLCONFIGDIR=/workspace/AV_Drone/runtime/matplotlib_full_analysis \
  -v /home3/deepblue/work/AV_Drone:/workspace/AV_Drone \
  -w /workspace/AV_Drone --entrypoint bash av-drone-ros:latest -c '
    source /opt/ros/humble/setup.bash
    export PYTHONPATH=/workspace/AV_Drone/src/drone_cslam:$PYTHONPATH
    python3 -m drone_cslam.offline_lidar_eval \
      --bag /workspace/AV_Drone/rosbags/20260911T091146Z_bed606a28e99_slam_debug \
      --config /workspace/AV_Drone/src/drone_cslam/config/lidar_no_gt_full.yaml \
      --output /workspace/AV_Drone/artifacts/full_map_reanalysis_20260911/lidar_registration
  '
```

`--inference-only`를 더하면 평가용 GT도 읽지 않고 pose·제약·지도 저장까지만 한다.
완전한 대시보드 비교 지표는 GT 사후 평가 단계까지 실행해야 생성된다.
별도 재분석 run은 새 대시보드에서 `분류 대기`로 발견된다. `이름·보관 관리`에서 목적과 이름을 입력한다.

## 8. 구현·검증 이력과 다음 판단

- waypoint 순차 진행을 추가하고 중간 목표 도달을 최종 임무 완료로 오인하지 않도록 했다.
- Gazebo entity가 아직 없는 경우 GT recorder가 오류 없이 재시도하도록 수정했다.
- GT 없이 동기화·keyframe 생성·후보 검색·LiDAR 정합·최적화하는 오프라인 경로를 추가했다.
- 최초 분석에서 이웃 보정의 translation 계수를 world 원점에서 비교하면 멀리 있는 장면에서 작은 yaw 차이가 과장되는 문제를 발견했다. 두 보정을 같은 지역 기준점에 적용해 비교하도록 수정했다.
- yaw 경계에서 정밀 정합 초기값을 연속 각도 구간으로 되돌렸다. 극미한 부동소수점 초과에 따른 SciPy 경고는 여전히 발생할 수 있다. 저장된 pair 0/11/18/27을 재계산해 동일 결과를 확인했으며 최대 경계 초과는 약 3.33e-16이었다.
- 이 과정에서 정합·일관성 임계값을 완화하지 않았다. 첫 분석 폴더도 삭제하지 않고 보존했다.
- 관련 cSLAM 검사 16개, 대시보드 검사 16개 통과. waypoint/GT startup 검사 및 실제 smoke·전체 길이 기록도 검증했다.
- 새 실행의 artifact 상위 폴더를 호스트 사용자 소유·그룹 상속으로 만들도록 해 후처리 출력의 권한 충돌을 줄였다.
- 데이터와 새 PX4/ROS 로그는 `/home3` 아래에 저장했다. 종료 후 루트 디스크는 약 289GiB, `/home3`는 약 2.2TiB 여유다. 기존 데이터 자동 삭제는 하지 않았다.

이번 요청 범위의 기록·분석·시각화는 여기서 완료한다.
다음 우선순위는 만남 횟수를 늘리기 전에 **한 공통 구간의 제약이 전체 지도를 어떻게 변형하는지**를 분석하는 것이다.
같은 bag으로 제약 가중치·오차 배분·국소/전역 지도 지표를 분리 검증하고,
설정을 별도 데이터에서 고정한 뒤 1/2/3 공통 구간과 반복 실행을 비교하는 것이 적절하다.
현재 한 번의 전체 길이 실행만으로 논문 수준의 일반화·항법 안전·실시간 통신 효율을 주장하지 않는다.
