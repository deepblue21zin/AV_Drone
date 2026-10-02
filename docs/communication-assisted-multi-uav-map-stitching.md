# 일자 환경을 위한 통신 보조 Multi-UAV Map Stitching 설계

> 기준일: 2026-08-26  
> 대상: 2-UAV LiDAR mapping, 중앙 map fusion, 향후 cooperative SLAM  
> 핵심 결론: **통신 자체가 drift를 줄이는 것이 아니라, 통신을 통해 전달한 odometry·keyframe과 드론 간 상대 관측으로 `inter-UAV constraint`를 만들고 pose graph를 최적화해야 한다.**

## 1. 받은 코멘트의 정확한 해석

SLAM의 누적 오차를 줄이려면 이미 지나간 위치와 현재 위치 사이에 새로운 제약이 필요하다.
그 제약은 다음 두 종류가 대표적이다.

1. 한 드론이 과거에 방문한 장소를 다시 관측하는 **intra-UAV loop closure**
2. 서로 다른 두 드론이 같은 장소·랜드마크 또는 서로의 상대 위치를 관측하는 **inter-UAV constraint**

따라서 각 드론이 반드시 같은 장소를 한 바퀴 돌아야 하는 것은 아니다. 드론1의 현재 관측과
드론2의 과거 또는 현재 관측이 같은 장소를 가리키면 그것도 pose graph의 loop를 만든다.
반대로 두 드론이 네트워크로 연결돼 있어도 map과 odometry만 전달하고 둘을 연결하는 상대 관측이
없다면 독립적인 두 좌표계의 drift는 줄어들지 않는다.

이 프로젝트에서 사용할 표현은 다음과 같다.

> 두 드론은 keyframe·submap·odometry를 공유하고, 공통 장소 인식 또는 드론 간 상대 센싱으로
> inter-UAV relative-pose constraint를 생성한다. 중앙 pose-graph optimizer가 이 제약을 이용해
> 각 로컬 궤적과 submap pose를 보정한 뒤 global OccupancyGrid를 다시 만든다.

`두 드론이 통신해서 지도를 합친다`만으로 설명하면 통신과 추정 문제가 섞이므로 위 문장을
연구 목표와 발표 자료의 기준 문장으로 사용한다.

## 2. 일자 환경에서 현재 방식이 어려운 이유

현재 두 기체는 `random_cylinders_double`에서 y=-7.5 m와 y=+7.5 m에 배치되고, 약 137 m를
같은 방향으로 진행한다. 이 구조에는 다음 문제가 동시에 있다.

- 출발점으로 돌아오는 폐곡선이 없어 각 기체의 self loop closure가 없다.
- 긴 구간에서 비슷한 벽·원통 배열이 반복되면 서로 다른 x 위치가 비슷하게 보이는
  perceptual aliasing이 생긴다.
- 두 lane 중심 간격은 15 m이고 현재 mapper의 최대 LiDAR 범위는 8 m이므로, 두 기체가 각 lane
  중심을 유지하면 강한 공통 LiDAR 관측 영역을 안정적으로 만들기 어렵다.
- 두 기체가 같은 속도와 방향으로 평행 이동하면 거리만 측정하는 상대 센서는 측면 간격만
  반복해서 보게 된다. 실제 추정에서는 상대 방향과 진행축 오차를 충분히 구분하기 어렵기 때문에
  속도 차이, 정지-이동, 측면 이동 또는 짧은 곡선 같은 excitation이 필요하다.
- 한 번 생성된 137 m 전체 OccupancyGrid에 rigid transform 하나만 적용하면 지도 내부에서 위치에
  따라 다르게 누적된 drift를 복구할 수 없다.

즉 문제는 단순히 `overlap이 적다`가 아니라 **서로 독립적인 두 궤적을 연결하는 관측 제약이 없고,
긴 일자 기하가 오차 방향을 충분히 관측시키지 못한다**는 것이다.

## 3. 현재 코드가 하는 일과 하지 않는 일

현재 `drone_map_fusion/map_fusion_node.py`는 다음 파이프라인이다.

```text
/drone1/{known_pose_map | slam/map} ─┐
                                     ├─ fixed spawn transform
/drone2/{known_pose_map | slam/map} ─┘       ↓
                                        grid projection
                                              ↓
                                      weighted log-odds fusion
                                              ↓
                                      /swarm/global_map
```

이미 구현된 기능:

- 두 local OccupancyGrid의 ROS 2 전송
- source timeout과 local-only fallback
- spawn에서 얻은 고정 `(x, y, yaw)` transform 적용
- source freshness/confidence weight 적용
- conflict cell과 fusion latency 기록

아직 없는 기능:

- 드론 간 상대 관측
- inter-UAV place recognition/loop closure
- keyframe 또는 submap 단위 pose graph
- transform 후보의 outlier rejection
- 최적화된 transform/covariance/revision
- trajectory drift를 반영한 submap 재배치

현재 `swarm_map -> droneN/map`은 static TF다. 향후 최적화 결과가 바뀌는 cooperative SLAM에서는
이를 그대로 갱신하면 안 된다. verified transform을 동적·versioned 상태로 관리하고 planner가
transform revision 변경을 인지하게 해야 한다.

## 4. 통신과 오차 보정을 분리한 설계

### 4.1 통신 계층

통신 계층은 데이터를 전달하고 다음 상태를 측정한다.

- peer 연결 여부
- message age, drop, 지연, bandwidth
- keyframe/submap revision
- clock offset 또는 timestamp validity

통신 연결만으로 registration confidence를 올리지 않는다.

### 4.2 제약 생성 계층

오차를 줄이려면 최소 하나의 실제 측정이 필요하다.

| 방법 | 생성되는 정보 | 일자 환경 적합성 | 프로젝트 사용 위치 |
|---|---|---|---|
| 공통 GNSS/RTK/외부 위치 | 각 map의 global pose | 높음 | absolute-reference 비교군 |
| 카메라/AprilTag로 상대 관측 | range, bearing, 상대 yaw/pose | 높음, 근거리 랑데부 필요 | 권장 실기 전환안 |
| UWB range + 양쪽 odometry 공유 | 거리 제약 | motion excitation 필요 | 권장 경량 실험안 |
| UWB range + bearing/heading | 상대 위치 제약 | range-only보다 안정적 | 권장 센서 조합 |
| 공통 장소의 LiDAR submap 정합 | 상대 SE(2) pose | 충분한 overlap·고유 기하 필요 | LiDAR-only 연구안 |
| 전체 OccupancyGrid ICP만 사용 | rigid 상대 pose 후보 | 반복 일자 환경에서 false merge 위험 | 보조 후보만 허용 |
| map/odom 통신만 수행 | 상대 제약 없음 | drift 보정 불가 | transport smoke test |

현재 센서 구성을 유지하면서 구조를 먼저 검증하려면 Gazebo truth로 **노이즈·지연·dropout이 포함된
가상 range/bearing 센서**를 만들어 relative observation만 발행할 수 있다. 이 값은 알고리즘에
정답 transform으로 직접 주입하지 않고 센서 측정처럼 취급한다. 최종 연구 결과에서는 이 단계를
`synthetic relative sensor` 비교군으로 명시하고, 실제 제안 방식은 UWB/비전/LiDAR front-end로
교체해야 한다.

### 4.3 추정 계층

권장 baseline은 현재 중앙 fusion 구조를 활용한 centralized SE(2) pose graph다.

```text
UAV별 odometry/keyframe edge
    Z(i,k -> i,k+1)

UAV별 self loop edge (존재할 때)
    Z(i,k -> i,m)

드론 간 relative/place edge
    Z(1,k -> 2,m)
                ↓
      robust outlier gate
                ↓
      centralized pose graph
                ↓
  optimized keyframe/submap poses
                ↓
    global OccupancyGrid 재투영
```

최적화 변수는 각 드론의 단일 map transform만이 아니라 keyframe/submap pose여야 한다. 한 개의
전체 grid를 rigid하게 옮기는 방식은 초기 상대 좌표계 정렬에는 쓸 수 있지만, 구간별로 누적된
scan-matching/odometry drift를 펴지 못한다.

loop candidate에는 Huber/Cauchy 또는 switchable constraint 계열의 robust loss를 사용하고,
최적화 전후 residual, inlier 수, covariance, transform jump를 confidence gate에 포함한다.
반복 원통 환경에서는 false positive 하나가 전체 지도를 망가뜨릴 수 있으므로 `정합 성공률`보다
`false merge 0회`를 더 우선하는 Gate를 둔다.

## 5. AV_Drone 권장 아키텍처

새 추정 기능은 raster 융합 코드와 분리한 `drone_cslam` 패키지로 두는 편이 안전하다.

```text
drone1 local mapper                         drone2 local mapper
  ├─ keyframes                               ├─ keyframes
  ├─ frozen submaps                          ├─ frozen submaps
  └─ local odometry                          └─ local odometry
              │                                  │
              └──────── ROS 2/DDS ───────────────┘
                              ↓
                  inter_robot_constraint_builder
                   ├─ time association
                   ├─ relative sensor adapter
                   ├─ place/submap candidate
                   └─ geometric/outlier gate
                              ↓
                    swarm_pose_graph_optimizer
                              ↓
                    verified transform manager
                     ├─ pose graph revision
                     ├─ covariance/confidence
                     └─ stale/degraded state
                              ↓
                         submap_projector
                              ↓
                       drone_map_fusion
                              ↓
                     /swarm/global_map
```

책임 분리:

- `drone_slam`: local odometry, keyframe, frozen submap 생성
- `drone_cslam`: inter-UAV constraint와 joint pose graph 추정
- `drone_map_fusion`: verified submap pose를 이용한 raster 합성
- `drone_bringup`: sensor/communication mode와 rendezvous mission 설정
- `drone_metrics`: constraint, optimizer, network KPI 기록

## 6. 메시지 계약 초안

실제 구현 시 `drone_interfaces` 같은 별도 interface package를 만든다. 최소 필드는 다음과 같다.

### `Keyframe2D`

```text
header
vehicle_id
keyframe_id
local_pose              # x, y, yaw
local_pose_covariance
submap_id
localization_status
```

### `Submap2D`

```text
header
vehicle_id
submap_id
revision
origin_pose
occupancy_grid_or_tile
descriptor_type
descriptor
```

### `RelativeObservation2D`

```text
header
observer_vehicle_id
target_vehicle_id
observer_keyframe_id
target_keyframe_id
range_m
bearing_rad
relative_yaw_rad         # 센서가 제공할 때만 valid
covariance
measurement_source       # synthetic, uwb, tag, lidar_place
quality
valid_until
```

### `InterRobotConstraint2D`

```text
header
from_vehicle/keyframe
to_vehicle/keyframe
relative_pose
covariance
inlier_count
overlap_ratio
rmse
confidence
verification_state       # CANDIDATE, VERIFIED, REJECTED
rejection_reason
```

### `OptimizedFrame2D`

```text
header
vehicle_id
map_to_swarm_transform
covariance
pose_graph_revision
confidence
state                    # VERIFIED, STALE, DEGRADED
```

초기 topic 제안:

```text
/droneN/cslam/keyframes
/droneN/cslam/submaps
/droneN/cslam/relative_observations
/droneN/cslam/heartbeat
/swarm/cslam/inter_robot_constraints
/swarm/cslam/optimized_frames
/swarm/cslam/pose_graph_revision
/swarm/cslam/status
```

raw LaserScan 전체를 상시 공유하지 않는다. 우선 keyframe metadata와 constraint 후보를 교환하고,
후보가 생겼을 때만 필요한 submap/descriptor를 요청하는 event-driven 구조로 시작한다.

## 7. 일자 맵에 맞춘 Active Rendezvous

완전한 한 바퀴 대신 짧은 anchor maneuver를 임무에 넣는다.

### 권장 anchor 구성

```text
x≈3 m    : 시작 anchor 및 timestamp/relative sensor 초기화
x≈45 m   : 중간 anchor A
x≈90 m   : 중간 anchor B
x≈135 m  : 목표 근처 anchor C
```

anchor 위치는 초기값이며 장애물 배치와 센서 range에 맞춰 조정한다.

### anchor 동작 예시

1. 먼저 도착한 드론은 hover한다.
2. 다른 드론은 안전한 범위에서 속도를 다르게 하거나 짧은 측면/arc maneuver를 수행한다.
3. 2~5초 동안 여러 relative observation과 양쪽 odometry를 수집한다.
4. 시간·기하·residual gate를 통과한 constraint만 pose graph에 넣는다.
5. optimizer가 새 revision을 발행하면 submap을 다시 투영한다.
6. confidence가 회복됐을 때 두 기체가 다음 구간으로 진행한다.

UWB range-only를 사용한다면 두 드론이 같은 속도로 평행 이동하는 상태만 반복하지 않는다.
range+bearing 또는 tag 기반 full relative pose를 사용해도 단발 측정 하나로 즉시 global map을
바꾸지 않고 여러 관측의 일관성을 확인한다.

## 8. 상태 머신과 fail-safe

```text
DISCONNECTED
    ↓ peer 발견
LOCAL_ONLY
    ↓ 상대 관측 후보
CANDIDATE
    ↓ 연속 검증 + observable geometry
VERIFIED
    ↓ pose graph 최적화
OPTIMIZED
    ↓ 통신/측정 timeout 또는 residual 증가
DEGRADED
    ├─ 복구 → VERIFIED
    └─ 장기 실패 → LOCAL_ONLY
```

원칙:

- 통신 단절 시 각 드론의 local mapping/navigation은 계속한다.
- constraint가 없으면 map을 억지로 stitch하지 않는다.
- transform confidence 미달 시 마지막 verified revision을 stale로 유지하거나 해당 source를 제외한다.
- transform jump가 planner에 바로 전달되지 않도록 revision 변경 이벤트와 replan handshake를 둔다.
- optimizer 실패가 비행 제어 주기를 막지 않도록 별도 process/executor에서 실행한다.

초기 튜닝값은 아래에서 시작하되 실험으로 확정한다.

| 항목 | 초기값 |
|---|---:|
| relative observation age | 0.20 s 이하 |
| timestamp association error | 50 ms 이하 |
| 연속 일관 관측 | 3회 이상 |
| submap overlap | 20% 이상 |
| LiDAR refinement RMSE | 0.30 m 이하 |
| transform translation jump | revision당 0.50 m 이하 |
| transform yaw jump | revision당 5° 이하 |
| peer/constraint timeout | 2.0 s |

`overlap`만으로 통과시키지 않고 pose-graph residual과 상대 센서 covariance를 함께 사용한다.

## 9. 구현 순서

### C0. 현재 기준선 고정

- known-pose operational fusion 5회 반복
- raw SLAM drift, fixed-transform fusion, network age를 같은 artifact에 기록
- 현 map-fusion을 `fixed_transform_baseline`으로 명명

완료 조건: 기존 Gate A/B/C가 반복 가능하고 다른 기능 변경과 분리된 커밋으로 고정됨.

### C1. 통신·상대 관측 contract

- `drone_interfaces` 메시지 정의
- odometry/keyframe/heartbeat exchange
- synthetic range+bearing adapter와 noise/dropout 파라미터
- time association 및 CSV logger

완료 조건: pose graph 적용 전에도 어떤 상대 관측이 언제 어느 keyframe을 연결하는지 재생 가능함.

### C2. 중앙 SE(2) pose graph

- odometry edge와 inter-UAV edge 구성
- robust loss/outlier gate
- optimized pose와 covariance/revision 발행
- ground truth는 평가에만 사용

완료 조건: 의도적으로 주입한 odometry drift가 relative constraint 이후 감소하고 false constraint가
global frame을 오염시키지 않음.

### C3. Submap 기반 global map 재구성

- 일정 거리/시간마다 frozen submap 생성
- optimized submap pose로 OccupancyGrid 재투영
- full-grid rigid fusion과 결과 A/B 비교

완료 조건: 구간별 drift가 있는 합성 데이터에서 단일 rigid transform보다 map quality가 개선됨.

### C4. Active rendezvous mission

- anchor zone과 hover/approach state 추가
- range-only일 때 observable motion maneuver 추가
- optimizer revision 이후 replan/continue handshake

완료 조건: 일자 137 m 조건에서 passive 직진보다 alignment와 repeatability가 개선되고 추가 mission
time을 수치로 보고할 수 있음.

### C5. 실제 front-end 전환

- UWB+odometry, camera/tag 또는 LiDAR inter-place 중 하나 선택
- synthetic adapter와 동일 message contract로 교체
- 통신 지연/loss 및 false match 실험

완료 조건: ground-truth-derived measurement 없이 동일 Gate를 통과함.

## 10. 실험 매트릭스

같은 world seed와 trajectory 조건에서 다음을 비교한다.

| ID | 조건 | 확인 목적 |
|---|---|---|
| E0 | 각 UAV local-only | drift 하한선 |
| E1 | map 통신 + fixed spawn transform | 현재 operational baseline |
| E2 | map/odom 통신만, 상대 제약 없음 | 통신 자체는 drift를 줄이지 않음을 확인 |
| E3 | passive 직진 + relative constraint | 직진 기하의 관측 한계 측정 |
| E4 | active rendezvous + relative constraint | 제안 방식 |
| E5 | intra-UAV return loop | 전통 loop closure 비교군 |
| E6 | E4 + delay/loss/outlier | robustness |

각 조건은 최소 10개 seed로 반복하고 invalid infrastructure run은 별도로 제외한다.

필수 KPI:

- trajectory ATE/RPE와 end-point drift
- known-pose reference 대비 occupied precision/recall/IoU
- inter-UAV translation/yaw error
- accepted/rejected constraint 수와 loop precision/recall
- pose-graph residual과 convergence latency
- map conflict/ghost/double-wall ratio
- rendezvous 추가 시간과 mission success
- DDS TX/RX bytes, p95 latency, drop rate
- fallback 횟수와 stale duration

## 11. 이 프로젝트의 최종 연구 주장 범위

다음 세 결과를 분리해 보고한다.

1. **Map sharing:** 두 드론의 map이 네트워크로 중앙 노드에 도착한다.
2. **Map fusion:** 알려진 transform 또는 verified transform으로 grid를 합성한다.
3. **Cooperative SLAM:** inter-UAV constraint가 joint trajectory/submap pose를 실제로 보정한다.

현재 프로젝트는 1과 known-transform 기반 2까지 구현돼 있다. 이 문서의 C1~C5가 완료돼야 3을
주장할 수 있다.

## 12. 선행연구와 설계 근거

- [Kimera-Multi](https://arxiv.org/abs/2106.14386)는 각 로봇의 로컬 추정에 inter-robot loop
  closure를 추가하고 robust distributed pose-graph optimization으로 전역 일관성을 만든다.
- [DOOR-SLAM](https://arxiv.org/abs/1909.12198)은 raw sensor 전체를 교환하지 않고도
  inter-robot loop closure를 찾고, pairwise consistency로 잘못된 제약을 제거하는 구조를 보인다.
- [Swarm-SLAM](https://arxiv.org/abs/2301.06230)은 ROS 2 기반으로 LiDAR를 포함한 sparse
  decentralized C-SLAM과 inter-robot loop-closure 우선순위화를 제시한다.
- [Multi-Robot Relative Pose Estimation in SE(2)](https://arxiv.org/abs/2401.15313)은 양쪽
  odometry가 공유될 때 range-only 또는 bearing-only 상대 관측의 observability 조건과, odometry를
  공유하지 않을 때 range+bearing이 필요한 조건을 분석한다.
- [Active Rendezvous for Multi-Robot PGO](https://arxiv.org/abs/1907.05538)는 특징이 부족한 환경에서
  통신 채널의 상대 위치 정보를 활용해 rendezvous와 pose-graph 정확도를 개선하는 방향을 보인다.

이 연구들을 그대로 이식하는 것이 아니라, 현재의 2D LiDAR/ROS 2/중앙 fusion 구조에 맞춰
`중앙 SE(2) pose graph + submap 재투영 + active rendezvous`를 최소 구현으로 채택한다.
