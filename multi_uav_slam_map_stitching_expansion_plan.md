# 다중 드론 SLAM 맵 스티칭 및 실시간 경로계획 확장 계획서

> 프로젝트: AV_Drone  
> 대상 환경: PX4 SITL + ROS 2 Humble + Gazebo Sim + LiDAR + MPPI/A*  
> 목표 기체 수: 1대 → 2대 → 4대  
> 핵심 확장: 드론별 로컬 SLAM 맵 생성, 실시간 맵 정합·융합, 공유 지도 기반 경로계획, 드론 간 충돌 회피

---

## 1. 핵심 방향

현재 단일 드론 자율주행 구조를 다음 순서로 확장한다.

1. 단일 드론 LiDAR SLAM baseline을 완성한다.
2. 2대 드론의 PX4 instance, ROS 2 namespace, TF frame을 완전히 분리한다.
3. Gazebo의 초기 spawn pose를 사용하는 known-pose map merge를 먼저 구현한다.
4. 각 드론의 `nav_msgs/OccupancyGrid`를 중앙 Map Fusion Node에서 실시간 융합한다.
5. 공유 Global Map에서 A* 전역 경로를 생성하고, 각 드론의 MPPI가 로컬 추종·회피를 담당하도록 분리한다.
6. known-pose 방식이 안정화된 뒤 unknown-pose map registration을 구현한다.
7. 마지막 단계에서 4대 확장, 통신 지연·패킷 손실·드론 고장 시나리오를 검증한다.

**카메라 원본 이미지 파노라마 스티칭은 주 경로로 사용하지 않는다.**  
항법용 지도는 LiDAR 기반 OccupancyGrid로 구성하고, 카메라는 추후 드론 간 동일 장소 인식 및 loop-closure 후보 생성용 보조 센서로 사용한다.

---

## 2. 현재 상태와 확장 필요성

### 2.1 현재 확보된 기능

- PX4 SITL 및 Gazebo Sim 기반 단일 드론 실행
- ROS 2 기반 perception, planning, safety, control, metrics 노드 구조
- Offboard 상태 머신 기반 이륙, 이동, 목표점 hover
- LiDAR scan 수신 및 최근접 장애물 거리 계산
- MPPI 기반 단일 드론 경로 생성·추종 실험
- 실행별 artifact, CSV, summary, event log 기록
- smoke test 및 experiment registry 기반 재현성 구조

### 2.2 현재 한계

- 각 드론이 독립적으로 생성한 SLAM map이 없음
- 현재 perception은 전체 지도보다 최근접 장애물 scalar 중심
- 멀티드론 namespace, PX4 instance, TF tree 분리가 미구현
- 드론 간 지도 공유 및 map merge가 없음
- 타 드론의 예측 궤적을 동적 장애물로 처리하지 않음
- Global Planner와 Local Planner의 역할 분리가 불완전
- 통신 지연, 맵 노후화, 정합 오류에 대한 fallback이 없음

---

## 3. 최종 목표 시스템

```text
                         ┌──────────────────────────────┐
                         │      Swarm Coordinator       │
                         │ mission / heartbeat / status │
                         └──────────────┬───────────────┘
                                        │
                              /swarm/global_map
                              /swarm/map_version
                              /swarm/fusion_status
                                        │
                ┌───────────────────────▼───────────────────────┐
                │          Central Map Fusion Node              │
                │ registration / confidence gate / grid fusion │
                └───────┬───────────┬───────────┬───────────────┘
                        │           │           │
             drone1/map ┘ drone2/map┘ drone3/map┘ drone4/map
                        │
        ┌───────────────▼────────────────┐
        │       Global A* Planner        │
        │ global map → sparse waypoints  │
        └───────────────┬────────────────┘
                        │
      ┌─────────────────▼─────────────────────────────────────┐
      │                  UAV i Local Stack                    │
      │                                                       │
      │ LiDAR + Odometry + IMU                                │
      │          ↓                                            │
      │ Local SLAM → local occupancy map                      │
      │          ↓                                            │
      │ MPPI Local Planner                                    │
      │  - global path tracking cost                          │
      │  - local obstacle cost                                │
      │  - peer predicted trajectory cost                     │
      │          ↓                                            │
      │ Safety Gate → Offboard Control → PX4 instance i       │
      └───────────────────────────────────────────────────────┘
```

---

## 4. 설계 원칙

### 4.1 각 드론의 로컬 자율성 유지

Global Map Server 또는 통신이 끊겨도 각 드론은 즉시 추락하거나 전체 임무를 중단하지 않아야 한다.

```text
Global map 정상
    → Global A* + Local MPPI

Global map timeout 또는 정합 confidence 저하
    → Local SLAM map + Local MPPI

Local SLAM 또는 pose 유실
    → Hover

장시간 복구 실패 또는 안전거리 미확보
    → Safe landing
```

### 4.2 Map과 Planner의 역할 분리

- SLAM: 센서 데이터로 로컬 지도와 pose 추정
- Map Fusion: 로컬 지도를 공통 좌표계에 정렬하고 융합
- Global A*: 공유 지도를 이용해 전역 경로 생성
- Local MPPI: 실시간 장애물, 경로 추종, 드론 간 충돌 회피
- Safety: stale data, collision risk, pose loss를 감시하고 명령 차단

### 4.3 Ground Truth 사용 범위 제한

Gazebo ground-truth pose는 다음 목적으로만 사용한다.

- known-pose baseline
- map registration 오차 평가
- trajectory 오차 평가
- 실험 정답 데이터 생성

Unknown-pose 제안 방식의 입력으로 ground truth transform을 직접 사용하지 않는다.

---

## 5. ROS 2 Namespace 및 TF 구조

### 5.1 권장 Namespace

```text
/drone1/...
/drone2/...
/drone3/...
/drone4/...

/swarm/...
```

예시 토픽:

```text
/drone1/scan
/drone1/odom
/drone1/slam/map
/drone1/slam/pose
/drone1/planner/global_path
/drone1/planner/predicted_trajectory
/drone1/control/trajectory_setpoint
/drone1/status/heartbeat

/swarm/global_map
/swarm/map_version
/swarm/map_fusion_status
/swarm/vehicle_states
```

### 5.2 TF Frame

```text
swarm_map
 ├─ drone1/map ─ drone1/odom ─ drone1/base_link ─ drone1/lidar_link
 ├─ drone2/map ─ drone2/odom ─ drone2/base_link ─ drone2/lidar_link
 ├─ drone3/map ─ drone3/odom ─ drone3/base_link ─ drone3/lidar_link
 └─ drone4/map ─ drone4/odom ─ drone4/base_link ─ drone4/lidar_link
```

### 5.3 필수 규칙

- 토픽뿐 아니라 모든 TF frame에 `droneN/` prefix를 적용한다.
- `map`, `odom`, `base_link`, `lidar_link`를 여러 드론이 공통 이름으로 사용하지 않는다.
- `swarm_map → droneN/map` transform은 Map Fusion 계층에서 관리한다.
- 각 드론은 자기 `droneN/map → droneN/odom`만 관리한다.
- Static TF와 dynamic TF publisher의 중복을 금지한다.

---

## 6. PX4 Multi-Instance 구성

각 기체는 다음 항목을 고유하게 가져야 한다.

| 항목 | drone1 | drone2 | drone3 | drone4 |
|---|---:|---:|---:|---:|
| PX4 instance | 0 | 1 | 2 | 3 |
| System ID | 1 | 2 | 3 | 4 |
| DDS client key | 1 | 2 | 3 | 4 |
| ROS namespace | `/drone1` | `/drone2` | `/drone3` | `/drone4` |
| Spawn 위치 | 독립 설정 | 독립 설정 | 독립 설정 | 독립 설정 |
| MAVLink/DDS port | 독립 설정 | 독립 설정 | 독립 설정 | 독립 설정 |

예시 설정:

```yaml
vehicles:
  - name: drone1
    px4_instance: 0
    system_id: 1
    dds_key: 1
    namespace: drone1
    spawn: [0.0, 0.0, 0.2]

  - name: drone2
    px4_instance: 1
    system_id: 2
    dds_key: 2
    namespace: drone2
    spawn: [0.0, 3.0, 0.2]

  - name: drone3
    px4_instance: 2
    system_id: 3
    dds_key: 3
    namespace: drone3
    spawn: [0.0, -3.0, 0.2]

  - name: drone4
    px4_instance: 3
    system_id: 4
    dds_key: 4
    namespace: drone4
    spawn: [-3.0, 0.0, 0.2]
```

### 완료 기준

- 4대의 state, pose, scan, command 토픽이 서로 섞이지 않는다.
- 한 기체의 Offboard 명령이 다른 기체에 전달되지 않는다.
- 각 드론의 arm, mode, heartbeat 상태가 독립 기록된다.
- TF tree에 duplicated frame 또는 multiple authority가 없다.

---

## 7. 단계별 구현 계획

# Phase 0. 단일 드론 SLAM Baseline

### 목표

현재 최근접 장애물 기반 perception을 실제 LiDAR SLAM map 생성 구조로 확장한다.

### 입력

```text
/drone1/scan
/drone1/odom
/drone1/imu
```

### 출력

```text
/drone1/slam/map
/drone1/slam/pose
TF: drone1/map → drone1/odom
```

### 구현 항목

- `slam_toolbox` 또는 동등한 2D SLAM 노드 연동
- 드론 고도를 고정하여 2D LiDAR 평면과 장애물 형상을 일치
- map resolution, update interval, scan rate 조정
- SLAM map 저장 및 재로드 기능 추가
- Gazebo ground-truth map과 SLAM map 비교 스크립트 작성
- 기존 MPPI plot에 SLAM map overlay 추가

### 완료 기준

- 목표 경로 전체에서 map topic이 끊기지 않는다.
- map→odom TF timeout이 발생하지 않는다.
- map coverage가 비행 거리 증가에 따라 정상 증가한다.
- SLAM map 위에서 A* 경로 생성이 가능하다.
- 동일 seed 10회 실행에서 map 생성 성공률 90% 이상이다.

---

# Phase 1. 2대 Multi-Vehicle 기반 구축

### 목표

지도 병합 전에 2대의 PX4, ROS 2, TF, control stack을 완전히 독립 실행한다.

### 구현 항목

- `multi_drone_autonomy.launch.py` 작성
- Python launch loop로 기체별 노드 생성
- 기체별 YAML parameter 파일 분리
- PX4 instance, System ID, port 분리
- MAVROS 유지 시 기체별 MAVROS namespace/port 분리
- uXRCE-DDS 전환 시 기체별 DDS namespace/client key 분리
- 기체별 artifact 디렉터리 생성
- 공통 run ID 아래 per-drone log 저장

### 권장 Artifact 구조

```text
artifacts/<run_id>/
├─ metadata.json
├─ swarm_summary.json
├─ fusion_metrics.csv
├─ network_metrics.csv
├─ drone1/
│  ├─ metrics.csv
│  ├─ summary.json
│  └─ events.log
├─ drone2/
│  ├─ metrics.csv
│  ├─ summary.json
│  └─ events.log
└─ plots/
```

### 완료 기준

- 2대 동시 이륙 및 독립 hover
- 각 드론의 scan, pose, command topic 정상 수신
- 다른 드론의 명령 수신 건수 0
- RTF, CPU, RAM, DDS throughput 기록 가능
- 단일 드론 대비 loop period p95 증가율 측정

---

# Phase 2. Known-Pose 실시간 Map Merge

### 목표

Gazebo spawn pose를 이용하여 두 로컬 OccupancyGrid를 공통 좌표계에 배치하고 실시간 융합한다.

### 좌표 변환

각 드론의 초기 위치가 알려져 있을 때 다음 transform을 계산한다.

```text
swarm_map → drone_i/map
```

개념식:

```text
T_swarm_map_to_local_map_i
  = T_swarm_map_to_base_i_at_start
    × inverse(T_local_map_i_to_base_i_at_start)
```

### Map Fusion 입력

```text
/drone1/slam/map
/drone2/slam/map
/swarm/map_transforms
```

### Map Fusion 출력

```text
/swarm/global_map
/swarm/map_version
/swarm/map_fusion_status
```

### Cell 융합 정책

- unknown cell은 기존 값을 덮어쓰지 않는다.
- occupied/free cell은 단순 overwrite보다 log-odds 또는 confidence weighted update를 사용한다.
- 센서 관측 시간이 오래된 cell은 낮은 confidence를 부여한다.
- 서로 충돌하는 map cell은 conflict flag를 남긴다.
- 맵 해상도가 다르면 global resolution로 resampling한다.

### 권장 상태 구조

```text
WAITING_MAPS
    ↓
TRANSFORM_READY
    ↓
FUSING
    ↓
HEALTHY

오류 발생:
DEGRADED
    ↓
LOCAL_ONLY_FALLBACK
```

### 완료 기준

- 두 드론의 지도 중첩 영역이 global map에서 이중 벽로 나타나지 않는다.
- map merge latency p95 측정 가능
- global map age가 설정 timeout 이내 유지
- 한 드론 map 입력 중단 시 나머지 map은 유지
- fusion node 재시작 후 latest map 복구 가능

---

# Phase 3. Global A* + Local MPPI 연결

### 목표

공유 Global Map을 이용해 전역 경로를 만들고, 각 드론은 MPPI로 해당 경로를 추종하면서 로컬 장애물을 회피한다.

### 처리 흐름

```text
/swarm/global_map
        ↓
Global A*
        ↓
/droneN/planner/global_path
        ↓
Waypoint Sampling / Path Corridor
        ↓
Local MPPI
        ↓
Safety Gate
        ↓
PX4 Offboard Setpoint
```

### MPPI 비용함수 확장

```text
J_total
 = J_goal
 + J_global_path
 + J_static_obstacle
 + J_unknown_space
 + J_control_effort
 + J_peer_collision
```

각 항목:

- `J_goal`: 최종 목표와의 거리
- `J_global_path`: A* 경로 corridor 이탈 비용
- `J_static_obstacle`: LiDAR 및 occupancy obstacle 비용
- `J_unknown_space`: 미탐색 공간 진입 비용
- `J_control_effort`: 급격한 속도·가속도 명령 억제
- `J_peer_collision`: 타 드론 예측 궤적과의 충돌 비용

### Replanning 조건

Global map이 갱신될 때마다 A*를 무조건 재실행하지 않는다.

다음 조건 중 하나가 만족될 때만 replanning한다.

- 현재 path corridor 내 occupied cell 증가
- 현재 경로가 끊김
- 신규 경로 길이가 기존보다 일정 비율 이상 개선
- 타 드론과 예상 충돌 발생
- goal 또는 task allocation 변경
- map transform revision 증가

### 초기 임계값 예시

```yaml
replanning:
  changed_cells_in_corridor: 20
  minimum_path_improvement_ratio: 0.08
  peer_min_separation_m: 2.0
  global_map_timeout_s: 2.0
  minimum_replan_interval_s: 0.5
```

### 완료 기준

- global path와 actual trajectory의 RMSE 기록
- map update로 경로가 막혔을 때 자동 우회
- replan latency 평균, p95, max 기록
- planner command discontinuity 제한
- local map만 사용할 때보다 mission success 또는 path quality 개선

---

# Phase 4. Unknown-Pose Map Registration

### 목표

드론의 초기 상대 위치를 주지 않고 각 local map의 상대 회전·이동을 추정하여 병합한다.

### 처리 단계

```text
OccupancyGrid A + OccupancyGrid B
        ↓
Occupied edge / contour extraction
        ↓
Coarse rotation-translation candidate
        ↓
RANSAC 또는 phase correlation
        ↓
ICP refinement
        ↓
Overlap / RMSE / inlier 검증
        ↓
Transform confidence gate
        ↓
swarm_map → droneN/map 등록
```

### 구현 전략

#### 1단계: 후보 생성

- OccupancyGrid를 8-bit image로 변환
- occupied 영역 edge 또는 distance transform 생성
- ORB/AKAZE/phase correlation 중 한 방법으로 후보 생성
- 여러 개의 transform candidate 유지

#### 2단계: 정밀 정합

- occupied cell을 2D point cloud로 변환
- ICP 또는 point-to-line ICP 적용
- translation, yaw, overlap, RMSE 계산

#### 3단계: Confidence Gate

```text
UNALIGNED
  ↓
CANDIDATE
  ↓
VERIFIED
  ↓
FUSED
  ↓ confidence 하락
DEGRADED
```

### 초기 Acceptance Threshold

| 항목 | 초기 기준 |
|---|---:|
| Map overlap ratio | 15% 이상 |
| ICP RMSE | 0.30 m 이하 |
| Translation jump | update당 0.50 m 이하 |
| Yaw jump | update당 5° 이하 |
| 연속 검증 | 3회 이상 |
| Transform timeout | 3 s 이하 |

### Fail-safe

- confidence 미달 transform은 global map에 반영하지 않는다.
- 기존 verified transform을 유지하되 stale 상태로 표시한다.
- 일정 시간 이상 confidence 복구 실패 시 해당 드론 map을 fusion에서 제외한다.
- 잘못된 정합 후보는 rejection reason과 함께 log로 저장한다.

### 완료 기준

- known-pose baseline 대비 translation/yaw error 정량화
- 반복 환경에서 false merge rate 측정
- 잘못된 transform이 global map에 반영된 횟수 0을 목표
- map registration success rate 및 convergence time 기록

---

# Phase 5. 드론 간 충돌 회피

### 목표

공유 지도와 별도로 타 드론을 시간에 따라 움직이는 동적 장애물로 처리한다.

### 공유 데이터

```text
/droneN/planner/predicted_trajectory
/droneN/status/pose
/droneN/status/velocity
/droneN/status/priority
```

### 예측 궤적 메시지 최소 필드

```text
header
vehicle_id
trajectory_id
timestamp
time_step
positions[]
velocities[]
confidence
valid_until
```

### 처리 원칙

- map에는 정적 장애물만 유지한다.
- 타 드론은 OccupancyGrid에 영구 기록하지 않는다.
- MPPI rollout 시간축에서 타 드론의 예측 위치와 separation을 평가한다.
- 예측 궤적이 stale이면 uncertainty radius를 증가시킨다.
- 교착 상태에서는 우선순위 기반 양보 규칙을 적용한다.

### 초기 안전 규칙

```yaml
peer_avoidance:
  hard_minimum_separation_m: 1.2
  preferred_separation_m: 2.0
  trajectory_timeout_s: 0.5
  stale_uncertainty_growth_mps: 0.5
  emergency_hover_on_conflict: true
```

### 완료 기준

- 최소 기체 간 거리 기록
- collision 0회
- near-miss 발생률 측정
- 교차 경로 시 deadlock 또는 oscillation 여부 확인
- trajectory topic 지연에 따른 안전성 저하 측정

---

# Phase 6. 4대 확장 및 통신 최적화

### 목표

2대에서 검증한 구조를 4대로 확장하고 DDS, CPU, Gazebo RTF 병목을 측정·완화한다.

### 데이터 주기 초기값

| 데이터 | 시작 주기 |
|---|---:|
| LiDAR scan | 10 Hz |
| Local SLAM update | 10 Hz |
| Local map publish | 1 Hz |
| Delta/submap share | 1~2 Hz |
| Global map publish | 1 Hz |
| Predicted trajectory | 5~10 Hz |
| Heartbeat | 2 Hz |
| Metrics | 5~10 Hz |

### QoS 권장

| Topic 유형 | Reliability | Durability | Depth |
|---|---|---|---:|
| LiDAR/Image | Best Effort | Volatile | 5 |
| Local/Global Map | Reliable | Transient Local | 1 |
| Map Delta | Reliable | Volatile | 5 |
| Predicted Trajectory | Best Effort | Volatile | 1 |
| Transform/Confidence | Reliable | Volatile | 10 |
| Failure/Event | Reliable | Transient Local | 10 |

### 통신량 절감

- 전체 map 반복 전송 대신 changed tile 또는 delta map 전송
- map compression 적용
- 동일 데이터의 중복 publisher 제거
- prediction horizon과 trajectory point 수 제한
- camera는 기본 비활성 또는 낮은 FPS 사용
- sensor topic과 event topic의 QoS를 구분
- global map update를 event-trigger 방식으로 제한

### Gazebo 최적화

- headless server 실행
- GUI는 필요 시에만 실행
- LiDAR ray 수와 update rate 최소화
- 카메라 해상도 및 FPS 제한
- 불필요한 contact sensor 및 high-rate plugin 비활성
- physics step size와 real-time update rate 조정
- RTF 0.8 미만 시 sensor update rate부터 감소

### 완료 기준

- 4대 동시 비행 성공
- Gazebo RTF 평균 0.8 이상 목표
- control loop deadline miss율 기록
- DDS bandwidth, CPU, RAM이 artifact에 저장
- 1대→2대→4대 확장에 따른 latency 증가량 측정

---

## 8. 이미지 스티칭 실험의 위치

Gazebo에서 RGB 카메라 영상 스티칭 자체는 가능하지만, 이를 항법용 주 지도 생성 방식으로 사용하지 않는다.

### 사용할 수 있는 목적

- 동일 장소 인식
- Inter-robot loop-closure 후보 생성
- 연구 결과 시각화
- 사람이 보는 panoramic inspection map
- LiDAR 정합 confidence를 보조하는 visual feature

### 항법용으로 바로 사용하기 어려운 이유

- 단안 영상의 scale 불확실성
- 비평면 장면의 parallax
- free/occupied/unknown 정보 부족
- 카메라 방향과 비행 자세에 따른 feature 변화
- 영상 대역폭과 Gazebo rendering 부하
- 조명·텍스처가 부족한 환경에서 정합 실패

### 카메라 실험을 추가하는 시점

1. LiDAR OccupancyGrid known-pose merge 완료
2. unknown-pose map registration 완료
3. false merge가 반복 환경에서 발생함을 확인
4. visual loop-closure가 실제로 false merge 감소에 기여하는지 A/B test

---

## 9. 패키지 및 파일 구조 제안

```text
src/
├─ drone_bringup/
│  ├─ launch/
│  │  ├─ single_drone_autonomy.launch.py
│  │  ├─ multi_drone_autonomy.launch.py
│  │  └─ swarm_mapping.launch.py
│  └─ config/
│     ├─ swarm.yaml
│     ├─ drone1.yaml
│     ├─ drone2.yaml
│     ├─ drone3.yaml
│     └─ drone4.yaml
│
├─ drone_slam/
│  ├─ launch/
│  │  └─ local_slam.launch.py
│  └─ config/
│     └─ slam_toolbox.yaml
│
├─ drone_map_fusion/
│  ├─ drone_map_fusion/
│  │  ├─ map_fusion_node.py
│  │  ├─ map_registration.py
│  │  ├─ occupancy_grid_utils.py
│  │  └─ confidence_gate.py
│  ├─ config/
│  │  └─ map_fusion.yaml
│  └─ test/
│     ├─ test_grid_transform.py
│     ├─ test_log_odds_fusion.py
│     └─ test_confidence_gate.py
│
├─ drone_global_planning/
│  ├─ drone_global_planning/
│  │  ├─ global_astar_node.py
│  │  └─ path_corridor.py
│  └─ config/
│     └─ global_planner.yaml
│
├─ drone_peer_avoidance/
│  ├─ drone_peer_avoidance/
│  │  ├─ trajectory_broadcaster.py
│  │  ├─ peer_trajectory_cache.py
│  │  └─ collision_cost.py
│  └─ config/
│     └─ peer_avoidance.yaml
│
└─ drone_metrics/
   ├─ swarm_metrics_logger.py
   └─ map_quality_evaluator.py
```

---

## 10. Map Fusion Node 요구사항

### Functional Requirements

| ID | 요구사항 |
|---|---|
| MF-001 | 2~4개의 OccupancyGrid를 구독할 수 있어야 한다. |
| MF-002 | 각 map frame을 `swarm_map` 기준으로 변환해야 한다. |
| MF-003 | known-pose 및 unknown-pose 정합 모드를 지원해야 한다. |
| MF-004 | unknown cell은 기존 관측 cell을 삭제하지 않아야 한다. |
| MF-005 | map conflict와 transform confidence를 기록해야 한다. |
| MF-006 | 최신 global map을 Transient Local QoS로 제공해야 한다. |
| MF-007 | stale 또는 invalid map을 자동 제외해야 한다. |
| MF-008 | map version을 단조 증가시켜 planner가 변경을 추적할 수 있어야 한다. |
| MF-009 | fusion failure 시 local-only fallback event를 발행해야 한다. |
| MF-010 | latency, input age, overlap, conflict count를 metrics로 기록해야 한다. |

### Non-Functional Requirements

| ID | 요구사항 |
|---|---|
| MNF-001 | 4대 기준 map fusion p95 latency를 측정 가능해야 한다. |
| MNF-002 | 입력 map 하나가 중단되어도 노드 전체가 종료되지 않아야 한다. |
| MNF-003 | 예외 발생 시 잘못된 global map을 발행하지 않아야 한다. |
| MNF-004 | 동적 메모리 증가 여부를 장시간 실험에서 확인해야 한다. |
| MNF-005 | 동일 입력과 seed에서 재현 가능한 결과를 생성해야 한다. |
| MNF-006 | 단위 테스트로 좌표 변환, grid index, log-odds saturation을 검증해야 한다. |

---

## 11. 실험 설계

### 11.1 1차 실험: Map Sharing 방식 비교

Planner는 `Global A* + Local MPPI`로 고정한다.

```text
Map 방식 3종
1. Local-only
2. Offline merge
3. Online merge

× 장애물 밀도 3종
1. Low
2. Medium
3. High

× 통신 조건 2종
1. Ideal
2. Delay/Loss

= 총 18조건
```

각 조건을 최소 10회 반복한다.

### 11.2 2차 실험: Scalability

```text
드론 수: 1, 2, 4
Map share rate: 0.5, 1, 2 Hz
```

### 11.3 3차 실험: Registration 방식 비교

```text
1. Ground-truth known pose
2. Occupancy image coarse alignment
3. Coarse alignment + ICP
4. Proposed confidence-gated registration
```

### 11.4 4차 실험: Failure 및 Network Fault

- 특정 드론 local map 중단
- 특정 드론 heartbeat timeout
- map packet 지연
- map packet loss
- 잘못된 transform candidate 주입
- global map server 재시작
- peer trajectory topic 중단

---

## 12. KPI

### 12.1 지도 성능

| KPI | 단위 |
|---|---|
| Occupancy IoU | % |
| Occupied precision/recall | % |
| Mapped coverage | % |
| Translation registration error | m |
| Yaw registration error | deg |
| Merge success rate | % |
| False merge rate | % |
| Conflict cell ratio | % |

### 12.2 경로 및 임무 성능

| KPI | 단위 |
|---|---|
| Mission success rate | % |
| Goal arrival time | s |
| Actual path length | m |
| Path efficiency | actual / shortest |
| Global path tracking RMSE | m |
| Replanning latency p50/p95/max | ms |
| Planner oscillation count | count |

### 12.3 안전 성능

| KPI | 단위 |
|---|---|
| Minimum obstacle distance | m |
| Minimum peer separation | m |
| Collision count | count |
| Near-miss count | count |
| Emergency hover count | count |
| Local-only fallback count | count |

### 12.4 시스템 성능

| KPI | 단위 |
|---|---|
| Map fusion latency p50/p95/max | ms |
| Global map age | ms |
| DDS bandwidth | MB/s |
| Topic drop count | count |
| CPU usage | % |
| RAM usage | MB |
| Gazebo RTF | ratio |
| Control loop deadline miss | % |

---

## 13. 필수 로그 필드

```text
timestamp
run_id
world_seed
planner_seed
vehicle_count
vehicle_id
map_version
local_map_stamp
global_map_stamp
map_age_ms
fusion_latency_ms
registration_mode
registration_confidence
overlap_ratio
registration_rmse
translation_error_m
yaw_error_deg
conflict_cell_count
global_path_length_m
actual_path_length_m
replan_reason
replan_latency_ms
minimum_obstacle_distance_m
minimum_peer_separation_m
dds_rx_bytes
dds_tx_bytes
cpu_percent
memory_mb
gazebo_rtf
fallback_state
failure_reason
```

---

## 14. 테스트 전략

### 14.1 단위 테스트

- OccupancyGrid index ↔ world coordinate 변환
- 회전·이동 transform 적용
- 서로 다른 resolution map resampling
- unknown/free/occupied cell 융합
- log-odds saturation
- stale map 제거
- confidence threshold boundary
- map version 증가
- invalid input 처리

### 14.2 통합 테스트

- 2대 map topic 동시 입력
- map source 하나의 지연·중단
- TF timeout
- 잘못된 frame ID
- global map subscriber late join
- planner가 새로운 map version을 인식
- fusion node 재시작 후 복구

### 14.3 시뮬레이션 회귀 테스트

- 고정 world seed
- 고정 planner seed
- 고정 spawn pose
- 동일 config hash
- 동일 git commit
- 동일 Docker image tag
- baseline 대비 KPI 자동 비교

---

## 15. 주요 리스크와 실패 모드

| 리스크 | 실패 결과 | 대응 |
|---|---|---|
| Topic만 namespace 분리 | 다른 드론 데이터 혼합 | TF frame, node name, parameter까지 prefix |
| PX4 port/System ID 충돌 | 잘못된 기체가 arm 또는 이동 | instance별 포트·ID 자동 검증 |
| TF multiple authority | pose jump 및 map 이중화 | transform publisher 소유권 명확화 |
| 잘못된 map registration | 가짜 벽, 경로 단절 | confidence gate 및 연속 검증 |
| 전체 map 고주기 전송 | DDS 병목, sensor drop | delta/submap 및 낮은 publish rate |
| map update마다 A* 실행 | CPU 폭증, path oscillation | event-triggered replanning |
| map correction 즉시 반영 | command discontinuity | transform smoothing 및 version gate |
| 타 드론을 static map에 기록 | ghost obstacle | peer trajectory 별도 관리 |
| 카메라를 4대 고FPS로 실행 | Gazebo RTF 급락 | ROI, 저해상도, 저FPS, 필요 시 활성 |
| Ground Truth 의존 | 연구 타당성 상실 | GT는 평가와 known-pose baseline에 한정 |
| 최종 성공만 기록 | 중간 불안정 구간 누락 | timeline, p95, fallback event 저장 |
| 한 번에 4대로 확장 | 디버깅 불가능 | 1대→2대→4대 단계별 승인 |

---

## 16. 구현 우선순위

### Sprint 1: 단일 SLAM 완성

- [ ] `/drone1/slam/map` 생성
- [ ] TF tree 검증
- [ ] SLAM map 저장
- [ ] ground-truth map 비교
- [ ] A*가 SLAM map을 입력으로 사용
- [ ] artifact에 map KPI 추가

### Sprint 2: 2대 Runtime 분리

- [ ] PX4 instance 2개 실행
- [ ] ROS namespace 분리
- [ ] TF frame prefix 적용
- [ ] 기체별 config 및 artifact 분리
- [ ] 2대 독립 이륙·hover smoke test

### Sprint 3: Known-Pose Map Fusion

- [ ] `map_fusion_node` 생성
- [ ] local map transform 구현
- [ ] global occupancy fusion 구현
- [ ] map version 및 status topic 구현
- [ ] stale input fallback 구현
- [ ] Local-only vs Online merge 비교

### Sprint 4: Global A* + MPPI

- [ ] global path topic 정의
- [ ] path corridor 생성
- [ ] MPPI global path cost 추가
- [ ] event-triggered replanning 구현
- [ ] replan latency 측정

### Sprint 5: Unknown-Pose Registration

- [ ] OccupancyGrid image 변환
- [ ] coarse transform 후보 생성
- [ ] ICP refinement
- [ ] confidence gate
- [ ] false merge rejection test
- [ ] known-pose 대비 오차 평가

### Sprint 6: Peer Avoidance 및 4대 확장

- [ ] predicted trajectory topic
- [ ] peer collision MPPI cost
- [ ] stale trajectory fallback
- [ ] 4대 launch
- [ ] 통신량 및 RTF 최적화
- [ ] 반복 실험 자동화

---

## 17. 단계별 Gate Review

다음 단계로 넘어가기 전에 각 Gate를 통과해야 한다.

### Gate A: Single SLAM

- map 생성 성공률 90% 이상
- TF timeout 0회
- map KPI 자동 생성

### Gate B: 2대 Namespace

- topic cross-talk 0건
- TF duplicate authority 0건
- 독립 Offboard 제어 성공

### Gate C: Known-Pose Fusion

- global map 정상 생성
- fusion p95 latency 기록
- map 입력 중단 시 local-only fallback

### Gate D: Planner Integration

- global path 자동 갱신
- blocked path 우회 성공
- replan latency 및 tracking RMSE 기록

### Gate E: Unknown-Pose Registration

- false merge rate 측정
- confidence rejection 정상 작동
- GT 대비 pose error 기준 충족

### Gate F: 4대 Scalability

- 4대 동시 임무 성공
- RTF, CPU, DDS bandwidth 기준 충족
- collision 0회

---

## 18. 완료 정의

프로젝트 확장은 다음 조건을 만족하면 완료로 판단한다.

- 4대 PX4 instance가 독립 실행된다.
- 각 드론이 독립 local SLAM map을 생성한다.
- local maps가 실시간 global OccupancyGrid로 병합된다.
- unknown-pose mode에서 transform confidence가 검증된다.
- Global A*와 Local MPPI가 역할 분리되어 동작한다.
- 타 드론 예측 궤적 기반 충돌 회피가 동작한다.
- map loss, communication delay, drone dropout 상황에서 fallback이 동작한다.
- 동일 조건 반복 실험으로 평균, 표준편차, p95가 자동 집계된다.
- 실행 결과가 git commit, config, seed, artifact와 연결된다.
- Local-only, offline merge, online merge 방식의 정량 비교 결과가 확보된다.

---

## 19. 예상 최종 성과 표현

> PX4 SITL, ROS 2, Gazebo 기반 4대 UAV 시뮬레이션에서 기체별 namespace와 TF tree를 분리하고, 각 UAV의 LiDAR SLAM OccupancyGrid를 실시간 정합·융합하는 공유 지도 시스템을 구축한다. 공유 Global Map 기반 A* 전역 경로와 MPPI 로컬 회피를 결합하며, map registration confidence gate, stale-data fallback, peer predicted trajectory 기반 충돌 회피를 적용한다. 성능은 지도 IoU, map merge p95 latency, DDS bandwidth, Gazebo RTF, 경로 추종 RMSE, 최소 기체 간 거리로 검증한다.

---

## 20. 이 경험의 기술적 차별점

이 프로젝트의 차별점은 단순히 Gazebo에 드론 여러 대를 띄우는 데 있지 않다.

- PX4 multi-instance와 ROS 2 namespace를 분리한 시스템 구성
- 다중 좌표계와 TF ownership 관리
- LiDAR SLAM map의 실시간 정합 및 confidence 기반 오류 차단
- DDS 통신량과 map update 주기의 최적화
- Global A*와 Local MPPI의 계층형 경로계획
- 타 드론 예측 궤적을 포함한 동적 충돌 회피
- 통신·SLAM·드론 고장 시 안전 fallback
- seed, git commit, config, artifact를 연결한 재현성 검증

이를 통해 자율주행 시스템 엔지니어 채용에서 **분산 시스템, 실시간 SW, SLAM, 경로계획, 안전 설계, 성능 검증을 하나의 시스템으로 통합한 경험**을 증명할 수 있다.
