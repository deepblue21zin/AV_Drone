# Multi-UAV LiDAR 공통관측 기반 SLAM Drift 보정 개발 및 검증 계획서

## 1. 연구 목적

현재 시스템은 두 UAV가 각각 `slam_toolbox` 기반 2D LiDAR SLAM을 수행하고, 초기 spawn transform을 이용해 두 `OccupancyGrid`를 고정 변환하여 stitching한다.

문제는 장거리 비행에서 각 UAV의 SLAM trajectory에 누적 drift가 발생할 경우, 전체 grid에 rigid transform 하나를 적용하는 현재 방식으로는 **지도 내부의 비선형적·구간별 오차를 보정할 수 없다는 점**이다.

따라서 이번 개발의 목표는 다음과 같이 정의한다.

> **두 UAV가 서로 공통으로 관측한 LiDAR 환경을 이용해 Inter-UAV 상대 pose constraint를 생성하고, 이를 joint pose graph에 추가하여 두 UAV의 differential SLAM drift와 map stitching 오차를 줄이는 것**

---

## 2. 이번 개발에서 해결하려는 것 / 해결하지 않는 것

### 해결 대상: Differential Drift

두 UAV가 서로 다르게 누적한 SLAM drift를 보정한다.

예:

\[
e_1=+5m,\qquad e_2=-1m
\]

이면

\[
e_{diff}=e_1-e_2=6m
\]

이 부분은 Inter-UAV constraint로 줄일 수 있다.

### 해결 대상 아님: Common-Mode Drift

두 UAV가 동일한 방향과 크기로 함께 틀어진 경우:

\[
e_1=+5m,\qquad e_2=+5m
\]

이면

\[
e_{diff}=0
\]

이므로 상대측정만으로는 두 UAV가 모두 +5m 틀렸다는 것을 알 수 없다.

따라서 이번 연구의 claim은

> **Global SLAM drift 제거**

가 아니라

> **Independent UAV SLAM 간 differential drift 감소 및 fused-map consistency 향상**

으로 제한한다.

---

## 3. 연구 가설

### H1. Differential drift 존재

현재 137 m 비행에서 UAV1과 UAV2의 SLAM 오차가 동일하게 증가하는 것이 아니라, 크기 또는 방향이 서로 다르게 누적된다.

### H2. 공통 LiDAR 관측으로 상대 pose를 얻을 수 있음

두 UAV가 동일한 정적 환경을 관측한 submap에 대해 registration을 수행하면

\[
Z_{ij}^{inter}
=
[\Delta x,\Delta y,\Delta\psi]
\]

형태의 상대 SE(2) constraint를 생성할 수 있다.

### H3. 여러 Inter-UAV constraint가 trajectory drift를 제한

시간 또는 공간적으로 떨어진 여러 위치에서 Inter-UAV loop constraint가 추가되면 두 trajectory 사이에 graph cycle이 형성되어 differential drift가 감소한다.

### H4. trajectory 보정이 map quality 개선으로 이어짐

최적화된 submap pose로 OccupancyGrid를 재투영하면 현재 fixed-transform stitching보다 다음 개선이 나타날 것으로 가정한다.

- Map IoU 증가
- Double-wall 감소
- Ghost obstacle 감소

---

## 4. 전체 시스템 구조

```text
 UAV 1                                      UAV 2
───────                                    ───────

2D LiDAR                                  2D LiDAR
   │                                         │
   ▼                                         ▼
Local SLAM                                Local SLAM
   │                                         │
   ▼                                         ▼
Keyframe / Submap                        Keyframe / Submap
   │                                         │
   └──────── Descriptor / Metadata ──────────┘
                    ROS 2 DDS
                       │
                       ▼
             Inter-UAV Match Detector
                       │
                       ▼
             Candidate Submap Pair
                       │
                       ▼
             RANSAC + ICP / NDT
                       │
                ┌──────┴──────┐
                │             │
             Reject        Accept
                              │
                              ▼
                  Relative SE(2) Factor
                              │
                              ▼
                    Joint Pose Graph
                              │
                              ▼
                 Optimized Submap Poses
                              │
                              ▼
                     Map Reprojection
                              │
                              ▼
               Corrected Multi-UAV Map
```

핵심은 다음과 같다.

```text
공통 LiDAR 데이터
→ map 자체를 그냥 합침 X

공통 LiDAR 데이터
→ 상대 pose 측정 생성
→ trajectory optimization
→ map 다시 생성 O
```

---

## 5. 개발 단계

## Stage 0. 현재 Baseline 고정

기존 시스템의 결과를 재현 가능한 기준선으로 고정한다.

### 저장 데이터

```text
/drone1/scan
/drone2/scan

UAV1 SLAM pose
UAV2 SLAM pose

UAV1 odometry
UAV2 odometry

map→odom TF

UAV1 OccupancyGrid
UAV2 OccupancyGrid

현재 fused OccupancyGrid

Gazebo GT pose
```

Gazebo Ground Truth는 **평가 전용**으로 사용하며 optimizer 입력에는 사용하지 않는다.

### 측정 지표

- ATE
- RPE
- Endpoint position error
- Endpoint yaw error
- Map IoU
- Chamfer distance
- Double-wall ratio

---

## Stage 1. 이 방법이 실제 문제에 적용 가능한지 검증

알고리즘 구현 전에 현재 오차를 differential component와 common component로 분해한다.

각 UAV의 위치 오차:

\[
e_1(t)=\hat p_1(t)-p_1^{GT}(t)
\]

\[
e_2(t)=\hat p_2(t)-p_2^{GT}(t)
\]

Differential component:

\[
e_{diff}(t)=e_1(t)-e_2(t)
\]

Common component:

\[
e_{common}(t)
=
\frac{e_1(t)+e_2(t)}{2}
\]

### 해석 예시

Inter-UAV correction의 가치가 큰 경우:

```text
UAV1 error       +6.0 m
UAV2 error       +1.5 m

Differential     4.5 m
Common           3.75 m
```

가치가 작은 경우:

```text
UAV1 error       +5.0 m
UAV2 error       +4.8 m

Differential     0.2 m
Common           4.9 m
```

### Go / No-Go 판단

> **후반 map error 중 differential component가 충분히 크게 존재하는가?**

를 우선 확인한다.

Differential drift가 거의 없다면 Inter-UAV correction보다 Local SLAM 자체 개선이 더 적절한 방향이다.

---

## Stage 2. LiDAR 공통 관측 가능성 분석

두 UAV가 동일한 환경 feature를 얼마나 많이 공통 관측하는지 확인한다.

실시간에 동시에 관측할 필요는 없다.

```text
t = 20 s
UAV1 → Area A 관측 → Submap A 저장

t = 45 s
UAV2 → Area A 관측 → Submap B 저장
```

위 상황도 valid overlap으로 본다.

### 분석 단위

137 m 구간을 예를 들어 다음과 같이 나눈다.

```text
0 ~ 20 m
20 ~ 40 m
40 ~ 60 m
...
120 ~ 137 m
```

각 구간에서 다음을 기록한다.

- UAV1 observed landmark 수
- UAV2 observed landmark 수
- shared landmark 수
- shared observation ratio
- valid candidate 수

### 예시

| 구간 | UAV1 | UAV2 | 공통 | Overlap |
|---|---:|---:|---:|---:|
| 0–20 m | 21 | 18 | 9 | 45% |
| 20–40 m | 24 | 20 | 6 | 27% |
| 40–60 m | 18 | 19 | 1 | 5% |

---

## Stage 3. Keyframe / Submap 생성

전체 OccupancyGrid를 직접 최적화하지 않고 각 UAV trajectory를 작은 submap 단위로 나눈다.

### Keyframe 초기 조건

- Translation: `0.5 ~ 1.0 m`
- Yaw: `5 ~ 10°`

둘 중 하나를 넘으면 새로운 keyframe을 생성한다.

### Submap 초기 조건

- 길이: `2 ~ 5 m`
- 또는 `10 ~ 30 scans`
- 생성 후 frozen 처리

```text
UAV1

A0 ─ A1 ─ A2 ─ A3 ─ A4


UAV2

B0 ─ B1 ─ B2 ─ B3 ─ B4
```

### Submap 데이터 구조

```text
uav_id
submap_id
timestamp
initial_pose
local_point_cloud
local_grid
descriptor
registration covariance
run_id
```

---

## Stage 4. Inter-UAV Submap Matching

### 4.1 통신 구조

모든 LaserScan을 지속적으로 DDS로 전송하지 않는다.

먼저 작은 descriptor와 metadata만 교환한다.

```text
UAV1
Submap → Descriptor ─────┐
                         │
                         │ ROS 2 DDS
                         │
UAV2                     │
Submap → Descriptor ─────┘
```

후보 pair가 발견될 때만 실제 submap point set을 요청한다.

이는 DDS bandwidth와 Multi-UAV scalability 측면에서 중요하다.

### 4.2 Candidate Detection

후보 submap 쌍을 찾는다.

```text
A2 ↔ B3
```

초기에는 복잡한 딥러닝 descriptor 대신 다음과 같은 단순 descriptor로 시작할 수 있다.

- occupancy shape
- point distribution
- landmark pattern
- centroid
- normal distribution

### 4.3 Geometric Verification

반복 원통 환경에서 false loop를 막기 위해 descriptor 결과를 바로 factor로 사용하지 않는다.

```text
Candidate
   ↓
trajectory / spatial prior gate
   ↓
RANSAC
   ↓
ICP 또는 NDT
   ↓
fitness / residual
   ↓
transform covariance
   ↓
temporal consistency
   ↓
ACCEPT / REJECT
```

---

## Stage 5. Relative SE(2) Constraint 생성

Registration이 성공하면 다음 상대 pose를 얻는다.

\[
Z_{ij}
=
[\Delta x,\Delta y,\Delta\psi]
\]

예:

```text
A2 ↔ B3

Δx   = 1.3 m
Δy   = 14.6 m
Δyaw = 2.1 deg
```

현재 SLAM pose들이 예상하는 상대관계는

\[
(X_i^1)^{-1}X_j^2
\]

이다.

Inter-UAV residual은 개념적으로 다음과 같이 정의한다.

\[
r_{ij}
=
\mathrm{Log}
\left[
Z_{ij}^{-1}
(X_i^1)^{-1}X_j^2
\right]
\]

이 residual을 Pose Graph의 Inter-UAV factor로 사용한다.

---

## Stage 6. Joint Pose Graph Optimization

Graph 구조 예시:

```text
UAV1

A0 ─ A1 ─ A2 ─ A3 ─ A4
     │         │
     │         │
     │         │
B0 ─ B1 ─ B2 ─ B3 ─ B4

UAV2
```

### Intra-UAV edge

- odometry
- local scan registration

### Inter-UAV edge

- common LiDAR environment registration에서 얻은 SE(2) constraint

목적함수:

\[
X^*=
\arg\min_X
\sum
\|r_{odom}\|_{\Sigma_o}^2
+
\sum
\|r_{scan}\|_{\Sigma_s}^2
+
\sum
\rho
\left(
\|r_{inter}\|_{\Sigma_i}^2
\right)
\]

초기 구현은 **GTSAM batch optimization**으로 수행한다.

Online iSAM2는 Offline 검증 이후로 미룬다.

---

## Stage 7. Map Reprojection

Pose Graph Optimization 결과는 occupancy grid 자체가 아니라 **수정된 submap pose 집합**이다.

예:

```text
Before

A0 ─ A1 ─ A2
          \
           A3
            \
             A4


After

A0 ─ A1 ─ A2 ─ A3 ─ A4
```

각 frozen submap을 optimized pose

\[
T_k^*
\]

를 사용해 world frame에 다시 투영한다.

```text
local submap
+
optimized pose
↓
global occupancy grid
```

제안 방식의 corrected map은 별도 topic으로 발행한다.

```text
/swarm/global_map_corrected
```

기존 baseline map은 유지한다.

```text
/swarm/global_map
```

---

## 6. ROS 2 노드 구조

권장 구조:

```text
/drone1/submap_builder
/drone2/submap_builder

/drone1/submap_descriptor
/drone2/submap_descriptor

/swarm/inter_uav_matcher
/swarm/constraint_validator
/swarm/joint_pose_graph
/swarm/map_projector
```

### 주요 Topic

```text
/drone1/slam/submap
/drone2/slam/submap

/drone1/slam/descriptor
/drone2/slam/descriptor

/swarm/loop_candidates
/swarm/inter_uav_constraints

/swarm/optimized_submap_poses
/swarm/global_map_corrected
```

---

## 7. ROS 2 QoS 및 통신 구조

### Descriptor

```yaml
reliability: reliable
durability: volatile
history: keep_last
depth: 10
```

### Submap Request / Response

```yaml
reliability: reliable
```

### LaserScan

```yaml
reliability: best_effort
history: keep_last
depth: 5
```

핵심 통신 구조:

```text
LaserScan 전체 지속 전송 X

Descriptor 지속 전송
        ↓
후보 발견
        ↓
Submap 요청
```

PX4 Micro XRCE-DDS 경로와 ROS 2 대용량 submap traffic이 동시에 동작할 경우, mapping traffic 때문에 Offboard setpoint와 vehicle status latency가 증가하지 않도록 전송률과 process 구조를 분리한다.

---

## 8. 검증 실험

## Experiment 1. Differential / Common Drift 분석

### 목적

현재 map drift가 Inter-UAV correction으로 해결 가능한 성분을 포함하는지 확인한다.

분석:

```text
e1(t)
e2(t)
e_diff(t)
e_common(t)
```

---

## Experiment 2. Synthetic Perfect Association 기반 Backend 검증

실제 LiDAR matcher를 만들기 전에 Ground Truth는 **어떤 submap끼리 같은 장소인지 label을 만드는 용도**로만 사용한다.

GT pose 자체를 factor로 넣지 않는다.

### 목적

> 실제 data association 문제를 배제했을 때 Joint PGO backend 자체가 differential drift를 줄일 수 있는가?

를 확인한다.

---

## Experiment 3. 실제 LiDAR Registration

GT label 없이 다음 구조로 candidate와 constraint를 생성한다.

```text
descriptor
   ↓
candidate
   ↓
RANSAC
   ↓
ICP / NDT
   ↓
gate
   ↓
inter-UAV factor
```

측정 항목:

- Correct match
- False match
- Missed match

---

## Experiment 4. Map Correction 비교

### B0 — Local Raw SLAM

```text
각 UAV 독립 SLAM
```

### B1 — Current Fixed Fusion

```text
Local SLAM
+
Static spawn transform
```

### B2 — PGO + Perfect Inter-UAV Association

성능 상한(oracle association) 확인.

### Proposed — PGO + Actual LiDAR Inter-UAV Matching

실제 제안 pipeline.

---

## 9. Ablation 실험

### A1. Inter-UAV Factor 개수

```text
0개
1개
3개
5개 이상
```

### A2. LiDAR Overlap 비율

```text
높음
중간
낮음
없음
```

### A3. False Match 비율

```text
0%
5%
10%
```

Robust gate의 안정성을 함께 평가한다.

---

## 10. 평가 지표

### Localization

- ATE
- RPE
- Endpoint drift

### Map Quality

- Map IoU
- Chamfer Distance
- Double-wall Ratio
- Ghost Obstacle Ratio

### Cooperative Matching

Precision:

\[
Precision
=
\frac{TP}{TP+FP}
\]

Recall:

\[
Recall
=
\frac{TP}{TP+FN}
\]

추가로 반드시 다음을 기록한다.

- False correction rate
- Catastrophic map corruption count

---

## 11. 성공 기준

최종 threshold는 baseline 통계를 확인한 뒤 결정한다.

초기 개발 목표:

| 항목 | 목표 |
|---|---|
| Differential drift | B1 대비 감소 |
| ATE | B1 대비 감소 |
| Endpoint error | B1 대비 감소 |
| Map IoU | 증가 |
| Double-wall | 감소 |
| Inter-UAV precision | 높게 유지 |
| Catastrophic false correction | 0 또는 매우 낮음 |
| Collision | 0 |

---

## 12. 실패 결과 해석

### Case A. Differential drift 자체가 작음

```text
Differential drift << Common drift
```

→ Inter-UAV matching은 현재 root cause의 핵심 해결책이 아님.

→ Local SLAM 자체 개선 또는 absolute anchor 방향으로 전환.

### Case B. Differential drift 큼 + LiDAR overlap 충분 + PGO 효과 있음

→ 현재 방향 그대로 진행.

### Case C. Differential drift 큼 + overlap 거의 없음

→ LiDAR + 통신만으로는 정보 부족.

→ UWB, anchor 또는 다른 센서 필요.

### Case D. Overlap 충분 + false match 과다

→ 반복 구조의 perceptual aliasing이 핵심 문제.

→ Matching / geometric verification 연구로 확장 가능.

---

## 13. 개발 우선순위

```text
1. Differential/Common drift 분석
             ↓
2. Shared observation 분석
             ↓
3. Offline submap 생성
             ↓
4. GT association 기반 PGO
             ↓
5. Map reprojection
             ↓
6. 실제 LiDAR matching
             ↓
7. Robust validation
             ↓
8. ROS 2 online화
```

**바로 ICP부터 구현하지 않는다.**

먼저 backend가 실제 문제를 해결할 수 있는지 검증한다.

---

## 14. 4주 개발 일정

### 1주차 — Feasibility

- 기존 137 m bag 선정
- UAV1/2 ATE 계산
- Differential/Common drift 분해
- Shared LiDAR observation 분석
- Submap 포맷 결정

결과물:

```text
drift_analysis.csv
shared_observation.csv
baseline_report.md
```

### 2주차 — Backend

- Offline submap 생성
- GTSAM SE(2) graph
- GT 기반 inter-UAV association
- 여러 factor 추가
- Optimized pose 출력

결과물:

```text
initial_graph.g2o
optimized_graph.g2o
optimized_submap_poses.csv
```

### 3주차 — Map Correction

- Optimized submap reprojection
- Corrected OccupancyGrid
- B0 / B1 / B2 정량 비교
- IoU / Double-wall 계산

이 단계에서 성능이 개선되지 않으면 연구 방향을 재검토한다.

### 4주차 — Actual Matching

- Descriptor
- Candidate search
- ICP / NDT
- RANSAC / gating
- Precision / Recall
- Proposed 전체 pipeline 검증

시간이 남는 경우 Online ROS 2 node로 확장한다.

---

## 15. 논문용 핵심 스토리

> 장거리 저특징 환경에서 두 UAV의 독립 2D LiDAR SLAM에 서로 다른 누적 drift가 발생하며, 현재의 fixed-transform map stitching은 이를 보정할 수 없다. 본 연구에서는 두 UAV가 시간적으로 서로 다른 시점에 관측한 공통 LiDAR submap을 통신으로 탐색하고, geometric registration으로 얻은 상대 SE(2) constraint를 joint pose graph에 추가한다. 이를 통해 UAV 간 differential trajectory drift를 억제하고, 보정된 submap pose를 이용해 global occupancy map을 재구성한다.

---

## 16. 이번 개발에서 의도적으로 제외하는 범위

이번 1차 개발에서는 다음을 제외한다.

- LiDAR Hessian / weak-direction 분석
- UWB
- Active rendezvous
- Hover / 속도차 motion excitation
- Information gain optimizer
- Real-time iSAM2
- 실제 상대센서
- 복잡한 recoverability manager

1차 목표는 다음 pipeline을 완성하는 것이다.

```text
LiDAR overlap
    ↓
Inter-UAV loop constraint
    ↓
Offline joint PGO
    ↓
Optimized submap pose
    ↓
Map reprojection
```

---

## 17. 최종 목표

### 기술적 목표

> 두 UAV의 공통 LiDAR 환경 관측으로 생성한 Inter-UAV SE(2) constraint를 사용하여 독립 SLAM 사이의 differential drift를 줄이고, 기존 fixed-transform map fusion보다 일관된 global map을 생성한다.

### 검증 목표

- Differential drift 감소
- ATE / RPE 감소
- Endpoint error 감소
- Map IoU 증가
- Double-wall / Ghost 감소
- False Inter-UAV constraint로 인한 catastrophic correction 방지

### 연구적 한계

- 공통 환경 관측이 전혀 없으면 LiDAR + 통신만으로 Inter-UAV spatial constraint를 만들 수 없다.
- 두 UAV가 동일한 방향과 크기로 drift하는 common-mode error는 pairwise Inter-UAV constraint만으로 제거할 수 없다.
- 반복 구조에서는 perceptual aliasing에 의한 false loop가 발생할 수 있다.

---

## 18. 채용/이력서 전환 문장

> 장거리 저특징 환경에서 독립 2D LiDAR SLAM의 누적 drift가 fixed-transform map fusion으로 보정되지 않는 문제를 분석하고, 통신으로 공유한 UAV별 LiDAR submap의 geometric correspondence를 이용해 Inter-UAV constraint를 생성하는 joint pose-graph 기반 지도 보정 구조를 개발하였다.

> ROS 2 기반 Multi-UAV 시스템에서 LiDAR data association, Inter-Robot Loop Closure, Pose Graph Optimization, Submap Reprojection을 연결하고, ATE·RPE·Map IoU·Double-wall·false correction rate를 기준으로 정량 검증하였다.

이 경험은 단순한 Multi-UAV 구동이 아니라 **분산 localization, 센서 융합, 통신 효율, graph optimization, map consistency를 하나의 시스템 문제로 해결한 경험**으로 설명할 수 있다.
