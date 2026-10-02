# 저특징 직선 환경의 Multi-UAV 오차 보정 연구 주제 및 선행연구 종합 정리

> 기준일: 2026-08-26  
> 대상 프로젝트: AV_Drone, 2-UAV, 2D LiDAR, ROS 2/PX4/Gazebo  
> 문서 목적: 지금까지 논의한 문제 정의, 핵심 개념, 선행연구, 신규성 판단, 추천 논문 주제와 검증 계획을 하나의 문서로 정리한다.  
> 중요 결론: **`LiDAR 퇴화를 감지하고 필요할 때 다른 로봇과 랑데부하여 보정한다`는 넓은 아이디어는 이미 선행연구가 있으므로 그대로 신규성을 주장할 수 없다.** 현재 가장 유망한 방향은 제한된 평행 비중첩 경로에서 `LiDAR 취약 방향`과 `드론 간 상대측정`의 **결합 관측가능성**을 분석하고, 실제로 정보가 추가되는 경우에만 최소 차선 내 협력 기동을 수행하는 문제로 좁히는 것이다.

---

## 1. 한눈에 보는 결론

### 1.1 처음 생각했던 문제

일반적인 SLAM은 한 드론이 과거에 지나간 장소로 돌아왔을 때 loop closure를 만들어 누적 오차를 줄인다. 그러나 현재 프로젝트의 드론은 긴 직선 구간을 한 방향으로 이동하며 출발 지점으로 돌아오지 않는다.

따라서 처음에는 다음 질문에서 출발했다.

> 한 드론이 과거 장소로 돌아오지 않아도 다른 드론이 대신 관측하거나, 두 드론이 필요할 때 만나 상대측정을 만들어 누적 오차를 보정할 수 있는가?

답은 **가능하다**이다. 다만 단순한 통신이나 지도 공유가 아니라 다음 중 하나가 필요하다.

1. 다른 드론이 같은 장소를 관측해 만드는 `inter-UAV loop closure`
2. 두 드론이 서로를 직접 관측해 만드는 `inter-UAV relative-pose constraint`
3. UWB range/bearing, 비전, AprilTag, LiDAR robot detection 등의 실제 상대측정
4. 이 제약을 각 드론의 odometry/keyframe과 함께 최적화하는 pose graph

### 1.2 문헌 조사 후 바뀐 판단

다음 아이디어는 이미 각각 선행연구가 있다.

- LiDAR의 퇴화 또는 취약 방향 검출
- 오차나 불확실성이 커질 때 능동 랑데부 요청
- 상대측정의 정보량이 커지도록 로봇의 위치나 경로 최적화
- 취약한 상태 차원만 다른 센서로 선택적으로 보완
- 특징이 부족한 터널에서 다른 로봇을 이동식 기준점으로 사용

특히 [Leap-SLAM](https://doi.org/10.1016/j.inffus.2026.104538)은 정보행렬로 터널의 LiDAR 퇴화를 분석하고, reflective marker를 가진 Beacon Robot과 Master Robot이 leapfrogging 방식으로 전진하며 외부 제약을 제공한다. 현재 아이디어와 매우 가깝다.

따라서 다음 주제는 **신규성이 부족하다**.

> 저특징 직선 환경에서 LiDAR 퇴화를 검출하고, 오차가 커지면 다른 드론과 랑데부하여 보정한다.

### 1.3 현재 추천 주제

가장 방어하기 좋은 후보 주제는 다음과 같다.

> **저특징 평행 복도 환경에서 2D LiDAR 퇴화와 드론 간 상대측정의 결합 관측가능성 분석 및 최소 차선 내 협력 기동**

영문 후보:

> **Joint Observability Analysis of 2D LiDAR Degeneracy and Inter-UAV Measurements with Minimal In-Lane Cooperative Maneuvers in Feature-Poor Parallel Corridors**

이 주제도 아직 `선행연구가 전혀 없는 것으로 확정된 주제`는 아니다. 현재 공개 문헌 검색에서 직접 일치하는 연구를 찾지 못한 **후보 공백(candidate gap)**이다. 논문 투고 전에는 IEEE Xplore, Scopus, Web of Science, Google Scholar의 인용·피인용 추적과 특허 검색이 추가로 필요하다.

---

## 2. 현재 프로젝트 조건과 실제 문제

현재 AV_Drone의 주요 조건은 다음과 같다.

| 항목 | 현재 조건 |
|---|---|
| 기체 수 | UAV 2대 |
| 환경 | 약 137 m의 긴 직선·반복 구조 |
| 진행 방식 | 두 드론이 같은 방향으로 진행하고 출발점으로 복귀하지 않음 |
| lane | 약 `y=-7.5 m`, `y=+7.5 m` |
| lane 중심 간격 | 약 15 m |
| 2D LiDAR 최대 범위 | 약 8 m |
| 현재 operational map | MAVROS odometry 기반 known-pose map fusion |
| 현재 raw SLAM | `slam_toolbox` scan matching을 진단 레이어로 병렬 유지 |
| 현재 핵심 실패 | 장거리 raw SLAM drift 및 실제 월드 정렬 품질 저하 |

현재 구조에서는 두 드론의 map을 전달하고 고정 spawn transform으로 합칠 수는 있다. 하지만 이것은 `cooperative SLAM`이 아니다. 두 궤적을 연결하는 측정 제약이 없으므로 각 드론 내부에서 누적된 drift는 줄어들지 않는다.

프로젝트의 구현 상태와 장거리 실험 근거는 다음 문서에 별도로 정리되어 있다.

- [프로젝트 현황 및 로드맵](../PROJECT_STATUS_AND_ROADMAP.md)
- [SLAM drift 원인 분석](../experiments/run_reports/2026-08-04_slam_root_cause_and_fallback.md)
- [통신 보조 Multi-UAV map stitching 설계](communication-assisted-multi-uav-map-stitching.md)

---

## 3. 기존 오차 보정과 다른 드론을 이용한 보정

### 3.1 전통적인 한 드론의 loop closure

```text
한 드론의 현재 관측
        ↓
과거에 같은 드론이 저장한 장소와 일치
        ↓
현재 pose와 과거 pose 사이에 loop edge 생성
        ↓
pose graph optimization
        ↓
중간에 누적된 궤적과 지도를 함께 수정
```

이 방식은 과거 장소를 다시 봐야 하므로 폐곡선 경로나 복귀 경로가 필요하다.

### 3.2 다른 드론이 과거 장소를 대신 관측하는 방법

드론 1이 돌아오지 않더라도 드론 2가 드론 1의 과거 장소를 관측하면 다음 제약을 만들 수 있다.

```text
드론 1의 과거 keyframe/submap ─────┐
                                    ├─ 같은 장소로 판정
드론 2의 현재 keyframe/submap ─────┘
                         ↓
              inter-UAV loop closure
                         ↓
              두 trajectory를 공동 보정
```

이를 `inter-robot loop closure` 또는 `inter-UAV place recognition constraint`라고 부른다.

### 3.3 두 드론이 서로를 직접 관측하는 방법

두 드론이 같은 환경 특징을 볼 필요 없이 서로를 직접 관측해도 된다.

가능한 센서 예:

- UWB range
- UWB range+bearing
- 카메라와 AprilTag
- 카메라 기반 상대 bearing/pose
- LiDAR에서 상대 드론 검출
- 통신 채널의 Wi-Fi AoA/CSI

이 경우 측정은 대략 다음 형태다.

\[
z_{12}=h(x_1,x_2)+n
\]

여기서 \(x_1,x_2\)는 두 드론의 pose이고, \(z_{12}\)는 거리·방향·상대 yaw 등의 측정이다. 이 측정을 pose graph factor로 넣어 두 궤적을 연결한다.

### 3.4 반드시 구분해야 할 세 단계

| 단계 | 의미 | drift 보정 여부 |
|---|---|---|
| Map sharing | map/keyframe/odometry를 네트워크로 전달 | 전달만으로는 불가 |
| Map fusion | 알려진 transform으로 raster map을 겹침 | 내부 trajectory drift는 보통 남음 |
| Cooperative SLAM | 실제 inter-UAV constraint로 joint trajectory/submap pose를 최적화 | 가능 |

`두 드론이 통신한다`는 사실만으로 위치 정보가 새로 생기지는 않는다. 오차 보정에는 반드시 물리적 관측에서 얻은 추가 제약이 필요하다.

---

## 4. 랑데부의 정확한 의미

### 4.1 쉬운 정의

랑데부(rendezvous)는 두 로봇이 **서로 관측하거나 데이터를 교환할 수 있는 상대 위치와 시점**을 의도적으로 만드는 행동이다.

꼭 두 드론이 완전히 같은 좌표에서 만나는 것을 뜻하지 않는다.

- 카메라로 서로 볼 수 있는 거리까지 접근
- UWB 측정 기하가 좋아지도록 서로 다른 위치에 배치
- 한 드론이 잠시 hover하고 다른 드론이 짧게 움직임
- 같은 장소를 서로 다른 시간에 방문하여 submap을 연결
- 통신 품질이 좋은 상대 위치를 만들고 정보를 교환

### 4.2 이 프로젝트에서의 랑데부 후보

현재 두 lane의 간격이 15 m이고 LiDAR 범위가 8 m이므로 lane 중심을 그대로 유지하면 안정적인 직접 LiDAR overlap이 어렵다. 따라서 랑데부를 사용한다면 다음 중 하나가 필요하다.

1. lane 내부 또는 안전 구간에서 제한된 측면 접근
2. 한 드론의 hover와 다른 드론의 짧은 arc/속도 변화
3. LiDAR보다 긴 거리에서 작동하는 UWB 또는 카메라 기반 상대측정
4. 미리 정한 anchor가 아니라 불확실성에 따라 요청하는 event-triggered observation

### 4.3 기존 논문에 필요할 때 랑데부하는 방법이 있는가?

**있다.** Wang 등은 각 로봇이 trajectory error를 감시하다가 정해진 임계치를 넘으면 능동 랑데부를 요청하고, Wi-Fi의 상대 위치 정보를 사용하여 동료 로봇을 선택·접근한 뒤 pose graph를 보정했다. 따라서 `필요할 때 랑데부` 자체는 신규 아이디어가 아니다.

- [Active Rendezvous for Multi-Robot Pose Graph Optimization using Sensing over Wi-Fi](https://arxiv.org/abs/1907.05538)

---

## 5. 취약한 방향 분석을 쉽게 설명하면

### 5.1 터널 비유

긴 직선 터널 안에서 눈을 감았다가 다시 떴다고 생각한다.

- 옆으로 이동하면 한쪽 벽이 가까워지고 반대쪽 벽은 멀어진다.
  - 센서 영상이 크게 변하므로 옆 방향 위치를 알아내기 쉽다.
- 앞으로 조금 이동하면 보이는 평행 벽 모양이 거의 같다.
  - 얼마나 앞으로 이동했는지 구별하기 어렵다.

```text
벽  ─────────────────────────────────────
                UAV  → → →  진행 방향
벽  ─────────────────────────────────────

횡방향 이동: 벽까지 거리가 크게 변함   → 강하게 관측됨
진행축 이동: scan 모양이 거의 같음      → 취약할 가능성이 큼
```

이때 LiDAR가 위치 변화를 잘 구분하지 못하는 pose-space 방향을 다음과 같이 부른다.

- 취약 방향
- 퇴화 방향(degenerate direction)
- 약관측 방향(weakly observable direction)
- 약제약 방향(weakly constrained direction)

### 5.2 scan matching 관점

scan matching은 후보 pose를 조금 바꿔가며 LiDAR 점들이 map에 얼마나 잘 맞는지 계산한다.

- pose를 조금만 바꿔도 정합 오차가 크게 증가한다.
  - 정답 주변의 비용 함수가 가파르다.
  - 해당 방향은 잘 관측된다.
- pose를 바꿔도 정합 오차가 거의 그대로다.
  - 비용 함수가 평평하다.
  - 여러 pose가 모두 비슷하게 맞아 해당 방향이 취약하다.

### 5.3 정보행렬과 고유값

scan matching의 국소 정보는 보통 다음 Hessian 근사로 표현한다.

\[
H_{L}=J^{T}WJ
\]

- \(J\): pose 변화가 scan residual을 얼마나 바꾸는지 나타내는 Jacobian
- \(W\): 각 측정의 신뢰도
- 큰 고유값: 비용이 빠르게 증가하는 강한 방향
- 작은 고유값: 비용이 거의 변하지 않는 취약 방향
- 해당 고유벡터: \([\Delta x,\Delta y,\Delta\psi]\) 중 어떤 조합이 취약한지 표시

예를 들어 가장 작은 고유값의 고유벡터가 다음과 같다면,

\[
v_{min}\approx[0.98,\;0.03,\;0.18]^T
\]

대부분 진행축 \(x\)이지만 yaw도 조금 섞인 방향이 취약하다는 뜻이다.

공분산은 개념적으로 정보행렬의 역과 관계가 있다.

\[
\Sigma \approx H^{-1}
\]

따라서 작은 고유값 방향은 큰 분산을 만들며, 2차원 그림에서는 길게 늘어난 uncertainty ellipse로 나타난다.

### 5.4 yaw를 항상 취약하다고 보면 안 되는 이유

두 개의 평행 벽이 충분히 길고 scan 품질이 좋으면 회전했을 때 벽 정렬이 크게 어긋나므로 yaw는 오히려 강하게 관측될 수 있다. 그러나 다음 조건에서는 yaw가 불안정하거나 진행축과 결합될 수 있다.

- 짧은 LiDAR range
- 희소한 각도 해상도
- 반복되는 원통이나 기둥
- 부분 가림
- 잘못된 correspondence
- 2D 투영으로 인한 정보 손실

따라서 `직선 복도이므로 x와 yaw가 무조건 취약하다`고 정하지 말고, 실제 \(H_L\)의 고유값과 고유벡터를 기록해야 한다.

### 5.5 퇴화와 반복장면 오인식은 다르다

| 문제 | 의미 | 비용 함수 모습 | 주요 위험 |
|---|---|---|---|
| Degeneracy | 정보가 부족해 여러 pose가 비슷하게 맞음 | 넓고 평평함 | 해당 방향 drift |
| Perceptual aliasing | 다른 기둥·장소를 같은 것으로 잘못 대응 | 틀린 위치에도 강한 최적점 | 갑작스러운 잘못된 보정 |

현재 반복 원통 환경에서는 두 현상이 함께 나타날 수 있으므로, 고유값 기반 퇴화 검출만으로 false match 문제까지 해결된다고 주장하면 안 된다.

---

## 6. 드론 간 측정이 취약 방향을 실제로 보완하는가?

다른 드론의 측정이 있다고 해서 항상 도움이 되는 것은 아니다. 핵심은 그 측정이 **현재 LiDAR가 약한 바로 그 방향에 새로운 정보**를 주는가이다.

LiDAR 정보행렬을 \(H_L\), 드론 간 상대측정의 정보행렬을 \(H_R\)이라고 하면 결합 정보는 개념적으로 다음과 같다.

\[
H_{total}=H_L+H_R
\]

LiDAR의 가장 취약한 방향 \(v_{min}\)에 상대측정이 주는 정보는 다음 값으로 평가할 수 있다.

\[
g=v_{min}^{T}H_Rv_{min}
\]

- \(g\)가 큼: 상대측정이 LiDAR 취약 방향을 잘 보완
- \(g\)가 작음: 만나거나 통신해도 필요한 방향 정보가 거의 증가하지 않음

이 값은 추천 연구 주제의 핵심 아이디어 후보다. 단순히 `불확실성이 커졌다`만 보는 것이 아니라 다음 두 조건을 함께 본다.

1. 지금 LiDAR가 실제로 어느 방향에서 약한가?
2. 후보 상대측정 또는 기동이 그 방향의 정보를 얼마나 증가시키는가?

### 6.1 range-only의 한계

두 드론이 15 m 간격을 유지하며 같은 속도로 평행 이동하면 range-only 센서는 거의 같은 측면 거리만 반복해서 측정한다. 진행축 오차를 구별하기 위한 정보가 부족할 수 있다.

```text
UAV 1  → → → → →
       15 m 유지
UAV 2  → → → → →

측정되는 거리와 형상이 거의 일정
→ 진행축 상대 오차를 분리하기 어려움
```

이 경우 속도 차이, 한쪽의 일시 hover, 제한된 측면 이동 또는 짧은 arc 같은 motion excitation이 필요할 수 있다.

### 6.2 공통모드 오차의 근본 한계

두 드론이 모두 같은 방향으로 2 m 잘못 추정되었다고 가정한다.

```text
실제:       UAV 1 ---------- UAV 2
추정:            UAV 1 ---------- UAV 2
                 둘 다 같은 만큼 이동
```

두 드론 사이의 상대거리와 상대방향은 그대로일 수 있다. 따라서 드론 간 상대측정만으로는 두 드론이 함께 움직인 `common-mode drift`를 제거할 수 없다.

이를 해결하려면 상황에 따라 다음 중 하나가 필요하다.

- 절대 anchor 또는 알려진 landmark
- 고정 UWB anchor
- GNSS/RTK 또는 external tracking
- 과거 map과의 신뢰할 수 있는 place constraint
- 서로 다른 오차 특성을 가진 추가 센서
- 세 번째 로봇과 충분한 측정 기하

논문에서는 `상대측정으로 모든 drift를 제거한다`고 주장하지 않고, **복구 가능한 상태 방향과 복구 불가능한 gauge/common-mode 방향을 구분**해야 한다.

---

## 7. 선행연구 분류 및 핵심 비교

### 7.1 퇴화·취약 방향 검출 및 처리

| 논문 | 문제 | 핵심 아이디어 | 해결 범위와 남은 한계 | 우리와의 관계 |
|---|---|---|---|---|
| [Degeneracy-Aware Factors with Applications to Underwater SLAM, IROS 2019](https://www.cs.cmu.edu/~kaess/pub/Hinduja19iros.pdf) | 기하가 부족할 때 ICP와 loop factor가 잘못된 방향까지 강하게 제약 | 퇴화-aware ICP와 부분 제약 loop factor로 잘 관측된 방향만 최적화 | 다른 로봇의 능동 기동은 다루지 않음 | `취약 방향만 사용`은 이미 존재 |
| [DARE-SLAM, 2021](https://arxiv.org/abs/2102.05117) | 지하의 모호하고 퇴화된 장소에서 loop closure 실패 | 퇴화 인지와 drift-resilient loop closing, multi-robot map 연결 | 주로 loop closure 검출·검증 | 반복 지형의 false loop 처리에 참고 |
| [X-ICP, IEEE TRO 2024](https://doi.org/10.1109/TRO.2023.3335691) | LiDAR ICP가 약제약 방향으로 발산 | correspondence가 principal direction에 주는 정보를 세밀하게 분석하고 constrained ICP 수행 | 단일 registration의 localizability 처리 | 취약 방향 검출 자체의 강한 선행연구 |
| [DALI-SLAM, 2025](https://doi.org/10.1016/j.isprsjprs.2025.01.036) | motion distortion, LiDAR 퇴화, 부정확한 pose-graph constraint | degeneracy-aware LIO와 multi-constraint PGO 결합 | 다른 드론의 능동 도움은 핵심이 아님 | LiDAR 퇴화 검출·PGO 설계 참고 |
| [Fuse Only What Matters, 2026](https://doi.org/10.1016/j.isprsjprs.2026.05.031) | 항상 모든 센서를 융합하면 계산량과 저품질 센서 잡음 증가 | 퇴화 시점과 상태 방향을 검출하고 필요한 차원에만 visual constraint를 선택적으로 결합 | 단일 플랫폼의 LiDAR-visual fusion | `약한 차원만 선택 보완`도 이미 있음 |

### 7.2 다중 로봇 C-SLAM과 inter-robot loop closure

| 논문 | 문제 | 핵심 아이디어 | 남은 한계 또는 차이 | 프로젝트에서의 의미 |
|---|---|---|---|---|
| [DOOR-SLAM, 2019](https://arxiv.org/abs/1909.12198) | 분산 환경에서 잘못된 inter-robot loop가 전체 graph를 손상 | 분산 place recognition과 pairwise consistency로 outlier 제거 | 능동 랑데부·퇴화 방향이 중심은 아님 | false merge gate의 기준 |
| [Kimera-Multi, 2021/2022](https://arxiv.org/abs/2106.14386) | 분산 다중 로봇의 전역 일관된 metric-semantic mapping | inter-robot loop closure, robust distributed PGO, mesh deformation | visual-inertial 중심, 공통 장소 필요 | 전체 C-SLAM 구조의 기준선 |
| [Swarm-SLAM, 2023](https://arxiv.org/abs/2301.06230) | 제한된 통신에서 확장 가능한 decentralized C-SLAM | sparse ROS 2 C-SLAM과 inter-robot loop-closure 우선순위화 | 취약 방향 기반 능동 기동이 핵심은 아님 | ROS 2 메시지·통신 구조 참고 |

### 7.3 필요할 때 랑데부하거나 상호관측하는 연구

| 논문 | 문제 | 언제/어떻게 도움을 요청했는가 | 성과와 한계 | 신규성 영향 |
|---|---|---|---|---|
| [Active Rendezvous for Multi-Robot PGO, 2019](https://arxiv.org/abs/1907.05538) | 특징이 부족하거나 map/location 없이 로봇 간 정보 교환과 PGO 보정 필요 | trajectory error가 threshold를 넘으면 동료를 선택하고 Wi-Fi AoA/CSI로 능동 접근·관측 | 실제 로봇에서 active rendezvous로 pose error 감소. Wi-Fi sensing과 정지/이동 역할 사용 | `오차가 클 때 랑데부`는 이미 있음 |
| [Active Collaborative Localization in Heterogeneous Robot Teams, 2023](https://arxiv.org/abs/2305.18193) | 경량 UAV의 VIO drift | Fisher information을 이용해 LiDAR UGV를 정보가 큰 위치에 능동 배치 | 이종 UGV-UAV와 visual observation 중심 | `도움 로봇의 위치 최적화`가 이미 있음 |
| [Preserving Relative Localization via Active Mutual Observation, IROS 2024](https://arxiv.org/abs/2407.01292) | 제한된 카메라 FoV 때문에 환경관측과 상호관측이 충돌 | Kalman Filter uncertainty에 따라 상호관측 시점과 yaw를 계획 | 실제 드론에서 drift를 최대 65% 감소. visual swarm 중심 | `불확실성 기반 드론 상호관측`이 이미 있음 |
| [Infrastructure-less UWB-based Active Relative Localization, 2024](https://arxiv.org/abs/2409.12780) | 고정 anchor 없이 UWB 상대위치 정확도를 높여야 함 | GDOP 계열 loss와 DRL로 상대 로봇의 위치오차가 작아지도록 능동 위치 조절 | UWB relative localization 중심, SLAM map 보정은 핵심 아님 | `UWB 측정 기하를 위한 능동 기동`이 이미 있음 |

### 7.4 상대측정의 기하·관측가능성 및 능동 경로 계획

| 논문 | 문제 | 핵심 아이디어 | 우리와의 관계 |
|---|---|---|---|
| [Ranging-Based Localizability Optimization, 2022](https://arxiv.org/abs/2202.00756) | range network의 위치 정확도가 로봇 배치 기하에 크게 의존 | CRLB와 graph rigidity 기반 localizability potential로 trajectory/deployment 계획 | 단순 거리측정도 배치가 나쁘면 도움이 안 된다는 근거 |
| [Multi-Robot Relative Pose Estimation in SE(2), 2024](https://arxiv.org/abs/2401.15313) | range-only/bearing-only/odometry 공유 조건별 상대 pose 관측가능성 | sensing·communication 구조별 EKF/PGO와 observability 비교 | 2D 상대측정 설계의 직접 참고문헌 |
| [Multi-Robot Collaborative Localization and Planning with Inter-Ranging, 2024](https://arxiv.org/abs/2406.16679) | 저텍스처·저조도에서 feature tracking 오차 증가 | UWB inter-ranging의 line-of-sight vector를 이용해 집단 localization error가 작아지는 경로를 분산 계획 | `정보량에 따른 능동 협력 경로`가 이미 있음 |
| [A Closed-Form 4-DoF Inter-Robot Pose Estimator, 2026](https://arxiv.org/abs/2606.26616) | bearing+odometry 상대 pose가 특정 motion에서 퇴화 | collinear 및 shape-preserving formation의 퇴화를 이론 분석하고 observability가 충분할 때 추정 실행 | 평행 동일속도 기동의 관측 한계와 직접 연관 |

### 7.5 현재 아이디어와 가장 가까운 연구

| 논문 | 기존 문제 | 해결 아이디어 | 현재 프로젝트와 같은 점 | 중요한 차이 |
|---|---|---|---|---|
| [Degradation-Aware Cooperative Multi-Modal GNSS-Denied Localization, 2025](https://arxiv.org/abs/2510.20480) | 서로 다른 로봇에 분산된 VIO/LIO/상대검출을 비동기적으로 융합하면서 각 센서 퇴화 대응 | scan-matching Hessian으로 LIO degradation을 평가하고 VIO, LIO, inter-robot 3D detection을 factor graph에서 adaptive fusion | `퇴화 감지 + 다른 로봇 관측 + factor graph` 조합이 매우 가까움 | 이종 UGV-UAV, 다중모달, 능동 rendezvous가 핵심은 아님 |
| [MR-FLOUR, 2025](https://www.sciencedirect.com/science/article/pii/S0957415825001199) | 사전 인프라 없이 다중 로봇 상대위치 추정 | LiDAR robot detection, odometry, UWB ranging, orientation rejection, PGO 융합 | LiDAR+UWB+odometry 상대 제약과 긴 복도 실험 | 지도의 LiDAR 취약 방향과 능동 기동은 핵심 아님 |
| [Leap-SLAM, 2026](https://doi.org/10.1016/j.inffus.2026.104538) | 특징 부족 터널에서 LiDAR-inertial SLAM의 종방향 drift | 정보행렬 degeneracy index, reflective marker를 가진 Beacon Robot, Master/Beacon leapfrogging, high-localizability residual | `터널 퇴화 검출 + 다른 로봇의 능동 외부 제약 + mapping`이 거의 일치 | 지상 Master/Beacon 역할, reflective marker, leapfrogging, 3D LiDAR-inertial. 현재 프로젝트는 대칭 2-UAV·2D LiDAR·평행 lane·무복귀를 목표 |

Leap-SLAM은 2026년 온라인 공개되어 Information Fusion Volume 136, 2026년 12월호로 표시된 연구다. 1,000 m 이상 특징 부족 터널에서 drift를 억제했으며, 현재 시점에서 가장 직접적인 비교 대상이다.

---

## 8. 지금 연구가 어디까지 진행되었는가?

2026년 현재 관련 연구는 다음 단계까지 진행되어 있다.

1. **퇴화 검출**
   - 정보행렬, Hessian, Jacobian, correspondence contribution으로 약제약 방향을 실시간 분석할 수 있다.
2. **퇴화된 방향만 선택적으로 처리**
   - 잘 관측된 상태 방향만 ICP/PGO에 사용하거나, 취약한 차원에만 다른 센서 제약을 넣는 연구가 있다.
3. **상대측정 관측가능성 분석**
   - range-only, bearing-only, odometry 공유 여부, 로봇 formation과 motion excitation에 따른 관측 조건이 연구되어 있다.
4. **능동 협력 위치·경로 계획**
   - Fisher information, CRLB, GDOP, uncertainty를 이용해 다른 로봇의 위치와 경로를 조절할 수 있다.
5. **필요할 때 상호관측·랑데부**
   - trajectory error 또는 filter uncertainty가 커질 때 다른 로봇을 호출하거나 카메라 방향을 바꾸는 시스템이 이미 있다.
6. **특징 부족 터널의 다중 로봇 퇴화 보완**
   - Leap-SLAM은 퇴화를 검출하고 이동식 Beacon Robot의 고관측성 특징을 이용해 실제 장거리 터널 mapping을 수행했다.

따라서 현재 연구의 질문은 더 이상 다음처럼 단순하지 않다.

> 다른 드론이 도와주면 drift를 줄일 수 있는가?

대신 다음 수준으로 구체화해야 한다.

> 주어진 임무·차선·센서 제약에서 어떤 상대측정이 LiDAR의 어느 취약 상태 방향을 실제로 관측 가능하게 만들며, 그 정보 이득을 얻기 위한 최소 기동과 비용은 얼마인가?

---

## 9. 신규성에 대한 정직한 판정

### 9.1 이미 선행연구가 있어 신규 주장으로 사용하기 어려운 항목

| 주장 후보 | 판정 | 이유 |
|---|---|---|
| LiDAR 정보행렬로 취약 방향을 찾는다 | 신규성 없음 | Degeneracy-Aware Factors, X-ICP, DALI-SLAM 등 |
| 오차가 threshold를 넘으면 랑데부한다 | 신규성 없음 | Active Rendezvous 2019 |
| uncertainty가 커질 때 드론끼리 관측한다 | 신규성 없음 | Active Mutual Observation 2024 |
| 상대측정에 유리하게 로봇 위치를 바꾼다 | 신규성 없음 | CRLB/Fisher/GDOP 기반 active localization 연구 |
| 취약한 차원에만 보조 제약을 넣는다 | 단독 신규성 없음 | Fuse Only What Matters 2026 |
| 터널에서 다른 로봇을 이동식 기준점으로 쓴다 | 매우 가까운 선행연구 있음 | Leap-SLAM 2026 |
| map과 odometry를 공유해 지도를 합친다 | 신규성 약함 | 다수 C-SLAM 및 map fusion 연구 |

### 9.2 현재 남아 있는 후보 공백

다음 조합은 현재 검토한 문헌에서 정확히 일치하는 연구를 찾지 못했지만, 아직 신규성 확정은 아니다.

1. 두 로봇 모두 임무와 mapping을 계속하는 **대칭 2-UAV 구조**
2. 전용 Beacon Robot, reflective marker, 고정 anchor가 없음
3. 2D LiDAR를 사용하는 두 UAV가 서로 겹치지 않는 평행 lane을 진행
4. 출발점 복귀, 역주행, lane 교차가 금지됨
5. LiDAR의 순간 취약 고유벡터와 후보 inter-UAV factor의 방향별 information gain을 결합
6. 보정 가능할 때만 최소 차선 내 속도·hover·측면·arc 기동을 선택
7. 상대측정으로 제거할 수 없는 common-mode/gauge 방향을 명시적으로 판별
8. 최종 효과를 trajectory뿐 아니라 occupancy map의 double-wall, ghost, IoU로 평가

하지만 `UAV`, `2D LiDAR`, `평행 lane`처럼 플랫폼과 시나리오만 다른 것은 강한 알고리즘 신규성이 아니다. 다음 중 최소 하나가 필요하다.

- 새로운 결합 관측가능성 정식화 또는 이론 결과
- 기존 기준보다 명확히 나은 event-trigger 또는 maneuver optimizer
- 복구 가능·불가능 방향을 판정하는 증명 가능한 조건
- 공개 가능한 benchmark/dataset과 재현성 높은 비교평가
- 계산량·통신량·임무시간을 동시에 제한하는 새로운 최적화 문제

### 9.3 확실히 선행논문이 없다고 말할 수 있는가?

**말할 수 없다.** 웹과 공개 preprint 검색으로 `없음`을 증명할 수는 없다. 정확한 표현은 다음과 같다.

> 2026-08-26까지 확인한 공개 문헌에서는 제안한 좁은 제약 조합과 직접 일치하는 연구를 발견하지 못했다. 그러나 인접한 핵심 구성 요소와 매우 가까운 선행연구가 다수 존재하므로, 최종 신규성은 체계적 데이터베이스 검색과 claim-level 비교 후 확정해야 한다.

논문 제안서나 발표에서 `최초`, `선행연구 없음`, `완전히 새로운 방식`이라는 표현은 추가 검토 전 사용하지 않는다.

---

## 10. 추천 연구문제와 가설

### RQ1. 취약 방향 검출

> 2D LiDAR scan matching의 정보행렬은 현재 직선·반복 구조에서 실제로 어떤 \(x,y,yaw\) 결합 방향이 취약한지 안정적으로 검출할 수 있는가?

가설:

- 진행축 성분이 큰 고유벡터에서 작은 고유값이 반복적으로 나타난다.
- 반복 원통은 단순 퇴화뿐 아니라 잘못된 local optimum을 만들기 때문에 별도의 aliasing gate가 필요하다.

### RQ2. 상대측정의 방향별 보완 능력

> UWB range, range+bearing 또는 상대 drone detection은 LiDAR의 취약 고유방향에 얼마나 많은 정보를 추가하는가?

가설:

- 같은 속도의 평행 비행에서 range-only의 진행축 information gain은 작다.
- range+bearing 또는 상대 longitudinal offset을 만드는 기동은 취약 방향의 최소 고유값을 증가시킨다.

### RQ3. 최소 협력 기동

> 역주행과 lane 교차 없이 필요한 information gain을 얻는 최소 기동은 무엇인가?

후보 action:

- 한 드론의 짧은 hover
- 두 드론의 속도 차이
- 안전 범위 내 측면 offset
- 짧은 S 또는 arc maneuver
- yaw-only mutual observation

### RQ4. event-trigger와 비용

> 언제 협력 기동을 요청해야 정확도 개선과 임무시간·에너지 비용의 균형이 가장 좋은가?

단순 covariance threshold가 아니라 다음 score를 후보로 한다.

\[
S(a)=\underbrace{\Delta I_{weak}(a)}_{취약방향\ 정보이득}
-\lambda_t\underbrace{C_{time}(a)}_{임무시간}
-\lambda_e\underbrace{C_{energy}(a)}_{에너지}
-\lambda_r\underbrace{C_{risk}(a)}_{충돌위험}
\]

### RQ5. 복구 불가능한 오차

> 상대측정만으로 제거할 수 없는 common-mode drift와 gauge freedom은 무엇이며, 시스템은 이를 어떻게 감지하고 `보정 불가`로 보고해야 하는가?

이 질문은 과장된 성능 주장을 막고, 언제 절대 anchor가 필요한지 결정하는 데 중요하다.

---

## 11. 제안 시스템 초안

```text
UAV 1: 2D LiDAR scan matching ── H_L1, weak eigenvector ─┐
                                                         │
UAV 2: 2D LiDAR scan matching ── H_L2, weak eigenvector ─┤
                                                         ↓
                                              observability monitor
                                                         │
inter-UAV sensor ─ candidate range/bearing information ──┤
                                                         ↓
                                      expected weak-direction gain
                                                         │
                   gain sufficient ──────────────────────┤
                   gain insufficient + action exists ───┤
                                                         ↓
                                            minimal maneuver selector
                                                         ↓
                         verified inter-UAV relative factor
                                                         ↓
                                   robust joint SE(2) pose graph
                                                         ↓
                                  optimized submap poses/revision
                                                         ↓
                                      global map re-projection
```

### 11.1 제약 생성 원칙

- 상대측정 한 번만으로 global map을 즉시 갱신하지 않는다.
- timestamp, covariance, 연속 일관성, residual, 기하 observability를 함께 확인한다.
- 반복 구조의 false correspondence는 robust loss만 믿지 않고 사전 gate로 제거한다.
- 잘 관측되지 않는 상태 차원에 과도한 confidence를 주지 않는다.
- 정보 이득이 없는 랑데부는 수행하지 않는다.

### 11.2 지도 보정 단위

137 m의 전체 OccupancyGrid에 rigid transform 하나만 적용해서는 구간별 누적 drift를 펼 수 없다. 따라서 최종 구조는 다음 단위를 가져야 한다.

- UAV별 keyframe pose
- frozen local submap
- intra-UAV odometry edge
- inter-UAV relative/place edge
- optimized submap pose
- pose-graph revision에 따른 global OccupancyGrid 재투영

---

## 12. 실험 설계

### 12.1 비교군

| ID | 조건 | 목적 |
|---|---|---|
| B0 | UAV별 local-only SLAM | drift 기준선 |
| B1 | map 공유 + fixed spawn transform | 현재 구현 기준선 |
| B2 | map/odom 통신만, inter-UAV 측정 없음 | 통신 자체가 drift를 줄이지 않음을 확인 |
| B3 | passive 평행 직진 + range-only | 나쁜 관측 기하의 한계 확인 |
| B4 | passive 평행 직진 + range+bearing | 센서 종류에 따른 차이 |
| B5 | scalar uncertainty threshold rendezvous | 기존 단순 event-trigger 비교군 |
| B6 | 고정 anchor 지점 rendezvous | 사전 계획 방식 비교군 |
| B7 | 제안 방향별 information-gain trigger + 최소 기동 | 제안 방식 |
| B8 | 한 드론이 실제로 복귀하는 self loop | 전통 loop closure 상한 비교 |
| B9 | 전용 Beacon/leapfrog 방식 | Leap-SLAM 개념과 비교 가능한 기준선 |

### 12.2 환경 변수

- 복도 길이: 짧음/중간/137 m 이상
- 반복 원통 간격과 seed
- lane 간격
- LiDAR range와 noise
- UWB range/bearing noise와 NLOS outlier
- 두 드론의 속도 차이
- 통신 delay/dropout
- 상대측정 가능 시간 창
- 기동 허용 폭과 최대 추가 시간

### 12.3 필수 지표

| 범주 | 지표 |
|---|---|
| 궤적 | ATE, RPE, endpoint drift, yaw error |
| 방향별 추정 | \(H_L\) eigenvalues, condition number, weak eigenvector stability |
| 정보 이득 | 기동 전후 최소 고유값, \(v_{min}^{T}H_Rv_{min}\), covariance reduction |
| 지도 | occupied precision/recall, IoU, Chamfer distance, double-wall/ghost ratio |
| 제약 품질 | accepted/rejected constraint, precision/recall, residual, outlier count |
| 시스템 | optimizer latency, DDS bytes, p95 delay, dropout, stale duration |
| 임무 비용 | 추가 시간, 이동거리, energy proxy, mission success rate |
| 안전 | minimum inter-UAV distance, maneuver abort, collision count |

각 조건은 같은 seed 집합으로 최소 10회 이상 반복하고, 평균뿐 아니라 분산과 실패율을 함께 보고한다.

### 12.4 반드시 확인할 ablation

1. eigenvalue threshold만 사용 vs eigenvector 방향 정보까지 사용
2. scalar covariance trigger vs weak-direction gain trigger
3. range-only vs range+bearing
4. 기동 없음 vs hover vs 속도 차이 vs 측면/arc
5. 전체 상태에 peer factor 적용 vs 관측 가능한 subspace에만 적용
6. 전체 grid rigid fusion vs optimized submap re-projection
7. robust loss만 사용 vs aliasing 사전 gate 추가

---

## 13. 구현 로드맵

### 단계 0. 현재 기준선 고정

- known-pose fusion과 raw `slam_toolbox`를 동일 run에서 기록
- 137 m 조건을 여러 seed로 반복
- 현재 failure를 trajectory와 map 지표로 고정

완료 기준: 통신/fusion 성공과 SLAM 정렬 실패를 분리해 재현할 수 있다.

### 단계 1. 2D LiDAR 취약 방향 계측기

- scan matching Jacobian 또는 대응점에서 \(H_L\) 계산
- eigenvalue/eigenvector/condition number topic과 CSV 기록
- 직선벽, 원통 반복, corner 환경에서 예상 방향과 비교

완료 기준: simulator ground truth를 사용하지 않고 퇴화 시점과 방향을 검출한다.

### 단계 2. synthetic relative sensor와 관측가능성 분석

- Gazebo truth로 noise/dropout이 포함된 range 및 bearing 측정만 생성
- truth transform을 optimizer에 직접 주입하지 않음
- \(H_R\), 결합 \(H_{total}\), weak-direction gain 계산

완료 기준: 어떤 formation과 motion이 진행축 정보를 추가하는지 정량 확인한다.

### 단계 3. joint SE(2) pose graph와 submap 재투영

- odometry/keyframe edge
- verified inter-UAV factor
- robust loss와 aliasing gate
- optimized submap pose와 revision 발행

완료 기준: 구간별 drift가 있는 합성·실제 run에서 rigid fusion보다 지도 품질이 개선된다.

### 단계 4. 최소 차선 내 기동

- action 후보별 predicted information gain과 비용 계산
- 안전 제약을 만족하는 action만 실행
- optimizer revision 이후 임무 재개

완료 기준: fixed anchor와 scalar uncertainty rendezvous보다 적은 추가 시간으로 같거나 나은 보정 성능을 얻는다.

### 단계 5. 실제 상대센서 front-end

- UWB range+bearing, tag, 비전 또는 LiDAR robot detection 중 하나 선택
- synthetic adapter와 같은 message contract 유지
- NLOS, false detection, 비동기 timestamp 실험

완료 기준: ground-truth-derived 상대측정 없이 주요 결과를 재현한다.

---

## 14. 논문에서 주장할 수 있는 것과 없는 것

### 14.1 구현 전부터 주장하면 안 되는 것

- 세계 최초의 능동 랑데부 SLAM
- 최초의 degeneracy-aware multi-robot SLAM
- 다른 드론과 만나면 모든 누적 오차 제거
- 통신만으로 drift 감소
- 2D LiDAR 직선 복도에서 yaw는 항상 퇴화
- 두 UAV 상대측정으로 global absolute drift 완전 복구

### 14.2 검증 후 가능한 주장 형태

다음은 조건을 정확히 붙일 때 가능한 후보 문장이다.

> 비중첩 평행 lane과 무복귀 제약 아래, 로컬 2D LiDAR의 약관측 고유방향과 후보 inter-UAV 측정의 방향별 정보 이득을 결합하여, 정보가 실제로 증가하는 경우에만 최소 차선 내 기동을 선택하는 방법을 제안한다.

> 제안 방법은 scalar uncertainty threshold 및 고정 rendezvous보다 적은 임무시간 증가로 진행축 drift와 map double-wall을 감소시켰다.

> 또한 pairwise relative measurement만으로 제거할 수 없는 common-mode 방향을 판별하여 불필요하거나 잘못된 보정을 억제한다.

이 문장들은 구현과 통계적 실험 결과가 확보된 뒤에만 최종 주장으로 사용한다.

---

## 15. 프로젝트형 연구와 논문형 연구의 선택

### 15.1 졸업작품·캡스톤 중심이면

다음 주제가 현실적이다.

> **저특징 직선 환경에서 복귀 없는 2-UAV 협력 보정 방법의 설계 및 비교평가**

기여:

- ROS 2/PX4/Gazebo 통합 시스템
- local-only, fixed map fusion, passive relative constraint, rendezvous, proposed maneuver 비교
- trajectory/map/통신/임무시간 정량평가
- 각 방법이 성공하는 조건과 실패하는 조건 정리

이 경우 새로운 수학 알고리즘을 과장하기보다 재현 가능한 시스템과 비교실험이 핵심 기여다.

### 15.2 학술논문 신규성이 목표이면

다음 주제로 더 좁힌다.

> **저특징 평행 복도에서 2D LiDAR와 inter-UAV 상대측정의 결합 관측가능성 및 임무비용 제약 최소 협력 기동**

필요 기여:

1. 결합 정보행렬 또는 factor-graph 관측가능성의 명확한 정식화
2. 상대측정이 LiDAR 약관측 subspace를 실제로 보완하는 조건
3. common-mode/gauge direction과 복구 가능 방향의 분리
4. lane·무복귀·안전·시간 제약을 포함한 최소 기동 선택
5. 최신 근접 연구, 특히 Leap-SLAM과의 직접 비교 또는 명확한 차이 증명

---

## 16. 추천 논문 구성

1. **Introduction**
   - 복귀 없는 직선 mapping의 필요성
   - 단순 map sharing과 cooperative correction의 차이
   - 기존 active rendezvous와 Leap-SLAM 이후에도 남는 제한 조건
2. **Related Work**
   - degeneracy-aware LiDAR registration
   - C-SLAM/inter-robot loop closure
   - active collaborative localization/rendezvous
   - relative measurement observability
3. **Problem Formulation**
   - 2-UAV SE(2) state
   - local LiDAR factor와 inter-UAV factor
   - lane/no-return/safety constraints
4. **Joint Observability Analysis**
   - local weak direction
   - peer measurement directional gain
   - common-mode unobservable subspace
5. **Minimal Cooperative Maneuver**
   - trigger, action set, cost, safety gate
6. **Cooperative Pose Graph and Map Reconstruction**
   - robust factor insertion
   - submap re-projection
7. **Experiments**
   - simulation, sensor ablation, baselines, repeated seeds
8. **Limitations**
   - absolute anchor 필요 조건
   - sensor NLOS/FoV/communication assumptions
9. **Conclusion**

---

## 17. 문헌 검색 범위와 후속 검색어

이 문서는 2026-08-26까지 공개된 논문 페이지, DOI metadata, arXiv와 프로젝트 문서를 중심으로 정리했다. `선행연구 없음`을 확정하기 위한 systematic review는 아니다.

후속 데이터베이스 검색 시 다음 Boolean query를 조합한다.

```text
(LiDAR degeneracy OR weakly constrained direction OR localizability)
AND (multi-robot OR multi-UAV OR cooperative localization)
AND (active rendezvous OR active observation OR motion planning)

(corridor OR tunnel OR feature-poor OR repetitive environment)
AND (inter-robot measurement OR UWB OR bearing OR relative pose)
AND (observability OR Fisher information OR information gain)

(parallel trajectory OR non-overlapping path OR no-return mission)
AND (cooperative SLAM OR relative localization)
```

반드시 확인할 경로:

- IEEE Xplore
- Scopus
- Web of Science
- Google Scholar의 cited-by/related articles
- 각 핵심 논문의 reference와 후속 인용
- arXiv 최신 preprint
- Google Patents, WIPO Patentscope

신규성 판단은 제목이나 keyword가 아니라 **논문의 claim, state, sensor, constraint, trigger, action, estimator, evaluation을 열 단위로 비교**해야 한다.

---

## 18. 핵심 참고문헌 목록

### Degeneracy와 localizability

1. Hinduja, A., Ho, B.-J., Kaess, M., “Degeneracy-Aware Factors with Applications to Underwater SLAM,” IROS, 2019. [Paper](https://www.cs.cmu.edu/~kaess/pub/Hinduja19iros.pdf)
2. Ebadi, K. et al., “DARE-SLAM: Degeneracy-Aware and Resilient Loop Closing in Perceptually-Degraded Environments,” 2021. [arXiv](https://arxiv.org/abs/2102.05117)
3. Tuna, T. et al., “X-ICP: Localizability-Aware LiDAR Registration for Robust Localization in Extreme Environments,” IEEE Transactions on Robotics, 2024. [DOI](https://doi.org/10.1109/TRO.2023.3335691)
4. Wu, W. et al., “DALI-SLAM: Degeneracy-aware LiDAR-inertial SLAM with Novel Distortion Correction and Accurate Multi-constraint Pose Graph Optimization,” ISPRS JPRS, 2025. [DOI](https://doi.org/10.1016/j.isprsjprs.2025.01.036)
5. “Fuse Only What Matters: Degeneracy-aware Multi-sensor Fusion for LiDAR-Inertial-Visual SLAM,” ISPRS JPRS, 2026. [DOI](https://doi.org/10.1016/j.isprsjprs.2026.05.031)

### Multi-robot SLAM과 map 연결

6. Lajoie, P.-Y. et al., “DOOR-SLAM: Distributed, Online, and Outlier Resilient SLAM for Robotic Teams,” 2019. [arXiv](https://arxiv.org/abs/1909.12198)
7. Tian, Y. et al., “Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems,” 2021/2022. [arXiv](https://arxiv.org/abs/2106.14386)
8. Lajoie, P.-Y., Beltrame, G., “Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems,” 2023. [arXiv](https://arxiv.org/abs/2301.06230)

### Active rendezvous와 collaborative localization

9. Wang, W. et al., “Active Rendezvous for Multi-Robot Pose Graph Optimization using Sensing over Wi-Fi,” ISRR, 2019. [arXiv](https://arxiv.org/abs/1907.05538)
10. Spasojevic, I. et al., “Active Collaborative Localization in Heterogeneous Robot Teams,” 2023. [arXiv](https://arxiv.org/abs/2305.18193)
11. Guo, L. et al., “Preserving Relative Localization of FoV-Limited Drone Swarm via Active Mutual Observation,” IROS, 2024. [arXiv](https://arxiv.org/abs/2407.01292)
12. Brunacci, V. et al., “Infrastructure-less UWB-based Active Relative Localization,” 2024. [arXiv](https://arxiv.org/abs/2409.12780)

### 상대측정 관측가능성과 능동 경로

13. Cano, J., Le Ny, J., “Ranging-Based Localizability Optimization for Mobile Robotic Networks,” 2022. [arXiv](https://arxiv.org/abs/2202.00756)
14. “Multi-Robot Relative Pose Estimation in SE(2) with Observability Analysis,” 2024. [arXiv](https://arxiv.org/abs/2401.15313)
15. Knowles, D., Dai, A., Gao, G., “Multi-Robot Collaborative Localization and Planning with Inter-Ranging,” 2024. [arXiv](https://arxiv.org/abs/2406.16679)
16. De, Q. et al., “A Closed-Form 4-DoF Inter-Robot Pose Estimator using Bearing-only Measurements,” 2026. [arXiv](https://arxiv.org/abs/2606.26616)

### 가장 가까운 최신 연구

17. Pritzl, V. et al., “Degradation-Aware Cooperative Multi-Modal GNSS-Denied Localization Leveraging LiDAR-Based Robot Detections,” 2025. [arXiv](https://arxiv.org/abs/2510.20480)
18. Shalihan, M. et al., “MR-FLOUR: Multi-robot Relative Localization Based on the Fusion of LiDAR, Odometry, and UWB Ranging,” 2025. [Article](https://www.sciencedirect.com/science/article/pii/S0957415825001199)
19. Chen, S. et al., “Leap-SLAM: Degeneracy-Mitigated Robust SLAM with Leapfrogging Multi-Robot Collaboration in Tunnels,” Information Fusion, 2026. [DOI](https://doi.org/10.1016/j.inffus.2026.104538)

---

## 19. 최종 의사결정 문장

현재 프로젝트의 연구 방향은 다음처럼 정리한다.

> 기존의 `필요할 때 랑데부하여 오차를 보정하는 Multi-UAV SLAM`은 이미 유사 선행연구가 있으므로 그 자체를 신규성으로 주장하지 않는다. 대신 저특징 평행 비중첩 경로와 무복귀 임무에서, 각 UAV의 2D LiDAR 약관측 방향과 inter-UAV 상대측정의 방향별 정보 이득을 공동 분석하고, 복구 가능한 경우에만 최소 차선 내 협력 기동을 수행하는 문제를 연구한다. 또한 상대측정만으로 복구할 수 없는 common-mode 방향을 명시하고, trajectory뿐 아니라 occupancy map 품질과 임무비용까지 비교평가한다.

이 문장을 이후 연구계획서, 발표자료, 구현 roadmap의 기준으로 사용하되, 최종 논문 신규성은 체계적 문헌·특허 검색 후 확정한다.
