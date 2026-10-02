# 복귀 없는 2-UAV 임무에서 시간차 공통 관측 submap 공유 기반 드리프트 보정: 선행연구 및 공백 분석

> 기준일: 2026-09-23
> 대상 프로젝트: AV_Drone, 2-UAV, 2D LiDAR, ROS 2/PX4/Gazebo Classic
> 선행 문서: [저특징 직선 환경 Multi-UAV 오차 보정 연구 주제 및 선행연구 종합 정리](research-topic-and-prior-art-summary.md) (2026-08-26, 랑데부/LiDAR 퇴화 중심)
> 근거 실험: [공통 통로 2구간 재실험](two_region_shared_corridor_20260912.md), [전체 맵 2단계 실험](full_map_stage2_experiment_20260911.md)

> **주의**: 이 문서의 논문 설명은 대부분 초록·검색 결과·프로젝트 페이지를 기준으로 정리했다.
> 전문을 읽지 않은 항목은 표에 `초록` 으로 표시했다. 특히 §4의 "가장 위협적인 선행연구" 3편은
> 신규성 주장 전에 반드시 전문을 읽고 claim 단위로 비교해야 한다. 웹 검색으로 "선행연구 없음"을 증명할 수는 없다.

---

## 1. 확정된 연구 흐름

1. **문제**: 기존 SLAM 오차 보정(loop closure)은 로봇이 과거 장소로 돌아와야 한다. 목표 지점까지 한 방향으로만 가는 임무에서는 자기 loop closure가 불가능하고 scan-matching SLAM drift가 누적된다.
2. **아이디어**: 2대의 드론이 같은 장소를 **서로 다른 시각에** 지나가도록 하고, 통신으로 LiDAR submap을 공유해 드론 간(inter-robot) 상대 pose 제약을 만들면, 복귀 없이도 오차를 줄일 수 있다.
3. **결과 (run `20260911T161139Z_9abd773a46f1`)**: GT 없이 공통 구간 A/B를 찾아 오프라인 pose graph로 보정.

| 조건 | 제약 수 | 종단 상대 위치 [m] | 상대 위치 RMSE [m] | 지도 Chamfer [m] |
|---|---:|---:|---:|---:|
| B0 raw SLAM | 0 | 6.538 | 2.963 | 1.493 |
| 앞 구간 A만 | 1 | 1.537 | 0.444 | 0.496 |
| 뒤 구간 B만 | 1 | **0.281** | 0.203 | 1.063 |
| A+B | 2 | 0.332 | **0.179** | **0.442** |

→ **구간의 위치와 개수에 따라 개선되는 지표가 다르다**는 관찰이 공백 논의의 핵심 단서다.

---

## 2. 선행연구 지도 (3개 축)

### 2.1 축 1 — Collaborative SLAM 시스템 (submap/descriptor 공유 + inter-robot loop closure)

| 시스템 | 연도·venue | 센서/로봇 | 공유 데이터 | inter-robot 제약·outlier 제거 | 겹침 발생 방식 | 링크 |
|---|---|---|---|---|---|---|
| DOOR-SLAM | 2020 RA-L | stereo, 지상+공중 | NetVLAD descriptor→feature | 분산 PCM | 우연 | https://arxiv.org/abs/1909.12198 |
| Kimera-Multi | 2021 ICRA / 2022 T-RO | VIO, 지상 | BoW→keyframe | 분산 GNC | 우연(탐사, revisit 포함) | https://arxiv.org/abs/2106.14386 |
| DiSCo-SLAM | 2022 RA-L | 3D LiDAR | Scan Context→keyframe | 2단계 최적화, 초기 pose 미지 | 우연 | https://github.com/RobustFieldAutonomyLab/DiSCo-SLAM |
| DCL-SLAM | 2024 IEEE Sensors J. | 3D LiDAR swarm | LiDAR-Iris→scan | 분산 PCM | 우연 | https://arxiv.org/abs/2210.11978 |
| Swarm-SLAM | 2024 RA-L | LiDAR/stereo/RGB-D | Scan Context/CosPlace→선택 후보만 | algebraic connectivity 예산 우선순위 + GNC | KITTI를 **시간 분할**해 로봇처럼 사용(우연적 시간차 겹침) | https://arxiv.org/abs/2301.06230 |
| LAMP 2.0 / Loop closure prioritization | 2022 RA-L | 3D LiDAR, SubT | keyed scan→기지국 | graph 정보량·관측성 기반 우선순위, GNC | 탐사 중 우연 | https://arxiv.org/abs/2205.13135 , https://arxiv.org/abs/2205.12402 |
| CCM-SLAM / COVINS | 2019 JFR / 2021 3DV | mono/VIO, UAV | keyframe | BoW+기하 검증, 서버 BA | 우연 | https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21854 , https://arxiv.org/abs/2108.05756 |
| D²SLAM | 2024 T-RO | VIO, UAV swarm | descriptor+keyframe | 분산 PGO | 함께 비행 | https://arxiv.org/abs/2211.01538 |
| Omni-swarm / Swarm-LIO2 / CoLRIO | 2022–2025 | VIO+UWB / LIO / LiDAR+UWB | 상태·상호관측·range | **상호관측·UWB** (장소 겹침 불필요) | **동시 근접** 필요 | https://arxiv.org/abs/2103.04131 , https://arxiv.org/abs/2409.17798 , https://arxiv.org/abs/2402.11790 |
| AutoMerge | 2023 T-RO | 3D LiDAR, 오프라인 | map segment 전체 | aliasing-robust 병합 | 시간 간격 큰 revisit(우연) | https://arxiv.org/abs/2207.06965 |
| Multi-robot LiDAR SLAM in tunnels | 2025 arXiv | LiDAR, 터널 | – | **반복 구조에서 loop false positive가 주 실패원** | 터널 | https://arxiv.org/abs/2507.21553 |
| Decentralized LiDAR-SLAM w/ certifiable PGO | 2026 ICRA WS | LiDAR | – | RBCD, DiSCo 대비 RMSE 48.9%↓ (초록) | – | https://arxiv.org/abs/2605.25051 |

**판정**: descriptor/submap 공유 → 기하 정합 → PCM/GNC → joint PGO 파이프라인은 **표준**이다. 본 프로젝트의 정합 gate + 이웃 일관성 검사는 PCM의 비공식 변형에 해당한다. **파이프라인 자체로는 신규성 주장 불가.**

### 2.2 축 2 — 겹침/loop closure 계획, 배치 이론, 무복귀 임무

| 논문 | 연도·venue | 핵심 | 본 연구와의 거리 | 링크 |
|---|---|---|---|---|
| Bai et al., Collaborative Graph Exploration w/ Reduced Pose-SLAM Uncertainty | 2024 IROS | 그래프 환경에서 loop edge를 경로에 추가, submodular 최적화 | **계획 측면 최근접**. 단 우회/revisit 허용, 그래프 탐사 (초록) | https://arxiv.org/abs/2407.01013 |
| Li et al., Distributed Multi-UGV Exploration w/ Loop-Aware Planning | 2026 IEEE TIE | cross-UGV loop 후보를 D-optimality로 평가해 task allocation/TSP에 반영, 시간차 교차 | **매우 가까움**. 단 탐사 문제, 고정 편도 경로 아님 (초록) | https://arxiv.org/abs/2606.11088 |
| Ding et al., Lightweight LiDAR Cooperative Localization for Leader-Follower | 2026 IEEE T-ITS | **"asynchronous view repetition"**: follower가 leader가 본 장면을 시간 지연 후 관측, joint graph | **시간차 겹침 개념 최근접**. 단 전 구간 연속 겹침(플래투닝), 구간 수·위치 설계 없음 (초록) | https://doi.org/10.1109/TITS.2025.3624568 |
| Krawciw et al., Sharing the Load (multi-rover teach&repeat) | 2025 arXiv | leader submap을 follower에 전송해 localization | joint drift 보정 아님 | https://arxiv.org/abs/2510.18766 |
| Kim & Eustice, Active visual SLAM for area coverage | 2015 IJRR | 탐사 vs revisit trade-off (단일 로봇) | revisit 필요 | https://journals.sagepub.com/doi/10.1177/0278364914547893 |
| Gao et al., Active Loop Closure for OSM-guided Mapping | 2024 arXiv | 불확실성 증가 시 revisit 재계획 | revisit 필요 | https://arxiv.org/abs/2407.17078 |
| Khosoussi et al., Designing Sparse Reliable Pose-Graph SLAM / Reliable Graphs for SLAM | 2016 WAFR / 2019 IJRR | weighted spanning tree 수 ↔ D-optimality, greedy edge 선택 보장 | **배치 이론의 기반** | https://arxiv.org/abs/1611.00889 , https://www.mit.edu/~mrrobot/assets/khosoussi19ijrr.pdf |
| Chen et al., Cramér–Rao Bounds and Optimal Design Metrics for Pose-Graph SLAM | 2021 T-RO | 가중 Laplacian 기반 FIM/CRLB, T/D-optimality | 구간 위치 효과를 해석적으로 예측하는 도구 | https://opus.lib.uts.edu.au/bitstream/10453/147657/3/Cram%C3%A9r%E2%80%93Rao%20Bounds.pdf |
| Placed & Castellanos, Optimality Criteria ↔ Connectivity Indices | 2023 RA-L | FIM 최적성 기준과 Laplacian 연결성 지표 관계 | 후보 점수화 근거 | https://arxiv.org/abs/2110.01289 |
| MAC: Graph Sparsification by Maximizing Algebraic Connectivity | 2024 arXiv | algebraic connectivity가 최악 오차 지배 | 선택 이론 | https://arxiv.org/abs/2403.19879 |
| Kim et al., Multiple Relative Pose Graphs for Robust Cooperative Mapping | 2010 ICRA | anchor node로 비동기 만남/공통 장소 결합 | **joint graph 표준 정식화** | https://publications.ri.cmu.edu/storage/publications/pub_files/2010/5/Kim10icra.pdf |
| Towards Optimal Beacon Placement for Range-Aided Localization | 2024 arXiv | 알려진 경로 위 beacon 위치 D-optimal 설계 | **"편도 경로 위 K개 구간 배치" 방법론 템플릿** | https://arxiv.org/abs/2405.11550 |
| da Silva et al., Intermittent Rendezvous exploration | 2023 arXiv | 통신용 랑데부 스케줄링 | 동시 만남, 통신 목적 | https://arxiv.org/abs/2309.13494 |

(참고 survey: C-SLAM https://arxiv.org/abs/2108.08325 , Active SLAM https://arxiv.org/abs/2207.00254)

### 2.3 축 3 — 통신·자원 제약, 반복 환경 장소인식

| 논문 | 연도·venue | 핵심 | 링크 |
|---|---|---|---|
| Giamou et al., Talk Resource-Efficiently to Me | 2018 ICRA | 후보 검증을 위한 최소 데이터 교환 계획 | https://arxiv.org/abs/1709.06675 |
| Tian et al., Budgeted Data Exchange / Resource-Aware C-LCD | 2018 RSS / IJRR | 통신 예산 하 submodular 후보 선택 | https://arxiv.org/abs/1806.00188 , https://arxiv.org/abs/1907.04904 |
| Cieslewski et al., Data-Efficient Decentralized Visual SLAM | 2018 ICRA | descriptor 먼저, 일치 시 전체 데이터 | https://arxiv.org/abs/1710.05772 |
| DRACo-SLAM / DRACo-SLAM2 | 2022 IROS / 2025 | 초저대역(수중 음향), descriptor→요청 시 point cloud, group-wise GCM | https://arxiv.org/abs/2210.00867 , https://arxiv.org/abs/2507.23629 |
| Lajoie et al., C-SLAM in Planetary Analogue | 2026 arXiv | 간헐 통신 영향, **실제 throughput/latency 로그 공개** | https://arxiv.org/abs/2601.21063 |
| PCM (Mangelson et al.) | 2018 ICRA | 최대 상호일관 loop 집합(max-clique) | https://ieeexplore.ieee.org/document/8460217/ |
| GNC (Yang et al.) | 2020 RA-L | 70–80% outlier 강건 최적화 | https://arxiv.org/abs/1909.08605 |
| FLIRT / Geometrical FLIRT Phrases | 2010 ISER / 2013 ICRA | 2D LiDAR 키포인트·장소인식 | http://ais.informatik.uni-freiburg.de/publications/papers/tipaldi13icra.pdf |
| Kim, Choi, Kim, Sequential Testing for LiDAR Loop Closure in Repetitive Environments | 2025 arXiv | 반복 구조에서 여러 프레임 증거를 쌓아 루프를 채택하는 순차 검정 (본문 확인) | https://arxiv.org/abs/2512.09447 |
| Zhang & Deng, Deep Compressed Communication for Multi-Robot 2D-LiDAR SLAM | 2024 Sensors | 2D occupancy map CNN 압축, UWB 12.5 kb/s | https://doi.org/10.3390/s24103154 |
| Wi-Closure | 2022 arXiv | 무선 신호로 후보 탐색 범위 축소 | https://arxiv.org/abs/2210.01320 |

---

## 3. 이미 해결된 것 vs 공백 후보

### 3.1 신규성으로 주장하기 어려운 것

| 주장 | 판정 | 근거 |
|---|---|---|
| 드론끼리 submap/descriptor를 공유해 inter-robot loop closure로 drift를 줄인다 | ✗ 표준 | DOOR, DiSCo, DCL, Swarm-SLAM, Kimera-Multi, LAMP 2.0 |
| 서로 다른 시각에 기록된 map/trajectory를 joint graph로 결합 | ✗ 표준 | Kim 2010, multi-session SLAM, AutoMerge |
| 반복 환경에서 정합 gate + 이웃 일관성으로 오검출 제거 | ✗ | PCM, GCM, GNC |
| 통신 예산 내에서 어떤 후보를 검증할지 선택 | ✗ | Tian, Giamou, LAMP prioritization, Swarm-SLAM |
| follower가 leader의 장면을 시간 지연 후 다시 봐서 위치 개선 | △ 매우 가까움 | Ding 2026 T-ITS (asynchronous view repetition) |
| loop closure가 생기도록 경로를 계획 | △ | Bai 2024, Li 2026, active SLAM 전반 (단 revisit/탐사 전제) |
| D-optimality/algebraic connectivity로 loop 가치를 평가 | ✗ (도구로 인용) | Khosoussi, Chen, Placed |

### 3.2 공백 후보 (이번 검색에서 직접 일치 연구를 찾지 못함, 신뢰도 중간)

**G1. 편도·무복귀 임무에서 "이산적 시간차 공통 관측 구간"의 개수와 위치 설계** ★ 가장 유망
- 두 드론 모두 목표로 진행하며 복귀·상호관측·동시 만남 없음.
- 설계 변수: 공통 관측 구간의 **수 K와 경로상 위치 s₁…s_K**, 시간차 Δt.
- 기존: Ding 2026은 전 구간 연속 겹침, Bai/Li는 탐사·우회 허용, beacon 배치 연구는 외부 beacon.

**G2. 개방 체인(open chain) 궤적에서 inter-robot 제약 위치에 따른 오차 감소의 이론/실험적 특성**
- 관측: 뒤 구간 B 단독이 종단 오차 최소, A+B가 지도 품질 최선.
- Laplacian/CRLB 이론(Khosoussi, Chen)으로 예측 가능하지만, 편도 2-체인 설정에서 이를 명시·검증한 논문은 찾지 못함.

**G3. 목표별(종단 오차 vs 전체 지도 품질) 배치 trade-off**
- 기존 active SLAM은 단일 스칼라(D/A-optimality)를 최적화. "도착 시점 상대 오차"와 "전체 지도 일관성"은 서로 다른 배치를 선호할 수 있음 → 다목적 설계.

**G4. 구간 선택과 정합 가능성(registrability) 결합**
- 반복 원통·퇴화 환경에서는 "이론적으로 좋은 위치"가 실제로 정합이 안 되거나 aliasing 위험이 높을 수 있음. 위치 가치 × 정합 성공 확률을 함께 고려한 배치는 찾지 못함.

**G5. (보조) 편도 임무에서 통신 지연·손실률과 drift 보정 효과의 체계적 관계**
- Lajoie 2026 실측 트레이스를 재생해 평가 가능. 단독 주제로는 약하고 G1의 현실성 검증 축으로 적합.

**G6. (보조) 2D LiDAR UAV 간 submap 공유 C-SLAM 자체**
- 직접 사례를 찾지 못했지만 "플랫폼/센서만 다름"은 약한 신규성이므로 단독 주장 금지.

---

## 4. 반드시 전문 확인이 필요한 최근접 선행연구

| 우선 | 논문 | 확인할 질문 |
|---|---|---|
| 1 | Ding et al., T-ITS 2026, *asynchronous view repetition* | 겹침 구간을 선택/배치하는가? 연속 겹침만인가? 구간 수·위치에 따른 오차 분석이 있는가? |
| 2 | Li et al., IEEE TIE 2026, loop-aware multi-UGV planning | 편도·고정 목표 임무에도 적용되는가? 시간차 교차를 의도적으로 설계하는가? 목적함수가 종단 오차인가 전체 불확실성인가? |
| 3 | Bai et al., IROS 2024 | loop edge가 inter-robot인가? 동시 방문 필요한가? revisit 없는 경우도 다루는가? |
| 4 | Beacon placement (arXiv 2405.11550) | 배치 정식화를 "두 번째 드론의 통과 구간"으로 그대로 옮길 수 있는가? |

추가로 Google Scholar/IEEE Xplore/Scopus에서 위 논문들의 **피인용(cited-by) 추적**과 키워드 검색이 필요하다:
`"inter-robot loop closure" placement`, `"asynchronous" overlap multi-robot SLAM one-way`,
`"loop closure placement" open trajectory`, `leader follower "view repetition"`, `"rendezvous-free" cooperative SLAM`.

---

## 5. 추천 연구 질문 프레이밍

> **복귀와 동시 만남이 없는 2-UAV 편도 임무에서, 시간차 공통 관측 구간의 개수와 위치를 어떻게 설계하면 통신·경로 비용 대비 종단 오차와 지도 일관성을 최대로 줄일 수 있는가?**

영문 후보:
> *Where to Overlap: Designing Time-Staggered Shared Observation Regions for Drift Reduction in Non-Revisiting Two-UAV LiDAR Missions*

### 5.1 이론 가설 (단순 선형 모델로 바로 검증 가능)

1차원 선형-가우시안 근사로 각 드론의 odometry edge를 단위 길이당 분산 q인 저항, 출발점 초기 정렬 prior를 두 체인 시작점 연결, 공통 관측 제약을 분산 r인 저항으로 보면
두 드론 종단 상대 위치 분산은 두 종단 노드 사이의 **effective resistance**다.

- 위치 s(경로 길이 L 중)에 제약 1개:
  R_end(s) ≈ [2q(L−s) + r] ∥ [2qL]  (∥ = 병렬 합성)
  → s가 L에 가까울수록(늦은 구간) 종단 상대 오차 감소. **실험의 "B 단독이 종단 최소"와 정성적으로 일치.**
- 전체 지도 품질은 모든 노드 쌍의 상대 분산 합(Kirchhoff index 유사)에 가깝다 → 제약을 **경로 전반에 분산**할수록 유리. **"A+B가 Chamfer 최선"과 정성적으로 일치.**

이 모델은 yaw 오차가 위치 오차로 비선형 증폭되는 효과를 무시하므로, 2D SE(2) pose graph 수치 실험(CRLB)으로 확장·검증해야 한다. 하지만 "관측된 현상 → 이론 설명 → 배치 최적화" 흐름을 만들 수 있다는 점이 G1–G3의 설득력을 높인다.

### 5.2 논문 기여 후보 구성

1. **정식화**: 편도 2-체인 pose graph에서 공통 관측 구간 배치 문제 (K, s, Δt, 경로 우회 비용, 통신량).
2. **분석**: 종단 오차 vs 전체 지도 품질의 최적 배치가 다름을 CRLB/effective resistance로 보이고 시뮬레이션으로 검증.
3. **알고리즘**: 정합 가능성(registrability, aliasing 위험)을 반영한 greedy/submodular 배치 계획.
4. **평가**: 동일 코드·설정 고정 반복 비행, 구간 수·위치 sweep, 통신 지연/손실 트레이스(Lajoie 2026) 재생.

---

## 6. 리뷰어가 지적할 약점과 대응

| 약점 | 대응 |
|---|---|
| 초기 상대 좌표를 안다고 가정 (DiSCo, AutoMerge는 미지 초기값 처리) | 가정 명시 + 초기값 perturbation 실험 또는 unknown-init 확장 |
| 오프라인 사후 처리 | 온라인 증분 PGO + 통신 지연 실험 |
| 1회 개발 비행 재분석, 수렴 한도 변경 이력 | 코드·설정 동결 후 반복 비행, 사전 등록한 판정 기준 |
| B1(PX4 외부 odometry)가 지도 지표에서 여전히 우수 | raw SLAM 개선 연구로 범위 한정, B1은 상한 참고로 제시 |
| 반복 원통 환경의 aliasing | PCM/GCM 비교, false-positive 주입 실험 |
| 경로 설계가 공통 구간을 "알고" 만듦 | 배치 설계는 계획 단계 입력, 정합·최적화에는 GT/지도 미사용임을 분리 보고 (현 실험 규칙 유지) |
| 시뮬레이션 전용 | 공개 데이터(S3E https://arxiv.org/abs/2210.13723 등) 시간 분할 재현 가능성 검토 |

---

## 7. 다음 할 일

1. §4의 최근접 4편 전문 읽기 → claim 단위 비교표 작성.
2. Scholar/Scopus 피인용 추적 및 §4 키워드 검색으로 G1–G3 재확인.
3. §5.1 선형 모델과 SE(2) CRLB 수치 실험으로 "위치 효과" 예측 곡선 작성 → 현재 A/B/A+B 결과와 비교.
4. 구간 위치 sweep 실험 설계 (예: 공통 구간 1개를 경로 20/40/60/80% 지점에 배치, 2개 조합) 및 반복 횟수 결정.
