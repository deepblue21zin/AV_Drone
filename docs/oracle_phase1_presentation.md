---
marp: true
theme: default
paginate: true
size: 16:9
style: |
  section {
    font-family: "Noto Sans CJK KR", "Noto Sans KR", sans-serif;
    font-size: 27px;
    color: #172033;
    padding: 46px 62px;
  }
  h1 { color: #123a63; font-size: 43px; }
  h2 { color: #123a63; font-size: 34px; }
  strong { color: #b42318; }
  table { font-size: 21px; width: 100%; }
  th { background: #eaf2f8; }
  blockquote { border-left: 8px solid #2e86c1; background: #f4f8fb; }
  .small { font-size: 19px; color: #52606d; }
  .good { color: #18794e; }
  .warn { color: #b42318; }
  .center { text-align: center; }
  .two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 36px; }
---

<!-- _class: lead -->

# One-way Multi-UAV LiDAR SLAM Drift Correction

## Phase 1 — Oracle constraint feasibility

2-UAV Gazebo/PX4/ROS 2 개발 결과와 다음 실험 설계  
2026-08-27 smoke run

<!--
발표 포인트: 지금 단계는 자동 장소 인식 성능 발표가 아니다. 올바른 inter-UAV constraint가 주어질 때 현재 SLAM drift가 보정 가능한지 확인한 단계다.
-->

---

# 먼저 결론

> 현재 결과는 **파이프라인 동작과 보정 가능성**은 보여주지만, 제안 알고리즘의 성능을 입증하지는 않는다.

- 두 UAV의 raw SLAM 상대 위치 종단 오차: **0.353 m**
- periodic oracle 보정 후: **0.133 m** → B0 대비 **62.2% 감소**
- 지도 Chamfer: **0.083 m → 0.035 m** → **58.4% 감소**
- 그러나 single factor가 periodic보다 종단 위치 오차가 작아 **정식 gate는 실패**
- 현재 run은 88.2초 부분 주행, periodic constraint도 2개뿐

<div class="small">따라서 발표 표현: “효과를 입증했다”가 아니라 “본 실험으로 진행할 근거를 얻었다.”</div>

---

# 왜 이 문제가 어려운가?

<div class="two-col">

<div>

### 단일 UAV 왕복 SLAM

- 전에 본 장소를 다시 방문
- loop closure로 누적 drift를 관측
- 과거 pose까지 graph optimization

</div>

<div>

### 현재 one-way Multi-UAV

- 각 UAV는 같은 장소로 돌아오지 않음
- self-loop correction 기회가 없음
- 서로 다른 UAV의 관측을 연결해야 함
- 원통은 회전 대칭이고 부분 중첩도 작음

</div>

</div>

> 핵심 질문: “신뢰 가능한 inter-UAV 상대 pose constraint가 존재한다면, one-way 누적 drift와 지도 왜곡을 줄일 수 있는가?”

---

# Phase 1에서 비교한 조건

| 조건 | 사용 pose/constraint | 역할 |
|---|---|---|
| B0 | 각 UAV의 raw `slam_toolbox` pose | 보정 전 기준선 |
| B1 | known spawn + PX4 odometry | 강한 비-SLAM 참조 기준선 |
| O-single | raw SLAM graph + GT 기반 inter-UAV factor 1개 | 한 번 연결했을 때의 효과 |
| O-periodic | raw SLAM graph + 약 10 m 간격 GT 기반 factor | 반복 연결의 효과 |

- GT는 **oracle factor 생성 및 평가에만 사용**
- GT를 센서 입력으로 사용하는 자동 보정법이라고 주장하지 않음
- 자동 원통 인식·descriptor·LiDAR registration은 Phase 2 범위

---

# 구현된 분석 파이프라인

![w:1100](../artifacts/2026-08-27_oracle_phase1_smoke_codex_v1/oracle_correction/presentation/01_pipeline.png)

<!--
발표 포인트: scan/odom/TF/GT를 같은 bag에 기록하고, scan timestamp에 맞춰 동기화한 뒤 동일 scan을 pose 조건별로 재투영했다. 따라서 조건별 차이가 scan 표본 차이에서 오지 않도록 했다.
-->

---

# 현재 smoke run의 범위와 데이터 유효성

| 항목 | UAV 1 | UAV 2 |
|---|---:|---:|
| source scans | 1,293 | 1,293 |
| common support / valid scans | 1,219 | 1,222 |
| startup trimmed | 74 | 71 |
| interpolation sync drop | 0 | 0 |
| keyframes | 14 | 13 |
| 실제 진행 거리 | 약 25.0 m | 약 23.1 m |

- 기록 시간: 약 **88.2 s**
- 시작 구간 trim 이후 동기화 누락률: **0%**
- 계획했던 30 m 완주 전 종료된 **부분 smoke run**
- 20–25.9 m 구간 shared voxel ratio 0.442, 0–20 m 구간은 0.113

<div class="warn">이 데이터는 코드와 지표 검증에는 충분하지만 통계적 성능 주장에는 부족하다.</div>

---

# 1. 궤적은 어떻게 볼 것인가?

![w:1100](../artifacts/2026-08-27_oracle_phase1_smoke_codex_v1/oracle_correction/presentation/02_trajectories.png)

- 전체 모양보다 **GT에서 벗어나는 방향과 구간**을 확인
- UAV별 ATE/RPE는 디버깅 지표
- 논문의 핵심은 두 UAV의 지도 정합에 직접 대응하는 **inter-UAV differential error**

---

# 2. Constraint가 어디에 들어갔는가?

![w:1100](../artifacts/2026-08-27_oracle_phase1_smoke_codex_v1/oracle_correction/presentation/04_oracle_factors.png)

- O-single: 공통 진행 구간의 중앙에 1개
- O-periodic: 약 10 m 간격, 현재 부분 run에서는 2개
- factor 수뿐 아니라 **공간 분포와 정보량**을 함께 봐야 함

<div class="small">원통/부분 중첩 기반 실제 matching으로 바뀌면 각 선에 confidence, overlap, registration residual을 함께 기록해야 한다.</div>

---

# 3. 핵심 지표: 두 UAV 사이 상대 drift

![w:950](../artifacts/2026-08-27_oracle_phase1_smoke_codex_v1/oracle_correction/presentation/03_differential_error.png)

- translation과 yaw를 **반드시 같이** 해석
- O-single은 종단 translation은 작지만 종단 yaw는 **1.746°**
- O-periodic 종단: translation **0.133 m**, yaw **0.374°**
- 단일 종단값만으로 순위를 정하지 말고 경로 전체 RMSE/maximum도 함께 보고

---

# 4. 지도는 TP/FP/FN으로 보여준다

![w:650](../artifacts/2026-08-27_oracle_phase1_smoke_codex_v1/oracle_correction/presentation/06_map_error_overlay.png)

<div class="small">초록=TP, 빨강=FP, 파랑=FN, 회색=공통 관측 영역. 발표에서는 error overlay, 논문에서는 원본 map과 overlay를 함께 제시한다.</div>

---

# 5. 정량 결과

![w:1100](../artifacts/2026-08-27_oracle_phase1_smoke_codex_v1/oracle_correction/presentation/07_metric_summary.png)

<div class="small">모든 값은 단일 partial smoke run 결과이며 평균±표준편차가 아니다.</div>

---

# 결과를 어떻게 해석해야 하나?

### 확인된 것

- raw SLAM 기반 두 지도 사이에는 진행과 함께 상대 drift가 누적됨
- oracle constraint가 들어간 graph가 정상 수렴함
- periodic oracle은 B0보다 상대 위치와 지도 Chamfer를 개선함
- scan 동기화 → keyframe → PGO → map reprojection → 평가가 end-to-end로 동작함

### 아직 확인되지 않은 것

- 실제 LiDAR만으로 동일 장소를 안정적으로 찾을 수 있는가?
- 낮은 overlap과 회전 대칭 원통에서 올바른 yaw를 선택할 수 있는가?
- periodic constraint가 single constraint보다 일관되게 좋은가?
- 서로 다른 seed와 장거리에서도 개선이 유지되는가?

---

# 왜 formal gate가 실패했는가?

![w:900](../artifacts/2026-08-27_oracle_phase1_smoke_codex_v1/oracle_correction/presentation/08_gate_summary.png)

<!--
발표 포인트: 실패를 숨기지 않는다. 짧은 데이터에서 single 1개, periodic 2개이고 서로 다른 노이즈 표본이 들어간다. endpoint translation만 보면 single이 우연히 유리할 수 있다. yaw와 map에서는 다른 양상이 나온다.
-->

---

# 본 실험에서는 이렇게 분석한다

1. **데이터 감사**: 공통 시간 구간, 누락률, 실제 주행 거리, keyframe/factor 수
2. **궤적 정확도**: UAV별 ATE/RPE와 endpoint error
3. **상대 drift**: 5 m 진행 구간별 translation/yaw error와 종단값
4. **지도 정확도**: Chamfer, occupied precision/recall/F1, coverage
5. **제약 품질**: factor별 overlap, residual, confidence, 공간 분포
6. **반복성**: 137 m × 최소 3 seeds, 가능하면 5 seeds
7. **통계**: seed별 paired improvement, mean±std 또는 median/IQR, 95% CI

> 1차 주 지표는 `inter-UAV differential translation/yaw`, 2차 지표는 map Chamfer와 occupied F1로 둔다.

---

# 다음 개발·실험 순서

| 순서 | 구현/실험 | 통과 조건 |
|---:|---|---|
| 1 | 30 m smoke 완주 재실행 | sync drop ≤1%, factor 수/위치 정상 |
| 2 | 137 m oracle 실험, 3–5 seeds | B0 대비 drift ≥30%, Chamfer ≥20% 개선 |
| 3 | periodic spacing/weight ablation | single 대비 후반 drift 일관 개선 |
| 4 | 원통 cluster + rotation search | 후보별 yaw score와 ambiguity 계산 |
| 5 | overlap/confidence gate | 낮은 overlap·다중 peak constraint 거부 |
| 6 | scan-derived factor로 oracle 대체 | oracle 대비 성능 gap 보고 |

- overlap이 너무 작으면 억지로 보정하지 않고 **no-update**가 정상 동작
- 회전 점수 하나만 보내지 말고 top-k yaw 후보와 confidence를 전달
- Future work로 미룰 수 있는 범위: learned descriptor, 통신량 최적화, 실시간 분산 PGO

---

<!-- _class: lead -->

# Take-away

> One-way Multi-UAV에서는 “같아 보이는 원통” 자체보다, **제약을 언제 믿고 언제 거부할지**가 핵심 연구 문제가 된다.

- Phase 1: 올바른 inter-UAV constraint가 있으면 보정 가능함을 검증
- Phase 2: 부분 중첩·회전 대칭에서 신뢰도 있는 constraint 생성
- 논문 기여점 후보: **ambiguity-aware overlap gating + pose graph correction**
