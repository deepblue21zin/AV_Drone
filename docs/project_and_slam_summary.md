# Drone_0716 (AV_Drone) 프로젝트 총정리 + SLAM 파트 상세

작성일: 2026-07-30

---

## 1. 이 프로젝트가 뭐하는 프로젝트인가

`Drone_0716`(저장소 내부 명칭: `AV_Drone`)은 **PX4 SITL + Gazebo Classic 11 + ROS 2 Humble + MAVROS**
기반으로 단일 드론의 자율 비행(장애물 회피 → 목표 도달 → 복귀)을 구현하고, 그 결과를 정량적으로
비교·분석하는 학부 연구/논문용 프로젝트다.

### 1-1. 팀 구성과 원래 목표

팀 키워드는 **회피(Gap), SLAM(Mapping), A\*, MPPI** 네 가지였다. 처음에는 "각자 이 알고리즘을
구현해보자"까지만 정해져 있었고, 이걸 하나로 묶는 최종 연구 질문은 없었다. 논의 끝에
(`docs/research_topic_direction.md`) 다음과 같은 하나의 실험 서사로 정리되었다.

> **연구 질문**: 관성이 있는 비행체가, 정보 없이(Gap) 첫 통과로 얻은 불완전한 지도를,
> 복귀 시 계획형 알고리즘(MPPI/A\*)이 실제로 얼마나, 어떤 조건에서 유의미하게 활용할 수 있는가?

네 알고리즘의 역할 분담:

| 요소 | 역할 |
|---|---|
| Gap (반응형 회피) | outbound(왕복 중 갈 때) 전담 — 지도 없이 첫 통과를 안전하게 마치고, 그 부산물로 불완전한 지도를 생성 |
| SLAM/Mapping | Gap 비행 중 관측된 라이다 데이터를 2D occupancy grid 지도로 정리·저장 |
| A\* | 지도가 주어졌을 때의 복귀 전략 후보 1 (전역 경로 탐색) |
| MPPI | 지도가 주어졌을 때의 복귀 전략 후보 2 (샘플링 기반 국소 최적화) |
| 드론(관성) | 반응형 vs 계획형의 차이가 지상 로봇보다 뚜렷하게 드러나는 실험 무대 |

핵심 통찰: "완전한 지도가 있으면 최적 경로가 더 빠른 건 당연하다"는 뻔한 질문이 아니라,
**"단 한 번의 편도 비행으로 얻은, 필연적으로 불완전한 지도가 복귀 성능에 실제로 얼마나
도움이 되는가, 그 이득은 어떤 조건(장애물 밀도·통로 폭·지도 커버리지)에서 사라지거나
역전되는가"**를 정량적으로 찾는 것이 논문의 실제 기여점이다.

### 1-2. 현재 검증된 런타임 (Active Baseline)

```text
Host Ubuntu 22.04
└─ Docker Compose
   ├─ sim  : PX4 SITL + Gazebo Classic 11 + iris_rplidar + obstacle_demo.world
   └─ ros  : ROS 2 Humble + MAVROS + autonomy nodes + ros_states
```

데이터 흐름 (회피 baseline 기준):

```text
Gazebo Classic LiDAR
  → /drone1/scan
  → drone_perception/lidar_obstacle_node       (가장 가까운 장애물 거리 추출)
  → /drone1/perception/nearest_obstacle_distance
  → drone_planning/local_planner_node          (Follow-the-Gap 반응형 회피)
  → /drone1/autonomy/cmd_vel
  → drone_safety/safety_monitor                (timeout/emergency stop 감시)
  → /drone1/safety/cmd_vel
  → drone_control/autonomy_manager             (비행 phase 관리, MAVROS setpoint 전달)
  → /mavros/setpoint_velocity/cmd_vel

PX4 SITL → MAVROS → /mavros/state, /mavros/local_position/pose → 전체 파이프라인 공용 입력

모든 스트림 → drone_metrics/metrics_logger → artifacts/<timestamp>_drone1/ (실험 기록)
```

이 baseline은 실제로 `HOVER_AT_GOAL` 도달, `goal_reached=true`, artifact 저장까지 확인된
**살아있는 single-UAV baseline**이다. 다중 드론, task reallocation, MPPI 복귀, failure-aware
continuation 등은 아직 상위 연구 단계로 미완성이다.

### 1-3. ROS 2 패키지 구성 (`src/`)

| 패키지 | 담당 |
|---|---|
| `drone_bringup` | 전체 launch 진입점, config, 팀별 개발 프로필 (`single_drone_avoidance_dev`, `single_drone_slam_dev`, `single_drone_mppi_dev`) |
| `drone_perception` | LiDAR 스캔에서 최근접 장애물 거리 추출 |
| `drone_planning` | Follow-the-Gap 반응형 회피(`local_planner_node`), A\* 전역 경로 탐색(`astar_global_planner_node`) |
| `drone_safety` | pose/scan/planner 커맨드 timeout 감시, 비상 정지(fail-safe) |
| `drone_control` | 비행 phase 상태머신(`WAIT_STREAM → OFFBOARD_ARM → TAKEOFF → HOVER_AFTER_TAKEOFF → FOLLOW_PLAN → HOVER_AT_GOAL`), MAVROS 인터페이스 |
| `drone_metrics` | 실행마다 `metadata.json` / `metrics.csv` / `summary.json` / `events.log` 저장 |
| `drone_slam` | **(이 문서 2절에서 상세)** LiDAR+pose 기반 2D occupancy grid 매핑, 지도 저장 |
| `mppi`, `mppi_lidar` | MPPI 기반 독립 미션 노드 (takeoff → mppi → hover/land), LiDAR 직접 소비 버전 |
| `ros_states` | topic/phase/artifact를 웹 대시보드로 시각화, 디버깅 세션 기록/리포트 생성 |

### 1-4. 팀 분리 개발 전략

같은 `drone1` 네임스페이스에서 한 번에 하나의 프로필만 실행하는 것을 원칙으로 하고,
담당자별로 소유 범위를 나눈다.

- 회피 담당: `src/drone_planning` + `single_drone_avoidance_dev.launch.py`
- **SLAM 담당: `src/drone_slam` + `single_drone_slam_dev.launch.py`**
- MPPI 담당: `src/mppi`, `src/mppi_lidar` + `single_drone_mppi_dev.launch.py`
- 통합 담당: `src/drone_bringup` + 추후 planner selector

나중 통합을 위해 topic 계약을 미리 예약해둔 상태다 (`/drone1/planner/avoid/cmd_vel`,
`/drone1/planner/mppi/cmd_vel`, `/drone1/planner/selected/cmd_vel` 등). 지금은 알고리즘별로
분리 실행하고, 이후 planner selector를 추가해 hard switch 방식으로 통합할 계획이다.

### 1-5. 실험/정량화 인프라

- `artifacts/<timestamp>_drone1/`: 실행마다 쌓이는 실험 기록 (metrics, trajectory, slam_summary 등)
- `experiments/experiment_matrix.yaml`: 실험 조건 정의
- `docs/gap_vs_mppi_quantification_plan.md`: Gap vs MPPI 정량 비교 실험 설계 (조건, 지표, 데이터 스키마)
- `scripts/quant_dashboard.py` + Streamlit 대시보드: condition/scenario/run 단위 비교 조회 (read-only)
- 논문용 최종 수치는 `paper_metrics.json`, `experiments/paper_outputs/summary_table.csv`,
  `experiments/paper_outputs/figure_manifest.csv` 기준으로 관리

---

## 2. 내가 맡은 SLAM 파트

### 2-1. 이 파트의 실제 목표

담당 패키지: **`src/drone_slam`**

가장 먼저 명확히 해야 할 것: 이 파트의 이름은 "SLAM"이지만, 엄밀히 말하면
**SLAM(Simultaneous Localization And Mapping) 중 Mapping(M) 쪽만** 실제로 풀고 있는
문제다. Localization은 시뮬레이터가 제공하는 ground-truth pose(`/mavros/local_position/pose`)를
그대로 가져다 쓰기 때문에, 이 파트가 실제로 새로 구현/증명해야 하는 부분은 아니다.
(이 구분은 `docs/research_topic_direction.md`에서 팀 논의 끝에 명시적으로 합의된 내용이다.)

**목표를 한 문장으로 정리하면:**

> Gap 기반 반응형 회피로 목표까지 편도 비행하는 동안 관측한 LiDAR 데이터를 2D occupancy grid
> 지도로 실시간 누적하고, 비행이 끝나는 시점에 그 지도를 파일로 저장해서, 이후 복귀 단계의
> 계획형 알고리즘(A\*, MPPI)이 재사용할 수 있는 고정 경로의 지도 산출물을 만드는 것.

이때 중요한 설계 전제 두 가지:

1. **완전한 지도를 만드는 게 목표가 아니다.** Gap 회피의 목적함수("장애물에서 최대한 멀리
   떨어진 넓은 틈으로 통과")는 지도를 촘촘히 채우는 것과는 방향이 다르기 때문에, Gap으로 만든
   지도는 필연적으로 사각지대·미관측 영역이 남는 **체계적으로 불완전한 지도**다. 이건 버그가
   아니라 연구 전제 자체다 — "완전한 지도가 아니라 안전 우선 회피 비행의 부산물로 얻어진
   현실적으로 불완전한 지도가 복귀에 얼마나 도움이 되는가"를 보는 게 이 프로젝트의 핵심 질문이기
   때문이다.
2. **인터페이스를 먼저 예약하고, 실제 구현으로 점진적으로 교체하는 전략.** 팀 분리 개발 초기에는
   `/drone1/slam/status`, `input_ready`, `map_ready`, `localization_ok`, `coverage` 같은 상태
   토픽 인터페이스만 먼저 잡아두는 **scaffold(자리표시) 노드**로 시작했고, 이후 실제 매핑 로직을
   구현한 노드로 교체하는 순서로 진행했다.

### 2-2. 실제로 구현한 것

`src/drone_slam/drone_slam/` 아래 3개의 노드로 구성된다.

#### (1) `slam_scaffold_node.py` — 초기 인터페이스 예약용 placeholder

- 실제 SLAM/매핑 로직은 전혀 없음. `/drone1/scan` + `/mavros/local_position/pose` 구독 여부만
  확인해서 `scaffold_inputs_ready` / `scaffold_waiting_inputs` 상태를 heartbeat로 publish.
- `map_ready`, `localization_ok`는 항상 `false`, `coverage`는 항상 `0.0`만 publish.
- 팀 분리 개발 극초반에 "회피 스택 없이 SLAM 입력 경로(scan/pose)만 먼저 확인"하려는 목적으로
  `single_drone_slam_dev.launch.py` 개발 프로필에서 사용.
- **현재는 실제 비행/매핑 실행에는 쓰이지 않음** — 아래 (2)로 대체됨. 개발 이력을 보여주는
  단계로 남아 있음.

#### (2) `simple_2d_mapping_node.py` — 실질적인 매핑 로직 (핵심 구현)

`/drone1/scan`(LaserScan) + `/mavros/local_position/pose`(PoseStamped)를 구독해서
**log-odds 기반 2D occupancy grid**를 실시간으로 누적하고 `/map`(`nav_msgs/OccupancyGrid`)으로
publish하는 노드.

구현 세부:

- **레이 캐스팅 기반 업데이트** (`_on_scan`): 매 스캔마다 라이다의 각 광선(ray)에 대해
  - 광선이 통과한 경로 상의 점들 → `_mark_free_ray`로 log-odds를 감소시켜 "빈 공간"으로 기록
  - 광선이 장애물에 부딪힌 최종 지점 → `_add_log_odds`로 log-odds를 증가시켜 "점유"로 기록
  - 로봇의 world 좌표계 각도는 pose에서 quaternion을 yaw로 변환(`quaternion_to_yaw`)한 값과
    라이다 각도를 합산해서 계산
- **log-odds 누적/클램핑**: 각 셀의 log-odds 값은 `-3.5 ~ 4.5` 범위로 clamp해서, 한 번의
  잘못된 관측으로 값이 무한정 커지거나 작아지지 않도록 함.
- **occupancy 값 변환** (`_to_occupancy_data`): 최종 publish 시 log-odds를 표준 OccupancyGrid
  인코딩(`-1`=미관측, `0`=빈공간, `1~100`=점유 확률)으로 변환. `abs(log_odds) < 0.25`면 아직
  충분히 관측되지 않은 것으로 보고 `-1`(unknown) 처리.
- **격자 파라미터화**: 해상도(`map_resolution`)와 범위(`map_min_x/max_x/min_y/max_y`)를 모두
  ROS 파라미터로 노출. 현재 설정(`drone1_avoidance_dev.yaml`)은 150m 코스 전체를 덮도록
  `x: -2~152`, `y: -16~16`로 맞춰져 있음.
- **입력 상태 모니터링**: scan/pose 각각의 마지막 수신 시각을 추적해서 `input_timeout_sec`
  이내에 둘 다 들어왔는지로 `input_ready` 판단, 이를 `/drone1/slam/status`,
  `/drone1/slam/input_ready`, `/drone1/slam/map_ready`, `/drone1/slam/localization_ok`로
  주기적으로(`publish_hz`, 기본 2Hz) publish.
- **명시적으로 하지 않는 것**: 이 노드 자체는 지도를 파일로 저장하지 않는다. 매핑 계산과
  파일 저장 책임을 분리해서, 매핑 노드는 계산에만 집중하도록 설계(다른 패키지의
  `metrics_logger` 패턴과 동일한 "관측 노드 + 별도 기록 노드" 구조).

#### (3) `map_saver_node.py` — 지도 파일 저장 전담

`/map`을 구독해 최신 OccupancyGrid를 계속 캐싱하고 있다가, 미션 phase
(`/drone1/mission/phase`)가 지정된 값(기본 `"LANDED"`)에 도달하는 **그 순간 딱 한 번만**
파일로 저장하는 노드.

- 저장 위치: `maps/<scenario_name>/<map_file_basename>_grid.npy` +
  `<map_file_basename>_meta.json` (기본 root: `/workspace/AV_Drone/maps`)
- `map_grid.npy`: `numpy.save()`로 저장된 `(height, width)` shape의 `int8` 2차원 배열.
  OccupancyGrid의 flat `data`를 `height × width`로 reshape한 것과 동일.
- `map_meta.json`: `resolution`, `width`, `height`, `origin_x/y/z`, `frame_id`, `scenario_name`,
  `saved_at`을 담아, grid 인덱스를 world 좌표로 역산할 수 있게 함
  (`world_x = origin_x + (col + 0.5) * resolution` 등).
- **파일명이 같으면 매 실행마다 덮어쓴다**는 게 의도된 설계다. 실험용
  `artifacts/<run_id>/`처럼 실행마다 새 폴더를 만들지 않고, SLAM을 다시 돌리면 최신 지도로
  갱신되어 이후 복귀 단계(A\*/MPPI)가 항상 같은 고정 경로 하나만 참조하면 되도록 만든 것.
- 한 번 저장하면 내부 플래그로 재저장을 막아, phase heartbeat가 계속 와도 중복 저장하지 않음.

### 2-3. 통합 지점 (현재 상태)

- `single_drone_avoidance_dev.launch.py`에서 `simple_2d_mapping_node` + `map_saver_node`가
  Gap 회피 스택과 **함께** 뜬다. 즉 SLAM 전용 실행이 따로 있는 게 아니라, 실제 outbound
  미션 비행 중에 지도가 같이 만들어지는 구조다.
- `single_drone_slam_dev.launch.py`는 여전히 `slam_scaffold_node` 단독 개발 프로필로 남아있다
  (회피 스택 없이 scan/pose 입력 경로만 확인하고 싶을 때 사용).
- 저장된 `maps/<scenario>/*_grid.npy` / `*_meta.json`을 A\*나 MPPI 쪽 노드가 아직 코드로
  직접 로드하는 부분은 확인되지 않았다 — 즉 **지도 저장까지는 구현되어 있지만, 복귀 단계
  알고리즘이 이 지도를 실제로 소비하는 연결은 아직 다음 작업 단계**로 보인다
  (`research_topic_direction.md` 10절의 "다음 실행 항목"에도 아직 체크되지 않은 항목으로
  남아 있음).

### 2-4. 요약: 목표 vs 구현 현황

| 항목 | 상태 |
|---|---|
| SLAM 입력 경로(scan/pose) 인터페이스 예약 | ✅ 완료 (`slam_scaffold_node`) |
| 실시간 log-odds 기반 2D occupancy grid 매핑 | ✅ 완료 (`simple_2d_mapping_node`) |
| 지도 상태 토픽(`status`/`input_ready`/`map_ready`/`localization_ok`) publish | ✅ 완료 |
| 미션 종료 시점 지도 파일 저장 (`.npy` + `.json` 메타) | ✅ 완료 (`map_saver_node`) |
| `coverage`(커버리지 정량 지표) 실제 계산 후 publish | ⚠️ scaffold 단계에서는 항상 0.0 — 실제 계산 로직은 미구현으로 보임 |
| A\*/MPPI가 저장된 지도를 로드해서 복귀 경로 계산에 사용 | ❌ 아직 연결 안 됨 (다음 통합 단계) |
| 실제 SLAM(Localization 포함)이 아닌 Mapping-only라는 점 | 팀 논의로 명시적으로 합의·문서화됨 |

---

## 3. 참고 문서 경로

- 프로젝트 전체 개요: `README.md`
- 아키텍처: `docs/architecture.md`
- SLAM 파트 코드 설명: `src/drone_slam/README.md`
- 연구 방향 논의 전체 기록: `docs/research_topic_direction.md`
- 정량 실험 설계: `docs/gap_vs_mppi_quantification_plan.md`
