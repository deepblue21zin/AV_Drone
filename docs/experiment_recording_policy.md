# Experiment Recording Policy

## Purpose

이 문서는 이 저장소에서 실험 기록을 어떤 계층으로 나눠 남길지 정의한다.
목표는 세 가지다.

- 디버깅과 재현성을 높인다.
- 현업 수준의 문제 추적과 회귀 검증 근거를 남긴다.
- 이후 논문 작성 시 결과 표, 그림, 주장과 실제 run artifact를 연결할 수 있게 한다.

핵심 원칙은 `artifacts`, `experiments`, `docs/change`를 하나로 묶어 보되, 서로 다른 책임을 갖게 분리하는 것이다.

## Why This Split Is Good

현업 관점에서 이 구조는 좋은 편이다. 이유는 다음과 같다.

- `artifacts/`는 자동 생성되는 기계 중심 증거다. 실행 중 일어난 사실을 남긴다.
- `experiments/`는 실험 단위의 장부다. 문제, 수정, 결과를 비교 가능하게 만든다.
- `docs/change/`는 사람 중심 변경 이력이다. 왜 바꿨는지를 설명한다.

이 셋을 한 파일에 섞어버리면 나중에 다음 문제가 생긴다.

- raw 증거와 해석이 뒤섞여서 검증이 어려워진다.
- 실패 원인과 수정 이유를 추적하기 힘들어진다.
- 논문 그림/표를 어떤 run에서 만들었는지 거슬러 올라가기 어렵다.

즉, 이 구조는 “하나의 체계”로 운영하되 “서로 다른 레이어”로 분리하는 것이 핵심이다.

## Recording Layers

### 1. `artifacts/`

역할:
- 실행별 자동 증거 보관

생성 주체:
- `metrics_logger`

현재 생성 파일:
- `metadata.json`
- `parameter_snapshot.json`
- `config_snapshots/`
- `metrics.csv`
- `events.log`
- `summary.json`

현재 구현 근거:
- `src/drone_metrics/drone_metrics/metrics_logger_node.py`

규칙:
- 한 번 생성된 artifact는 수정하지 않는다.
- 실행 1회 = artifact 1개 디렉터리로 본다.
- artifact는 “실험의 사실”을 남기고, 해석이나 서술은 넣지 않는다.

### 2. `experiments/`

역할:
- 실험 장부, 집계표, ledger, plot index

생성 주체:
- `scripts/smoke_test_single_drone.sh`
- `scripts/update_experiment_registry.py`
- `scripts/generate_artifact_plots.py`

현재 파일 구조:
- `index.csv` / `index.md`
- `scenario_table.csv` / `scenario_table.md`
- `ledger.csv` / `ledger.md`
- `plots/`

규칙:
- experiment row는 artifact를 참조해야 한다.
- issue / fix / notes는 experiments ledger에 남긴다.
- 팀 단위 판단은 artifact 하나가 아니라 experiments 장부를 기준으로 한다.

### 3. `docs/change/`

역할:
- 사람이 읽는 변경 이력
- 코드/설정/문서/환경 변경 이유 기록

규칙:
- 날짜별 파일로 누적한다.
- “무엇을 바꿨는지”뿐 아니라 “왜 바꿨는지”, “남은 리스크가 무엇인지”를 적는다.
- 실험 결과를 직접 저장하지는 않되, 관련 artifact나 ledger와 연결될 수 있어야 한다.

## Current Coverage Matrix

| Category | Current Status | What Exists Now | What Is Missing / Next Step |
| --- | --- | --- | --- |
| Run metadata | Good | commit, branch, dirty, scenario, world, model, topic names, started_at, baseline/planner version, host_name, parameter snapshot path | Docker image tag, ROS distro, simulator build id를 더 추가하면 좋음 |
| Raw time series | Partial | `metrics.csv`에 pose/scan/planner/safe count와 obstacle, phase가 주기적으로 기록됨 | full raw topic recording 없음, 실제 pose 값/scan 값 원시 데이터 없음, rosbag2 필요 |
| Event log | Partial | `events.log`에 phase 변경과 safety event가 남고, `summary.json`에 `failure_code`와 `last_safety_reason`이 기록됨 | offboard 요청, arm 요청, service response, task reassignment, operator intervention 기록 필요 |
| Outcome KPI | Partial-Good | `summary.json`에 runtime, goal_reached, pose/scan count, p99 period, obstacle distance, safety_intervention_count, failure_code 존재 | collision 여부, mission timeout, real-time factor, energy 관련 지표 필요 |
| Issue/Fix ledger | Partial | `experiments/ledger.csv` 구조와 update script 존재 | 모든 의미 있는 실행에서 일관되게 기록하는 습관과 자동화 필요 |
| Seed | Partial | `experiment_seed`가 metadata/summary/index에 기록됨 | simulator 내부 seed와 randomized scenario seed를 더 연결하면 좋음 |
| Scenario taxonomy | Partial-Good | `scenario_manifest`와 copied manifest file로 obstacle, start/goal, communication profile을 남긴다 | scenario variant 집합과 멀티드론 taxonomy를 더 늘리면 좋음 |
| N repeated trials | Partial | experiments 구조는 집계 가능 | 최소 반복 횟수 규칙과 반복 실행 자동화 필요 |
| Baseline comparison | Partial | `baseline_name`, `planner_name`, `planner_version`, `controller_version`가 metadata/experiments에 기록됨 | 실제 baseline 간 자동 비교/집계 리포트는 추가 필요 |
| Failure labeling | Partial-Good | `failure_code`가 summary와 experiments ledger/index에 반영됨 | collision, mission timeout, task reassignment failure까지 taxonomy를 넓혀야 함 |
| Artifact-to-paper mapping | Missing | 직접 연결 구조 없음 | figure/table가 어떤 run_id에서 나왔는지 연결 파일 필요 |

## What Is Already Working Now

현재 기준으로 이미 하고 있는 것:

- 실행마다 artifact 디렉터리를 만들고 있다.
- artifact에 metadata, parameter snapshot, copied config snapshots, summary, events, metrics를 남기고 있다.
- smoke test 성공/실패를 기반으로 experiment registry를 갱신할 수 있다.
- `issue`, `fix`, `notes`를 ledger에 남길 수 있는 스크립트가 있다.
- 코드/설정 변경은 `docs/change/`에 누적할 수 있는 구조를 만들었다.

즉, 지금은 완전한 연구 실험 플랫폼은 아니지만,
`실행 증거 -> 실험 장부 -> 변경 이력`의 기본 3층 구조는 이미 시작점이 갖춰져 있다.

## What Should Be Added Next

우선순위 순서대로 추천하면 다음과 같다.

### Priority 1: Reproducibility (next strengthening)

- 기존 parameter snapshot과 config snapshot을 모든 실행 경로에서 빠짐없이 남기도록 유지
- simulator 내부 seed와 randomized scenario seed까지 연결
- failure code taxonomy를 collision / mission timeout / task reassignment failure까지 확장

### Priority 2: Raw Replay Capability

- `rosbag2` recording 추가
  - 최소 대상 토픽
  - `/mavros/state`
  - `/mavros/local_position/pose`
  - `/drone1/scan`
  - `/drone1/autonomy/cmd_vel`
  - `/drone1/safety/cmd_vel`
  - `/drone1/perception/nearest_obstacle_distance`
  - `/drone1/mission/phase`
  - `/drone1/safety/event`
- sim / ros 텍스트 로그를 artifact 안으로 복사
  - `docker compose logs sim`
  - autonomy launch stdout/stderr
  - ros_states health snapshot

### Priority 3: Experiment Governance

- smoke test 외 수동 실행도 registry에 기록되게 보강
- `baseline_name`, `planner_name`, `planner_version` 필드 추가
- scenario manifest 파일 도입
  - 시작점, 목표점, 장애물 배치, 드론 수, 통신 제약, 목적을 명시

### Priority 4: Paper Readiness

- 반복 실행 기본 규칙 정의
  - 예: 모든 주장에 대해 최소 10회 반복
- figure/table mapping 파일 추가
  - 예: `experiments/paper_map.csv`
  - `figure_id`, `table_id`, `run_id`, `artifact_path`, `plot_path`, `notes`
- baseline 비교 결과를 scenario 단위로 자동 집계

## Recommended Operating Rule

실행과 기록은 아래 순서를 기본으로 한다.

1. 코드나 설정을 바꿨으면 `docs/change/YYYY-MM-DD.md`에 변경 이유를 남긴다.
2. 드론 실행 시 artifact가 생성되는지 먼저 확인한다.
3. 중요한 실행은 `smoke_test_single_drone.sh` 또는 registry update로 `experiments/` 장부에 등록한다.
4. 실패 실행도 버리지 말고 issue/fix/notes와 함께 ledger에 남긴다.
5. 논문 후보 실험은 seed, scenario manifest, 반복 횟수, plot mapping까지 같이 남긴다.

## Required Minimum Per Meaningful Run

“의미 있는 실행”마다 최소한 아래는 남겨야 한다.

- artifact 1개
- `result` 값이 들어간 experiment row 1개
- issue/fix/notes 중 하나 이상
- 어떤 코드/설정 변경이 있었으면 docs/change entry 1개

## Practical Rules For This Repo

### Use `artifacts/` for facts

여기에는 아래만 둔다.

- 기계가 자동 생성한 결과
- 수치 요약
- 이벤트 로그
- raw 또는 raw에 가까운 증거

### Use `experiments/` for comparison

여기에는 아래를 둔다.

- 성공/실패 집계
- 시나리오별 pass rate
- 문제 -> 수정 -> 결과 ledger
- plot과 표를 만드는 인덱스

### Use `docs/change/` for engineering narrative

여기에는 아래를 둔다.

- 왜 바꿨는지
- 어떤 판단이 있었는지
- 어떤 리스크가 남는지
- 다음 액션이 무엇인지

## Suggested Future Fields

향후 아래 필드를 점진적으로 추가하는 것을 권장한다.

- `baseline_name`
- `planner_version`
- `controller_version`
- `seed`
- `failure_code`
- `collision`
- `mission_timeout`
- `task_reassignment_count`
- `task_reassignment_latency_s`
- `fallback_triggered`
- `bag_path`
- `sim_log_path`
- `ros_log_path`
- `paper_figure_refs`

## Multi-Drone / Paper Extension Notes

이 프로젝트가 이후 multi-UAV + MPPI + task reallocation 논문 방향으로 가려면 다음 이벤트가 반드시 기록되어야 한다.

- 어떤 드론이 danger 상태를 감지했는가
- danger 판정 기준은 무엇이었는가
- task가 어떤 규칙으로 다른 드론에 재할당되었는가
- 재할당까지 걸린 시간은 얼마인가
- 원래 드론과 대체 드론의 경로 비용/시간/안전도 차이는 무엇인가
- MPPI 복귀가 baseline 대비 어떤 이득을 냈는가

즉, 단순 성공/실패를 넘어 “의사결정 이벤트”를 기록해야 논문 가치가 생긴다.

## Bottom Line

현업 관점에서 `artifacts + experiments + docs/change`를 묶는 방식은 좋다.
다만 이 셋은 하나의 폴더로 합치라는 뜻이 아니라,
하나의 운영 체계로 보고 책임을 분리해서 관리하는 게 맞다.

- `artifacts/` = 증거
- `experiments/` = 비교와 집계
- `docs/change/` = 변경 이유와 판단 이력

이 구조가 자리 잡으면,
나중에 “이 결과가 왜 나왔는지”, “무엇을 고쳤는지”, “논문 그림이 어느 run에서 나온 것인지”를 모두 되짚을 수 있다.
