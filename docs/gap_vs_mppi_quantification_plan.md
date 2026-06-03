# Gap Return vs SLAM-MPPI Return Quantification Plan

## 1. Research Question

연구 질문:

> LiDAR 기반 Gap 회피로 목표점까지 이동하며 얻은 환경 정보를 이용해, 복귀 단계에서 MPPI를 적용하면 단순 Gap 기반 복귀보다 더 효율적이고 안정적인가?

비교는 아래처럼 고정한다.

```text
공통 outbound:
이륙 -> 목표점 이동 = 현재 LiDAR Gap 회피 알고리즘

비교 A:
목표점 도달 -> Gap 회피 방식으로 복귀

비교 B:
목표점 도달 -> SLAM/지도 기반 MPPI로 복귀
```

이 구조를 쓰는 이유는 outbound 조건을 같게 만들어야 return planner 차이만 분리해서 볼 수 있기 때문이다.

## 2. Experiment Conditions

기본 조건은 아래 2개다.

| condition_id | outbound | mapping | return | 목적 |
| --- | --- | --- | --- | --- |
| `gap_return_baseline` | Gap avoidance | optional logging only | Gap avoidance | 현재 알고리즘 기준선 |
| `slam_mppi_return` | Gap avoidance | enabled | MPPI return | 제안 방식 |

world 조건은 최소 3개로 둔다.

| scenario_id | world | 목적 |
| --- | --- | --- |
| `static_obstacle_world` | `obstacle_demo` | 기본 장애물 회피 검증 |
| `random_corridor_world` | `random_corridor_generated` | 긴 corridor에서 반복 실험 |
| `partial_block_return_world` | 추후 추가 | 복귀 경로 일부 차단 상황 |

초기 반복 규칙:

```text
conditions 2개 x scenarios 2개 x seeds 5개 = 최소 20 runs
```

논문 제출 전 권장 규칙:

```text
conditions 2개 x scenarios 3개 x seeds 10개 = 60 runs 이상
```

## 3. Database Strategy

초기에는 별도 DB 서버를 두지 않고 file-based database로 간다.

이유:

- Git repo와 artifact 폴더만 있으면 재현 가능하다.
- 학부 논문/학술대회 수준에서는 CSV/JSON 기반 장부가 검증과 공유에 더 단순하다.
- 나중에 run 수가 많아지면 같은 스키마를 SQLite로 옮기면 된다.

데이터는 4계층으로 나눈다.

```text
raw layer:
  rosbag2, docker logs, ROS launch logs

run artifact layer:
  metrics.csv, trajectory.csv, summary.json, paper_metrics.json

experiment registry layer:
  experiments/index.csv, ledger.csv, scenario_table.csv

paper output layer:
  summary_table.csv, figures, figure_manifest.csv

visualization layer:
  Streamlit dashboard, ros_states dashboard
```

Streamlit 사용 원칙:

```text
Streamlit은 데이터를 생성하거나 수정하지 않는다.
Streamlit은 artifacts/와 experiments/를 읽어서 비교, 필터링, 추적만 한다.
논문 숫자의 원본은 항상 paper_metrics.json, summary_table.csv, figure_manifest.csv다.
```

## 4. Run Artifact Schema

실행 1회는 반드시 `run_id` 하나를 가진다.

```text
artifacts/<run_id>/
  metadata.json
  parameter_snapshot.json
  config_snapshots/
  metrics.csv
  trajectory.csv
  events.log
  planner_debug.jsonl
  slam_summary.json
  phase_summary.json
  paper_metrics.json
  summary.json
  rosbag2/
  logs/
```

`metadata.json` 필수 필드:

| field | 설명 |
| --- | --- |
| `run_id` | 실행 고유 ID |
| `started_at` | 실행 시작 시각 |
| `git_commit` | 실행 당시 commit hash |
| `git_branch` | 실행 당시 branch |
| `git_dirty` | 수정 미커밋 여부 |
| `scenario_id` | 실험 시나리오 ID |
| `condition_id` | 비교 조건 ID |
| `world_name` | Gazebo world |
| `seed` | 실험 seed |
| `return_mode` | `avoid` 또는 `mppi` |
| `outbound_mode` | 기본 `avoid` |
| `mapping_enabled` | SLAM/mapping 사용 여부 |
| `rosbag_path` | raw replay bag 경로 |
| `config_snapshot_dir` | 실행 당시 config snapshot |

## 5. ROS Bag Recording Contract

논문용 원자료로 남길 최소 topic:

```text
/mavros/state
/mavros/local_position/pose
/drone1/scan
/drone1/perception/nearest_obstacle_distance
/drone1/autonomy/cmd_vel
/drone1/safety/cmd_vel
/drone1/mission/phase
/drone1/mission/goal_reached
/drone1/safety/event
/drone1/planner/avoid/debug/selected_gap
/drone1/planner/avoid/debug/score_terms
/drone1/planner/avoid/debug/mode
/drone1/planner/avoid/debug/escape_active
/drone1/slam/status
/drone1/slam/input_ready
/drone1/slam/map_ready
/drone1/slam/localization_ok
/drone1/slam/coverage
/map
```

MPPI 쪽 topic이 확정되면 아래도 추가한다.

```text
/drone1/mppi/cmd_vel
/drone1/mppi/debug/cost
/drone1/mppi/debug/best_trajectory
/drone1/mppi/debug/sample_count
```

## 6. Registry Tables

현재 `experiments/index.csv`를 중심 테이블로 유지하고 필드를 확장한다.

### 6-1. `experiments/index.csv`

실행별 대표 row.

추가해야 할 필드:

| field | 설명 |
| --- | --- |
| `condition_id` | `gap_return_baseline`, `slam_mppi_return` |
| `scenario_id` | `static_obstacle_world`, `random_corridor_world` |
| `world_name` | Gazebo world |
| `seed` | 반복 실험 seed |
| `outbound_mode` | `avoid` |
| `return_mode` | `avoid` 또는 `mppi` |
| `mapping_enabled` | true/false |
| `rosbag_path` | replay 원자료 |
| `paper_metrics_path` | 논문 지표 JSON |
| `figure_dir` | run별 figure 경로 |

### 6-2. `experiments/ledger.csv`

문제, 수정, 재실행 결과를 남긴다.

필수 운영 규칙:

```text
실패 run도 삭제하지 않는다.
실패 원인과 수정 내용을 ledger에 남긴다.
같은 issue가 반복되면 동일 failure_code로 묶는다.
```

### 6-3. `experiments/paper_outputs/summary_table.csv`

논문 표에 직접 들어갈 집계 결과.

| field | 설명 |
| --- | --- |
| `scenario_id` | 시나리오 |
| `condition_id` | 비교 조건 |
| `metric` | 지표 이름 |
| `runs` | 전체 반복 횟수 |
| `valid` | 유효 run 수 |
| `mean` | 평균 |
| `std` | 표준편차 |
| `min` | 최소 |
| `max` | 최대 |

### 6-4. `experiments/paper_outputs/figure_manifest.csv`

논문 그림과 artifact 연결.

| field | 설명 |
| --- | --- |
| `figure_id` | 예: `fig_trajectory_overlay_random_corridor` |
| `figure_path` | 생성된 그림 경로 |
| `source_run_ids` | 사용된 run_id 목록 |
| `scenario_id` | 시나리오 |
| `condition_id` | 조건 |
| `script` | 생성 스크립트 |
| `notes` | 그림 해석 메모 |

## 7. Paper Metrics Definition

논문에 쓸 핵심 지표는 `paper_metrics.json`에 저장한다.

| metric | 정의 | 방향 |
| --- | --- | --- |
| `return_success` | 복귀 성공 여부 | 높을수록 좋음 |
| `return_success_rate` | 조건별 복귀 성공률 | 높을수록 좋음 |
| `return_time_s` | 복귀 시작부터 home 도달까지 시간 | 낮을수록 좋음 |
| `return_path_length_m` | 복귀 구간 실제 이동거리 | 낮을수록 좋음 |
| `return_path_efficiency` | 직선 복귀거리 / 실제 복귀거리 | 높을수록 좋음 |
| `min_obstacle_distance_m` | 전체 비행 중 장애물 최소 거리 | 높을수록 좋음 |
| `return_min_obstacle_distance_m` | 복귀 중 장애물 최소 거리 | 높을수록 좋음 |
| `safety_intervention_count` | safety 개입 횟수 | 낮을수록 좋음 |
| `control_effort` | 명령 입력 변화량 또는 제어 에너지 | 낮을수록 좋음 |
| `escape_count` | escape mode 진입 횟수 | 낮을수록 좋음 |
| `pose_period_p99_s` | pose topic p99 period | 낮을수록 안정 |
| `scan_period_p99_s` | scan topic p99 period | 낮을수록 안정 |
| `map_coverage` | mapping coverage | 높을수록 좋음 |
| `localization_ok_rate` | localization OK 비율 | 높을수록 좋음 |

`return_path_efficiency` 계산:

```text
return_path_efficiency = straight_line_home_distance_m / return_path_length_m
```

`localization_ok_rate` 계산:

```text
localization_ok_rate = localization_ok_true_samples / localization_ok_total_samples
```

## 8. Code Modification Plan

### P0. Config 분리

대상:

- `src/drone_bringup/config/drone1_autonomy.yaml`
- `src/drone_bringup/config/drone1_mppi_known_world.yaml`
- `src/drone_bringup/config/drone1_avoidance_dev.yaml`
- `experiments/experiment_matrix.yaml`

해야 할 일:

```text
return_mode: avoid | mppi
outbound_mode: avoid
mapping_enabled: true | false
condition_id: gap_return_baseline | slam_mppi_return
scenario_id: static_obstacle_world | random_corridor_world
experiment_seed: int
world_name: obstacle_demo | random_corridor_generated
```

목표:

```text
같은 launch 구조에서 config만 바꿔 A/B 실험을 돌릴 수 있게 한다.
```

### P1. `metrics_logger_node.py` 보강

대상:

- `src/drone_metrics/drone_metrics/metrics_logger_node.py`

추가해야 할 내부 상태:

```text
return_started_at
return_completed_at
return_start_pose
return_end_pose
return_path_length_m
return_min_obstacle_distance_m
localization_ok_samples
localization_ok_true_samples
map_coverage_samples
rosbag_path
condition_id
scenario_id
world_name
```

추가해야 할 `paper_metrics.json` 필드:

```text
return_success
return_time_s
return_path_length_m
return_path_efficiency
return_min_obstacle_distance_m
localization_ok_rate
map_coverage_mean
map_coverage_final
rosbag_path
```

주의:

```text
outbound path와 return path를 phase 기준으로 분리해야 한다.
FOLLOW_PLAN, MAPPING_TO_GOAL = outbound
RETURN_HOME_AVOID, RETURN_HOME_MPPI = return
```

### P1. Quantification Runner 추가

새 스크립트:

```text
scripts/run_quant_experiment.sh
```

역할:

```text
1. git 상태 기록
2. PX4_SITL_WORLD 설정
3. docker compose sim/ros 실행
4. ROS workspace build
5. autonomy launch 실행
6. rosbag2 record 시작
7. goal/return 완료 또는 timeout까지 대기
8. rosbag 종료
9. plot 생성
10. experiments registry 업데이트
```

예상 사용법:

```bash
./scripts/run_quant_experiment.sh \
  --scenario random_corridor_world \
  --world random_corridor_generated \
  --condition gap_return_baseline \
  --return-mode avoid \
  --seed 0 \
  --timeout 240

./scripts/run_quant_experiment.sh \
  --scenario random_corridor_world \
  --world random_corridor_generated \
  --condition slam_mppi_return \
  --return-mode mppi \
  --seed 0 \
  --timeout 240
```

### P1. Experiment Matrix Runner 추가

새 스크립트:

```text
scripts/run_experiment_matrix.py
```

입력:

```text
experiments/experiment_matrix.yaml
```

역할:

```text
condition x scenario x seed 조합을 자동 실행한다.
각 run 결과를 experiments/index.csv에 누적한다.
중간 실패가 있어도 다음 seed로 넘어갈 수 있게 한다.
```

### P1. Rosbag replay 추가

새 스크립트:

```text
scripts/replay_quant_run.sh
```

사용법:

```bash
./scripts/replay_quant_run.sh artifacts/<run_id>
```

역할:

```text
ros2 bag play artifacts/<run_id>/rosbag2 --clock
ros_states launch
run_id 기준 report 연결
```

목표:

```text
논문 그림의 근거가 되는 run을 언제든 다시 보여줄 수 있게 한다.
```

### P2. Plot generation 보강

대상:

- `scripts/generate_artifact_plots.py`
- `scripts/generate_paper_figures.py`
- `scripts/analyze_experiment_batch.py`

추가해야 할 그림:

```text
condition별 return_success_rate bar
condition별 return_time_s bar 또는 box
condition별 return_path_length_m bar 또는 box
condition별 return_path_efficiency bar
condition별 min_obstacle_distance_m bar
trajectory overlay
return trajectory overlay
mission phase timeline
failure_code distribution
```

논문용 그림은 반드시 `figure_manifest.csv`에 source run을 남긴다.

### P2. `ros_states` 보강

대상:

- `src/ros_states/ros_states/ros_monitor.py`
- `src/ros_states/templates/index.html`

추가하면 좋은 패널:

```text
current condition_id
return_mode
mapping status
MPPI status
return progress
latest artifact path
latest rosbag path
paper_metrics preview
```

역할:

```text
실험 중 상태 확인과 실패 원인 확인용이다.
논문 원자료 저장은 rosbag2와 artifacts가 담당한다.
```

### P2. Streamlit quantification dashboard

새 스크립트:

```text
scripts/quant_dashboard.py
```

역할:

```text
experiments/index.csv, ledger.csv, scenario_table.csv를 한 화면에서 본다.
artifacts/*/paper_metrics.json과 summary.json을 자동 스캔한다.
condition/scenario/result/run_id로 필터링한다.
Gap return baseline과 SLAM-MPPI return의 평균 지표를 비교한다.
선택한 run의 artifact 경로와 paper_metrics를 바로 확인한다.
```

실행:

```bash
python3 -m pip install -r requirements-dashboard.txt
./scripts/run_quant_dashboard.sh
```

직접 실행:

```bash
streamlit run scripts/quant_dashboard.py -- --repo-root .
```

검증:

```bash
python3 scripts/quant_dashboard.py --check-data --repo-root .
```

운영 원칙:

```text
Streamlit dashboard는 read-only다.
데이터 생성은 run_quant_experiment.sh, metrics_logger, update_experiment_registry.py가 담당한다.
논문에 들어갈 최종 숫자는 dashboard에서 직접 복사하지 말고 summary_table.csv에서 가져온다.
dashboard는 실험 탐색과 교수/팀원 설명용으로 쓴다.
```

## 9. Recommended Implementation Order

1. `experiments/experiment_matrix.yaml`를 연구 조건 기준으로 재정의한다.
2. `metrics_logger_node.py`에 return 구간 지표를 추가한다.
3. `run_quant_experiment.sh`를 만들어 rosbag2와 artifact 생성을 자동화한다.
4. `update_experiment_registry.py`에 condition/scenario/world/rosbag 필드를 추가한다.
5. `generate_paper_figures.py`에 condition 비교 그래프를 추가한다.
6. `replay_quant_run.sh`를 만들어 artifact 단위 replay를 지원한다.
7. `quant_dashboard.py`로 생성된 결과를 한 화면에서 검토한다.
8. 최소 2 conditions x 2 scenarios x 5 seeds를 돌려 첫 비교표를 만든다.

## 10. Done Criteria

이 quantification 환경은 아래가 되면 1차 완료로 본다.

```text
한 명령으로 gap_return_baseline run을 실행할 수 있다.
한 명령으로 slam_mppi_return run을 실행할 수 있다.
각 run이 artifact와 rosbag2를 남긴다.
paper_metrics.json에 return 지표가 들어간다.
experiments/index.csv에 condition/scenario/seed/rosbag이 누적된다.
summary_table.csv에 조건별 평균과 표준편차가 생성된다.
trajectory overlay와 return 지표 비교 그래프가 자동 생성된다.
선택한 run을 replay해서 ros_states로 확인할 수 있다.
Streamlit dashboard에서 condition별 결과를 필터링하고 artifact까지 역추적할 수 있다.
```

## 11. Paper Claim Mapping

논문 주장과 데이터 연결은 아래처럼 관리한다.

| claim | 필요한 데이터 | 그림/표 |
| --- | --- | --- |
| MPPI return이 Gap return보다 복귀 시간이 짧다 | `return_time_s` | condition별 bar/box |
| MPPI return이 경로 길이를 줄인다 | `return_path_length_m`, `return_path_efficiency` | return path 비교 |
| MPPI return이 안전성을 유지한다 | `min_obstacle_distance_m`, `safety_intervention_count` | safety metric table |
| SLAM 정보가 복귀 최적화에 쓰인다 | `map_coverage`, `/map`, `localization_ok_rate` | map coverage timeline |
| 결과가 재현 가능하다 | `git_commit`, `seed`, `rosbag_path`, `config_snapshots` | artifact manifest |

핵심 운영 원칙:

```text
논문에 들어가는 모든 숫자는 paper_metrics.json 또는 summary_table.csv에서만 가져온다.
논문에 들어가는 모든 그림은 figure_manifest.csv를 통해 run_id로 역추적 가능해야 한다.
```
