# Experiment Tracking

이 디렉터리는 자동 실험 추적 결과를 모아두는 위치다.

메모:

- 실행 중 생성되는 원본 지표와, 사람이 편집하는 표시용 메타데이터를 분리한다.
- `dashboard_catalog.json`: 실험 표시 이름·목적·메모·보관 상태. 편집 전 값은 `dashboard_catalog_history/`에 백업된다.
- `index.csv`, `index.md`, `scenario_table.csv`, `scenario_table.md`, `ledger.csv`, `ledger.md`, `plots/`는 실행 후 생성되는 generated output이므로 `.gitignore`에 포함한다.

주요 파일:

- `index.csv`: 실행별 요약 장부. baseline, planner version, seed, failure_code, snapshot 경로까지 포함한다.
- `index.md`: `index.csv`를 사람이 보기 쉽게 변환한 표
- `scenario_table.csv`: 시나리오별 pass/fail 집계표
- `scenario_table.md`: 시나리오별 집계표의 Markdown 뷰
- `ledger.csv`: `문제 -> 수정 -> 재실행 -> 결과`를 한 줄로 남기는 실험 ledger. failure_code와 baseline도 함께 남긴다.
- `ledger.md`: ledger의 Markdown 뷰
- `paper_outputs/`: 논문용 집계표와 비교 figure를 생성하는 위치

조회 도구:

- `scripts/quant_dashboard.py`: 실험별 조회와 이름·목적 편집을 제공한다. 측정 데이터는 읽기 전용이며 라벨/보관 상태만 저장한다.
- 기본 화면: `실험 보기` / `이름·보관 관리`. 사용법은 [실험 목록·라벨 관리](../docs/dashboard_experiment_management.md) 참고.
- `실험 해석 가이드`: [B0/B1·GT·보정 조건과 그래프 해석 README](../docs/slam_experiment_guide.md)를 화면에서 읽거나 다운로드한다.
- `Oracle Drift Correction` 탭: `artifacts/<RUN_ID>/oracle_correction`을 읽어
  B0/B1/O-single/O-periodic 상대 drift, trajectory, map, factor와 gate를 비교한다.
- `LiDAR Registration` 탭: `artifacts/<RUN_ID>/lidar_registration`을 읽어
  GT-associated keyframe pair의 LiDAR 정합 전/후 오차, overlap, confidence gate,
  R-single/R-periodic trajectory와 map을 비교한다.
- 위 기존 탭은 `기존 상세 분석 도구`에 유지한다. 새 화면의 `LiDAR 상세`는 GT 평가 전용 N-single과 과거 GT-associated 조건을 명확히 구분한다.

권장 사용 흐름:

1. 드론 실행
2. `./scripts/smoke_test_single_drone.sh`
3. 최신 artifact 아래 `plots/` 자동 생성 확인
4. `experiments/index.csv`, `experiments/scenario_table.csv`, `experiments/ledger.csv` 확인

대시보드로 확인:

```bash
python3 -m pip install --target .dashboard-deps -r requirements-dashboard.txt
./scripts/run_quant_dashboard.sh
```

직접 실행하고 싶으면:

```bash
streamlit run scripts/quant_dashboard.py -- --repo-root .
```

Streamlit 설치 없이 데이터 스캔만 검증:

```bash
python3 scripts/quant_dashboard.py --check-data --repo-root .
```

추가 메모:

- smoke test에서 `--issue`, `--fix`, `--notes`, `--scenario`를 주면 ledger에 함께 저장된다.
- artifact에는 `parameter_snapshot.json`과 `config_snapshots/`가 함께 남아 재현성 근거를 보강한다.
- registry update는 `--failure-code` override를 받을 수 있지만, 기본은 artifact summary의 `failure_code`를 사용한다.
- 기존 artifact를 다시 스캔해서 장부를 재생성하려면 `python3 scripts/update_experiment_registry.py --scan-artifacts artifacts`를 사용한다.
- Streamlit의 이름 편집과 보관은 원본 숫자를 변경하지 않는다. 논문 숫자의 원본은 `paper_metrics.json`, 분석 `metrics.json`, `summary_table.csv`, `figure_manifest.csv`로 둔다.
