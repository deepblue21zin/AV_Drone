# Experiment Tracking

이 디렉터리는 자동 실험 추적 결과를 모아두는 위치다.

메모:

- `README.md`만 저장소에 포함한다.
- `index.csv`, `index.md`, `scenario_table.csv`, `scenario_table.md`, `ledger.csv`, `ledger.md`, `plots/`는 실행 후 생성되는 generated output이므로 `.gitignore`에 포함한다.

주요 파일:

- `index.csv`: 실행별 요약 장부
- `index.md`: `index.csv`를 사람이 보기 쉽게 변환한 표
- `scenario_table.csv`: 시나리오별 pass/fail 집계표
- `scenario_table.md`: 시나리오별 집계표의 Markdown 뷰
- `ledger.csv`: `문제 -> 수정 -> 재실행 -> 결과`를 한 줄로 남기는 실험 ledger
- `ledger.md`: ledger의 Markdown 뷰

권장 사용 흐름:

1. 드론 실행
2. `./scripts/smoke_test_single_drone.sh`
3. 최신 artifact 아래 `plots/` 자동 생성 확인
4. `experiments/index.csv`, `experiments/scenario_table.csv`, `experiments/ledger.csv` 확인

추가 메모:

- smoke test에서 `--issue`, `--fix`, `--notes`, `--scenario`를 주면 ledger에 함께 저장된다.
- 기존 artifact를 다시 스캔해서 장부를 재생성하려면 `python3 scripts/update_experiment_registry.py --scan-artifacts artifacts`를 사용한다.
