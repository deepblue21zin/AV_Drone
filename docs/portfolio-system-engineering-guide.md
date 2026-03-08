# Portfolio System Engineering Guide

이 문서는 드론, 로봇, 자율주행, 임베디드 프로젝트를 취업용 포트폴리오로 발전시킬 때 어떤 식으로 설계하고 무엇을 증명해야 하는지 정리한 재사용 가능한 가이드다.

이 문서는 특정 프로젝트 전용 문서가 아니라, 다른 프로젝트에도 그대로 적용할 수 있는 원칙 문서다.

## 1. 포트폴리오 프로젝트의 목표를 다시 정의해야 한다

취업용 프로젝트는 단순히 "동작한다"로 끝나면 약하다.  
현업 관점에서는 다음을 보여줘야 한다.

- 재현 가능한 실행 환경
- 정량적으로 측정된 성능
- 실패 상황에서의 안전 동작
- 반복 실험과 자동 평가 체계
- 구조화된 로그와 결과물
- 설계 의사결정의 근거

즉, 좋은 프로젝트의 목표는 다음처럼 바뀌어야 한다.

- 나쁜 목표: 데모를 성공시킨다
- 좋은 목표: 데모를 재현 가능하게 만들고, 성능과 안정성을 수치로 증명한다

## 2. 반드시 수집해야 하는 데이터

다음 데이터는 거의 모든 자율 시스템 프로젝트에서 중요하다.

### 실행 메타데이터

- 실행 시각
- git commit hash
- branch
- 실험 ID
- 시나리오 이름
- 파라미터 파일
- 난수 시드
- 실행 환경 정보

### 성능 데이터

- 성공/실패 여부
- 목표 도달 시간
- 충돌 여부
- 최소 장애물 거리
- 경로 길이
- 평균 속도
- 미션 완료율

### 실시간성 데이터

- loop 주기 평균
- loop 주기 p95
- loop 주기 p99
- loop 주기 worst-case
- callback 처리 시간 평균
- callback 처리 시간 p99
- missed deadline count

### 시스템 자원 데이터

- CPU 사용률
- 메모리 RSS
- peak memory
- 컨테이너 자원 사용량

### 안전 데이터

- fail-safe trigger 횟수
- trigger 사유
- recovery 횟수
- recovery 성공/실패
- emergency stop 발생 여부

## 3. 프로젝트가 반드시 남겨야 하는 산출물

한 번 실험할 때 아래 산출물이 자동 생성되도록 만드는 것이 좋다.

```text
artifacts/
└─ run_2026-03-08_153000/
   ├─ metadata.json
   ├─ metrics.csv
   ├─ summary.json
   ├─ events.log
   ├─ rosbag/
   └─ plots/
      ├─ trajectory.png
      ├─ latency_histogram.png
      └─ resource_usage.png
```

권장 포맷:

- `JSON`: 구조화된 결과
- `CSV`: 표와 통계 처리
- `ROS bag`: 원시 증거
- `PNG/PDF`: 면접용 시각 자료

## 4. 면접에서 강한 지표

현업에서 설득력 있는 지표는 보통 아래다.

### 기능 성능

- 성공률
- 충돌률
- 목표 도달 시간
- 목표 도달 거리 오차
- 최소 안전 거리

### 실시간성

- planner latency mean / p99 / max
- perception latency mean / p99 / max
- control loop jitter
- worst-case response time

### 안정성

- 100회 반복 실험 성공률
- 노이즈 환경에서 성공률
- 장애물 배치 변경 시 성공률
- 센서 drop 상황에서 recovery 성공률

### 안전성

- sensor timeout 감지 시간
- emergency brake 반응 시간
- stale command fallback 성공 여부
- fail-safe 이후 safe state 진입 시간

이 중 `p99`, `max`, `worst-case`, `100회 반복`, `recovery rate`는 특히 강하다.

## 5. fail-safe는 필수다

자율 시스템 프로젝트에서는 정상 동작보다 실패 시 동작이 더 중요할 수 있다.

최소한 아래 상황은 감시해야 한다.

- pose timeout
- sensor timeout
- planner command timeout
- autopilot disconnected
- obstacle too close
- invalid state estimation

Fail-safe 설계 원칙:

- 감지 조건이 명확해야 한다
- 대응 동작이 명확해야 한다
- 로그로 남아야 한다
- recovery 조건도 정의되어야 한다

예시:

```text
condition: pose update > 0.5s stale
action: publish zero command
log: [SAFETY] pose timeout triggered
recovery: pose stream stable for 1.0s
```

## 6. 자동화가 있어야 한다

현업 수준으로 보이려면 반복 실험 자동화가 필요하다.

필수 자동화:

- 실험 N회 반복 실행
- 결과 자동 저장
- 실패 케이스 자동 분류
- summary table 자동 생성
- 그래프 자동 생성

좋은 인터페이스 예:

```bash
ros2 run my_metrics experiment_runner --runs 100 --scenario corridor --seed 42
```

좋은 출력 예:

```text
Scenario: corridor_static
Runs: 100
Success rate: 96%
Collision rate: 2%
Timeout rate: 2%
Mean time-to-goal: 18.4s
P95 time-to-goal: 22.1s
Min obstacle distance: 0.82m
Planner latency mean/p99/max: 8.1 / 14.7 / 18.2 ms
Failsafe triggers: 7
Recovery success: 6/7
```

## 7. 구조를 이렇게 나누면 좋다

대부분의 자율 시스템 프로젝트는 아래 구조로 나누면 관리가 쉽다.

```text
bringup/
control/
planning/
perception/
safety/
metrics/
interfaces/
simulation/
```

역할:

- `bringup`: launch, config, scenario
- `control`: actuator/autopilot interface
- `planning`: planning and decision making
- `perception`: sensor processing
- `safety`: watchdog and fail-safe
- `metrics`: logging and evaluation
- `interfaces`: shared msg/srv/action
- `simulation`: world/model/sensor

## 8. 프로젝트 설명에서 좋은 스토리라인

면접에서 좋은 설명은 다음 구조다.

1. 문제 정의
2. 시스템 아키텍처
3. 핵심 구현
4. 정량 평가
5. 실패 대응
6. 개선 방향

예시:

- Docker로 시뮬레이션 환경을 재현 가능하게 만들었다
- perception/planning/control을 분리했다
- 센서 기반 장애물 회피를 구현했다
- 100회 자동 실험으로 성공률과 p99 latency를 측정했다
- sensor timeout과 planner timeout에 대한 fail-safe를 설계했다
- 멀티드론 확장을 고려한 namespace 구조를 도입했다

## 9. 다른 프로젝트에도 적용되는 체크리스트

### 구조

- 실행 환경이 고정되어 있는가
- 패키지 책임이 분리되어 있는가
- 설정 파일이 코드와 분리되어 있는가

### 측정

- 핵심 성능 지표가 정의되어 있는가
- p99 / worst-case를 측정하는가
- 실험 결과가 파일로 남는가

### 안정성

- timeout 감시가 있는가
- safe fallback이 있는가
- 로그와 이벤트 기록이 남는가

### 자동화

- 반복 실험이 가능한가
- summary가 자동으로 생성되는가
- 결과 그래프를 자동 생성할 수 있는가

## 10. 결론

취업용 프로젝트는 기능보다 시스템 엔지니어링 역량을 보여줘야 한다.  
즉, "만들었다"보다 아래를 증명해야 한다.

- 어떻게 구조화했는가
- 어떻게 측정했는가
- 어떻게 실패에 대비했는가
- 어떻게 반복 검증했는가

이 문서의 내용을 기준으로 프로젝트를 설계하면, 방산/로봇/자율주행/임베디드 계열 포트폴리오에서 설득력이 훨씬 높아진다.
