# 4종 구간 월드 따라가기 실험

작성일: 2026-10-06. 목적: 퇴화(단서 부족)와 착각(닮은 곳 반복)을 따로 떼어 보는 실험용 월드와 비행 설정.
기존 무작위 원통 월드에서는 두 점수가 거의 같은 정보였다(Spearman −0.91,
`experiments/2026-09-27_alias_predictor_offline/`). 이 월드는 그 둘을 구간 단위로 분리한다.

## 1. 월드

드론1 차선(world y = −7.5 m) 주변의 x = 15–135 m를 30 m 구간 4개로 나눈다.

| 구간 | 원통 / 30 m | 배치 규칙 | 의도 |
|---|---:|---|---|
| RU 원통 많음·유일 | 15 | 무작위, 원통 사이 2 m 이상 | 대조군 |
| RR 원통 많음·반복 | 14 | 원통 2개 묶음을 4 m마다 복사, 묶음마다 ±0.15 m 흔들기 | 퇴화 점수는 안전, 착각 위험 높음 (핵심) |
| SU 원통 적음·유일 | 6 | 무작위, 원통 사이 4 m 이상 | 퇴화 위험 |
| SR 원통 적음·반복 | 6 | 원통 1개를 5 m마다 복사, ±0.15 m 흔들기 | 퇴화 + 착각 |

- 반복 간격 4–5 m는 정합 탐색 범위(±6 m) 안에 닮은 곳이 들어오도록 정한 값이다.
- 완전 복사는 1·2등 점수가 같아 검사가 항상 거르므로, “거의 같은” 복사를 쓴다.
- 드론1 쪽 원통은 차선에서 옆으로 2.2–6.0 m 띠에만 놓는다. 표면까지 1.7 m 이상 떨어져
  `obstacle_stop_distance`(1.25 m)와 `gap_min_clearance`(1.6 m)보다 멀다.
- |y| < 1.5 m는 비우고, 드론2 쪽(y = 1.5–13.5 m)에는 드론2 SLAM용 무작위 원통 35개를 둔다.
- x < 10 m에는 원통이 없다. 드론2가 여기서 드론1 차선으로 건너간다.
- 벽·물리 설정·`gazebo_ros_state` 플러그인은 `random_cylinders_double.world`를 그대로 복사했다.

| 월드 | 15–45 m | 45–75 m | 75–105 m | 105–135 m | 용도 | 시드 |
|---|---|---|---|---|---|---:|
| `zoned_corridor_w1` | RU | RR | SU | SR | 설계용 | 101 |
| `zoned_corridor_w2` | RR | SR | RU | SU | 설계용 | 202 |
| `zoned_corridor_w3` | SU | RU | SR | RR | 평가용 | 303 |
| `zoned_corridor_w4` | SR | SU | RR | RU | 평가용 | 404 |

각 구간 종류가 네 위치에 한 번씩 온다(4×4 라틴 방진). 점수의 k·임계값 같은 설정은 w1·w2에서만
정하고 w3·w4로 평가한다.

### 생성과 사전 확인

```bash
cd /home3/deepblue/work/AV_Drone
python3 scripts/generate_zoned_corridor_world.py            # 월드 4개 생성
python3 scripts/generate_zoned_corridor_world.py --check    # + 가상 LiDAR 확인 (numpy 필요)
```

생성물: `sim_assets/worlds/zoned_corridor_w{1..4}.world`, `_positions.txt`, `_layout.json`.
사전 확인 결과: `experiments/2026-10-06_zoned_world_design_check/design_check.json`.

가상 LiDAR 확인(360빔·8 m, 앞뒤 5 m submap, 0.2 m voxel, 구간 경계 ±7 m 제외, 구간당 20개):

| 구간 | α 중앙값 | 착각 점수(±6 m) 중앙값 | 최소 점 수 |
|---|---:|---:|---:|
| RU | 0.31 | 0.58 | 185 |
| RR | 0.27 | 0.94 | 196 |
| SU | 0.15 | 0.76 | 144 |
| SR | 0.12 | 0.95 | 143 |

RR은 α가 RU와 비슷한데 착각 점수만 높다. 모든 submap이 최소 점 수(80)를 넘는다.
가상 스캔이므로 실제 비행 submap과 값이 다를 수 있다.

## 2. 비행 설정 (따라가기)

`src/drone_bringup/config/swarm_two_uav_zoned_trailing_w{1..4}.yaml`

- 드론1: (3, −7.5) 출발, world y = −7.5 m 차선으로 직진. 목표에서는 차선 밖 (146, −3)에 정지.
- 드론2: (3, +7.5) 이륙 후 30초 대기, (9, −7.5)로 건너가 드론1 차선을 따라감. (142, −7.5)에 정지.
- 경로에서 원통 표면까지 최소 1.75 m.
- 정합·검사·pose graph 설정(`src/drone_cslam/config/`), 기체 모델, 회피 설정은 바꾸지 않았다.
- 경로와 월드 배치는 LiDAR 후보 검색·정합·pose graph에 전달하지 않는다. 구간 이름은 오프라인 채점에만 쓴다.

## 3. 실행

실행기는 sim 컨테이너의 `PX4_SITL_WORLD`·기체 수·spawn이 설정 파일과 같아야 시작한다.
2026-10-06 기준 `av_drone-sim-1`은 `PX4_SITL_WORLD=donut`, 1대 설정이고 `av_drone-ros-1`은 실행 중이다.
따라서 아래 순서가 필요하다. **컨테이너 재생성은 직접 확인 후 실행한다.** sim의 `/root`는
`av_drone_sim-home` volume이라 재생성해도 남지만, 다른 작업이 이 컨테이너를 쓰고 있지 않은지 먼저 확인한다.

```bash
cd /home3/deepblue/work/AV_Drone

# 1) sim을 월드 w1·2대 설정으로 다시 만든다 (시작하지 않음). DISPLAY 등은 평소 값 사용.
PX4_SITL_WORLD=zoned_corridor_w1 VEHICLE_COUNT=2 PX4_INSTANCE_BASE=2 \
  docker compose up --no-start --force-recreate sim

# 2) 실행기는 멈춘 ros 컨테이너를 요구한다
docker stop av_drone-ros-1

# 3) 검사만 (컨테이너를 시작하지 않음)
python3 scripts/run_guarded_experiment.py \
  --manifest src/drone_bringup/config/swarm_two_uav_zoned_trailing_w1.yaml --check

# 4) 비행 (tmux 안에서 권장)
python3 scripts/run_guarded_experiment.py \
  --manifest src/drone_bringup/config/swarm_two_uav_zoned_trailing_w1.yaml
```

w2–w4는 1)의 `PX4_SITL_WORLD`와 3)·4)의 설정 파일 이름만 바꾼다. 계획: 월드당 2회, 총 8회.

### 첫 비행에서 확인할 것

- 두 드론 모두 `HOVER_AT_GOAL`, rosbag 정상 종료
- 드론2가 x < 15 m에서 드론1 차선에 들어왔는지, 이후 두 드론 간격이 10 m 이상인지
- 원통 근처에서 회피 기동·정지가 없었는지 (최소 LiDAR 거리)
- 반복 구간에서 각 드론의 자기 SLAM이 크게 튀지 않았는지 (B0 궤적)

## 4. 다음 단계

비행 후 `offline_lidar_eval`로 드론 간 후보 정합을 만들고, 구간 종류별로
채택·정답 / 탈락 / 채택·오답을 센다. 같은 쌍에 초기값 오차 σ를 넣어 다시 정합하는 분석과
±kσ 착각 점수의 AuROC 비교는 `experiments/2026-09-27_alias_predictor_offline/analyze.py`를 확장해 진행한다.
