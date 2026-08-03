# AV_Drone 프로젝트 상세 현황 및 발전 방향

> 기준 시각: 2026-08-03 17:10 KST
> 기준 브랜치: `feature/multi-uav-map-fusion`
> 기준 커밋: `0ecd3a2` (`Implement two-UAV known-pose map fusion`)
> 조사 범위: 저장소 소스·설정·Git 변경 상태·실행 중 Docker/ROS 그래프·누적 artifact·실험 집계·로컬 테스트

이 문서는 “계획상 무엇을 하려는가”만 적은 문서가 아니다. 현재 저장소에 실제로 들어 있는 코드, 아직 커밋되지 않은 수정, 지금 실행 중인 프로세스, 과거 실행 결과를 서로 대조해 다음 네 가지를 구분한다.

- **구현 완료**: 코드가 있고 최소 단위 검증 또는 실행 근거가 있다.
- **부분 완료**: 구성 요소는 있으나 전체 임무 성공까지 확인되지 않았다.
- **수정 중**: working tree에만 존재하며 아직 기준 커밋으로 고정되지 않았다.
- **미구현/미검증**: 문서의 장기 목표에는 있으나 현재 연구 baseline으로 주장할 수 없다.

---

## 1. 한눈에 보는 결론

현재 AV_Drone은 더 이상 단순한 단일 드론 예제만 있는 저장소는 아니다. 단일 UAV의 PX4 SITL–MAVROS–LiDAR–회피–안전–제어–metrics 파이프라인을 기반으로, **2대 UAV의 독립 PX4/MAVROS 실행, 드론별 지도 생성, known-pose 기반 중앙 OccupancyGrid 융합, source timeout 시 local-only fallback**까지 코드가 확장되어 있다.

그러나 현재를 “2대 드론 협업 자율주행 완성” 상태라고 부르면 안 된다. 가장 정확한 표현은 다음과 같다.

> **2-UAV known-pose map-fusion 연구용 plumbing과 장시간 융합 실행 근거는 확보했지만, 실제 SLAM localization의 신뢰성, 두 기체의 정상 임무 완주, global map을 사용하는 A*–MPPI 연결, 드론 간 충돌 회피, 4대 확장은 아직 완료되지 않았다.**

현재 가장 큰 차이는 “맵이 합쳐지는가”와 “합쳐진 맵을 이용해 전체 임무를 안정적으로 성공하는가” 사이에 있다. 전자는 상당 부분 구현되었고 artifact도 남아 있다. 후자는 최신 장시간 실행에서 두 기체 모두 정상 성공으로 끝나지 않았으므로 아직 연구 과제다.

### 현재 성숙도 요약

| 영역 | 상태 | 근거 및 해석 |
|---|---|---|
| 단일 UAV Gazebo Classic/PX4 baseline | 구현 완료에 가까움 | 기존 문서와 과거 artifact에서 scan, pose, OFFBOARD, goal hover, logging 확인 |
| LiDAR 기반 reactive avoidance | 구현 및 반복 수정 중 | `local_planner_node.py`가 실제 LaserScan을 사용하며 현재도 gap 탐색 수정이 working tree에 존재 |
| MPPI/A* 실험 도구 | 부분 완료 | 패키지와 정량화 도구가 있고 2026-07 실험 결과가 있으나 표본이 조건당 1회인 결과도 많음 |
| 2-UAV PX4/MAVROS 분리 | 부분 완료 | instance/SYSID/port/namespace 설계와 실행 스크립트가 있으나 현재 launcher 교체가 미커밋 상태 |
| 드론별 local map | 부분 완료 | known-pose mapper와 `slam_toolbox`가 병렬 실행되도록 구성됨 |
| known-pose map fusion | 구현 및 실행 근거 있음 | `drone_map_fusion`, 단위 테스트 4개 통과, 장시간 `HEALTHY` artifact 존재 |
| 실제 scan-matching SLAM | 불안정/미검증 | 최신 장시간 artifact에서 두 기체 모두 `map_ready=true`이지만 `localization_ok=false` |
| fusion fallback | 코드 구현, 제한적 실행 근거 | stale source 제외 및 `LOCAL_ONLY_FALLBACK` 상태 구현, 일부 artifact에 fallback 1회 기록 |
| global map 기반 A* | 미연결 | A* 관련 코드는 있으나 `/swarm/global_map` 소비 파이프라인은 현재 launch에 없음 |
| Global A* + Local MPPI | 미구현 | 설계 문서의 다음 핵심 Gate |
| unknown-pose registration | 미구현 | known spawn transform만 사용 |
| peer trajectory 충돌 회피 | 미구현 | 다른 기체를 시간축 동적 장애물로 처리하지 않음 |
| 4-UAV 확장 | 미구현 | 현재 sim entrypoint는 `VEHICLE_COUNT=2`만 허용하고 그 외 multi count를 거부 |

---

## 2. 이 순간의 실제 실행 상태

2026-08-03 17:10 KST에 Docker와 ROS 그래프를 직접 조회한 결과는 다음과 같다.

### 2.1 Docker 상태

- `av_drone-sim-1`: 실행 중
- `av_drone-ros-1`: 실행 중
- 두 컨테이너 모두 조회 시점 기준 약 4분 동안 `Up`

즉 시뮬레이션 컨테이너와 ROS 작업 컨테이너 자체는 살아 있다.

### 2.2 ROS 상태

현재 ROS 컨테이너에는 전체 autonomy/SLAM/fusion launch가 올라와 있지 않다.

확인된 주요 노드와 토픽:

- `/drone1/gazebo_ros_rplidar`
- `/drone2/gazebo_ros_rplidar`
- `/drone1/scan`: publisher 1개, subscriber 0개
- `/drone2/scan`: publisher 1개, subscriber 0개

확인되지 않은 것:

- MAVROS 인스턴스
- `autonomy_manager`
- `local_planner`
- `safety_monitor`
- `slam_toolbox`
- `map_fusion`
- `/swarm/fusion_status`
- `/swarm/map_version`

따라서 **현재 순간의 상태는 “시뮬레이터와 LiDAR publisher는 실행 중이지만 ROS 자율주행 스택은 대기 중”**이다. 컨테이너가 `Up`이라고 해서 비행·SLAM·융합이 실행 중인 것은 아니다.

추가로 `/drone3/gazebo_ros_rplidar`도 ROS discovery에서 관찰되었다. 현재 repository의 2-UAV manifest와 맞지 않는 참가자이므로 다음 중 하나일 가능성이 있다.

- 같은 ROS domain에 붙은 다른 컨테이너/프로세스
- 이전 실행에서 남은 ROS 2 daemon/discovery 정보
- 현재 프로젝트 외부의 Gazebo ROS participant

이는 즉시 기능 장애라는 뜻은 아니지만, 실험 재현성을 위해서는 **ROS_DOMAIN_ID 격리, daemon 재시작, 실험 전 node/topic inventory 고정**이 필요하다는 신호다.

---

## 3. Git과 작업 트리 상태

### 3.1 브랜치와 커밋 계보

현재 브랜치:

```text
feature/multi-uav-map-fusion
```

현재 HEAD:

```text
0ecd3a2  2026-08-03  Implement two-UAV known-pose map fusion
```

직전 흐름:

```text
0ecd3a2  two-UAV known-pose map fusion
ab6c656  quantification 변경 snapshot
48ca4cb  feature/mppi 최신 변경 병합
9366180  MPPI 실험 시나리오와 tooling 확장
a593b27  pure MPPI 및 A-star planning 지원
```

즉 현재 브랜치는 단일 드론/MPPI/정량화 작업 위에 멀티 UAV 지도 융합 기능을 얹은 형태다. `feature/multi-uav-map-fusion`에는 설정된 upstream branch가 보이지 않으므로, 현재 HEAD와 working tree가 원격에 안전하게 보존되었다고 가정해서는 안 된다.

### 3.2 기준 커밋 `0ecd3a2`가 추가한 핵심

이 커밋은 26개 파일에 약 1,575줄을 추가했다. 핵심 내용은 다음과 같다.

- `drone_map_fusion` ROS 2 패키지 신설
- OccupancyGrid 좌표 투영과 weighted log-odds fusion
- conflict cell 집계
- `/swarm/global_map`, `/swarm/map_version`, `/swarm/fusion_status` 발행
- stale source 제외와 `LOCAL_ONLY_FALLBACK`
- 2-UAV bringup launch 및 manifest
- known-pose mapper, SLAM health, map artifact recorder
- `slam_toolbox` 비동기 실행 설정
- 드론별 TF chain 구성
- `fusion_metrics.csv`, `swarm_summary.json` 기록
- 두 기체용 Gazebo world 및 LiDAR model 분리 기초

### 3.3 아직 커밋되지 않은 변경

이 문서를 만들기 직전 작업 트리에는 다음이 있었다.

- 수정 10개
- 삭제 1개
- untracked 최상위 항목 6개

수정 중인 핵심 방향:

1. **멀티 PX4 실행기 교체**
   - 기존 `prepare_swarm_models.py`를 삭제하는 방향
   - 새 `multi_px4_entrypoint.sh`로 Gazebo server, PX4 instance 2/3, model spawn을 직접 제어
   - 기존 포트를 점유하는 다른 실험과 충돌하지 않도록 SYSID/port 범위를 변경

2. **실험 world와 이동 범위 확대**
   - 전용 짧은 `swarm_two_lane`에서 `random_cylinders_double`로 전환
   - spawn을 `(3, -7.5)`와 `(3, 7.5)`로 변경
   - local goal을 `(137, 0, 3)`으로 확대
   - local/global map bound를 약 150 m 전방까지 확대

3. **ROS clock/TF 정렬 수정**
   - MAVROS pose의 epoch 계열 timestamp와 Gazebo scan의 simulation time 불일치를 피하도록 `pose_odom_tf_node`가 node clock으로 TF/odom stamp를 생성
   - `slam_toolbox`와 pose TF node에 `use_sim_time=true` 추가
   - 이는 scan이 TF cache보다 오래된 것으로 거절되는 문제를 겨냥한 수정이다.

4. **local planner 수정**
   - `guidance_mode`와 forward/escape 관련 파라미터 추가
   - direct-goal 계산에 현재 active goal을 명시적으로 전달
   - gap 구간의 start/end 검출 로직 보완
   - 최신 장시간 실행에서 나타난 과도한 escape와 목표 미도달 문제를 해결하려는 진행 중 수정으로 해석된다.

5. **대시보드/모니터의 multi-UAV artifact 지원**
   - `artifacts/<run_id>/<droneN>/` 중첩 구조 인식
   - vehicle별 run identity 표시
   - `swarm_summary.json` fusion KPI 표시
   - 진행 중 run을 실패로 계산하지 않도록 분리
   - 대용량 trajectory를 사용자가 선택할 때만 읽고 downsample
   - `ros_states`가 namespaced node와 중첩 artifact를 찾도록 수정

이 변경들은 방향은 타당하지만 아직 하나의 검증된 커밋으로 묶이지 않았다. 다음 실험 전에 “어떤 수정이 어떤 실패를 해결했는지”를 작은 커밋과 Gate 결과로 고정해야 한다.

---

## 4. 현재 시스템 아키텍처

### 4.1 실행 환경

현재 active runtime의 중심은 다음 조합이다.

- Host: Ubuntu 계열 환경
- Container orchestration: Docker Compose
- Flight stack: PX4 SITL v1.14.3
- Simulator: Gazebo Classic 11
- Middleware: ROS 2 Humble + Fast DDS
- Flight bridge: MAVROS
- Network: host network + host IPC
- 센서: Gazebo ROS LiDAR plugin이 `sensor_msgs/LaserScan` 직접 발행
- 연구 데이터: `artifacts/`, `experiments/`, `rosbags/`

일부 오래된 문서 제목이나 계획서에는 “Gazebo Sim”이 적혀 있지만, 현재 실행 스크립트와 README 기준 active path는 **Gazebo Classic**이다.

### 4.2 컨테이너 책임

```text
Host
└─ Docker Compose
   ├─ sim
   │  ├─ Gazebo Classic server/client
   │  ├─ PX4 SITL instance 1~N
   │  ├─ vehicle/LiDAR model spawn
   │  └─ /droneN/scan publish
   │
   └─ ros
      ├─ MAVROS instance per vehicle
      ├─ perception/planning/safety/control
      ├─ local mapping + slam_toolbox
      ├─ central map fusion
      ├─ metrics/artifact recording
      └─ ros_states/quant dashboard
```

`sim`은 물리·센서·PX4를 담당하고, `ros`는 자율주행과 연구 측정을 담당한다. 두 컨테이너가 같은 ROS domain에서 UDP Fast DDS로 통신하므로 domain 오염과 포트 충돌에 특히 민감하다.

### 4.3 단일 UAV 데이터 흐름

```text
Gazebo LiDAR
  -> /drone1/scan
  -> lidar_obstacle_node
  -> local_planner_node
  -> safety_monitor
  -> autonomy_manager
  -> MAVROS velocity setpoint
  -> PX4 SITL

MAVROS pose/state + planner/safety/mission topics
  -> metrics_logger
  -> artifacts/<run>/...
```

단일 UAV에서는 perception, planning, safety, control이 분리되어 있다. 이는 과거의 MPPI 단일 노드가 비행 상태 머신과 planner를 함께 갖던 구조보다 멀티드론 확장에 적합하다.

### 4.4 2-UAV mapping/fusion 데이터 흐름

각 차량마다 다음 노드가 namespace 아래 생성된다.

```text
/droneN
├─ mavros
├─ pose_odom_tf
├─ base_to_lidar_tf
├─ known_pose_mapper
├─ slam/slam_toolbox
├─ slam_health
├─ map_artifact_recorder
├─ lidar_obstacle
├─ local_planner
├─ safety_monitor
├─ autonomy_manager
└─ metrics_logger
```

중앙에는 한 개의 `map_fusion` 노드가 있다.

```text
/drone1/mapping/known_pose_map ─┐
                               ├─ map_fusion ─> /swarm/global_map
/drone2/mapping/known_pose_map ─┘             ├> /swarm/map_version
                                              └> /swarm/fusion_status
```

launch argument `fusion_source`에 따라 입력을 바꿀 수 있다.

- `known_pose`: deterministic baseline mapper 사용; 현재 기본값
- `slam`: `slam_toolbox`의 scan-matching map 사용; 비교 실험용

두 mapper는 동시에 실행되므로 source만 바꿔도 센서·비행 trajectory baseline을 유지하려는 설계다.

### 4.5 TF 소유권

의도된 TF chain은 다음과 같다.

```text
swarm_map
├─ drone1/map -> drone1/odom -> drone1/base_link -> drone1/lidar_link
└─ drone2/map -> drone2/odom -> drone2/base_link -> drone2/lidar_link
```

소유권:

| Transform | Publisher |
|---|---|
| `swarm_map -> droneN/map` | `map_fusion` static broadcaster |
| `droneN/map -> droneN/odom` | `slam_toolbox` |
| `droneN/odom -> droneN/base_link` | `pose_odom_tf` |
| `droneN/base_link -> droneN/lidar_link` | static transform publisher |

이 구조에서 가장 위험한 문제는 동일 transform의 multiple authority와 서로 다른 clock domain이다. 현재 미커밋 TF timestamp 수정은 후자를 해결하는 작업이다.

---

## 5. 패키지별 상세 상태

| 패키지 | 현재 역할 | 현재 판단 |
|---|---|---|
| `drone_bringup` | single/multi launch, YAML, MAVROS 포함 실행 | 핵심 orchestration. 단일과 2-UAV 진입점이 있으나 multi 실행은 수동 절차이며 `start.sh`와 통합되지 않음 |
| `drone_control` | OFFBOARD arm/takeoff/mission phase, setpoint 전달 | 단일 baseline에서 사용됨. multi launch에서 vehicle namespace별 인스턴스 생성 |
| `drone_perception` | LaserScan에서 최근접 장애물 추출 | 단순하지만 동작 가능한 baseline. map-level perception이나 tracking은 없음 |
| `drone_planning` | reactive local planner, A* global planner 코드 | reactive planner가 active. global planner와 swarm map의 정식 통합은 미완료 |
| `drone_safety` | pose/scan/planner timeout 및 emergency stop | 기본 fail-safe 존재. peer collision, global map stale 계층은 없음 |
| `drone_metrics` | trajectory, phase, summary, event, config snapshot | 재현성 기반이 좋지만 장시간 로그 폭증과 metric integrity 문제가 드러남 |
| `drone_slam` | pose→odom/TF, known-pose mapper, SLAM health, map 저장 | 멀티 mapping 기반은 구현. 실제 `slam_toolbox` localization은 최신 run에서 불량 |
| `drone_map_fusion` | local grid 투영, weighted fusion, conflict/stale 상태, summary | Gate C의 중심. 단위 테스트 및 장시간 실행 근거 있음 |
| `mppi` | known-world MPPI 및 flight wrapper | 기존 단일 UAV 연구 자산. swarm global path follower로는 아직 분리/연결되지 않음 |
| `mppi_lidar` | LiDAR 직접 사용 MPPI 실험 구현 | 별도 독립 경로. 공통 planner contract와 통합 필요 |
| `A_star` | map-aware A* 실험 패키지 | 알고리즘 자산은 있으나 이름 규칙과 패키지 구조 정리, `/swarm/global_map` 통합 필요 |
| `ros_states` | ROS/비행 상태 웹 모니터 | 단일 namespace 전제에서 multi namespace/artifact 지원으로 수정 중 |

### 5.1 `drone_map_fusion` 구현 세부

현재 fusion node는 다음 기능을 갖는다.

- 2개 이상 source를 parameter array로 구성 가능
- source map의 frame ID 검증
- 각 source의 `(x, y, yaw)` known transform 적용
- source grid resolution/origin/yaw를 global grid로 투영
- unknown cell 보존
- free/occupied threshold 분리
- log-odds 기반 weighted 합성
- source age에 따른 freshness weight 감소
- free와 occupied vote가 겹치는 conflict cell 집계
- source timeout 시 stale 제외
- source가 하나라도 있으면 global map 계속 발행
- 모든 source가 있으면 `HEALTHY`
- 일부 source만 있으면 `LOCAL_ONLY_FALLBACK`
- 아무 source도 없으면 `WAITING_MAPS`
- map version 단조 증가
- Transient Local map publisher
- fusion latency, input age, observed/conflict cell을 CSV와 JSON에 기록

현재 구현 경계:

- unknown-pose transform 추정은 하지 않는다.
- transform confidence gate는 source confidence parameter 수준이며 registration 검증기는 없다.
- per-cell observation timestamp/temporal decay map은 없다.
- delta map/tile 전송 없이 매 주기 전체 grid를 처리한다.
- process restart 후 자체 저장 map을 복구하는 persistence 계층은 별도로 검증되지 않았다.

---

## 6. 실험 및 artifact가 말해 주는 것

### 6.1 저장 데이터 규모

조사 시점 로컬 데이터 규모:

| 경로 | 크기 | 비고 |
|---|---:|---|
| `artifacts/` | 약 2.2 GB | `.gitignore` 대상, 실행별 metrics/map/log |
| `.logs/` | 약 168 MB | startup/launch 로그 |
| `dashboard_archive/` | 약 143 MB | untracked archive |
| `rosbags/` | 약 55 MB | untracked bag |
| `experiments/` | 약 25 MB | 일부 generated output/untracked 결과 포함 |

최신 장시간 swarm run 하나에서 특히 큰 파일:

- drone1 `planner_debug.jsonl`: 약 842 MB
- drone2 `planner_debug.jsonl`: 약 835 MB
- 각 drone `trajectory.csv`: 약 116 MB
- 각 drone `metrics.csv`: 약 5 MB
- `fusion_metrics.csv`: 약 5 MB
- `launch.log`: 약 8 MB

이 수치는 현재 logging이 장시간 실험에서 연구 데이터보다 디스크 병목을 먼저 만들 수 있음을 보여 준다. debug log sampling, size/time rotation, 종료 조건, run retention policy가 필요하다.

### 6.2 map fusion의 긍정적 근거

최신 장시간 artifact:

```text
artifacts/2026-08-03_03-51-08_swarm_random_live_tf/
```

`swarm_summary.json` 핵심 값:

| 항목 | 값 |
|---|---:|
| map version / fusion samples | 42,826 |
| 최종 state | `HEALTHY` |
| active sources | drone1, drone2 |
| stale sources | 없음 |
| fusion latency p95 | 약 132.09 ms |
| latest conflict ratio | 약 0.00810, 즉 0.81% |
| WAITING_MAPS count | 5 |
| HEALTHY count | 42,826 |

이는 다음을 입증한다.

- 두 source map을 장시간 연속으로 받았다.
- global map version이 지속 증가했다.
- 1 Hz 수준 fusion은 장시간 유지되었다.
- conflict metric과 latency metric이 실제 artifact로 남는다.
- 장시간 실행 종료 직전에도 source 두 개가 active였다.

과거 짧은 run의 p95는 약 30~84 ms 범위였고, expanded map을 사용한 장시간 run은 약 132 ms였다. map 크기와 관측 cell 수 확대가 latency 상승 원인일 가능성이 크지만, CPU/RTF와 함께 측정하지 않았으므로 현재는 인과관계가 아닌 추정으로만 봐야 한다.

일부 짧은 artifact에는 `LOCAL_ONLY_FALLBACK=1`이 기록되어 있다. fallback 상태 전이가 실제로 한 번 발생한 근거는 되지만, 의도적으로 source를 중단하고 global map continuity와 복구까지 자동 판정한 Gate test 결과는 별도로 고정할 필요가 있다.

### 6.3 최신 장시간 run이 임무 성공을 입증하지 못하는 이유

같은 run의 vehicle summary는 다음과 같다.

#### drone1

- runtime: 약 42,832초, 약 11.9시간
- 최종 phase: `RETURN_HOME_AVOID`
- `goal_reached=false`
- `failure_code=POSE_TIMEOUT`
- `slam_map_ready=true`
- `slam_localization_ok=false`
- SLAM coverage: 약 0.586
- escape count: 377,153
- 기록된 total path length: 약 125 km

#### drone2

- runtime: 약 42,832초
- 최종 phase: `MAPPING_TO_GOAL`
- `goal_reached=false`
- `failure_code=POSE_TIMEOUT`
- `slam_map_ready=true`
- `slam_localization_ok=false`
- SLAM coverage: 약 0.562
- escape count: 372,401
- 기록된 total path length: 약 9.4 km

이 값은 정상적인 137 m outbound/return 임무의 결과로 볼 수 없다. 특히 매우 큰 path length, 반복 arm/takeoff phase, 수십만 회 escape, phase time과 outbound time의 불일치는 다음 문제를 의심하게 한다.

- simulator/PX4/ROS stack 재시작 또는 재연결 후 같은 `RUN_ID`를 계속 사용
- pose frame reset 또는 큰 position jump를 path length에 그대로 누적
- mission phase 재진입 시 metrics latch/reset 불완전
- local planner가 장애물 구간에서 escape 상태를 과도하게 반복
- 실제 SLAM transform 불안정 또는 clock mismatch
- 종료 조건/mission timeout 부재

따라서 이 run은 **map fusion 장시간 동작 증거**로는 사용할 수 있지만, **비행 성공률·경로 길이·planner 우수성의 유효한 논문 샘플**로 사용하면 안 된다.

### 6.4 기존 A*/MPPI 정량화 결과의 경계

2026-07-18 bag 분석 표에는 다음 조건이 있다.

- `lidar_only_mppi_return`
- `partial_map_mppi_return`
- `partial_map_astar_return`

현재 표는 조건별 `runs=1`인 항목이 많고 success/return_success가 0이다. mission time이나 path length 비교 숫자는 생성되었지만, 표본 수와 성공 판정 때문에 통계적 결론을 내릴 단계는 아니다.

좋은 점은 artifact→CSV→Markdown→dashboard로 이어지는 정량화 파이프라인이 존재한다는 것이다. 다음 과제는 숫자를 더 많이 만드는 것이 아니라, **유효 run 기준과 자동 failure classification을 먼저 확정한 후 같은 seed/조건 반복을 채우는 것**이다.

---

## 7. 테스트 및 검증 상태

조사 과정에서 수행한 로컬 검증:

| 검증 | 결과 | 해석 |
|---|---|---|
| Python AST parse | 84개 Python 파일 통과 | 현재 source에 문법 오류는 발견되지 않음 |
| map fusion unit test | 4개 모두 통과 | translation, rotation, unknown 보존, conflict, invalid input 검증 |
| 전체 `pytest` | 실행 불가 | 현재 host Python environment에 `pytest`가 없음 |
| `compileall` | 일부 실패 | 코드 문법이 아니라 root 소유 `__pycache__`에 쓸 권한이 없어 실패 |
| Docker container 상태 | 확인 | sim/ros 컨테이너 Up |
| live swarm fusion topic | 현재 없음 | 전체 launch가 실행되지 않은 시점이므로 정상적인 대기 상태 |

현재 테스트의 구조적 공백:

- 2-UAV end-to-end 자동 smoke test 없음
- PX4 port/SYSID/namespace collision 자동 검사 없음
- TF multiple authority 자동 검사 없음
- scan publisher 정확히 1개인지 자동 Gate 없음
- 두 기체 arm/takeoff/goal/return timeout 자동 판정 없음
- source kill→fallback→recovery 자동 integration test 없음
- `slam` source와 `known_pose` source의 동일 trajectory A/B runner 없음
- 장시간 log growth와 memory growth test 없음
- CI에서 colcon build/test를 반복한다는 근거 없음

---

## 8. 현재 핵심 문제와 기술 부채

### P0. 실행 기준선이 아직 dirty working tree에 있음

핵심 runtime, clock, planner, dashboard 수정이 커밋되지 않았다. 현재 상태에서 새로운 실험을 계속하면 어떤 코드가 결과를 만들었는지 추적하기 어려워진다.

필요 조치:

- runtime 수정, planner 수정, dashboard 수정, 문서 수정을 논리적 커밋으로 분리
- 각 커밋마다 짧은 Gate 결과 첨부
- branch upstream 설정 또는 안전한 원격 백업
- artifact metadata에 source commit과 dirty diff hash를 함께 저장

### P0. “fusion 성공”과 “mission 성공”이 분리되어 있음

최신 artifact는 fusion은 `HEALTHY`지만 두 기체 mission은 실패했다. 중앙 지도가 살아 있는 것만으로 연구 목표를 달성한 것은 아니다.

필요 조치:

- mission timeout 설정
- 한 run에서 simulator/ROS launch 재시작을 허용하지 않거나 restart epoch를 기록
- pose discontinuity 감지 후 path metric segment 분리
- escape state rate-limit과 stuck detector
- outbound/return phase별 성공 조건 재검증

### P0. 실제 SLAM localization 불안정

두 기체 모두 `map_ready=true`, `localization_ok=false`다. 현재 기본 fusion source가 known-pose mapper이므로 global map은 만들어져도 실제 scan-matching SLAM이 성공했다는 뜻은 아니다.

필요 조치:

- `/clock`, scan stamp, odom/TF stamp를 같은 기준으로 검증
- `tf2_echo`, `view_frames`, slam_toolbox diagnostics를 artifact로 저장
- scan queue drop와 transform timeout count 기록
- known-pose map과 slam map의 IoU/translation/yaw error 비교
- `fusion_source:=slam`을 Gate C와 별도 Gate로 관리

### P1. 실험 환경 격리 부족

의도하지 않은 drone3 node가 discovery되었다. host networking과 공유 DDS domain은 편리하지만, 동시에 여러 실험을 돌릴 때 오염되기 쉽다.

필요 조치:

- run별 또는 작업별 ROS_DOMAIN_ID 할당 정책
- 실험 시작 전 `ros2 node list`, topic publisher GID, domain ID snapshot
- PX4 instance/port 예약표 자동 검증
- 동일 domain의 예상 밖 namespace가 있으면 launch 중단

### P1. 로그와 artifact가 과도하게 증가

한 장시간 run이 수 GB에 접근한다. 이는 dashboard 지연, 디스크 고갈, 분석 실패를 만든다.

필요 조치:

- `planner_debug.jsonl` sampling 또는 event-only 기록
- 파일당 최대 크기와 rotation
- trajectory는 고정 주기 downsample과 Parquet 선택 검토
- run 종료 시 압축/요약 후 raw retention 등급 적용
- artifacts quota와 자동 경고
- 10~20분 mission timeout을 기본값으로 설정

### P1. 시작 경로가 단일/멀티로 나뉨

`start.sh`는 single drone package subset을 빌드하고 `single_drone_autonomy.launch.py`를 실행한다. 2-UAV fusion은 문서의 수동 command를 사용해야 한다.

필요 조치:

- `./start.sh --profile single`
- `./start.sh --profile swarm-known-pose`
- `./start.sh --profile swarm-slam`
- profile별 build package와 readiness Gate 통합
- `stop.sh`가 multi launch와 rosbag/dashboard까지 정리하도록 확장

### P1. host-specific Docker 설정

`docker-compose.yml`에는 `/dev/nvidia3`와 특정 NVIDIA library version path가 고정되어 있다. 다른 GPU 번호, driver version, headless CI에서 그대로 재현되지 않을 수 있다.

필요 조치:

- GPU를 optional compose profile로 분리
- NVIDIA Container Toolkit 방식으로 driver library 직접 bind를 최소화
- CPU/headless test profile 추가
- host prerequisite checker 제공

### P2. 문서 사이의 상태 불일치

기존 README와 architecture 문서는 single-UAV active path를 중심으로 설명하고, 장기 계획서의 Sprint 체크박스는 실제 구현이 들어왔는데도 모두 미완료로 남아 있다. 반대로 Gate C 문서는 구현 경계를 비교적 잘 설명하지만 최신 미커밋 runtime을 기준으로 수정 중이다.

이 문서를 당분간 상위 현황 문서로 사용하고, 검증된 커밋 이후 README의 “현재 기준 스택”과 각 roadmap checkbox를 동기화해야 한다.

---

## 9. 권장 발전 방향

발전 순서는 기능 수를 빠르게 늘리는 순서가 아니라, 각 단계가 다음 단계의 신뢰 가능한 입력이 되도록 구성해야 한다.

### Phase 1. 현재 2-UAV known-pose baseline 고정

목표: dirty working tree를 재현 가능한 Gate C 기준선으로 만든다.

작업:

1. 새 multi PX4 launcher의 instance/SYSID/UDP mapping 자동 검증
2. `random_cylinders_double`에서 2대 scan publisher가 각각 정확히 1개인지 확인
3. clock/TF 수정 검증
4. planner gap/goal 수정에 대한 regression test 추가
5. 10~15분 제한 mission runner 작성
6. artifact 로그 rotation 적용
7. 수정 내용을 작은 커밋으로 고정

완료 기준:

- 예상 외 `/droneN` participant 0개
- 두 MAVROS 모두 connected, OFFBOARD, armed
- 두 scan topic 5분 이상 지속, publisher 각 1개
- TF multiple authority 0개
- 두 기체가 정해진 goal/return 시퀀스를 timeout 안에 완료
- `fusion_state=HEALTHY` 95% 이상
- source 한 개 중단 시 2초 이내 fallback, global map 발행 지속
- 재실행 5회 중 유효 run 5회

### Phase 2. 실제 SLAM Gate 분리 및 완성

목표: known-pose mapper가 아니라 `slam_toolbox` map을 fusion source로 사용할 수 있게 한다.

작업:

1. scan/odom/TF timestamp audit tool 추가
2. localization health 정의를 boolean 하나가 아니라 TF age, scan accepted rate, pose jump로 확장
3. known-pose map을 ground truth reference로 사용해 SLAM map 정량화
4. map 저장/재로드와 restart recovery 검증
5. 동일 seed 10회 A/B 실행

완료 기준:

- `localization_ok` 95% 이상 유지
- TF timeout/scan rejection 0 또는 허용 임계치 이하
- 동일 seed 10회 map 생성 성공률 90% 이상
- known-pose 대비 map IoU, occupied precision/recall 보고 가능
- 두 local SLAM map의 fusion에서 이중 벽/ghost obstacle 비율 정량화

### Phase 3. Global A*와 Local MPPI 연결

목표: `/swarm/global_map`이 단순 시각화 결과가 아니라 실제 의사결정 입력이 되게 한다.

권장 구조:

```text
/swarm/global_map + /swarm/map_version
  -> global A*
  -> /droneN/planner/global_path
  -> waypoint/corridor sampler
  -> local MPPI
  -> safety
  -> PX4
```

작업:

- A* package를 공통 `nav_msgs/Path` contract로 정리
- map version이 바뀌어도 corridor가 막힐 때만 replan
- MPPI cost에 global path, unknown space, static obstacle, control effort 반영
- planner output을 mission wrapper에서 분리
- reactive planner/MPPI/A*+MPPI를 동일 safety/control contract로 비교

완료 기준:

- global path 생성 성공률
- replan p50/p95/max latency
- path tracking RMSE
- blocked corridor 자동 우회
- command discontinuity와 oscillation 임계치 통과
- local-only 대비 mission success 또는 path quality 개선

### Phase 4. Peer-aware safety와 협업

목표: 다른 드론을 static map의 장애물이 아니라 시간에 따라 움직이는 agent로 처리한다.

작업:

- `/droneN/planner/predicted_trajectory`
- pose/velocity/valid-until/confidence contract
- MPPI의 peer collision cost
- stale trajectory uncertainty 증가
- priority 기반 deadlock 해소
- emergency hover/landing policy

완료 기준:

- collision 0회
- hard minimum separation 1.2 m 이상
- crossing, merge, head-on 시나리오에서 deadlock/oscillation 측정
- peer topic delay/loss에서도 안전 상태 전이 확인

### Phase 5. Unknown-pose registration

목표: spawn pose를 fusion 입력으로 직접 사용하지 않고 local map 상대 transform을 추정한다.

권장 순서:

1. OccupancyGrid edge/distance transform
2. coarse rotation/translation candidate
3. ICP refinement
4. overlap/RMSE/inlier 평가
5. 연속 confidence gate
6. known-pose transform과 오차 평가

중요 원칙:

- ground truth는 입력이 아니라 평가에만 사용
- confidence 미달 transform은 global map에 반영하지 않음
- verified transform jump 제한
- false merge를 success보다 더 심각한 failure로 취급

### Phase 6. 4-UAV와 통신 최적화

2대 Gate가 안정되기 전에는 4대로 확장하지 않는 편이 좋다. 4대는 단순 loop count 증가가 아니라 Gazebo RTF, LiDAR 부하, DDS bandwidth, TF 수, log volume이 함께 증가하는 단계다.

작업:

- manifest 기반 N-vehicle launcher 일반화
- full map 대신 delta/tile 공유
- sensor와 map QoS 분리
- CPU/RAM/DDS/RTF metrics 추가
- GUI 없는 CI/headless profile
- 1→2→4 scaling report

완료 기준:

- 4대 namespace/port/TF 완전 분리
- 평균 RTF 0.8 이상 목표
- control deadline miss 측정 및 허용 범위 통과
- 한 기체 failure가 나머지 기체 mission을 중단시키지 않음

---

## 10. 바로 실행할 우선 작업 목록

### 오늘 해야 할 일

- [ ] 현재 ROS domain의 예상 밖 drone3 participant 원인 확인
- [ ] current working tree를 기능별 diff로 검토
- [ ] `multi_px4_entrypoint.sh`를 포함해 runtime 변경 커밋
- [ ] clock/TF 변경만 별도 커밋
- [ ] local planner 변경에 최소 단위/regression test 추가
- [ ] 10~15분 mission timeout 설정
- [ ] planner debug log sampling/rotation 적용

### 다음 1주

- [ ] 2-UAV end-to-end smoke script 작성
- [ ] Gate A/B/C 자동 판정 JSON 생성
- [ ] source kill/fallback/recovery test 자동화
- [ ] `fusion_source=known_pose` 5회 반복 성공
- [ ] `fusion_source=slam` timestamp/TF 문제 해결
- [ ] README와 roadmap 상태 동기화

### 다음 2~4주

- [ ] actual SLAM map quality 10회 반복 평가
- [ ] global A*가 `/swarm/global_map`을 사용하도록 연결
- [ ] MPPI planner core와 flight mission wrapper 분리
- [ ] global path tracking cost 추가
- [ ] local-only vs online-fusion A/B 실험

---

## 11. 권장 실행 및 확인 절차

현재 multi-UAV 경로는 `start.sh`가 아니라 아래 수동 절차를 기준으로 한다.

### 11.1 2-UAV simulation 시작

```bash
export RUN_ID="$(date +%Y-%m-%d_%H-%M-%S)_swarm"
VEHICLE_COUNT=2 \
PX4_SITL_WORLD=random_cylinders_double \
RUN_ID="$RUN_ID" \
docker compose up -d --build sim ros
```

### 11.2 ROS package build 및 launch

```bash
docker compose exec ros bash -lc '
  source /opt/ros/humble/setup.bash
  cd /workspace/AV_Drone
  colcon build --symlink-install --packages-up-to drone_bringup
  source install/setup.bash
  ros2 launch drone_bringup multi_drone_slam_fusion.launch.py \
    run_id:="${RUN_ID}" fusion_source:=known_pose
'
```

실제 SLAM source 비교 시:

```bash
fusion_source:=slam
```

### 11.3 최소 Gate 확인

```bash
ros2 topic hz /drone1/scan
ros2 topic hz /drone2/scan
ros2 topic hz /drone1/mapping/known_pose_map
ros2 topic hz /drone2/mapping/known_pose_map
ros2 topic hz /swarm/global_map
ros2 topic echo /swarm/map_version --once
ros2 topic echo /swarm/fusion_status --once
ros2 run tf2_ros tf2_echo drone1/map drone1/base_link
ros2 run tf2_ros tf2_echo drone2/map drone2/base_link
```

반드시 함께 확인할 항목:

- scan publisher가 vehicle별 정확히 1개인지
- `/drone1/mavros`와 `/drone2/mavros`가 서로 다른 system ID를 쓰는지
- node name과 TF frame이 모두 namespace 처리되었는지
- `map_version`이 단조 증가하는지
- `fusion_status`의 input age가 timeout보다 작은지
- mission 종료 시 각 drone summary와 swarm summary가 같은 run 아래 생성되는지

---

## 12. 실험 결과를 유효하다고 판단하는 기준

앞으로는 artifact가 생성되었다는 이유만으로 실험을 성공 처리하지 않는다.

### 유효 run 필수 조건

- git commit이 기록되고 dirty 여부가 명시됨
- world/config snapshot과 hash 존재
- 예상한 vehicle 수와 namespace가 일치
- simulator/ROS restart 없이 한 mission epoch로 종료
- pose jump가 허용 임계치 이하
- scan/pose p99 period가 허용 범위 이내
- mission success/failure code가 종료 시점에 확정
- fusion source와 registration mode가 명확
- planner/debug 파일이 정상 종료 및 flush됨

### 2-UAV known-pose 성공 조건 예시

- 양쪽 connected/armed/OFFBOARD
- 양쪽 목표 도달 및 필요 시 return 완료
- collision 0
- pose/scan timeout 0
- map source 두 개 active 비율 95% 이상
- fusion p95가 정한 budget 이하
- conflict ratio가 사전 기준 이하
- fallback 주입 실험에서는 global map 연속 발행

### 결과 제외 조건

- 동일 RUN_ID 아래 process restart가 섞임
- path length가 pose reset 때문에 비정상적으로 증가
- mission phase가 timeout 없이 무한 지속
- `localization_ok=false`인데 SLAM 성공 표본으로 분류
- 진행 중 run을 fail 또는 success 통계에 포함
- 조건당 1회 결과로 우수성을 일반화

---

## 13. 문서별 역할 정리

현재 문서가 많으므로 역할을 다음처럼 나누는 것이 좋다.

| 문서 | 앞으로의 역할 |
|---|---|
| `PROJECT_STATUS_AND_ROADMAP.md` | 현재 상태, 우선순위, 검증 경계의 상위 문서 |
| `README.md` | 처음 실행하는 사람을 위한 짧은 Quick Start |
| `docs/architecture.md` | 검증된 active architecture와 topic/TF contract |
| `docs/multi-uav-mapping-gate-c.md` | 2-UAV known-pose Gate 실행 절차 |
| `multi_uav_slam_map_stitching_expansion_plan.md` | 장기 연구 설계와 Phase 0~6 요구사항 |
| `docs/problem.md` | 현재 재현되는 문제, 증거, 수정, 결과 ledger |
| `experiments/` | 자동 생성되는 실험 집계와 논문용 결과 |

문서 안의 절대 경로 `/home/deepblue/AV_Drone`는 현재 실제 workspace `/home3/deepblue/work/AV_Drone`와 다르므로, 가능한 경우 repository-relative path로 정리하는 것이 좋다.

---

## 14. 최종 판단

이 저장소의 강점은 다음과 같다.

- 단일 드론 센서–계획–안전–제어 계층이 이미 분리되어 있다.
- 실험 artifact와 config snapshot을 남기는 습관이 코드에 반영되어 있다.
- 2-UAV namespace, TF, mapping, fusion으로 넘어가는 실제 코드가 존재한다.
- map fusion은 단위 테스트와 장시간 `HEALTHY` artifact를 모두 갖고 있다.
- known-pose baseline 이후 unknown-pose, global planning, peer avoidance로 가는 연구 방향이 명확하다.

현재 약점은 다음과 같다.

- 최신 runtime fixes가 dirty working tree에 있고 기준선이 아직 고정되지 않았다.
- 장시간 fusion 성공과 실제 비행 mission 성공 사이의 간극이 크다.
- 실제 SLAM localization이 아직 신뢰 가능한 상태가 아니다.
- global map이 planner 의사결정에 아직 연결되지 않았다.
- 자동 end-to-end Gate와 실험 격리가 부족하다.
- 로그 증가 속도와 metric integrity가 장시간 연구를 방해한다.

가장 좋은 다음 방향은 새로운 알고리즘을 바로 더 추가하는 것이 아니다. 먼저 **2-UAV known-pose baseline을 짧고 반복 가능한 자동 실험으로 고정하고, 실제 SLAM과 mission failure를 분리해 해결한 뒤, global A*–local MPPI 연결로 넘어가는 것**이다. 이 순서를 지키면 이후 unknown-pose registration과 4-UAV 확장이 연구 결과로서도 설명 가능하고 디버깅 가능한 구조가 된다.
