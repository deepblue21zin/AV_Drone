# 복구가능성 기반 Multi-UAV SLAM 지도 보정: 단계별 구현 및 실험 계획

> 대상 프로젝트: `AV_Drone`  
> 기준 환경: PX4 SITL + ROS 2 Humble + Gazebo Classic + 2-UAV + 2D LiDAR  
> 기준 시나리오: 약 137 m의 저특징·반복 구조, 서로 떨어진 평행 lane, one-way 비행  
> 문서 목적: 현재의 fixed-transform map fusion을 실제 trajectory/submap 보정 구조로 확장하고, 논문 실험까지 이어지는 구현 순서와 판정 기준을 정의한다.

---

## 1. 한눈에 보는 전체 목표

현재 시스템은 각 UAV가 만든 지도를 알려진 spawn transform으로 `swarm_map`에 투영해 합친다. 이 방식은 두 지도의 초기 좌표계를 맞추는 데는 유효하지만, 비행 중 누적된 구간별 SLAM drift는 수정하지 못한다.

제안하는 최종 흐름은 다음과 같다.

```text
UAV별 LaserScan + odometry
              ↓
       keyframe / local submap
              ↓
 intra-UAV odometry·scan factors
              ↓
LiDAR 약관측 방향과 불확실성 계산
              ↓
후보 inter-UAV 측정의 방향별 정보 이득 계산
              ↓
정보가 충분하면 측정 사용
정보가 부족하면 최소 협력 기동 수행
              ↓
    joint SE(2) pose graph 최적화
              ↓
 최적화된 submap pose로 지도 재투영
              ↓
       보정된 global map 발행
```

핵심은 단순히 상대측정을 추가하는 것이 아니다. 다음 세 질문에 순서대로 답해야 한다.

1. 현재 SLAM 오차는 어느 상태 방향에서 증가하는가?
2. 드론 간 측정이 그 방향에 실제 정보를 추가하는가?
3. 정보가 부족하다면 어떤 최소 기동으로 관측가능성을 만들 수 있는가?

---

## 2. 현재 시스템의 상태와 연구 출발점

### 2.1 현재 확보된 기능

- 두 UAV의 PX4/MAVROS/ROS namespace 분리
- UAV별 2D LiDAR와 odometry 입력
- UAV별 `slam_toolbox` 지도 및 진단 데이터 생성
- MAVROS odometry 기반 `known_pose` occupancy map 생성
- static spawn transform 기반 중앙 map fusion
- fusion 상태, conflict ratio, latency 기록
- rosbag, artifact, 실험 registry 및 dashboard 구조

### 2.2 현재 확인된 실패

기존 137 m 실험에서 raw SLAM의 0.3 m 정렬률은 UAV별로 40.6%, 29.3%까지 감소했고, `map→odom` translation correction은 각각 9.063 m, 1.929 m까지 누적됐다. 반면 scan matching을 끈 대조군은 odometry 기준 지도에 잘 정렬됐다.

이 결과는 현재의 주된 문제가 map fusion 투영 자체가 아니라, 각 UAV 내부 scan matching과 pose graph가 만든 drift라는 것을 의미한다.

### 2.3 현재 map fusion으로 보정할 수 없는 이유

현재 fusion은 UAV별 완성된 `OccupancyGrid`에 하나의 rigid transform을 적용한다.

```text
잘못 휘어진 UAV 지도 + static transform
                 ↓
전체 지도의 위치만 이동/회전
                 ↓
지도 내부의 구간별 drift는 그대로 유지
```

따라서 137 m 지도 내부에서 앞부분은 맞고 뒷부분은 3 m 틀어진 경우, 전체 grid를 1 m 이동시켜도 두 구간을 동시에 맞출 수 없다. 이를 해결하려면 지도를 여러 submap으로 나누고 각 submap의 pose를 독립 변수로 최적화해야 한다.

---

## 3. 연구 가설과 논문용 질문

### RQ1. SLAM 약관측 방향 검출

> 2D LiDAR scan matching의 정보행렬 또는 공분산으로 현재 직선·반복 환경에서 약한 `x`, `y`, `yaw` 결합 방향을 안정적으로 검출할 수 있는가?

예상 결과:

- 긴 평행 구조에서는 진행축 성분이 큰 방향이 약해질 가능성이 높다.
- 반복 원통 환경에서는 단순 퇴화뿐 아니라 잘못된 correspondence에 의한 false optimum도 나타날 수 있다.

### RQ2. 상대측정의 보정 가능성

> UWB range, range+bearing 또는 상대 UAV pose 측정이 현재 LiDAR 약관측 방향에 실제로 정보를 추가하는가?

예상 결과:

- 같은 속도의 평행 직진에서 range-only 측정은 진행축 drift 보정 정보가 작을 수 있다.
- 상대 longitudinal offset이 변하도록 hover 또는 속도 차이를 만들면 정보량이 증가할 수 있다.

### RQ3. 최소 협력 기동

> 복귀·역주행·lane 교차 없이 약관측 방향의 정보 이득을 만드는 최소 기동은 무엇인가?

후보:

- 기동 없음
- 한 UAV의 짧은 hover
- UAV 간 일시적인 속도 차이
- 허용 범위 내 측면 offset
- 짧은 S 또는 arc 기동
- bearing/vision을 사용할 때 yaw-only 상호관측

### RQ4. 복구 불가능한 오차

> 두 UAV가 함께 같은 방향으로 틀어지는 common-mode drift를 상대측정만으로 제거할 수 있는가?

예상 결과:

- pairwise relative measurement는 주로 두 UAV 간 differential drift를 줄인다.
- 두 UAV가 함께 동일하게 틀어진 common-mode는 절대 anchor, 과거 submap, GNSS/RTK 또는 고정 landmark 없이 남을 수 있다.

---

## 4. 전체 구현 및 실험 순서

```text
Stage 0  현재 baseline 고정
Stage 1  offline submap pose-graph 보정
Stage 2  LiDAR 퇴화·약관측 방향 계측
Stage 3  inter-UAV 측정과 관측가능성 실험
Stage 4  방향별 정보 이득 기반 최소 기동
Stage 5  online incremental 보정
Stage 6  실제 상대센서 및 확장 실험
```

중요한 우선순위는 `Stage 1 → Stage 2 → Stage 3`이다. 지도 보정 backend가 없는 상태에서 active maneuver를 먼저 구현하면, 측정값이 좋아져도 실제 지도 수정으로 이어지지 않는다.

---

## 5. Stage 0: 현재 baseline 고정

### 5.1 목적

제안 방식 적용 전의 오차 분포와 실패율을 재현 가능한 숫자로 고정한다. 한 번의 성공·실패가 아니라 동일 설정 반복 결과를 기준선으로 사용한다.

### 5.2 구현 작업

현재 실행 구조를 유지하면서 다음 항목이 같은 run에 기록되도록 확인한다.

- `/drone1/scan`, `/drone2/scan`
- UAV별 odometry 및 local pose
- UAV별 raw SLAM pose와 map
- UAV별 `map→odom` transform
- UAV별 known-pose map
- fused known-pose map
- Gazebo ground truth: 알고리즘 입력이 아닌 평가 전용
- parameter snapshot과 scenario seed

### 5.3 실험 조건

최소 세 종류의 환경을 사용한다.

| 환경 | 목적 |
|---|---|
| 특징이 충분한 corner/비대칭 환경 | 퇴화 검출기의 negative control |
| 긴 평행 벽 또는 단순 복도 | 진행축 기하 퇴화 확인 |
| 반복 원통/기둥 환경 | 퇴화와 perceptual aliasing 동시 확인 |

비행 길이는 다음처럼 단계화한다.

- 짧은 구간: 20~30 m
- 중간 구간: 45~70 m
- 전체 구간: 약 137 m

각 조건은 동일 seed 집합으로 최소 10회 반복한다. 개발 중에는 3회로 빠르게 확인할 수 있지만, 논문 결과는 10회 이상을 권장한다.

### 5.4 실행 예시

현재 진단 스크립트와 manifest를 이용할 수 있다.

```bash
cd /home3/deepblue/work/AV_Drone
./scripts/run_slam_root_cause_condition.sh baseline_r1
```

정확한 condition 이름과 옵션은 실행 전 스크립트 상단의 usage를 확인한다.

```bash
sed -n '1,25p' scripts/run_slam_root_cause_condition.sh
ls experiments/slam_root_cause/manifests
```

기존 multi-UAV launch를 수동 실행하는 경우에는 run ID와 fusion source를 명시한다.

```bash
export RUN_ID="$(date +%Y-%m-%d_%H-%M-%S)_slam_baseline"
ros2 launch drone_bringup multi_drone_slam_fusion.launch.py \
  run_id:="$RUN_ID" \
  fusion_source:=slam
```

### 5.5 필수 기준 지표

#### 궤적 지표

- ATE RMSE
- RPE translation/yaw
- endpoint position error
- endpoint yaw error
- traveled distance 대비 drift 비율

#### 지도 지표

- occupied-cell precision/recall
- map IoU
- occupied-cell Chamfer distance
- double-wall ratio
- ghost obstacle ratio
- free/occupied conflict ratio

#### 안정성 지표

- run 성공률과 실패율
- `map→odom` translation/yaw 최대값
- localization loss 발생 시점
- metric의 seed 간 표준편차와 최악값

### 5.6 완료 기준

- 동일 설정에서 최소 10회의 artifact가 생성된다.
- 모든 run에서 동일한 평가 스크립트가 지표를 계산한다.
- `비행/통신 성공`, `map fusion 성공`, `raw SLAM 정렬 성공`이 서로 다른 판정 필드로 분리된다.
- 실패가 특정 UAV ID, lane 또는 단일 seed에만 종속되지 않는지 확인한다.

### 5.7 실패 결과의 해석

- 짧은 구간부터 무너지면 sensor/TF/timestamp/deskew 문제를 먼저 의심한다.
- 긴 구간에서만 무너지면 누적 scan matching drift와 graph constraint 부족 가능성이 높다.
- lane swap 시 실패 UAV가 lane을 따라가면 환경 기하 영향이 크다.
- 같은 설정의 결과 분산이 크면 알고리즘의 local optimum 민감성을 별도 결과로 보고한다.

---

## 6. Stage 1: Offline submap pose-graph 지도 보정

### 6.1 목적

기존 rosbag만 사용해 다음 질문을 먼저 검증한다.

> 여러 시점의 inter-UAV constraint를 추가하고 joint pose graph를 최적화하면 fixed-transform grid fusion보다 지도 오차가 줄어드는가?

이 단계에서는 active maneuver와 실시간 제어를 구현하지 않는다.

### 6.2 Submap 생성

전체 occupancy grid를 직접 최적화하지 않고 UAV별 local submap을 생성한다.

권장 초기 설정:

- keyframe 간격: 이동 0.5~1.0 m 또는 yaw 5~10도
- submap 길이: 2~5 m 또는 10~30개의 scan
- submap은 생성 후 frozen 처리
- 각 submap은 local frame 안의 grid 또는 point set으로 저장

Submap 데이터 예시:

```text
SubmapRecord
├─ uav_id
├─ submap_id
├─ start_stamp / end_stamp
├─ initial_pose_SE2
├─ local_occupancy_grid 또는 local_points
├─ keyframe scan 목록
├─ scan Hessian/covariance
└─ source bag/run ID
```

초기에는 rosbag을 읽는 offline Python/C++ 도구로 구현한다. 온라인 ROS node는 offline 결과가 검증된 뒤 만든다.

### 6.3 Pose graph 구성

각 UAV의 submap pose를 graph node로 둔다.

```text
UAV 1: A0 ─ A1 ─ A2 ─ A3 ─ A4
UAV 2: B0 ─ B1 ─ B2 ─ B3 ─ B4
```

기본 factor:

1. UAV별 odometry factor
2. 인접 submap scan-registration factor
3. 검증된 intra-UAV loop factor가 있으면 추가
4. inter-UAV range/range+bearing/place factor

SE(2) 상태는 다음과 같다.

\[
x_k^i=[p_x,p_y,\psi]^T
\]

기본 최적화 목적함수는 다음과 같이 구성한다.

\[
X^*=\arg\min_X
\sum \|r_{odom}\|_{\Sigma_o}^2
+\sum \rho_s(\|r_{scan}\|_{\Sigma_s}^2)
+\sum \rho_r(\|r_{inter}\|_{\Sigma_r}^2)
\]

첫 UAV의 첫 pose 또는 명시적인 시작 anchor를 고정해 global gauge freedom을 제거한다. 이 고정은 실제 위치 정답을 주입하는 것이 아니라 좌표계 원점을 정의하는 용도다.

### 6.4 Synthetic inter-UAV sensor 생성

초기에는 Gazebo ground truth에서 실제 센서처럼 보이는 상대측정을 생성한다.

잘못된 방법:

```text
optimizer에 UAV의 ground-truth pose를 직접 factor로 입력
```

올바른 방법:

```text
ground truth로 두 UAV 사이의 거리/방향을 계산
                 ↓
Gaussian noise, bias, dropout, timestamp offset 추가
                 ↓
range 또는 range+bearing measurement만 optimizer에 입력
```

Range 모델:

\[
z_r=\|p_i-p_j\|+n_r
\]

Range+bearing 모델:

\[
z_{rb}=\begin{bmatrix}
\|p_i-p_j\| \\
\mathrm{atan2}(y_j-y_i,x_j-x_i)-\psi_i
\end{bmatrix}+n_{rb}
\]

초기 noise sweep 예시:

- range 표준편차: 0.05, 0.10, 0.30, 0.50 m
- bearing 표준편차: 1, 3, 5, 10도
- dropout: 0, 10, 30, 50%
- outlier: 전체 측정의 0, 5, 10%
- timestamp offset: 0, 50, 100, 200 ms

### 6.5 단일 연결과 다중 연결 비교

반드시 다음을 분리해서 실험한다.

#### E1-A. inter-UAV factor 없음

```text
A0 ─ A1 ─ A2 ─ A3
B0 ─ B1 ─ B2 ─ B3
```

두 graph가 독립적이다.

#### E1-B. inter-UAV factor 한 개

```text
A0 ─ A1 ─ A2 ─ A3
          │
B0 ─ B1 ─ B2 ─ B3
```

두 좌표계를 연결하는 데는 도움이 되지만 내부 drift를 식별할 중복 제약이 부족할 수 있다.

#### E1-C. 떨어진 위치의 inter-UAV factor 여러 개

```text
A0 ─ A1 ─ A2 ─ A3
     │         │
B0 ─ B1 ─ B2 ─ B3
```

graph cycle이 생기므로 두 UAV의 상대적 drift 불일치를 최적화할 수 있다.

### 6.6 잘못된 factor 방어

반복 구조에서는 단일 ICP score만 믿지 않는다. 다음 gate를 순서대로 둔다.

1. timestamp 및 최대 속도 gate
2. 측정 범위와 FoV gate
3. registration Hessian/condition gate
4. 연속 측정 일관성 gate
5. pairwise consistency 또는 cycle consistency
6. robust loss/ switchable constraint
7. 최적화 후 residual 재검사

Outlier 실험에서 하나의 잘못된 factor가 전체 지도를 무너뜨리는지 반드시 확인한다.

### 6.7 최적화 후 지도 재생성

최적화 결과는 submap pose 집합이다.

```text
보정 전 pose: T_A0, T_A1, T_A2, ...
보정 후 pose: T*_A0, T*_A1, T*_A2, ...
```

각 frozen submap을 보정된 pose로 global grid에 다시 투영한다.

```text
for each submap S_k:
    global_cells = project(S_k, optimized_pose[k])
    update global log-odds
```

이때 map revision 번호를 증가시키고, global planner가 이전 지도와 새 지도를 구분할 수 있게 한다.

### 6.8 비교 실험

| ID | 방법 | 확인 질문 |
|---|---|---|
| E1-0 | UAV별 raw SLAM | 원래 drift는 얼마인가? |
| E1-1 | current fixed fusion | 현재 방식의 한계는 무엇인가? |
| E1-2 | submap graph, inter factor 없음 | submap 분할 자체의 효과는 있는가? |
| E1-3 | inter factor 1개 | 좌표계 연결만으로 drift가 펴지는가? |
| E1-4 | inter factor 여러 개 | cycle이 생길 때 trajectory/map이 개선되는가? |
| E1-5 | 여러 factor + robust gate | outlier가 있을 때도 안정적인가? |

### 6.9 완료 기준

- 합성 drift가 있는 graph에서 최적화 결과가 정답 방향으로 수렴한다.
- 여러 inter-UAV factor가 한 개 factor보다 구간별 drift를 더 잘 줄인다.
- fixed grid fusion보다 map IoU가 증가하고 double-wall/ghost ratio가 감소한다.
- 잘못된 factor 비율이 설정 범위 안일 때 catastrophic map corruption이 발생하지 않는다.
- 같은 입력 rosbag에 대해 결과가 재현 가능하다.

### 6.10 실패 결과의 해석

- trajectory는 좋아지고 map은 나빠지면 submap origin/TF/reprojection 오류 가능성이 높다.
- graph는 수렴하지만 ground truth 오차가 커지면 factor covariance 또는 false constraint가 잘못 설정됐을 수 있다.
- 한 UAV만 좋아지고 다른 UAV가 나빠지면 두 odometry source의 confidence를 동일하게 둔 것이 원인일 수 있다.
- 모든 UAV가 같은 방향으로 틀어진 상태가 유지되면 common-mode unobservability일 수 있다.

---

## 7. Stage 2: LiDAR 약관측 방향 계측

### 7.1 목적

`SLAM이 불안정하다`는 scalar 판정에서 벗어나, 현재 pose의 어떤 방향이 약한지 기록한다.

### 7.2 정보행렬 계산

Scan matching residual을 (r), pose를 (x=[x,y,\psi])라고 하면 다음 근사를 사용한다.

\[
H_L=J^T WJ, \qquad J=\frac{\partial r}{\partial x}
\]

고유값 분해:

\[
H_L=V\Lambda V^T
\]

- 큰 고유값: 해당 방향으로 pose를 바꾸면 residual이 크게 변하므로 잘 관측됨
- 작은 고유값: pose가 바뀌어도 residual 변화가 작으므로 약관측
- 작은 고유값의 고유벡터: 약한 `x`, `y`, `yaw` 결합 방향

### 7.3 단위와 정규화 주의

Translation은 m, yaw는 rad이므로 raw Hessian eigenvalue를 그대로 비교하면 단위 영향이 섞인다. 다음 중 하나를 사용한다.

- translation/rotation characteristic scale을 사용한 state normalization
- translation과 rotation sub-block을 분리한 분석
- solver가 제공하는 marginal covariance 사용
- 논문에서 사용한 normalization과 threshold를 명시

Threshold를 한 환경에 맞춘 뒤 모든 환경에 무조건 적용하지 않는다.

### 7.4 출력 데이터

권장 ROS topic 또는 artifact 필드:

```text
/droneN/slam/observability
├─ stamp
├─ eigenvalues[3]
├─ weak_eigenvector[3]
├─ condition_number
├─ translation_score
├─ rotation_score
├─ degeneracy_state
└─ matching_residual
```

CSV에는 다음을 함께 기록한다.

- UAV pose와 velocity
- 환경/seed/run ID
- scan point 수와 valid correspondence 수
- eigenvalues/eigenvectors
- ground-truth local error 증가량: 평가 전용
- 이후 1, 2, 5초의 RPE 증가량

### 7.5 실험 순서

#### E2-A. 구조가 충분한 환경

Corner, 비대칭 장애물, 다양한 normal 방향이 있는 환경에서 false alarm이 적어야 한다.

#### E2-B. 평행 벽 환경

진행축 약관측 방향이 지속적으로 검출되는지 확인한다.

#### E2-C. 반복 원통 환경

작은 고유값과 갑작스러운 잘못된 local optimum을 분리해서 기록한다.

#### E2-D. 속도와 LiDAR range sweep

- 비행 속도
- scan rate
- LiDAR 최대 range
- angular resolution
- noise

를 바꿔 퇴화 검출이 sensor 설정에 얼마나 민감한지 본다.

### 7.6 검출기 평가 방법

퇴화 label을 ground truth로 직접 정의하기 어렵기 때문에 다음 두 기준을 병행한다.

1. 기하학적 예상 방향과 weak eigenvector의 각도 비교
2. 검출 score가 이후 RPE/ATE 증가를 얼마나 잘 예측하는지 평가

가능한 지표:

- weak direction angular error
- drift event precision/recall
- AUROC 또는 AUPRC
- detection lead time
- seed 간 threshold 안정성

### 7.7 완료 기준

- 특징이 충분한 환경과 퇴화 환경의 score 분포가 구분된다.
- weak eigenvector가 실제 error growth direction과 통계적으로 연관된다.
- 반복 구조의 false match를 `degeneracy` 하나로 잘못 설명하지 않는다.
- 실시간 또는 목표 주기에서 계산 가능하다.

---

## 8. Stage 3: Inter-UAV 측정과 관측가능성 분석

### 8.1 목적

상대측정이 존재한다는 사실과 SLAM 오차 보정에 유용하다는 사실을 구분한다.

### 8.2 방향별 정보 이득

LiDAR의 가장 약한 방향을 (v_{weak}), 후보 상대측정의 정보행렬을 (H_R(a))라고 하면 간단한 score는 다음과 같다.

\[
g(a)=v_{weak}^T H_R(a)v_{weak}
\]

Graph 전체의 약관측 부분공간을 (W)라고 하면 다음을 사용할 수 있다.

\[
\Delta I_W(a)=
\log\det(W^T(H_G+H_R(a))W+\epsilon I)
-\log\det(W^TH_GW+\epsilon I)
\]

실제 구현에서는 candidate maneuver 시간창의 모든 예상 측정을 함께 linearize한 뒤 information gain을 계산한다.

### 8.3 Formation 및 기동 실험

#### E3-A. 같은 속도의 평행 직진 + range-only

```text
UAV 1  ─────────────→
        일정한 lane 간격
UAV 2  ─────────────→
```

예상:

- range 값이 거의 일정하다.
- 횡방향 상대 위치에는 정보가 있어도 진행축 상대오차 정보는 작을 수 있다.
- graph에 많은 range factor를 넣어도 rank가 실질적으로 증가하지 않을 수 있다.

#### E3-B. 한 UAV hover + range-only

```text
UAV 1  ─────● hover
UAV 2  ─────────────→
```

예상:

- 상대 longitudinal offset이 시간에 따라 변한다.
- 여러 range 측정의 기하가 다양해져 relative pose 추정이 개선될 수 있다.

#### E3-C. 속도 차이 + range-only

UAV 1과 UAV 2의 속도를 짧은 시간 다르게 적용한다. 임무 중단 없이 정보를 만들 수 있는지 본다.

#### E3-D. 평행 직진 + range+bearing

Range-only보다 적은 기동으로 진행축 정보를 얻는지 확인한다.

#### E3-E. 측면 offset 또는 짧은 arc

정보 이득은 클 수 있지만 안전·시간 비용도 증가한다. lane 제한 안에서 가능한 범위만 시험한다.

### 8.4 Common-mode 실험

의도적으로 두 UAV odometry에 같은 진행축 bias를 주입한다.

```text
UAV 1 estimated x = true x + common bias
UAV 2 estimated x = true x + common bias
```

비교 조건:

1. inter-UAV range만 사용
2. inter-UAV range+bearing 사용
3. 시작 pose anchor만 사용
4. 중간의 fixed anchor 또는 known landmark 추가

예상:

- 상대측정은 두 UAV 사이의 일관성을 높이지만 공통 bias는 남을 수 있다.
- 중간 absolute factor가 들어가면 common-mode drift가 줄어든다.

이 실험은 제안 방식의 한계를 정직하게 보여주면서, `보정 가능/불가능 판정`의 필요성을 증명한다.

### 8.5 측정 모델 비교표

| 측정 | 장점 | 약점 | 현재 프로젝트에서의 역할 |
|---|---|---|---|
| range-only | 가볍고 UWB로 구현 용이 | 기동 기하에 매우 민감 | 관측가능성 연구에 가장 적합 |
| range+bearing | 진행축 정보 확보가 쉬움 | 센서와 calibration 복잡 | 성능 상한 및 실용 대안 |
| 상대 SE(2) pose | graph factor가 직접적 | 카메라/tag/FoV 의존 | 보정 backend 검증에 유용 |
| 같은 장소 submap match | 환경 자체를 이용 | 반복 구조 false match 위험 | overlap이 생기는 확장 시나리오 |

### 8.6 완료 기준

- 기동별 predicted information gain과 실제 covariance/ATE 감소가 양의 상관을 보인다.
- passive range-only가 약한 조건을 재현한다.
- hover 또는 속도 차이가 특정 약관측 방향의 rank/eigenvalue를 증가시킨다.
- common-mode 조건에서 `보정 불가` 또는 `절대 anchor 필요` 판정이 동작한다.

---

## 9. Stage 4: 방향별 정보 이득 기반 최소 협력 기동

### 9.1 목적

고정된 위치에서 항상 rendezvous하지 않고, 현재 약한 방향과 후보 기동의 예상 정보 이득을 이용해 필요한 경우에만 최소 기동을 선택한다.

### 9.2 후보 action set

초기에는 연속 최적화보다 작은 discrete action set이 안전하고 재현성이 높다.

```text
A0: continue
A1: UAV 1 hover 1 s
A2: UAV 1 hover 2 s
A3: UAV 2 hover 1 s
A4: UAV 2 hover 2 s
A5: UAV 1 speed -20%, UAV 2 speed 유지
A6: UAV 2 speed -20%, UAV 1 speed 유지
A7: 허용 폭 내 lateral offset
A8: 짧은 arc
```

### 9.3 Action score

\[
S(a)=\Delta I_W(a)
-\lambda_t C_{time}(a)
-\lambda_e C_{energy}(a)
-\lambda_r C_{risk}(a)
-\lambda_c C_{communication}(a)
\]

각 비용은 실험 전에 정규화한다.

- (C_{time}): 목표 도달시간 증가
- (C_{energy}): 이동거리·가감속·hover 시간을 이용한 proxy
- (C_{risk}): 최소 UAV 간 거리, 장애물 여유, lane 이탈 가능성
- (C_{communication}): 측정/graph 전송량과 예상 지연

### 9.4 Trigger 조건

다음 조건을 모두 만족할 때만 협력 기동을 요청한다.

1. LiDAR 약관측 상태가 일정 시간 이상 지속
2. 예상 error growth가 설정 범위보다 큼
3. 현재 passive 측정의 정보 이득이 부족
4. 후보 기동 중 최소 하나가 요구 정보 이득을 제공
5. 안전·통신·배터리 조건 만족
6. cooldown 시간 이후

반대로 다음이면 기동하지 않는다.

- SLAM이 정상
- passive 측정만으로 충분
- 후보 기동으로도 target weak direction 정보가 증가하지 않음
- common-mode라 peer measurement만으로 복구 불가
- 장애물 또는 통신 문제로 안전 조건 미충족

### 9.5 상태 머신

```text
NORMAL_MAPPING
    ↓ degeneracy persistent
EVALUATE_RECOVERABILITY
    ├─ passive factor sufficient → APPLY_FACTOR
    ├─ maneuver useful           → REQUEST_MANEUVER
    └─ unrecoverable             → REPORT_UNRECOVERABLE

REQUEST_MANEUVER
    ↓ peer accepted + safety valid
EXECUTE_MANEUVER
    ↓ enough measurements
VERIFY_FACTOR
    ├─ accepted → OPTIMIZE_AND_REPROJECT
    └─ rejected → RESUME_WITHOUT_CORRECTION

OPTIMIZE_AND_REPROJECT
    ↓ revision published
NORMAL_MAPPING
```

### 9.6 안전 제약

- minimum inter-UAV distance
- lane boundary
- obstacle clearance
- maximum hover duration
- maximum additional mission time
- stale pose/scan/UWB 시 maneuver 중단
- peer response timeout 시 local-only mode 복귀

### 9.7 비교 실험

| 방법 | Trigger | Maneuver |
|---|---|---|
| Fixed schedule | 일정 거리마다 | 고정 hover |
| Scalar threshold | covariance trace 임계값 | 고정 hover |
| Proposed | weak-direction information gain | 최소 score action |
| Oracle | ground truth error 기준 | 가장 좋은 action |

Oracle은 성능 상한 평가에만 쓰며 제안 알고리즘 입력으로 사용하지 않는다.

### 9.8 완료 기준

- fixed schedule보다 적은 기동 횟수 또는 추가 시간으로 같거나 낮은 ATE를 달성한다.
- scalar threshold보다 map IoU 또는 double-wall ratio가 개선된다.
- 정보 이득이 없는 상황에서 불필요한 rendezvous를 억제한다.
- 안전거리 위반과 lane 이탈이 없다.

---

## 10. Stage 5: Online incremental pose graph와 map revision

### 10.1 목적

Offline에서 검증한 보정을 실시간 ROS 2 pipeline으로 옮긴다.

### 10.2 권장 node 구조

```text
/droneN/submap_builder
    └─ frozen submap + initial pose

/droneN/observability_monitor
    └─ weak direction + covariance

/swarm/relative_measurement_adapter
    └─ synchronized range/bearing factor candidate

/swarm/recoverability_manager
    └─ information gain + maneuver decision

/swarm/joint_pose_graph
    └─ optimized submap poses + revision

/swarm/submap_projector
    └─ revised global OccupancyGrid
```

### 10.3 권장 topic contract

```text
/droneN/slam/submap
/droneN/slam/submap_pose
/droneN/slam/observability
/swarm/relative_measurements
/swarm/factor_candidates
/swarm/recoverability_status
/swarm/maneuver_request
/swarm/maneuver_status
/swarm/optimized_submap_poses
/swarm/map_revision
/swarm/global_map_corrected
```

### 10.4 Incremental 최적화

초기 구현은 GTSAM batch optimizer로 시작하고, 결과가 맞으면 iSAM2로 전환한다.

권장 순서:

1. Offline batch GTSAM
2. rosbag replay 중 batch window
3. iSAM2 incremental update
4. live simulation

최적화 latency가 planner 주기를 방해하지 않도록 별도 thread/process에서 수행한다. map revision이 발생할 때 planner가 갑자기 큰 pose jump를 받지 않도록 pose/map version 정책을 명시한다.

### 10.5 Map revision 정책

- optimizer result가 confidence gate를 통과할 때만 revision 증가
- 수정량이 너무 크면 즉시 적용하지 않고 `SUSPECTED_CORRECTION` 상태로 기록
- global planner는 revision 변경을 감지해 path를 다시 생성
- local planner는 짧은 시간 local map/odom을 계속 사용
- optimization 실패 시 마지막 valid revision 유지

### 10.6 통신 장애 실험

- delay: 0, 50, 100, 300, 500 ms
- dropout: 0, 10, 30, 50%
- burst loss
- 한 UAV의 submap 전송 중단
- out-of-order message
- clock offset

평가:

- map consistency
- stale factor rejection
- optimizer latency p50/p95
- DDS bandwidth
- local-only fallback 성공률

---

## 11. Stage 6: 실제 상대센서 적용

### 11.1 센서 우선순위

현재 lane 간격이 약 15 m이고 LiDAR 최대 range가 약 8 m이므로, 현재 geometry를 유지하면 LiDAR로 peer를 직접 안정적으로 관측하기 어렵다.

권장 순서:

1. Gazebo synthetic UWB range
2. Gazebo synthetic range+bearing
3. 실제 UWB range
4. UWB-AoA 또는 카메라/tag bearing
5. LiDAR range 증가 또는 lane 간격 변경 후 peer detection

### 11.2 Adapter 구조

Synthetic sensor와 real sensor가 동일한 내부 message contract를 사용하도록 한다.

```text
synthetic UWB ─┐
actual UWB ────┼→ relative_measurement_adapter → graph factor
camera/tag ────┘
```

공통 필드:

- source UAV / target UAV
- source/target timestamp
- measurement type
- range, bearing 또는 relative pose
- covariance
- sensor health
- line-of-sight/NLOS flag
- sequence ID

### 11.3 Sim-to-real 검증

- simulation에서 real sensor noise 분포를 재현
- hardware stationary calibration
- known baseline 거리에서 bias 측정
- timestamp/clock synchronization 검증
- NLOS outlier 수집
- 동일한 offline bag replay로 factor gate 검증
- 짧은 거리 실제 실험 후 긴 one-way 실험으로 확장

---

## 12. 최종 비교군 설계

| ID | 조건 | 목적 |
|---|---|---|
| B0 | UAV별 local raw SLAM | drift 기준선 |
| B1 | current fixed spawn map fusion | 현재 구현 기준선 |
| B2 | map/odometry 통신만, inter factor 없음 | 통신 자체는 정보를 만들지 않음을 확인 |
| B3 | passive parallel + range-only | 나쁜 관측 기하의 한계 |
| B4 | passive parallel + range+bearing | 센서 종류의 효과 |
| B5 | 고정 거리/시간마다 hover | scheduled rendezvous 기준선 |
| B6 | scalar covariance threshold + hover | 기존 단순 trigger 기준선 |
| B7 | weak-direction gain + 최소 기동 | 제안 방식 |
| B8 | 한 UAV가 복귀하여 self-loop 수행 | 전통 loop closure 상한 |
| B9 | dedicated beacon/leapfrog 방식 | Beacon 계열 상한 비교 |
| B10 | 중간 fixed anchor 추가 | common-mode 제거 상한 |

모든 조건에서 센서 noise, environment seed, 출발 위치와 mission goal을 가능한 한 동일하게 유지한다.

---

## 13. Ablation 실험

제안 방식의 어떤 요소가 실제 성능을 만드는지 다음처럼 분리한다.

1. eigenvalue threshold만 사용 vs eigenvector 방향까지 사용
2. scalar covariance trigger vs directional gain trigger
3. range-only vs range+bearing
4. passive vs hover vs speed difference vs lateral/arc
5. 모든 상태 차원에 factor 적용 vs 검증된 observable subspace에만 적용
6. 전체 OccupancyGrid rigid transform vs submap pose 재투영
7. robust loss만 사용 vs 사전 consistency gate 추가
8. common-mode 판정 없음 vs 판정 있음
9. 모든 submap 전송 vs 정보가 큰 submap만 전송
10. batch optimizer vs incremental optimizer

---

## 14. 평가 지표와 계산 방법

### 14.1 궤적

#### ATE

시간 정렬과 좌표계 정렬 후 추정 pose와 ground truth 사이의 전역 오차를 계산한다.

#### RPE

고정 시간 또는 거리 간격의 상대 이동 오차를 계산해 local drift를 평가한다.

#### Endpoint drift

one-way 비행에서는 마지막 위치 오차가 실제 임무 관점에서 중요하다.

### 14.2 지도

#### Occupied IoU

\[
IoU=\frac{|M_{est}^{occ}\cap M_{gt}^{occ}|}
{|M_{est}^{occ}\cup M_{gt}^{occ}|}
\]

#### Chamfer distance

추정 occupied cell과 ground-truth occupied cell 사이의 최근접 거리 평균 또는 percentile을 사용한다.

#### Double-wall ratio

하나의 실제 벽 주변에 평행한 occupied band가 두 개 이상 생긴 셀 비율을 측정한다.

#### Ghost obstacle ratio

ground truth obstacle에서 설정 거리 이상 떨어진 estimated occupied cell의 비율을 측정한다.

### 14.3 관측가능성과 factor 품질

- 최소 고유값
- condition number
- weak eigenvector 방향
- candidate action 전후 정보 이득
- graph rank/nullspace dimension
- factor precision/recall
- accepted/rejected factor 수
- normalized residual/NIS

### 14.4 임무 비용과 안전

- 목표 도달시간 증가
- 추가 이동거리
- hover 총시간
- energy proxy
- maneuver 횟수
- minimum inter-UAV distance
- obstacle clearance
- lane violation
- mission abort/collision 횟수

### 14.5 시스템

- optimizer p50/p95 latency
- map reconstruction latency
- DDS bytes/s
- CPU/memory
- map update rate
- stale duration
- fallback count

---

## 15. 통계 분석

### 15.1 반복과 seed

- 개발 확인: 조건당 3회
- 중간 결과: 조건당 5회
- 논문 결과: 조건당 최소 10회, 가능하면 20회
- 모든 방법에 동일 seed set 사용

### 15.2 보고 방식

평균만 보고하지 않는다.

- mean ± standard deviation
- median과 IQR
- 95% confidence interval
- worst-case와 failure rate
- seed별 paired plot

방법 간 비교는 같은 seed를 공유하므로 paired test를 우선 고려한다. 분포 정규성이 불분명하면 Wilcoxon signed-rank test와 effect size를 함께 보고할 수 있다.

### 15.3 성공 정의 예시

최종 threshold는 baseline 분포를 확인한 뒤 확정하되 다음 형태로 정의한다.

- B1 대비 ATE 또는 endpoint drift 유의 감소
- B1 대비 map IoU 증가
- double-wall/ghost ratio 감소
- B6보다 같은 정확도에서 추가 시간이 감소하거나, 같은 추가 시간에서 정확도 향상
- collision/lane violation 0회
- catastrophic false correction 발생률이 허용치 이하

---

## 16. 권장 artifact 구조

```text
artifacts/<run_id>/
├─ manifest.yaml
├─ parameter_snapshot.json
├─ rosbag/
├─ ground_truth/
├─ drone1/
│  ├─ trajectory.csv
│  ├─ observability.csv
│  ├─ submaps/
│  └─ local_graph.g2o
├─ drone2/
│  ├─ trajectory.csv
│  ├─ observability.csv
│  ├─ submaps/
│  └─ local_graph.g2o
├─ swarm/
│  ├─ relative_measurements.csv
│  ├─ factor_decisions.csv
│  ├─ maneuver_events.csv
│  ├─ optimized_graph.g2o
│  ├─ optimized_submap_poses.csv
│  ├─ map_revision_history.csv
│  └─ corrected_map.npy
├─ metrics/
│  ├─ trajectory_metrics.json
│  ├─ map_metrics.json
│  ├─ observability_metrics.json
│  ├─ mission_metrics.json
│  └─ system_metrics.json
└─ plots/
```

모든 결과에는 다음 provenance를 남긴다.

- git commit
- dirty worktree 여부
- world/seed
- manifest와 parameter snapshot
- sensor noise model
- graph/optimizer 설정
- run 시작·종료 시각

---

## 17. 권장 코드 확장 위치

현재 패키지 구조를 유지한다면 다음처럼 역할을 분리할 수 있다.

```text
src/drone_slam/
├─ submap_builder_node
├─ observability_monitor_node
└─ local_factor_exporter

src/drone_map_fusion/
├─ relative_measurement_adapter
├─ joint_pose_graph_node
├─ recoverability_manager_node
├─ corrected_map_projector_node
└─ map_revision_manager

src/drone_control 또는 drone_planning/
└─ cooperative_maneuver_executor
```

기존 `map_fusion_node`는 B1 baseline으로 유지한다. 바로 삭제하거나 덮어쓰지 말고, 제안 방식은 새로운 corrected map topic으로 병렬 실행한다.

```text
/swarm/global_map                # 기존 fixed fusion baseline
/swarm/global_map_corrected      # 제안 submap graph 방식
```

이렇게 해야 동일한 run에서 두 방법을 공정하게 비교할 수 있다.

---

## 18. 구현 우선순위와 현실적인 마일스톤

### Milestone 1. Offline graph proof

- 기존 rosbag에서 submap 생성
- synthetic inter-UAV factor 생성
- batch PGO
- corrected map 재생성

완료 조건: fixed fusion보다 지도 지표 개선.

### Milestone 2. Observability proof

- LiDAR Hessian/covariance 기록
- weak direction과 실제 drift 방향 비교
- passive/hover/speed action 정보량 비교

완료 조건: predicted gain과 실제 오차 감소의 연관성 확인.

### Milestone 3. Proposed trigger

- recoverability 판정
- discrete action selector
- scalar threshold 기준선과 비교

완료 조건: 더 적은 임무비용으로 같거나 나은 지도 정확도.

### Milestone 4. Online system

- incremental graph
- map revision
- planner/fallback 연동
- 통신 지연·dropout 실험

완료 조건: live simulation에서 안전하게 반복 성공.

### Milestone 5. Real sensor or strong simulation study

- 실제 UWB/vision adapter 또는 현실적 noise/NLOS 모델
- 다양한 환경·seed·sensor 조건
- 통계 분석과 논문 figure 생성

---

## 19. 주요 위험과 대응

### 위험 1. 다른 UAV의 측정을 넣었지만 지도 오차가 줄지 않음

가능한 원인:

- factor 한 개만 있어 graph cycle이 없음
- relative measurement가 LiDAR 약관측 방향과 직교
- 두 UAV의 common-mode drift
- factor covariance가 지나치게 큼
- submap reprojection이 구현되지 않음

대응:

- factor 수와 공간적 간격 확인
- (v_{weak}^TH_Rv_{weak}) 기록
- graph nullspace 분석
- 중간 fixed anchor 대조군 추가

### 위험 2. 보정 후 좋은 UAV까지 나빠짐

가능한 원인:

- 두 UAV factor에 동일 confidence를 부여
- 잘못된 inter-UAV association
- robust loss만으로 큰 outlier를 방어하려 함

대응:

- UAV별 odometry/scan covariance를 실제 residual에 맞춰 calibration
- consistency gate와 switchable constraint 추가
- factor acceptance/rejection 로그 보존

### 위험 3. Hessian eigenvalue가 환경마다 스케일이 다름

가능한 원인:

- point 수, range, translation/yaw 단위가 섞임

대응:

- normalization 명시
- covariance 또는 sub-block score 병행
- 절대 threshold 대신 baseline-normalized score 검토

### 위험 4. 반복 구조에서 false loop가 많음

대응:

- geometric score 하나만 사용하지 않음
- temporal/motion/cycle consistency 추가
- 여러 연속 factor의 합의가 있을 때만 map revision

### 위험 5. Active maneuver가 임무 성능을 크게 악화

대응:

- discrete action과 최대 추가시간 제한
- passive measurement가 충분하면 기동하지 않음
- information gain per cost로 평가
- hover와 속도 차이를 먼저 비교

---

## 20. 논문에서 가능한 주장과 피해야 할 주장

### 검증 후 가능한 주장 후보

> 비중첩 평행 lane과 무복귀 제약 아래, 로컬 2D LiDAR의 약관측 방향과 inter-UAV 측정의 방향별 정보 이득을 공동 분석해 보정 가능성을 판정하고, 필요한 경우에만 최소 차선 내 협력 기동을 선택한다.

> 제안 방법은 fixed map fusion과 scalar uncertainty trigger보다 적은 임무비용으로 differential drift와 map double-wall을 감소시킨다.

> 또한 pairwise relative measurement로 제거할 수 없는 common-mode 방향을 검출해 정보가 없는 잘못된 보정을 억제한다.

### 피해야 할 주장

- 다른 드론과 통신하면 SLAM drift가 제거된다.
- 상대측정 하나로 두 전체 지도를 보정할 수 있다.
- range-only 측정은 항상 진행축 오차를 보정한다.
- pairwise relative measurement로 global absolute drift를 모두 제거한다.
- 2D LiDAR 직선 비행에서는 `x` 또는 yaw가 항상 퇴화한다.
- active rendezvous 또는 degeneracy-aware fusion 자체가 최초다.

---

## 21. 바로 시작할 작업 체크리스트

### 첫 구현 주간

- [ ] 기존 137 m rosbag 하나를 기준 데이터로 선정
- [ ] LaserScan과 pose를 시간 정렬하는 offline loader 작성
- [ ] UAV별 keyframe/submap 포맷 결정
- [ ] submap 초기 pose와 local grid 저장
- [ ] odometry edge만 있는 SE(2) graph 생성
- [ ] graph serialization과 시각화

### 두 번째 구현 주간

- [ ] synthetic range/range+bearing 생성기
- [ ] inter-UAV factor 한 개와 여러 개 비교
- [ ] batch PGO
- [ ] optimized submap pose 저장
- [ ] corrected OccupancyGrid 재투영
- [ ] B1 fixed fusion과 map metric 비교

### 세 번째 구현 주간

- [ ] scan Hessian/covariance export
- [ ] eigenvalue/eigenvector 기록
- [ ] corner/parallel/repetitive 환경 비교
- [ ] weak direction과 future RPE 연관 분석

### 네 번째 이후

- [ ] passive/hover/speed action 정보 이득 비교
- [ ] common-mode bias 실험
- [ ] recoverability trigger
- [ ] online ROS node와 map revision
- [ ] 통신 장애와 실제 센서 확장

---

## 22. 서버에서 로컬 PC로 파일 옮기기

이 문서의 서버 절대경로는 다음과 같다.

```text
/home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md
```

아래 명령은 **서버가 아니라 로컬 PC의 터미널에서 실행**한다.

### 22.1 `scp`로 문서 한 개 내려받기

서버 계정이 `deepblue`, 서버 주소가 `166.104.230.197`인 경우:

```bash
scp deepblue@166.104.230.197:/home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md .
```

로컬의 Downloads 폴더로 저장하려면 Linux/macOS에서 다음처럼 실행한다.

```bash
scp deepblue@166.104.230.197:/home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md ~/Downloads/
```

Windows PowerShell에서 OpenSSH `scp`를 사용하는 경우:

```powershell
scp deepblue@166.104.230.197:/home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md "$HOME\Downloads"
```

서버 계정 또는 주소가 다르면 `deepblue@166.104.230.197` 부분만 실제 SSH 접속 정보로 바꾼다.

### 22.2 SSH 포트가 기본 22번이 아닌 경우

예를 들어 SSH 포트가 2222라면 대문자 `-P`를 사용한다.

```bash
scp -P 2222 deepblue@166.104.230.197:/home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md ~/Downloads/
```

### 22.3 `rsync`로 내려받기

같은 파일을 여러 번 갱신해서 받을 때는 `rsync`가 편리하다.

```bash
rsync -avP deepblue@166.104.230.197:/home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md ~/Downloads/
```

SSH 포트가 다른 경우:

```bash
rsync -avP -e "ssh -p 2222" deepblue@166.104.230.197:/home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md ~/Downloads/
```

### 22.4 특정 실험 artifact 폴더 전체 내려받기

`<RUN_ID>`를 실제 run ID로 바꾼다.

```bash
scp -r deepblue@166.104.230.197:/home3/deepblue/work/AV_Drone/artifacts/<RUN_ID> ~/Downloads/
```

중단 후 이어받기와 진행률 확인이 필요하면 `rsync`를 사용한다.

```bash
rsync -avP deepblue@166.104.230.197:/home3/deepblue/work/AV_Drone/artifacts/<RUN_ID>/ ~/Downloads/<RUN_ID>/
```

### 22.5 서버에서 경로와 파일 크기 확인

로컬에서 복사하기 전에 서버 터미널에서 다음을 실행할 수 있다.

```bash
ls -lh /home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md
```

문서의 절대경로 확인:

```bash
realpath /home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md
```

### 22.6 SSH config 별칭을 사용하는 경우

로컬의 `~/.ssh/config`에 서버가 다음처럼 등록되어 있다고 가정한다.

```sshconfig
Host av-drone
    HostName 166.104.230.197
    User deepblue
```

그러면 명령을 짧게 쓸 수 있다.

```bash
scp av-drone:/home3/deepblue/work/AV_Drone/docs/recoverability-aware-multi-uav-slam-implementation-and-experiment-plan.md ~/Downloads/
```

---

## 23. 참고할 현재 프로젝트 문서

- `docs/research-topic-and-prior-art-summary.md`
- `docs/communication-assisted-multi-uav-map-stitching.md`
- `docs/multi-uav-mapping-gate-c.md`
- `experiments/run_reports/2026-08-04_slam_root_cause_and_fallback.md`
- `multi_uav_slam_map_stitching_expansion_plan.md`

이 문서의 Stage 0~1은 현재 known-pose/fixed-fusion baseline을 보존하면서 새로운 submap graph 방식을 병렬로 추가하는 것을 전제로 한다.
