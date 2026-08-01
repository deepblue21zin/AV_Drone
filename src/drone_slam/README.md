# drone_slam

Gap 기반 outbound 비행 중 관측한 LiDAR 데이터를 2D occupancy grid 지도로 축적하고,
비행이 끝나면 그 지도를 파일로 저장하는 패키지. localization은 시뮬레이터가 제공하는
`/mavros/local_position/pose`(ground truth)를 그대로 쓰기 때문에, 이 패키지가 실제로
푸는 문제는 SLAM 중 **Mapping** 쪽뿐이다 (자세한 연구 배경은 `docs/research_topic_direction.md` 참고).

## 코드 파일 구성

### `drone_slam/__init__.py`
빈 패키지 초기화 파일. 로직 없음.

### `drone_slam/slam_scaffold_node.py` — 실행: `slam_scaffold_node`
실제 SLAM/매핑을 하지 않는 **placeholder 노드**. 팀 분리 개발 초기에 scan+pose 입력 경로와
`/drone1/slam/*` 상태 토픽(`status`, `input_ready`, `map_ready`, `localization_ok`, `coverage`)의
인터페이스만 미리 잡아두기 위한 자리표시용이다. `map_ready`/`localization_ok`는 항상 `false`,
`coverage`는 항상 `0.0`만 publish한다. `single_drone_slam_dev.launch.py`(회피 스택 없이 SLAM
입력 경로만 확인하는 개발 프로필)에서 쓰인다. **실제 비행/매핑 실행에는 안 쓰인다** —
`simple_2d_mapping_node`로 대체됨.

### `drone_slam/simple_2d_mapping_node.py` — 실행: `simple_2d_mapping_node`
실질적인 매핑 로직. `/drone1/scan`(LaserScan) + `/mavros/local_position/pose`(PoseStamped)를
구독해서 log-odds 기반 2D occupancy grid를 누적하고, `/map`(`nav_msgs/OccupancyGrid`)으로 계속
publish한다.

- 스캔이 들어올 때마다 각 레이저 광선을 따라 통과 지점은 "빈 공간"으로, 장애물에 맞은 지점은
  "점유"로 log-odds를 갱신 (`_on_scan` → `_mark_free_ray` / `_add_log_odds`)
- 격자 범위/해상도는 파라미터로 설정 (`map_resolution`, `map_min_x/max_x/min_y/max_y`) — 범위
  밖 좌표는 조용히 무시됨. 지금 `drone1_avoidance_dev.yaml`은 150m 코스 전체를 덮도록
  `-2 ~ 152 (x)`, `-16 ~ 16 (y)`로 설정되어 있음.
- `publish_hz`(기본 2Hz)마다 누적된 grid를 OccupancyGrid로 변환해서 publish. 셀 값 인코딩은
  아래 "저장되는 지도 파일" 절과 동일 (`-1`/`0`/`1~100`).
- 이 노드가 자체적으로 지도를 파일로 저장하지는 않는다 — 그건 `map_saver_node`의 책임.

### `drone_slam/map_saver_node.py` — 실행: `map_saver_node`
`/map`을 구독해서 최신 메시지를 계속 캐싱해두다가, `/drone1/mission/phase`가 지정된 값
(기본 `"LANDED"`)이 되는 순간 **딱 한 번** 파일로 저장하는 노드. `simple_2d_mapping_node`와는
분리된 별도 노드로 두어서(이 프로젝트의 `metrics_logger`와 같은 "관측 노드 + 별도 기록 노드"
패턴), 매핑 노드 자체는 계산에만 집중하게 했다.

- 트리거: `mission_phase_topic`(기본 `/drone1/mission/phase`)이 `save_on_phase`(기본
  `"LANDED"`)와 일치하는 순간. 그 시점까지 받은 가장 최신 `/map` 메시지를 저장한다.
- 저장 위치/파일: 아래 절 참고.
- 한 번 저장하면 내부 플래그로 재저장하지 않음 (phase heartbeat가 계속 와도 무시).

## 저장되는 지도 파일

### 위치

```
maps/<scenario_name>/
  <map_file_basename>_grid.npy
  <map_file_basename>_meta.json
```

- 루트는 파라미터 `maps_root` (기본값 `/workspace/AV_Drone/maps`, 즉 컨테이너 안에서 이 레포
  루트 바로 아래 `maps/` 디렉터리)
- 하위 폴더명은 `scenario_name` 파라미터 값 (지금 설정 기준 `single_drone_obstacle_demo`)
- 파일명 접두어는 `map_file_basename` 파라미터 (기본값 `"map"` → `map_grid.npy`/`map_meta.json`).
  `drone1_avoidance_dev.yaml`엔 지금 `"obstacle_demo_v2"`로 설정되어 있어서 실제로는
  `obstacle_demo_v2_grid.npy` / `obstacle_demo_v2_meta.json`으로 저장된다 — world 버전별로
  지도를 구분해서 남기고 싶을 때 이 값만 바꾸면 됨.
- **`map_file_basename`이 같으면 매 실행마다 같은 파일을 덮어쓴다.** 실험용
  `artifacts/<run_id>/`처럼 실행마다 새 폴더가 생기지 않는다 — SLAM을 다시 돌리면 최신
  지도로 갱신되고, 이후 실험 노드들은 이 고정된 경로 하나만 보면 되도록 의도한 설계.

### `map_grid.npy`

`numpy.save()`로 저장된 2차원 `int8` 배열, **shape = `(height, width)`** (row-major, y가 행,
x가 열). `OccupancyGrid.data`(1차원 flat 배열)를 `height x width`로 reshape한 것과 동일.

셀 값 의미 (`simple_2d_mapping_node._to_occupancy_data()`와 동일한 인코딩, Foxglove에서 보이는
회색/흰색/검정과 대응):

| 값 | 의미 | Foxglove에서 |
|---|---|---|
| `-1` | 미관측(unknown) | 회색 |
| `0` | 빈 공간(free) | 흰색 |
| `1 ~ 100` | 점유 확률(숫자가 클수록 장애물일 가능성 높음) | 검정(진할수록 높은 확률) |

로드 예시:
```python
import numpy as np
grid = np.load("maps/single_drone_obstacle_demo/obstacle_demo_v2_grid.npy")  # map_file_basename 기준
grid.shape   # (267, 1284)  == (height, width)
grid.dtype   # dtype('int8')
```

### `map_meta.json`

`map_grid.npy`를 실제 world 좌표로 재해석하는 데 필요한 메타데이터. 예시:

```json
{
  "resolution": 0.12,
  "width": 1284,
  "height": 267,
  "origin_x": -2.0,
  "origin_y": -16.0,
  "origin_z": 0.0,
  "frame_id": "map",
  "scenario_name": "single_drone_obstacle_demo",
  "saved_at": "2026-07-17T14:32:05"
}
```

- `resolution`: 셀 한 칸의 한 변 길이(m/cell)
- `origin_x/y/z`: `map_grid[0, 0]`(배열의 첫 행·첫 열) 셀의 world 좌표 — 즉 grid 인덱스
  `(row, col)`을 world 좌표로 바꾸려면:
  ```python
  world_x = origin_x + (col + 0.5) * resolution
  world_y = origin_y + (row + 0.5) * resolution
  ```
- `frame_id`: 이 지도가 정의된 좌표계 이름 (지금은 항상 `"map"`, TF 트리는 안 씀 — 이 프로젝트의
  모든 위치 관련 토픽이 같은 `"map"` frame_id 문자열을 공유하는 것으로 사실상 정합성을 맞춤)
- `scenario_name`, `saved_at`: 이 지도가 어떤 시나리오/언제 생성됐는지 추적용

## 관련 launch/설정

- `single_drone_avoidance_dev.launch.py`: `simple_2d_mapping_node` + `map_saver_node`가
  Gap 회피 스택과 함께 뜬다 (SLAM 전용 실행 없이, 실제 미션 비행 중에 지도를 같이 만듦).
- `drone1_avoidance_dev.yaml`: `map_topic`, `map_frame_id`, `map_resolution`,
  `map_min_x/max_x/min_y/max_y` 등 매핑 관련 파라미터, `scenario_name`, `mission_phase_topic`
  (map_saver가 재사용).
- `single_drone_slam_dev.launch.py` / `drone1_slam_dev.yaml`: `slam_scaffold_node` 단독 개발용
  프로필 (실제 매핑 아님, 위 참고).
