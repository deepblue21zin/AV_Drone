# single_drone_obstacle_demo 지도 파일

`map_saver_node`(`drone_slam` 패키지)가 Gap 회피 비행 중 만든 occupancy grid를 착륙 시점에
저장한 결과물. `ros2 launch drone_bringup single_drone_avoidance_dev.launch.py`를 다시 돌리면
이 폴더의 두 파일이 최신 결과로 덮어써진다.

## 이 폴더의 파일

| 파일 | 내용 |
|---|---|
| `map_grid.npy` | 지도 격자 데이터 본체 (숫자 배열) |
| `map_meta.json` | 그 격자를 실제 world 좌표로 해석하는 데 필요한 메타데이터 |

## 1. `map_meta.json` 읽는 법

그냥 텍스트 파일이라 아무 에디터로 열면 됩니다. 지금 저장된 내용 예시:

```json
{
  "resolution": 0.11999999731779099,
  "width": 1284,
  "height": 267,
  "origin_x": -2.0,
  "origin_y": -16.0,
  "origin_z": 0.0,
  "frame_id": "map",
  "scenario_name": "single_drone_obstacle_demo",
  "saved_at": "2026-07-17T13:48:31"
}
```

- `resolution`: 격자 한 칸의 한 변 길이 (미터/칸). `0.1199999...`처럼 `0.12`가 아니라 미세하게
  다른 건 원본 ROS 메시지가 float32라서 생기는 정밀도 오차 — 정상, 무시해도 됨.
- `width`, `height`: 격자의 가로(칸 수), 세로(칸 수)
- `origin_x`, `origin_y`, `origin_z`: 격자의 `[0, 0]` 칸(맨 왼쪽 아래)이 실제 world 좌표계에서
  어디에 해당하는지
- `frame_id`: 좌표계 이름 (이 프로젝트는 항상 `"map"`)
- `scenario_name`: 어느 시나리오/월드에서 만든 지도인지
- `saved_at`: 저장된 시각 (로컬 시각 기준)

## 2. `map_grid.npy` 읽는 법 (Python)

```python
import numpy as np

grid = np.load("map_grid.npy")

print(grid.shape)  # (267, 1284)  == (height, width)  ← map_meta.json 값과 반드시 일치해야 정상
print(grid.dtype)  # int8
```

`np.load()` 한 줄이면 바로 2차원 배열로 들어옵니다. **`.npy`는 바이너리 파일이라 텍스트
에디터로 열면 깨져 보이는 게 정상**이고, 반드시 numpy(또는 numpy 호환 도구)로 읽어야 합니다.

### 각 셀 값의 의미

| 값 | 의미 | Foxglove 색 |
|---|---|---|
| `-1` | 미관측 (한 번도 안 본 영역) | 회색 |
| `0` | 빈 공간 (지나갈 수 있음) | 흰색 |
| `1 ~ 100` | 장애물일 확률 (%). 클수록 확실한 장애물 | 검정 (진할수록 높음) |

### 배열 인덱스 순서 주의

`grid[row, col]` = `grid[y_index, x_index]` 순서입니다 (numpy의 기본 `(height, width)` 관례).
`x`, `y` 순서로 헷갈리기 쉬우니 주의.

## 3. 격자 인덱스 ↔ 실제 world 좌표 변환

`map_meta.json`의 `resolution`, `origin_x`, `origin_y`를 이용:

```python
import json
import numpy as np

grid = np.load("map_grid.npy")
meta = json.load(open("map_meta.json"))

resolution = meta["resolution"]
origin_x = meta["origin_x"]
origin_y = meta["origin_y"]

# 격자 인덱스 -> world 좌표 (칸의 중심 기준)
def grid_to_world(row: int, col: int) -> tuple[float, float]:
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y

# world 좌표 -> 격자 인덱스
def world_to_grid(x: float, y: float) -> tuple[int, int]:
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row, col

# 예: 목표 지점(140, 0) 근처가 실제로 빈 공간(0)으로 관측됐는지 확인
row, col = world_to_grid(140.0, 0.0)
print(grid[row, col])
```

## 4. 자주 쓸만한 확인/분석 코드

**전체 통계 (관측 비율, 장애물 개수 등):**
```python
total = grid.size
unknown = int((grid == -1).sum())
free = int((grid == 0).sum())
occupied = int((grid >= 1).sum())

print(f"전체 칸: {total}")
print(f"미관측: {unknown} ({100*unknown/total:.1f}%)")
print(f"빈 공간: {free} ({100*free/total:.1f}%)")
print(f"장애물(추정): {occupied} ({100*occupied/total:.1f}%)")
print(f"커버리지(관측된 비율) = {100*(total-unknown)/total:.1f}%")
```

**이미지로 눈으로 확인 (matplotlib):**
```python
import matplotlib.pyplot as plt

display = grid.astype(float)
display[grid == -1] = 128   # 미관측 -> 회색
display[grid == 0] = 255    # 빈 공간 -> 흰색
# grid >= 1 인 곳은 이미 1~100이라 그대로 두면 어둡게(=진한 회색~검정) 보임

plt.imshow(255 - display, cmap="gray", origin="lower")
plt.title("single_drone_obstacle_demo map")
plt.savefig("map_preview.png")
```

## 참고

코드 쪽(어떤 노드가 어떻게 이 파일을 만드는지)은 `src/drone_slam/README.md`를 참고하세요.
