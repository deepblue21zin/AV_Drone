#!/usr/bin/env python3
import argparse
import math
import random
from pathlib import Path


# ============================================================
# 반복 실험용 파라미터
# 아래 값만 바꾼 뒤 이 파일을 실행하면 된다.
#
# 생성 파일:
# - random_cylinders_double_<시드>.world
# - random_cylinders_double_<시드>_positions.txt
# - random_cylinders_double_<시드>_map.png
# ============================================================

# None이면 실행할 때마다 새 시드를 자동 생성한다.
# 같은 월드를 재생성하려면 정수 시드를 지정한다.
DEFAULT_SEED = None

DEFAULT_NUM_OBSTACLES = 100
# 장애물 표면 사이의 최소 여유 거리 [m]
DEFAULT_MIN_GAP = 4.0

# 실제 파일명 뒤에는 항상 _<시드>가 자동으로 붙는다.
DEFAULT_WORLD_NAME = "random_cylinders_double"

# 이 파일을 /workspace/AV_Drone/sim_assets/worlds 안에 둘 예정이므로
# 기본 출력 폴더는 스크립트가 위치한 worlds 폴더로 고정한다.
DEFAULT_OUTPUT_DIR = str(Path(__file__).resolve().parent)

# 맵 크기
DEFAULT_X_MIN = 0.0
DEFAULT_X_MAX = 150.0
DEFAULT_Y_MIN = -15.0
DEFAULT_Y_MAX = 15.0

# 원기둥 크기
# 실제 원기둥 형상은 sim_assets/models/cylinder_r05_h5/model.sdf가 결정한다.
# 여기 값은 랜덤 배치 시 간격 계산 및 로그 기록용이다.
DEFAULT_OBSTACLE_RADIUS = 0.5
DEFAULT_OBSTACLE_HEIGHT = 5.0

# 벽에서 원기둥 중심이 최소로 떨어지는 여유 거리
DEFAULT_WALL_MARGIN = 2.5

# 시작/목표 위치와 X축 기준 장애물 생성 금지 구역 [m]
DEFAULT_START_X = 0.0
DEFAULT_START_Y = 0.0
DEFAULT_GOAL_X = 150.0
DEFAULT_GOAL_Y = 0.0

# 장애물 표면이 아래 구간 안으로 들어가지 않도록 배치한다.
# 시작 구역: x_min ~ 10 m, 목표 구역: 140 m ~ x_max
DEFAULT_OBSTACLE_AREA_X_MIN = 10.0
DEFAULT_OBSTACLE_AREA_X_MAX = 140.0

DEFAULT_MAX_TRIALS = 100000

# 고정 원기둥 모델 이름
CYLINDER_MODEL_URI = "model://cylinder_r05_h5"


# ============================================================
# INTERNAL FUNCTIONS
# ============================================================

def distance_2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def is_valid_position(pos, placed, min_center_dist):
    for p in placed:
        if distance_2d(pos, p) < min_center_dist:
            return False

    return True


def make_cylinder_includes(points):
    lines = []

    for i, (x, y) in enumerate(points):
        lines.append(
            f'    <include>'
            f'<name>cylinder_{i:02d}</name>'
            f'<uri>{CYLINDER_MODEL_URI}</uri>'
            f'<pose>{x:.3f} {y:.3f} 2.5 0 0 0</pose>'
            f'</include>'
        )

    return "\n".join(lines)


def make_world(points, world_name):
    cylinder_includes = make_cylinder_includes(points)

    return f'''<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="{world_name}">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <model name="corridor_floor">
      <static>true</static>
      <pose>75 0 0.01 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>150 30 0.02</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>150 30 0.02</size></box></geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>

    <model name="left_wall">
      <static>true</static>
      <pose>75 15.15 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>150 0.3 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>150 0.3 5</size></box></geometry>
        </visual>
      </link>
    </model>

    <model name="right_wall">
      <static>true</static>
      <pose>75 -15.15 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>150 0.3 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>150 0.3 5</size></box></geometry>
        </visual>
      </link>
    </model>

    <model name="start_wall">
      <static>true</static>
      <pose>0 0 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.3 30.3 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.3 30.3 5</size></box></geometry>
        </visual>
      </link>
    </model>

    <model name="end_wall">
      <static>true</static>
      <pose>150 0 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.3 30.3 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.3 30.3 5</size></box></geometry>
        </visual>
      </link>
    </model>

{cylinder_includes}

  </world>
</sdf>
'''


def validate_args(args):
    if args.num_obstacles < 0:
        raise ValueError("num-obstacles는 0 이상이어야 합니다.")
    if args.min_gap < 0.0 or args.obstacle_radius <= 0.0:
        raise ValueError("min-gap은 0 이상, obstacle-radius는 0보다 커야 합니다.")
    if args.x_min >= args.x_max or args.y_min >= args.y_max:
        raise ValueError("맵의 최솟값은 최댓값보다 작아야 합니다.")
    if args.wall_margin < args.obstacle_radius:
        raise ValueError(
            "wall-margin은 장애물 반지름 이상이어야 벽과 겹치지 않습니다."
        )
    if not (
        args.x_min
        <= args.obstacle_area_x_min
        < args.obstacle_area_x_max
        <= args.x_max
    ):
        raise ValueError(
            "장애물 생성 X 범위는 맵 범위 안에서 min < max여야 합니다."
        )


def generate_points(args, rng):
    obstacle_radius = args.obstacle_radius

    # 표면 간 최소간격을 중심 간 최소거리로 변환
    # 예: 반지름 0.5m, 표면 간격 4m
    # 중심 간 최소거리 = 0.5 + 4.0 + 0.5 = 5.0m
    min_center_dist = args.min_gap + 2.0 * obstacle_radius

    # 장애물의 표면까지 생성 허용 구역 안에 있도록 반지름만큼 안쪽에 둔다.
    x_min = args.obstacle_area_x_min + obstacle_radius
    x_max = args.obstacle_area_x_max - obstacle_radius
    y_min = args.y_min + args.wall_margin
    y_max = args.y_max - args.wall_margin

    if x_min > x_max or y_min > y_max:
        raise ValueError("장애물을 배치할 수 있는 유효 영역이 없습니다.")

    points = []

    for i in range(args.num_obstacles):
        placed = False

        for _ in range(args.max_trials):
            x = rng.uniform(x_min, x_max)
            y = rng.uniform(y_min, y_max)
            candidate = (x, y)

            if is_valid_position(candidate, points, min_center_dist):
                points.append(candidate)
                placed = True
                break

        if not placed:
            raise RuntimeError(
                f"Failed to place obstacle {i}. "
                f"배치하지 못했습니다. 장애물 수 또는 최소 간격을 줄이세요."
            )

    return points


def write_position_log(path, args, seed, points):
    min_center_dist = None

    for i in range(len(points)):
        for j in range(i):
            d = distance_2d(points[i], points[j])
            if min_center_dist is None or d < min_center_dist:
                min_center_dist = d

    if min_center_dist is None:
        min_center_dist = 0.0

    min_surface_gap = min_center_dist - 2.0 * args.obstacle_radius

    with path.open("w") as f:
        f.write(f"seed: {seed}\n")
        f.write(f"num_obstacles: {args.num_obstacles}\n")
        f.write(f"map_x: {args.x_min} ~ {args.x_max}\n")
        f.write(f"map_y: {args.y_min} ~ {args.y_max}\n")
        f.write(f"obstacle_radius_for_spacing: {args.obstacle_radius}\n")
        f.write(f"obstacle_height_for_log: {args.obstacle_height}\n")
        f.write(f"model_uri: {CYLINDER_MODEL_URI}\n")
        f.write(f"requested_min_surface_gap: {args.min_gap}\n")
        f.write(f"start: ({args.start_x}, {args.start_y})\n")
        f.write(f"goal: ({args.goal_x}, {args.goal_y})\n")
        f.write(
            f"obstacle_free_start_x: "
            f"{args.x_min} ~ {args.obstacle_area_x_min}\n"
        )
        f.write(
            f"obstacle_free_goal_x: "
            f"{args.obstacle_area_x_max} ~ {args.x_max}\n"
        )
        f.write(f"actual_min_center_distance: {min_center_dist:.3f}\n")
        f.write(f"actual_min_surface_gap: {min_surface_gap:.3f}\n")
        f.write("\n")

        for i, (x, y) in enumerate(points):
            f.write(f"cylinder_{i:02d}: x={x:.3f}, y={y:.3f}\n")


def write_map_image(path, args, seed, points):
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError("2D PNG 생성에 Pillow가 필요합니다.") from exc

    pixels_per_meter = 10
    margin_left = 75
    margin_right = 30
    margin_top = 55
    margin_bottom = 60
    plot_width = round((args.x_max - args.x_min) * pixels_per_meter)
    plot_height = round((args.y_max - args.y_min) * pixels_per_meter)
    width = margin_left + plot_width + margin_right
    height = margin_top + plot_height + margin_bottom

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)

    def to_pixel(x, y):
        px = margin_left + (x - args.x_min) * pixels_per_meter
        py = margin_top + (args.y_max - y) * pixels_per_meter
        return px, py

    def circle_box(center, radius):
        px, py = to_pixel(*center)
        radius_px = radius * pixels_per_meter
        return (
            px - radius_px,
            py - radius_px,
            px + radius_px,
            py + radius_px,
        )

    plot_box = (
        margin_left,
        margin_top,
        margin_left + plot_width,
        margin_top + plot_height,
    )
    draw.rectangle(plot_box, fill="#fafafa", outline="#263238")

    # 10 m 격자와 좌표 눈금
    first_x_tick = math.ceil(args.x_min / 10.0) * 10
    first_y_tick = math.ceil(args.y_min / 10.0) * 10
    for x in range(int(first_x_tick), math.floor(args.x_max) + 1, 10):
        px, _ = to_pixel(x, args.y_min)
        draw.line((px, margin_top, px, margin_top + plot_height), fill="#dddddd")
        draw.text((px - 8, margin_top + plot_height + 8), str(x), fill="#263238")
    for y in range(int(first_y_tick), math.floor(args.y_max) + 1, 10):
        _, py = to_pixel(args.x_min, y)
        draw.line((margin_left, py, margin_left + plot_width, py), fill="#dddddd")
        draw.text((margin_left - 32, py - 6), str(y), fill="#263238")

    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    overlay_draw = ImageDraw.Draw(overlay)
    forbidden_zone_specs = (
        (
            args.x_min,
            args.obstacle_area_x_min,
            "#2e7d32",
            "Start obstacle-free zone",
        ),
        (
            args.obstacle_area_x_max,
            args.x_max,
            "#c62828",
            "Goal obstacle-free zone",
        ),
    )
    for zone_x_min, zone_x_max, color, _ in forbidden_zone_specs:
        left, top = to_pixel(zone_x_min, args.y_max)
        right, bottom = to_pixel(zone_x_max, args.y_min)
        overlay_draw.rectangle(
            (left, top, right, bottom),
            fill=color + "20",
            outline=color + "cc",
        )
    image = Image.alpha_composite(image.convert("RGBA"), overlay)
    draw = ImageDraw.Draw(image)

    for i, (x, y) in enumerate(points):
        draw.ellipse(
            circle_box((x, y), args.obstacle_radius),
            fill="#37474f",
            outline="black",
        )
        px, py = to_pixel(x, y)
        draw.text((px + 5, py - 12), str(i), fill="#263238")

    marker_specs = (
        ((args.start_x, args.start_y), "#2e7d32", "Start"),
        ((args.goal_x, args.goal_y), "#c62828", "Goal"),
    )
    for center, color, label in marker_specs:
        px, py = to_pixel(*center)
        draw.line((px - 7, py, px + 7, py), fill=color, width=3)
        draw.line((px, py - 7, px, py + 7), fill=color, width=3)
        label_x = px + 9 if px < width / 2 else px - 42
        draw.text((label_x, py - 18), label, fill=color)

    title = (
        f"Random cylinder world | seed={seed} | obstacles={len(points)} | "
        f"min surface gap={args.min_gap:g} m"
    )
    draw.text((margin_left, 18), title, fill="#111111")
    draw.text(
        (margin_left + plot_width / 2 - 20, height - 24),
        "X [m]",
        fill="#111111",
    )
    draw.text((12, margin_top + plot_height / 2), "Y [m]", fill="#111111")
    image.convert("RGB").save(path, format="PNG")


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)

    parser.add_argument("--num-obstacles", type=int, default=DEFAULT_NUM_OBSTACLES)
    parser.add_argument("--min-gap", type=float, default=DEFAULT_MIN_GAP)

    parser.add_argument("--x-min", type=float, default=DEFAULT_X_MIN)
    parser.add_argument("--x-max", type=float, default=DEFAULT_X_MAX)
    parser.add_argument("--y-min", type=float, default=DEFAULT_Y_MIN)
    parser.add_argument("--y-max", type=float, default=DEFAULT_Y_MAX)

    parser.add_argument("--obstacle-radius", type=float, default=DEFAULT_OBSTACLE_RADIUS)
    parser.add_argument("--obstacle-height", type=float, default=DEFAULT_OBSTACLE_HEIGHT)

    parser.add_argument("--wall-margin", type=float, default=DEFAULT_WALL_MARGIN)

    parser.add_argument("--start-x", type=float, default=DEFAULT_START_X)
    parser.add_argument("--start-y", type=float, default=DEFAULT_START_Y)

    parser.add_argument("--goal-x", type=float, default=DEFAULT_GOAL_X)
    parser.add_argument("--goal-y", type=float, default=DEFAULT_GOAL_Y)
    parser.add_argument(
        "--obstacle-area-x-min",
        type=float,
        default=DEFAULT_OBSTACLE_AREA_X_MIN,
    )
    parser.add_argument(
        "--obstacle-area-x-max",
        type=float,
        default=DEFAULT_OBSTACLE_AREA_X_MAX,
    )

    parser.add_argument("--output-dir", type=str, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--world-name", type=str, default=DEFAULT_WORLD_NAME)

    parser.add_argument("--max-trials", type=int, default=DEFAULT_MAX_TRIALS)

    return parser.parse_args()


def main():
    args = parse_args()
    validate_args(args)

    if args.seed is None:
        seed = random.SystemRandom().randint(0, 999999999)
    else:
        seed = args.seed

    rng = random.Random(seed)

    # 출력 파일만 새로 만들며 cylinder 모델 원본은 수정하지 않는다.
    worlds_dir = Path(args.output_dir).resolve()
    worlds_dir.mkdir(parents=True, exist_ok=True)

    points = generate_points(args, rng)

    output_stem = f"{args.world_name}_{seed}"
    world_path = worlds_dir / f"{output_stem}.world"
    log_path = worlds_dir / f"{output_stem}_positions.txt"
    image_path = worlds_dir / f"{output_stem}_map.png"

    world_path.write_text(make_world(points, output_stem))
    write_position_log(log_path, args, seed, points)
    write_map_image(image_path, args, seed, points)

    print("[OK] random corridor world generated")
    print(f"seed: {seed}")
    print(f"num_obstacles: {args.num_obstacles}")
    print(f"min_surface_gap: {args.min_gap}")
    print(f"world: {world_path}")
    print(f"log: {log_path}")
    print(f"2d_map: {image_path}")
    print("[INFO] cylinder model files were not modified")
    print(f"[INFO] cylinder model uri: {CYLINDER_MODEL_URI}")


if __name__ == "__main__":
    main()
