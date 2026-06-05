#!/usr/bin/env python3
import argparse
import math
import random
from pathlib import Path


# ============================================================
# USER CONFIG
# 여기 값만 바꾸면 기본 생성 조건이 바뀐다.
#
# 목적:
# - 고정 원기둥 모델(sim_assets/models/cylinder_r05_h5)은 건드리지 않는다.
# - 이 스크립트가 있는 worlds 폴더에 obstacle_demo.world만 새로 생성한다.
# - obstacle_demo_positions.txt는 좌표 확인용으로 같이 갱신한다.
# ============================================================

DEFAULT_SEED = None

DEFAULT_NUM_OBSTACLES = 50
DEFAULT_MIN_GAP = 4.0

# 이 이름으로 생성하면 PX4_SITL_WORLD=obstacle_demo 설정과 바로 매칭된다.
DEFAULT_WORLD_NAME = "obstacle_demo"

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

# 시작점 / 목표점 주변 장애물 금지 구역
DEFAULT_START_X = 0.0
DEFAULT_START_Y = 0.0
DEFAULT_START_SAFE_RADIUS = 8.0

DEFAULT_GOAL_X = 150.0
DEFAULT_GOAL_Y = 0.0
DEFAULT_GOAL_SAFE_RADIUS = 8.0

DEFAULT_MAX_TRIALS = 100000

# 고정 원기둥 모델 이름
CYLINDER_MODEL_URI = "model://cylinder_r05_h5"


# ============================================================
# INTERNAL FUNCTIONS
# ============================================================

def distance_2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def is_valid_position(pos, placed, min_center_dist, safe_zones):
    for p in placed:
        if distance_2d(pos, p) < min_center_dist:
            return False

    for center, radius in safe_zones:
        if distance_2d(pos, center) < radius:
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


def make_world(points):
    cylinder_includes = make_cylinder_includes(points)

    return f'''<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
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


def generate_points(args):
    obstacle_radius = args.obstacle_radius

    # 표면 간 최소간격을 중심 간 최소거리로 변환
    # 예: 반지름 0.5m, 표면 간격 4m
    # 중심 간 최소거리 = 0.5 + 4.0 + 0.5 = 5.0m
    min_center_dist = args.min_gap + 2.0 * obstacle_radius

    x_min = args.x_min + args.wall_margin
    x_max = args.x_max - args.wall_margin
    y_min = args.y_min + args.wall_margin
    y_max = args.y_max - args.wall_margin

    safe_zones = [
        ((args.start_x, args.start_y), args.start_safe_radius),
        ((args.goal_x, args.goal_y), args.goal_safe_radius),
    ]

    points = []

    for i in range(args.num_obstacles):
        placed = False

        for _ in range(args.max_trials):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            candidate = (x, y)

            if is_valid_position(candidate, points, min_center_dist, safe_zones):
                points.append(candidate)
                placed = True
                break

        if not placed:
            raise RuntimeError(
                f"Failed to place obstacle {i}. "
                f"Try reducing DEFAULT_NUM_OBSTACLES or DEFAULT_MIN_GAP."
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
        f.write(f"actual_min_center_distance: {min_center_dist:.3f}\n")
        f.write(f"actual_min_surface_gap: {min_surface_gap:.3f}\n")
        f.write("\n")

        for i, (x, y) in enumerate(points):
            f.write(f"cylinder_{i:02d}: x={x:.3f}, y={y:.3f}\n")


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
    parser.add_argument("--start-safe-radius", type=float, default=DEFAULT_START_SAFE_RADIUS)

    parser.add_argument("--goal-x", type=float, default=DEFAULT_GOAL_X)
    parser.add_argument("--goal-y", type=float, default=DEFAULT_GOAL_Y)
    parser.add_argument("--goal-safe-radius", type=float, default=DEFAULT_GOAL_SAFE_RADIUS)

    parser.add_argument("--output-dir", type=str, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--world-name", type=str, default=DEFAULT_WORLD_NAME)

    parser.add_argument("--max-trials", type=int, default=DEFAULT_MAX_TRIALS)

    return parser.parse_args()


def main():
    args = parse_args()

    if args.seed is None:
        seed = random.randint(0, 999999999)
    else:
        seed = args.seed

    random.seed(seed)

    # 이 스크립트는 worlds 폴더 안에서 obstacle_demo.world만 갱신한다.
    # models/cylinder_r05_h5/model.sdf, model.config는 절대 수정하지 않는다.
    worlds_dir = Path(args.output_dir).resolve()
    worlds_dir.mkdir(parents=True, exist_ok=True)

    points = generate_points(args)

    world_path = worlds_dir / f"{args.world_name}.world"
    log_path = worlds_dir / f"{args.world_name}_positions.txt"

    world_path.write_text(make_world(points))
    write_position_log(log_path, args, seed, points)

    print("[OK] random corridor world generated")
    print(f"seed: {seed}")
    print(f"num_obstacles: {args.num_obstacles}")
    print(f"min_surface_gap: {args.min_gap}")
    print(f"world: {world_path}")
    print(f"log: {log_path}")
    print("[INFO] cylinder model files were not modified")
    print(f"[INFO] cylinder model uri: {CYLINDER_MODEL_URI}")


if __name__ == "__main__":
    main()
