#!/usr/bin/env python3
"""Generate a Gazebo Classic corridor world with reproducible random cylinders."""

import argparse
import math
import random
from pathlib import Path


DEFAULT_SEED = None
DEFAULT_NUM_OBSTACLES = 50
DEFAULT_MIN_GAP = 4.0

DEFAULT_WORLD_NAME = "random_corridor_generated"
DEFAULT_OUTPUT_DIR = "sim_assets"

DEFAULT_X_MIN = 0.0
DEFAULT_X_MAX = 150.0
DEFAULT_Y_MIN = -15.0
DEFAULT_Y_MAX = 15.0

DEFAULT_OBSTACLE_RADIUS = 0.5
DEFAULT_OBSTACLE_HEIGHT = 5.0
DEFAULT_WALL_MARGIN = 2.5

DEFAULT_START_X = 0.0
DEFAULT_START_Y = 0.0
DEFAULT_START_SAFE_RADIUS = 8.0

DEFAULT_GOAL_X = 150.0
DEFAULT_GOAL_Y = 0.0
DEFAULT_GOAL_SAFE_RADIUS = 8.0

DEFAULT_MAX_TRIALS = 100000


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
            "    <include>"
            f"<name>cylinder_{i:02d}</name>"
            "<uri>model://cylinder_r05_h5</uri>"
            f"<pose>{x:.3f} {y:.3f} 2.5 0 0 0</pose>"
            "</include>"
        )
    return "\n".join(lines)


def make_wall(name, pose, size):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
        </visual>
      </link>
    </model>"""


def make_world(points, args):
    cylinder_includes = make_cylinder_includes(points)
    length = args.x_max - args.x_min
    width = args.y_max - args.y_min
    center_x = (args.x_min + args.x_max) / 2.0
    center_y = (args.y_min + args.y_max) / 2.0
    wall_height = args.obstacle_height
    wall_z = wall_height / 2.0
    wall_thickness = 0.3
    side_wall_y = width / 2.0 + wall_thickness / 2.0
    start_wall_x = args.x_min - wall_thickness / 2.0
    end_wall_x = args.x_max + wall_thickness / 2.0

    return f'''<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="{args.world_name}">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <scene>
      <ambient>0.45 0.45 0.45 1.0</ambient>
      <background>0.82 0.88 0.95 1.0</background>
      <shadows>true</shadows>
    </scene>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>75.0 0.0 90.0 0.0 1.35 0.0</pose>
        <view_controller>fps</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <model name="corridor_floor">
      <static>true</static>
      <pose>{center_x:.3f} {center_y:.3f} 0.01 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{length:.3f} {width:.3f} 0.02</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.3f} {width:.3f} 0.02</size></box></geometry>
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

{make_wall("left_wall", f"{center_x:.3f} {side_wall_y:.3f} {wall_z:.3f} 0 0 0", f"{length:.3f} {wall_thickness:.3f} {wall_height:.3f}")}

{make_wall("right_wall", f"{center_x:.3f} {-side_wall_y:.3f} {wall_z:.3f} 0 0 0", f"{length:.3f} {wall_thickness:.3f} {wall_height:.3f}")}

{make_wall("start_wall", f"{start_wall_x:.3f} {center_y:.3f} {wall_z:.3f} 0 0 0", f"{wall_thickness:.3f} {width + wall_thickness:.3f} {wall_height:.3f}")}

{make_wall("end_wall", f"{end_wall_x:.3f} {center_y:.3f} {wall_z:.3f} 0 0 0", f"{wall_thickness:.3f} {width + wall_thickness:.3f} {wall_height:.3f}")}

{cylinder_includes}

  </world>
</sdf>
'''


def make_model_config():
    return '''<?xml version="1.0" ?>
<model>
  <name>cylinder_r05_h5</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>
  <author>
    <name>AV_Drone</name>
    <email>none</email>
  </author>
  <description>Static cylinder obstacle, radius 0.5m, height 5m.</description>
</model>
'''


def make_model_sdf(args):
    return f'''<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="cylinder_r05_h5">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{args.obstacle_radius:.3f}</radius>
            <length>{args.obstacle_height:.3f}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{args.obstacle_radius:.3f}</radius>
            <length>{args.obstacle_height:.3f}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.75 0.75 0.75 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''


def generate_points(args):
    min_center_dist = args.min_gap + 2.0 * args.obstacle_radius
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
        for _ in range(args.max_trials):
            candidate = (random.uniform(x_min, x_max), random.uniform(y_min, y_max))
            if is_valid_position(candidate, points, min_center_dist, safe_zones):
                points.append(candidate)
                break
        else:
            raise RuntimeError(
                f"Failed to place obstacle {i}. "
                "Try reducing --num-obstacles or --min-gap."
            )
    return points


def write_position_log(path, args, seed, points):
    min_center_dist = None
    for i in range(len(points)):
        for j in range(i):
            d = distance_2d(points[i], points[j])
            min_center_dist = d if min_center_dist is None else min(min_center_dist, d)

    min_center_dist = min_center_dist or 0.0
    min_surface_gap = min_center_dist - 2.0 * args.obstacle_radius

    with path.open("w") as f:
        f.write(f"seed: {seed}\n")
        f.write(f"num_obstacles: {args.num_obstacles}\n")
        f.write(f"map_x: {args.x_min} ~ {args.x_max}\n")
        f.write(f"map_y: {args.y_min} ~ {args.y_max}\n")
        f.write(f"obstacle_radius: {args.obstacle_radius}\n")
        f.write(f"obstacle_height: {args.obstacle_height}\n")
        f.write(f"requested_min_surface_gap: {args.min_gap}\n")
        f.write(f"actual_min_center_distance: {min_center_dist:.3f}\n")
        f.write(f"actual_min_surface_gap: {min_surface_gap:.3f}\n\n")
        for i, (x, y) in enumerate(points):
            f.write(f"cylinder_{i:02d}: x={x:.3f}, y={y:.3f}\n")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
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
    seed = random.randint(0, 999999999) if args.seed is None else args.seed
    random.seed(seed)

    output_dir = Path(args.output_dir)
    worlds_dir = output_dir / "worlds"
    models_dir = output_dir / "models" / "cylinder_r05_h5"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    models_dir.mkdir(parents=True, exist_ok=True)

    points = generate_points(args)
    world_path = worlds_dir / f"{args.world_name}.world"
    log_path = worlds_dir / f"{args.world_name}_positions.txt"
    model_config_path = models_dir / "model.config"
    model_sdf_path = models_dir / "model.sdf"

    world_path.write_text(make_world(points, args))
    model_config_path.write_text(make_model_config())
    model_sdf_path.write_text(make_model_sdf(args))
    write_position_log(log_path, args, seed, points)

    print("[OK] random corridor world generated")
    print(f"seed: {seed}")
    print(f"num_obstacles: {args.num_obstacles}")
    print(f"min_surface_gap: {args.min_gap}")
    print(f"world: {world_path}")
    print(f"model_config: {model_config_path}")
    print(f"model_sdf: {model_sdf_path}")
    print(f"log: {log_path}")


if __name__ == "__main__":
    main()
