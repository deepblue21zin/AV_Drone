#!/usr/bin/env python3
"""Generate corridor worlds whose drone1-side segments separate degeneracy from aliasing.

Each world splits x=15..135 m into four 30 m zones of different types:
  RU  rich, unique     - many cylinders, random
  RR  rich, repeated   - a 2-cylinder tile copied every 4 m (near-copies, small jitter)
  SU  sparse, unique   - few cylinders, random
  SR  sparse, repeated - one cylinder copied every 5 m (near-copies, small jitter)
Zone order follows a 4x4 Latin square over worlds w1..w4, so every type appears once at
every position. Walls, physics and the gazebo_ros_state plugin are copied from the
template world; only the cylinders change.

  python3 scripts/generate_zoned_corridor_world.py            # write w1..w4
  python3 scripts/generate_zoned_corridor_world.py --check    # + virtual LiDAR check (numpy)
"""

import argparse
import json
import math
import random
import re
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
TEMPLATE = PROJECT / "sim_assets/worlds/random_cylinders_double.world"
OUTPUT_DIR = PROJECT / "sim_assets/worlds"
CHECK_OUTPUT = PROJECT / "experiments/2026-10-06_zoned_world_design_check/design_check.json"

DRONE1_LANE_Y = -7.5
LANE_CLEARANCE_M = 2.2        # surface >= 1.7 m from the lane: above obstacle_stop_distance (1.25 m) and gap_min_clearance (1.6 m)
BAND_OUTER_M = 6.0            # cylinders stay within 6 m of the drone1 lane
CENTER_FREE_Y = 1.5           # |y| < 1.5 m stays empty
START_FREE_X = 10.0           # drone2 crosses to the drone1 lane before this x
ZONES = [(15.0, 45.0), (45.0, 75.0), (75.0, 105.0), (105.0, 135.0)]
LATIN = {
    "w1": ["RU", "RR", "SU", "SR"],
    "w2": ["RR", "SR", "RU", "SU"],
    "w3": ["SU", "RU", "SR", "RR"],
    "w4": ["SR", "SU", "RR", "RU"],
}
LATIN["p1"] = ["RU", "RR", "SU", "SR"]   # pilot world for approach flights (same zone order as w1)
LATIN["p2"] = ["RU", "RR", "RU", "RR"]   # approach pilot 2: rich zones only, cylinders between the lanes
LATIN["p3"] = ["RU", "RR", "RU", "RR"]   # approach pilot 3: p2's drone1 side (same seed), denser drone2 side
# approach main experiment: p3 layout, two zone orders x two seeds, so each zone type is approached near and far
LATIN.update({"a1": ["RU", "RR", "RU", "RR"], "a2": ["RR", "RU", "RR", "RU"],
              "a3": ["RU", "RR", "RU", "RR"], "a4": ["RR", "RU", "RR", "RU"]})
# zone-selection replication (2026-10-09): same rule as a1..a4 with new seeds
LATIN.update({"a5": ["RU", "RR", "RU", "RR"], "a6": ["RR", "RU", "RR", "RU"],
              "a7": ["RU", "RR", "RU", "RR"], "a8": ["RR", "RU", "RR", "RU"]})
# three-candidate selection (2026-10-09): every mix of unique/repeated over the first three zones that has
# at least one of each; the fourth zone is not a candidate and is always unique
LATIN.update({"b1": ["RU", "RU", "RR", "RU"], "b2": ["RU", "RR", "RU", "RU"], "b3": ["RR", "RU", "RU", "RU"],
              "b4": ["RU", "RR", "RR", "RU"], "b5": ["RR", "RU", "RR", "RU"], "b6": ["RR", "RR", "RU", "RU"]})
SEEDS = {"w1": 101, "w2": 202, "w3": 303, "w4": 404, "p1": 501, "p2": 502, "p3": 502,
         "a1": 611, "a2": 612, "a3": 613, "a4": 614, "a5": 615, "a6": 616, "a7": 617, "a8": 618,
         "b1": 621, "b2": 622, "b3": 623, "b4": 624, "b5": 625, "b6": 626}
ROLE = {"w1": "design", "w2": "design", "w3": "evaluation", "w4": "evaluation",
        "p1": "approach_pilot", "p2": "approach_pilot", "p3": "approach_pilot",
        "a1": "approach_main", "a2": "approach_main", "a3": "approach_main", "a4": "approach_main",
        "a5": "approach_selection", "a6": "approach_selection", "a7": "approach_selection", "a8": "approach_selection",
        **{f"b{i}": "approach_selection3" for i in range(1, 7)}}
# "trailing": drone2 follows the drone1 lane (w1..w4). "approach": drone2 keeps its own lane (y=+7.5)
# and dips to the centre line (y=0), so both must be clear and drone2 must not see drone1-side
# cylinders from its lane (LiDAR 8 m): centre clearance 2.2 m, drone2-side cylinders only beyond y=9.7 m.
# "approach_between": as "approach", but drone1-side cylinders sit only between the two lanes
# (y = -5.3 .. -2.2). From y=0 drone2 then sees every cylinder drone1 sees; with p1 it saw only half of
# them and no wall, and its submaps in the RR zone fell below minimum_submap_points.
# "approach_between_dense": drone2's own lane also needs along-track features. With 25 cylinders (p2)
# drone2's raw SLAM slipped ~3 m along x within the first 30 m.
LAYOUT = {"p1": "approach", "p2": "approach_between", "p3": "approach_between_dense",
          "a1": "approach_between_dense", "a2": "approach_between_dense",
          "a3": "approach_between_dense", "a4": "approach_between_dense",
          "a5": "approach_between_dense", "a6": "approach_between_dense",
          "a7": "approach_between_dense", "a8": "approach_between_dense",
          **{f"b{i}": "approach_between_dense" for i in range(1, 7)}}
DENSE_DRONE2_SIDE = {"count": 60, "min_gap_m": 2.0}
BETWEEN_ONLY = False
APPROACH = {"center_free_y": 2.2, "drone2_side": {"count": 25, "min_gap_m": 3.0, "y": (9.7, 13.5)}}
ZONE_RULES = {
    # count, min gap for random zones; period, tile size for repeated zones
    "RU": {"count": 15, "min_gap_m": 2.0},
    "SU": {"count": 6, "min_gap_m": 4.0},
    "RR": {"period_m": 4.0, "tile_count": 2, "tile_gap_m": 1.6},
    "SR": {"period_m": 5.0, "tile_count": 1, "tile_gap_m": 1.6},
}
REPEAT_JITTER_M = 0.15
DRONE2_SIDE = {"count": 35, "min_gap_m": 3.0, "y": (1.5, 13.5), "x": (START_FREE_X, 142.0)}
BUFFER_COUNT = {"start": (START_FREE_X, 15.0, 2), "goal": (135.0, 142.0, 3)}
MIN_CENTER_DIST_M = 1.2       # any two cylinders (r=0.5 m) keep a visible gap


def lane_offset_y(rng):
    side = rng.choice((-1.0, 1.0))
    if BETWEEN_ONLY:
        side = 1.0
    # towards the corridor centre the band ends at the centre-free line (6.0 m for the trailing layout)
    outer = BAND_OUTER_M if side < 0 else min(BAND_OUTER_M, -CENTER_FREE_Y - DRONE1_LANE_Y)
    return DRONE1_LANE_Y + side * rng.uniform(LANE_CLEARANCE_M, outer)


def valid_drone1_side(x, y):
    offset = abs(y - DRONE1_LANE_Y)
    return LANE_CLEARANCE_M <= offset <= BAND_OUTER_M and y <= -CENTER_FREE_Y and x >= START_FREE_X


def far_enough(point, placed, gap):
    return all(math.hypot(point[0] - q[0], point[1] - q[1]) >= gap for q in placed)


def random_points(rng, x0, x1, count, gap, make_y, placed, max_trials=20000):
    out = []
    for _ in range(max_trials):
        if len(out) == count:
            return out
        point = (rng.uniform(x0 + 0.5, x1 - 0.5), make_y(rng))
        if far_enough(point, placed + out, gap):
            out.append(point)
    raise RuntimeError(f"could not place {count} cylinders in x=[{x0}, {x1}]")


def repeated_points(rng, x0, x1, rule, placed):
    period = rule["period_m"]
    tile = random_points(rng, 0.0, period, rule["tile_count"], rule["tile_gap_m"], lane_offset_y, [])
    tile = [(min(max(x, 0.5), period - 0.5), y) for x, y in tile]
    out = []
    start = x0
    while start + period <= x1 + 1e-9:
        for tx, ty in tile:
            for _ in range(100):  # near-copy: redraw jitter until it stays legal
                p = (start + tx + rng.gauss(0.0, REPEAT_JITTER_M), ty + rng.gauss(0.0, REPEAT_JITTER_M))
                if valid_drone1_side(*p) and far_enough(p, placed + out, MIN_CENTER_DIST_M):
                    out.append(p)
                    break
            else:
                raise RuntimeError(f"could not jitter tile cylinder at x={start + tx:.2f}")
        start += period
    return out


def build_layout(name):
    global CENTER_FREE_Y, BETWEEN_ONLY
    layout_kind = LAYOUT.get(name, "trailing")
    approach = layout_kind.startswith("approach")
    CENTER_FREE_Y = APPROACH["center_free_y"] if approach else 1.5
    BETWEEN_ONLY = layout_kind.startswith("approach_between")
    drone2_side = {**DRONE2_SIDE, **(APPROACH["drone2_side"] if approach else {}),
                   **(DENSE_DRONE2_SIDE if layout_kind.endswith("_dense") else {})}
    rng = random.Random(SEEDS[name])
    cylinders = []

    def add(points, side, zone):
        cylinders.extend({"x": x, "y": y, "side": side, "zone": zone} for x, y in points)

    placed = lambda: [(c["x"], c["y"]) for c in cylinders]
    zones = []
    for index, (zone_type, (x0, x1)) in enumerate(zip(LATIN[name], ZONES)):
        rule = ZONE_RULES[zone_type]
        if "period_m" in rule:
            points = repeated_points(rng, x0, x1, rule, placed())
        else:
            points = random_points(rng, x0, x1, rule["count"], rule["min_gap_m"], lane_offset_y, placed())
        add(points, "drone1", index)
        zones.append({"index": index, "type": zone_type, "x_min": x0, "x_max": x1, "count": len(points)})
    for label, (x0, x1, count) in BUFFER_COUNT.items():
        add(random_points(rng, x0, x1, count, 2.5, lane_offset_y, placed()), "drone1", label)
    y0, y1 = drone2_side["y"]
    add(random_points(rng, *drone2_side["x"], drone2_side["count"], drone2_side["min_gap_m"],
                      lambda r: r.uniform(y0, y1), placed()), "drone2", "drone2_side")
    for c in cylinders:
        if c["side"] == "drone1" and not valid_drone1_side(c["x"], c["y"]):
            raise RuntimeError(f"cylinder outside drone1 band: {c}")
    return {
        "world_name": f"zoned_corridor_{name}",
        "role": ROLE[name],
        "layout": layout_kind,
        "seed": SEEDS[name],
        "zone_order": LATIN[name],
        "zones": zones,
        "rules": {"zones": ZONE_RULES, "repeat_jitter_m": REPEAT_JITTER_M,
                  "drone1_lane_y": DRONE1_LANE_Y, "lane_clearance_m": LANE_CLEARANCE_M,
                  "band_outer_m": BAND_OUTER_M, "center_free_y": CENTER_FREE_Y,
                  "start_free_x": START_FREE_X},
        "cylinders": cylinders,
    }


def write_world(layout, template_text):
    lines = [line for line in template_text.splitlines() if "<name>cylinder_" not in line]
    body = "\n".join(lines)
    includes = "\n".join(
        "    <include>"
        f"<name>cylinder_{i:03d}</name>"
        "<uri>model://cylinder_r05_h5</uri>"
        f"<pose>{c['x']:.3f} {c['y']:.3f} 2.5 0 0 0</pose>"
        "</include>"
        for i, c in enumerate(layout["cylinders"]))
    end = body.rindex("  </world>")
    return body[:end].rstrip() + "\n\n" + includes + "\n\n" + body[end:] + "\n"


def write_positions(layout):
    head = [f"world: {layout['world_name']}", f"seed: {layout['seed']}",
            f"zone_order: {' '.join(layout['zone_order'])}", f"num_obstacles: {len(layout['cylinders'])}", ""]
    rows = [f"cylinder_{i:03d}: x={c['x']:.3f}, y={c['y']:.3f}, side={c['side']}, zone={c['zone']}"
            for i, c in enumerate(layout["cylinders"])]
    return "\n".join(head + rows) + "\n"


# ---------- virtual LiDAR check (same scoring as the 2026-09-27 offline study) ----------
def design_check(layouts):
    import numpy as np

    rng = np.random.default_rng(0)
    angles = np.deg2rad(np.arange(0.0, 360.0, 1.0))
    radius, max_range = 0.5, 8.0

    def scan(px, py, centers):
        dx, dy = np.cos(angles), np.sin(angles)
        d = np.full(len(angles), np.inf)
        for cx, cy in centers:
            if (cx - px) ** 2 + (cy - py) ** 2 > (max_range + radius) ** 2:
                continue
            ox, oy = px - cx, py - cy
            b = ox * dx + oy * dy
            disc = b * b - (ox * ox + oy * oy - radius * radius)
            t = np.where(disc >= 0, -b - np.sqrt(np.maximum(disc, 0.0)), np.inf)
            d = np.minimum(d, np.where(t > 0, t, np.inf))
        for wall_y in (-15.0, 15.0):
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (wall_y - py) / dy
            d = np.minimum(d, np.where(t > 0, t, np.inf))
        d = np.where(d <= max_range, d + rng.normal(0.0, 0.01, len(d)), np.nan)
        keep = ~np.isnan(d)
        return np.column_stack([px + d[keep] * dx[keep], py + d[keep] * dy[keep]])

    def voxel(points, size=0.2):
        keys = np.floor(points / size).astype(int)
        _, index = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(index)]

    def submap(x, centers):
        scans = [scan(sx, DRONE1_LANE_Y, centers)[::2] for sx in np.arange(x - 5.0, x + 5.01, 0.9)]
        return voxel(np.vstack(scans)) - np.array([x, DRONE1_LANE_Y])

    def alpha(points, neighbor_m=0.6):
        d2 = ((points[:, None, :] - points[None, :, :]) ** 2).sum(-1)
        normals = []
        for i in range(len(points)):
            near = points[d2[i] < neighbor_m ** 2]
            if len(near) < 3:
                continue
            w, v = np.linalg.eigh(np.cov((near - near.mean(0)).T))
            if w[1] > 0:
                normals.append(v[:, 0])
        if len(normals) < 10:
            return 0.0
        n = np.array(normals)
        w = np.linalg.eigvalsh(n.T @ n)
        return float(w[0] / w[1])

    class Grid:
        def __init__(self, points, res=0.1, hit_m=0.3):
            self.res, self.lo = res, points.min(0) - 10.0
            shape = np.ceil((points.max(0) + 10.0 - self.lo) / res).astype(int) + 1
            occ = np.zeros(shape, bool)
            ij = np.floor((points - self.lo) / res).astype(int)
            occ[ij[:, 0], ij[:, 1]] = True
            k = int(round(hit_m / res))
            self.hit = np.zeros_like(occ)
            for a in range(-k, k + 1):
                for b in range(-k, k + 1):
                    if a * a + b * b <= k * k:
                        self.hit[max(a, 0):shape[0] + min(a, 0), max(b, 0):shape[1] + min(b, 0)] |= \
                            occ[max(-a, 0):shape[0] + min(-a, 0), max(-b, 0):shape[1] + min(-b, 0)]
            self.shape = shape

        def score(self, batch):
            ij = np.floor((batch - self.lo) / self.res).astype(int)
            inside = (ij[..., 0] >= 0) & (ij[..., 1] >= 0) & (ij[..., 0] < self.shape[0]) & (ij[..., 1] < self.shape[1])
            ij = np.where(inside[..., None], ij, 0)
            return (self.hit[ij[..., 0], ij[..., 1]] & inside).mean(-1)

    def alias(grid, points, x, window_m=6.0, exclude_m=1.5):
        here = np.array([x, DRONE1_LANE_Y])
        own = float(grid.score((points + here)[None])[0])
        steps = np.arange(-window_m, window_m + 0.01, 0.5)
        gx, gy = np.meshgrid(steps, steps, indexing="ij")
        offsets = np.column_stack([gx.ravel(), gy.ravel()])
        best = 0.0
        for yaw in np.deg2rad(np.arange(-30, 31, 5)):
            c, s = np.cos(yaw), np.sin(yaw)
            rotated = points @ np.array([[c, s], [-s, c]]) + here
            far = (np.abs(offsets[:, 0]) > exclude_m) | (np.abs(offsets[:, 1]) > exclude_m) | (abs(np.rad2deg(yaw)) >= 10)
            if far.any():
                best = max(best, float(grid.score(rotated[None] + offsets[far][:, None, :]).max()))
        return best / own if own > 0 else 0.0

    rows = []
    for layout in layouts:
        centers = [(c["x"], c["y"]) for c in layout["cylinders"]]
        anchors = np.arange(12.0, 140.0, 3.0)
        maps = {x: submap(x, centers) for x in anchors}
        grid = Grid(np.vstack([m + np.array([x, DRONE1_LANE_Y]) for x, m in maps.items()]))
        for x, points in maps.items():
            zone = next((z for z in layout["zones"] if z["x_min"] + 7.0 <= x <= z["x_max"] - 7.0), None)
            if zone is None:
                continue  # submaps near zone boundaries mix two types
            rows.append({"world": layout["world_name"], "x": float(x), "type": zone["type"],
                         "points": int(len(points)), "alpha": alpha(points), "alias": alias(grid, points, x)})
    summary = {}
    for zone_type in ("RU", "RR", "SU", "SR"):
        sel = [r for r in rows if r["type"] == zone_type]
        if not sel:
            continue  # approach worlds have no sparse zones
        summary[zone_type] = {"submaps": len(sel), "min_points": min(r["points"] for r in sel),
                              "alpha_median": float(np.median([r["alpha"] for r in sel])),
                              "alias_median": float(np.median([r["alias"] for r in sel]))}
    return rows, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--worlds", nargs="+", default=["w1", "w2", "w3", "w4"], choices=list(LATIN))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--check", action="store_true", help="run the virtual LiDAR zone check (needs numpy)")
    parser.add_argument("--check-output", type=Path, default=CHECK_OUTPUT)
    args = parser.parse_args()

    template = TEMPLATE.read_text()
    if len(re.findall(r"<name>cylinder_", template)) == 0:
        raise RuntimeError("template has no cylinder includes; refusing to guess its structure")
    layouts = []
    for name in args.worlds:
        layout = build_layout(name)
        stem = args.output_dir / layout["world_name"]
        stem.with_suffix(".world").write_text(write_world(layout, template))
        Path(f"{stem}_positions.txt").write_text(write_positions(layout))
        Path(f"{stem}_layout.json").write_text(json.dumps(layout, indent=1) + "\n")
        layouts.append(layout)
        counts = " ".join(f"{z['type']}={z['count']}" for z in layout["zones"])
        print(f"{layout['world_name']}: {len(layout['cylinders'])} cylinders ({counts})")
    if args.check:
        rows, summary = design_check(layouts)
        out = args.check_output
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({"summary": summary, "rows": rows}, indent=1) + "\n")
        for zone_type, s in summary.items():
            print(f"{zone_type}: submaps={s['submaps']} min_points={s['min_points']} "
                  f"alpha={s['alpha_median']:.2f} alias(+-6m)={s['alias_median']:.2f}")
        print(f"check written: {out}")


if __name__ == "__main__":
    main()
