#!/usr/bin/env python3
"""Generate a 12-row slit corridor with mirror-paired gap layouts."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "sim_assets/worlds/obstacle_demo.world"
OUTPUT = ROOT / "sim_assets/worlds/slit_corridor_double_symmetric.world"

# The second half is the reverse-order y -> -y mirror of the first half.
# This preserves global symmetry without creating repetitive adjacent pairs.
BASE_GAPS = [
    # Mix the earlier multi-slit walls with the current sparse/one-sided walls.
    [(-12.0, -7.0), (4.0, 10.0)],
    [(-6.0, -1.0), (11.0, 14.0)],
    [(-14.0, -9.0), (1.0, 5.0), (6.0, 10.0)],
    [(-8.0, -3.0), (11.0, 14.0)],
    [(-14.0, -9.0), (-2.0, 2.0), (4.0, 9.0)],
    [(-8.0, -3.0)],
]

X_POSITIONS = [9.0, 21.0, 31.0, 44.0, 55.0, 68.0, 79.0, 90.0, 102.0, 111.0, 121.0, 131.0]


def mirrored(gaps):
    return sorted([(-high, -low) for low, high in gaps])


def blocked_segments(gaps, low=-15.0, high=15.0):
    segments = []
    cursor = low
    for gap_low, gap_high in sorted(gaps):
        if gap_low > cursor:
            segments.append((cursor, gap_low))
        cursor = max(cursor, gap_high)
    if cursor < high:
        segments.append((cursor, high))
    return segments


def model(row, segment, x, y0, y1):
    center = 0.5 * (y0 + y1)
    length = y1 - y0
    return f'''    <model name="slit_row_{row:02d}_seg_{segment:02d}">
      <static>true</static>
      <pose>{x:.3f} {center:.3f} 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.3 {length:.3f} 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.3 {length:.3f} 5</size></box></geometry>
          <material>
            <ambient>0.30 0.43 0.55 1</ambient>
            <diffuse>0.36 0.56 0.72 1</diffuse>
          </material>
        </visual>
      </link>
    </model>'''


def main():
    text = SOURCE.read_text()
    prefix = text.split("    <!-- Row 01:", 1)[0]
    layouts = list(BASE_GAPS) + [mirrored(gaps) for gaps in reversed(BASE_GAPS)]

    rows = []
    for index, (x, gaps) in enumerate(zip(X_POSITIONS, layouts), start=1):
        gap_text = ", ".join(f"[{a:g},{b:g}]" for a, b in gaps)
        rows.append(f"    <!-- Row {index:02d}: mirror-paired openings y={gap_text} -->")
        for segment, (y0, y1) in enumerate(blocked_segments(gaps), start=1):
            rows.append(model(index, segment, x, y0, y1))

    OUTPUT.write_text(prefix + "\n".join(rows) + "\n  </world>\n</sdf>\n")
    print(f"world: {OUTPUT}")
    print(f"rows: {len(layouts)}")
    print("symmetry: second half is the reverse-order y-mirror of the first half")


if __name__ == "__main__":
    main()
