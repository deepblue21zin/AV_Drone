"""Parse SDF collision geometry for map-aware planners."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass
class CircleObstacle:
    x: float
    y: float
    radius: float


@dataclass
class RectangleObstacle:
    x: float
    y: float
    half_x: float
    half_y: float
    yaw: float


@dataclass
class WorldGeometry:
    rectangles: list
    circles: list


def _tag(element):
    return element.tag.split("}", 1)[-1] if "}" in element.tag else element.tag


def _child(element, name):
    return next((item for item in list(element) if _tag(item) == name), None)


def _text(element, name, default=""):
    item = _child(element, name)
    return (item.text or "").strip() if item is not None else default


def _pose(element):
    values = [float(value) for value in _text(element, "pose", "0 0 0 0 0 0").split()]
    return (values + [0.0] * 6)[:6]


def load_collision_circles(world_path, sample_spacing=2.0):
    """Read all static collision boxes/cylinders; model names are irrelevant."""
    path = Path(world_path)
    if not path.exists():
        raise FileNotFoundError("world file not found: {}".format(path))
    root = ET.parse(str(path)).getroot()
    obstacles = []
    seen = set()

    for model in root.iter():
        if _tag(model) != "model":
            continue
        if _text(model, "static", "false").lower() not in {"1", "true"}:
            continue
        model_x, model_y, _z, _r, _p, yaw = _pose(model)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        for collision in model.iter():
            if _tag(collision) != "collision":
                continue
            geometry = _child(collision, "geometry")
            if geometry is None:
                continue
            cylinder = _child(geometry, "cylinder")
            if cylinder is not None:
                radius = float(_text(cylinder, "radius", "0"))
                candidates = [(model_x, model_y, radius)] if radius > 0.0 else []
            else:
                box = _child(geometry, "box")
                if box is None:
                    continue
                size = [float(value) for value in _text(box, "size").split()]
                if len(size) < 3:
                    continue
                size_x, size_y, size_z = size[:3]
                if size_z <= 0.1 and size_x >= 5.0 and size_y >= 5.0:
                    continue
                major, minor = max(size_x, size_y), min(size_x, size_y)
                radius = max(0.05, minor * 0.5)
                count = max(1, int(math.ceil(major / max(sample_spacing, 0.1))))
                candidates = []
                for index in range(count):
                    offset = -0.5 * major + (index + 0.5) * major / count
                    lx = offset if size_x >= size_y else 0.0
                    ly = 0.0 if size_x >= size_y else offset
                    x = model_x + cos_yaw * lx - sin_yaw * ly
                    y = model_y + sin_yaw * lx + cos_yaw * ly
                    candidates.append((x, y, radius))

            for x, y, radius in candidates:
                key = (round(x, 4), round(y, 4), round(radius, 4))
                if key not in seen:
                    seen.add(key)
                    obstacles.append(CircleObstacle(x, y, radius))
    return obstacles


def load_world_geometry(world_path):
    """Return exact collision rectangles/circles from static SDF models."""
    path = Path(world_path)
    if not path.exists():
        raise FileNotFoundError("world file not found: {}".format(path))
    root = ET.parse(str(path)).getroot()
    rectangles = []
    circles = []

    for model in root.iter():
        if _tag(model) != "model":
            continue
        if _text(model, "static", "false").lower() not in {"1", "true"}:
            continue
        model_x, model_y, _z, _roll, _pitch, model_yaw = _pose(model)

        for collision in model.iter():
            if _tag(collision) != "collision":
                continue
            geometry = _child(collision, "geometry")
            if geometry is None:
                continue
            collision_x, collision_y, _cz, _cr, _cp, collision_yaw = _pose(collision)
            cos_yaw, sin_yaw = math.cos(model_yaw), math.sin(model_yaw)
            world_x = model_x + cos_yaw * collision_x - sin_yaw * collision_y
            world_y = model_y + sin_yaw * collision_x + cos_yaw * collision_y
            yaw = model_yaw + collision_yaw

            cylinder = _child(geometry, "cylinder")
            if cylinder is not None:
                radius = float(_text(cylinder, "radius", "0"))
                if radius > 0.0:
                    circles.append(CircleObstacle(world_x, world_y, radius))
                continue

            box = _child(geometry, "box")
            if box is None:
                continue
            size = [float(value) for value in _text(box, "size").split()]
            if len(size) < 3:
                continue
            size_x, size_y, size_z = size[:3]
            if size_z <= 0.1 and size_x >= 5.0 and size_y >= 5.0:
                continue
            rectangles.append(
                RectangleObstacle(world_x, world_y, size_x * 0.5, size_y * 0.5, yaw)
            )

    return WorldGeometry(rectangles=rectangles, circles=circles)
