import json
import math
from pathlib import Path

import numpy as np


class SavedSlamMap:
    def __init__(self, grid_path, meta_path):
        self.grid_path = Path(grid_path)
        self.meta_path = Path(meta_path)
        self.grid = np.load(str(self.grid_path))
        with self.meta_path.open("r", encoding="utf-8") as stream:
            meta = json.load(stream)
        self.resolution = float(meta["resolution"])
        self.origin_x = float(meta["origin_x"])
        self.origin_y = float(meta["origin_y"])
        self.frame_id = str(meta.get("frame_id", "map"))
        self.height, self.width = self.grid.shape

    def world_to_grid(self, x, y):
        return (
            int(round((x - self.origin_x) / self.resolution)),
            int(round((y - self.origin_y) / self.resolution)),
        )

    def grid_to_world(self, cell):
        return (
            self.origin_x + cell[0] * self.resolution,
            self.origin_y + cell[1] * self.resolution,
        )

    def blocked(self, occupied_threshold=50, unknown_is_blocked=True):
        result = self.grid >= int(occupied_threshold)
        if unknown_is_blocked:
            result = np.logical_or(result, self.grid < 0)
        return result


def inflate_mask(mask, radius_cells):
    radius_cells = max(0, int(radius_cells))
    if radius_cells == 0:
        return mask.copy()
    inflated = mask.copy()
    height, width = mask.shape
    for dy in range(-radius_cells, radius_cells + 1):
        max_dx = int(math.floor(math.sqrt(radius_cells ** 2 - dy ** 2)))
        for dx in range(-max_dx, max_dx + 1):
            source_y0, source_y1 = max(0, -dy), min(height, height - dy)
            source_x0, source_x1 = max(0, -dx), min(width, width - dx)
            target_y0, target_y1 = source_y0 + dy, source_y1 + dy
            target_x0, target_x1 = source_x0 + dx, source_x1 + dx
            inflated[target_y0:target_y1, target_x0:target_x1] |= mask[source_y0:source_y1, source_x0:source_x1]
    return inflated
