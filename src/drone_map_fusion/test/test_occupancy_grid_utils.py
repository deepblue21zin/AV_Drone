import math
import unittest

import numpy as np

from drone_map_fusion.occupancy_grid_utils import (
    GridSpec,
    fuse_projected_grids,
    project_occupancy_grid,
)


class OccupancyGridUtilsTest(unittest.TestCase):
    def setUp(self):
        self.spec = GridSpec(1.0, 0.0, 6.0, 0.0, 6.0)

    def test_translation_and_unknown_preservation(self):
        source = [-1, 0, 100, -1]
        projected = project_occupancy_grid(
            source, 2, 2, 1.0, (0.0, 0.0, 0.0), (1.0, 2.0, 0.0), self.spec
        )
        grid, conflicts, observed = fuse_projected_grids([projected], [1.0], self.spec)
        self.assertEqual(observed, 2)
        self.assertEqual(conflicts, 0)
        self.assertEqual(grid[2, 2], 5)
        self.assertEqual(grid[3, 1], 95)
        self.assertEqual(grid[0, 0], -1)

    def test_rotation(self):
        projected = project_occupancy_grid(
            [100], 1, 1, 1.0, (0.0, 0.0, 0.0), (3.0, 1.0, math.pi / 2), self.spec
        )
        self.assertEqual(projected.indices.tolist(), [1 * self.spec.width + 2])

    def test_conflicting_sources_are_counted(self):
        free = project_occupancy_grid(
            [0], 1, 1, 1.0, (0.0, 0.0, 0.0), (1.0, 1.0, 0.0), self.spec
        )
        occupied = project_occupancy_grid(
            [100], 1, 1, 1.0, (0.0, 0.0, 0.0), (1.0, 1.0, 0.0), self.spec
        )
        grid, conflicts, observed = fuse_projected_grids(
            [free, occupied], [1.0, 1.0], self.spec
        )
        self.assertEqual(conflicts, 1)
        self.assertEqual(observed, 1)
        self.assertEqual(grid[1, 1], 50)

    def test_invalid_data_size(self):
        with self.assertRaises(ValueError):
            project_occupancy_grid(
                [0], 2, 2, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), self.spec
            )


if __name__ == "__main__":
    unittest.main()
