import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "quant_dashboard.py"
SPEC = importlib.util.spec_from_file_location("quant_dashboard", MODULE_PATH)
quant_dashboard = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(quant_dashboard)


class QuantDashboardCoordinateTest(unittest.TestCase):
    def test_local_to_world_applies_translation_and_rotation(self):
        x, y = quant_dashboard.local_to_world(
            2.0,
            1.0,
            {"x": 3.0, "y": -4.0, "yaw": math.pi / 2.0},
        )
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, -2.0)

    def test_metadata_transform_has_priority(self):
        transform, source = quant_dashboard.resolve_world_from_local(
            Path("/unused"),
            {
                "trajectory_frame_id": "drone1/odom",
                "world_frame_id": "swarm_map",
                "world_from_local": {"x": 3, "y": -7.5, "yaw": 0},
            },
            {},
            "drone1",
        )
        self.assertEqual(source, "metadata")
        self.assertEqual(transform, {"x": 3.0, "y": -7.5, "yaw": 0.0})

    def test_legacy_manifest_spawn_is_used(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            manifest = root / "swarm.yaml"
            manifest.write_text(
                "vehicles:\n"
                "  - name: drone2\n"
                "    spawn: [3.0, 7.5, 0.83, 1.57079632679]\n",
                encoding="utf-8",
            )
            transform, source = quant_dashboard.resolve_world_from_local(
                root,
                {"config_snapshot_files": {"scenario_manifest": "swarm.yaml"}},
                {},
                "drone2",
            )
        self.assertEqual(source, "scenario_manifest")
        self.assertEqual(transform["x"], 3.0)
        self.assertEqual(transform["y"], 7.5)
        self.assertAlmostEqual(transform["yaw"], math.pi / 2.0)

    def test_missing_transform_is_explicit(self):
        transform, source = quant_dashboard.resolve_world_from_local(
            Path("/unused"), {}, {}, "drone1"
        )
        self.assertIsNone(transform)
        self.assertEqual(source, "unavailable")

    def test_load_run_reports_ignores_invalid_json_shape(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            report_dir = root / "experiments" / "run_reports"
            report_dir.mkdir(parents=True)
            (report_dir / "valid.json").write_text(
                json.dumps({"source_run_id": "run-1", "title": "report"}),
                encoding="utf-8",
            )
            (report_dir / "invalid.json").write_text(
                json.dumps({"title": "missing run id"}),
                encoding="utf-8",
            )

            reports = quant_dashboard.load_run_reports(root)

        self.assertEqual(len(reports), 1)
        self.assertEqual(reports[0]["source_run_id"], "run-1")
        self.assertTrue(reports[0]["report_path"].endswith("valid.json"))

    def test_occupancy_snapshot_points_applies_origin_and_world_transforms(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            grid_path = Path(temp_dir) / "slam_grid.npy"
            np.save(grid_path, np.asarray([[-1, 100], [0, 50]], dtype=np.int8))
            snapshot = {
                "grid_path": grid_path,
                "layer_id": "drone1/slam",
                "snapshot_index": 1,
                "elapsed_sec": 30.0,
                "meta": {
                    "frame_id": "drone1/map",
                    "resolution": 1.0,
                    "coverage": 0.75,
                    "origin": {"x": 10.0, "y": 20.0, "yaw": math.pi / 2.0},
                },
            }
            points, summary = quant_dashboard.occupancy_snapshot_points(
                snapshot,
                {"x": 3.0, "y": -4.0, "yaw": 0.0},
            )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0]["x"], 12.5)
        self.assertAlmostEqual(points[0]["y"], 17.5)
        self.assertEqual(summary["observed_cells"], 3)
        self.assertEqual(summary["occupied_cells"], 1)

    def test_projected_conflict_points_locates_disagreement(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            free_path = root / "free.npy"
            occupied_path = root / "occupied.npy"
            target_path = root / "target.npy"
            np.save(free_path, np.asarray([[0]], dtype=np.int8))
            np.save(occupied_path, np.asarray([[100]], dtype=np.int8))
            np.save(target_path, np.full((4, 4), -1, dtype=np.int8))
            source_meta = {
                "resolution": 1.0,
                "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            }
            target = {
                "grid_path": target_path,
                "meta": {
                    "resolution": 1.0,
                    "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                },
            }
            identity = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            points, summary = quant_dashboard.projected_conflict_points(
                [
                    ({"grid_path": free_path, "meta": source_meta}, identity),
                    ({"grid_path": occupied_path, "meta": source_meta}, identity),
                ],
                target,
            )

        self.assertEqual(summary["source_overlap_cells"], 1)
        self.assertEqual(summary["conflict_cells"], 1)
        self.assertAlmostEqual(points[0]["x"], 0.5)
        self.assertAlmostEqual(points[0]["y"], 0.5)

    def test_project_snapshot_to_target_uses_vehicle_translation(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source_path = root / "source.npy"
            target_path = root / "target.npy"
            np.save(source_path, np.asarray([[100]], dtype=np.int8))
            np.save(target_path, np.full((3, 5), -1, dtype=np.int8))
            source = {
                "grid_path": source_path,
                "meta": {
                    "resolution": 1.0,
                    "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                },
            }
            target = {
                "grid_path": target_path,
                "meta": {
                    "resolution": 1.0,
                    "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                },
            }
            projected = quant_dashboard.project_snapshot_to_target(
                source,
                {"x": 2.0, "y": 1.0, "yaw": 0.0},
                target,
            )

        self.assertEqual(projected.shape, (3, 5))
        self.assertEqual(projected[1, 2], 100)
        self.assertEqual(int(np.count_nonzero(projected >= 0)), 1)

    def test_dilate_boolean_mask_makes_single_map_cell_visible(self):
        mask = np.zeros((7, 7), dtype=bool)
        mask[3, 3] = True

        expanded = quant_dashboard.dilate_boolean_mask(mask, radius=1)

        self.assertEqual(int(np.count_nonzero(expanded)), 9)
        self.assertTrue(np.all(expanded[2:5, 2:5]))
        self.assertFalse(expanded[0, 0])


if __name__ == "__main__":
    unittest.main()
