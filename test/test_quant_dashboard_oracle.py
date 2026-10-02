import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "quant_dashboard.py"
SPEC = importlib.util.spec_from_file_location("quant_dashboard", MODULE_PATH)
quant_dashboard = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(quant_dashboard)


class QuantDashboardOracleTest(unittest.TestCase):
    def test_find_oracle_runs_respects_active_run_allowlist(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for run_id in ["included", "excluded"]:
                artifact = root / "artifacts" / run_id / "oracle_correction"
                artifact.mkdir(parents=True)
                (artifact / "metrics.json").write_text(
                    json.dumps({"conditions": {"B0": {}}}),
                    encoding="utf-8",
                )
                (artifact / "manifest.json").write_text(
                    json.dumps({"schema_version": 1}),
                    encoding="utf-8",
                )
            experiments = root / "experiments"
            experiments.mkdir()
            (experiments / "dashboard_active_runs.txt").write_text(
                "included\n", encoding="utf-8"
            )

            runs = quant_dashboard.find_oracle_runs(root)

        self.assertEqual([path.parent.name for path in runs], ["included"])

    def test_oracle_metrics_and_differential_are_flattened(self):
        metrics = {
            "conditions": {
                "B0": {
                    "differential": {
                        "endpoint_translation_m": 0.4,
                        "endpoint_yaw_deg": 1.2,
                        "translation_m": {"rmse": 0.3},
                        "yaw_deg": {"rmse": 0.8},
                        "samples": [
                            {
                                "progress_m": 5.0,
                                "translation_error_m": 0.2,
                                "yaw_error_deg": 0.5,
                            }
                        ],
                    },
                    "map": {
                        "occupied_chamfer_m": 0.08,
                        "occupied_f1": 0.7,
                    },
                }
            }
        }

        metric_rows = quant_dashboard.oracle_metric_rows(metrics)
        differential_rows = quant_dashboard.oracle_differential_rows(metrics)

        self.assertEqual(metric_rows[0]["condition"], "B0")
        self.assertEqual(metric_rows[0]["endpoint_translation_m"], 0.4)
        self.assertEqual(metric_rows[0]["occupied_chamfer_m"], 0.08)
        self.assertEqual(differential_rows[0]["progress_m"], 5.0)
        self.assertEqual(differential_rows[0]["yaw_error_deg"], 0.5)

    def test_trajectory_loader_adds_gt_once_from_b0(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            artifact = Path(temp_dir)
            trajectories = artifact / "trajectories"
            trajectories.mkdir()
            fields = [
                "condition",
                "vehicle",
                "keyframe",
                "progress_m",
                "x",
                "y",
                "gt_x",
                "gt_y",
            ]
            with (trajectories / "B0.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerow({
                    "condition": "B0",
                    "vehicle": "drone1",
                    "keyframe": 0,
                    "progress_m": 0.0,
                    "x": 3.1,
                    "y": -7.6,
                    "gt_x": 3.0,
                    "gt_y": -7.5,
                })

            rows = quant_dashboard.oracle_trajectory_rows(artifact)

        self.assertEqual(len(rows), 2)
        self.assertEqual({row["series"] for row in rows}, {"B0", "GT"})

    def test_factor_loader_attaches_keyframe_coordinates(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            artifact = Path(temp_dir)
            keyframes = artifact / "keyframes.csv"
            keyframes.write_text(
                "node,gt_x,gt_y\n0,1.0,2.0\n1,4.0,6.0\n",
                encoding="utf-8",
            )
            (artifact / "factors.jsonl").write_text(
                json.dumps({
                    "condition": "O-periodic",
                    "type": "oracle_inter_uav",
                    "source": 0,
                    "target": 1,
                    "measurement": [3.0, 4.0, 0.1],
                })
                + "\n",
                encoding="utf-8",
            )

            rows = quant_dashboard.oracle_factor_rows(artifact)

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["condition"], "O-periodic")
        self.assertEqual(rows[0]["source_x"], 1.0)
        self.assertEqual(rows[0]["target_y"], 6.0)

    def test_find_lidar_registration_runs_respects_allowlist(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for run_id in ["included", "excluded"]:
                artifact = root / "artifacts" / run_id / "lidar_registration"
                artifact.mkdir(parents=True)
                (artifact / "metrics.json").write_text(
                    json.dumps({"conditions": {"B0": {}}}),
                    encoding="utf-8",
                )
                (artifact / "manifest.json").write_text(
                    json.dumps({"schema_version": 1}),
                    encoding="utf-8",
                )
            experiments = root / "experiments"
            experiments.mkdir()
            (experiments / "dashboard_active_runs.txt").write_text(
                "included\n", encoding="utf-8"
            )

            runs = quant_dashboard.find_lidar_registration_runs(root)

        self.assertEqual([path.parent.name for path in runs], ["included"])

    def test_lidar_registration_rows_have_typed_values(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            artifact = Path(temp_dir)
            (artifact / "registrations.csv").write_text(
                "condition,pair,target_progress_m,overlap_ratio,accepted\n"
                "R-periodic,2,30.0,0.42,True\n",
                encoding="utf-8",
            )

            rows = quant_dashboard.lidar_registration_rows(artifact)

        self.assertEqual(rows[0]["pair"], 2.0)
        self.assertEqual(rows[0]["target_progress_m"], 30.0)
        self.assertAlmostEqual(rows[0]["overlap_ratio"], 0.42)
        self.assertTrue(rows[0]["accepted"])

    def test_no_gt_condition_is_not_hidden_by_legacy_condition_list(self):
        metrics = {"conditions": {"B0": {}, "B1": {}, "N-single": {
            "differential": {"endpoint_translation_m": 1.2, "samples": [
                {"progress_m": 30, "translation_error_m": 1.0, "yaw_error_deg": 2.0}]}}}}
        rows = quant_dashboard.lidar_metric_rows(metrics)
        self.assertEqual([row["condition"] for row in rows], ["B0", "B1", "N-single"])
        self.assertEqual(quant_dashboard.lidar_differential_rows(metrics)[0]["condition"], "N-single")


if __name__ == "__main__":
    unittest.main()
