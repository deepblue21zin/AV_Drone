import importlib.util
import json
import sys
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from dashboard_catalog import (
    CatalogConflict, CatalogError, active_ids, discover_experiments,
    display_label, entry_for, load_catalog, save_entries,
)
from quant_dashboard import artifact_dirs, find_lidar_registration_runs, find_map_debug_runs


def make_run(root, name="run-one"):
    run = root / "artifacts" / name
    (run / "drone1").mkdir(parents=True)
    (run / "drone1/metadata.json").write_text(json.dumps({"run_id": name, "drone_name": "drone1"}))
    (run / "drone1/paper_metrics.json").write_text("{}")
    (run / "lidar_registration").mkdir()
    (run / "lidar_registration/metrics.json").write_text(json.dumps({"conditions": {"B0": {}}}))
    (run / "lidar_registration/manifest.json").write_text("{}")
    (run / "drone1/maps").mkdir()
    (run / "drone1/maps/slam_meta.json").write_text("{}")
    (run / "drone1/maps/slam_grid.npy").write_bytes(b"untouched map fixture")
    return run


class DashboardCatalogTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.run = make_run(self.root)

    def tearDown(self):
        self.temp.cleanup()

    def test_discovery_excludes_backup_tree(self):
        backup = self.root / "artifacts/old_docker_images_backup/very/deep/drone1"
        backup.mkdir(parents=True)
        (backup / "metadata.json").write_text("{}")
        self.assertEqual(set(discover_experiments(self.root)), {"run-one"})

    def test_label_persists_without_modifying_experiment_files(self):
        before = {p: p.read_bytes() for p in self.run.rglob("*") if p.is_file()}
        saved = save_entries(self.root, {"run-one": {"label": "한글 이름", "purpose": "왜 보정하는가?"}}, 0)
        self.assertEqual(saved["revision"], 1)
        self.assertEqual(display_label(load_catalog(self.root), "run-one"), "한글 이름")
        self.assertEqual(before, {p: p.read_bytes() for p in self.run.rglob("*") if p.is_file()})

    def test_all_archived_does_not_fall_back_to_showing_everything(self):
        save_entries(self.root, {"run-one": {"visibility": "archive", "archive_reason": "준비 점검"}}, 0)
        self.assertEqual(active_ids(self.root), set())
        self.assertEqual(artifact_dirs(self.root), [])
        self.assertEqual(find_lidar_registration_runs(self.root), [])
        self.assertEqual(find_map_debug_runs(self.root), [])
        self.assertTrue(self.run.is_dir())

    def test_restore_and_backup_previous_revision(self):
        save_entries(self.root, {"run-one": {"visibility": "archive", "archive_reason": "준비"}}, 0)
        save_entries(self.root, {"run-one": {"visibility": "active"}}, 1)
        self.assertEqual(active_ids(self.root), {"run-one"})
        backup = self.root / "experiments/dashboard_catalog_history/revision_000001.json"
        self.assertEqual(json.loads(backup.read_text())["runs"]["run-one"]["visibility"], "archive")

    def test_stale_editor_cannot_overwrite_another_save(self):
        save_entries(self.root, {"run-one": {"label": "먼저 저장"}}, 0)
        with self.assertRaises(CatalogConflict):
            save_entries(self.root, {"run-one": {"label": "오래된 창"}}, 0)
        self.assertEqual(display_label(load_catalog(self.root), "run-one"), "먼저 저장")

    def test_atomic_write_failure_preserves_original_and_can_retry(self):
        save_entries(self.root, {"run-one": {"label": "원래 이름"}}, 0)
        with patch("dashboard_catalog._atomic_json", side_effect=OSError("disk error")):
            with self.assertRaises(OSError):
                save_entries(self.root, {"run-one": {"label": "새 이름"}}, 1)
        self.assertEqual(display_label(load_catalog(self.root), "run-one"), "원래 이름")
        saved = save_entries(self.root, {"run-one": {"label": "새 이름"}}, 1)
        self.assertEqual(saved["revision"], 2)

    def test_archive_requires_a_reason(self):
        with self.assertRaises(CatalogError):
            save_entries(self.root, {"run-one": {"visibility": "archive"}}, 0)
        self.assertEqual(load_catalog(self.root)["revision"], 0)

    def test_invalid_ids_labels_and_fields_are_rejected(self):
        for update in ({"../escape": {"label": "bad"}}, {"missing": {"label": "bad"}},
                       {"run-one": {"label": "  "}}, {"run-one": {"label": "a" * 121}},
                       {"run-one": {"group": "invented"}}, {"run-one": {"metric": 0}}):
            with self.subTest(update=update), self.assertRaises(CatalogError):
                save_entries(self.root, update, 0)

    def test_corrupt_catalog_is_never_overwritten(self):
        directory = self.root / "experiments"
        directory.mkdir()
        path = directory / "dashboard_catalog.json"
        path.write_text("not json")
        with self.assertRaises(CatalogError):
            save_entries(self.root, {"run-one": {"label": "name"}}, 0)
        self.assertEqual(path.read_text(), "not json")

    def test_duplicate_labels_remain_distinguishable(self):
        make_run(self.root, "run-two")
        catalog = save_entries(self.root, {name: {"label": "같은 이름"} for name in ("run-one", "run-two")}, 0)
        self.assertNotEqual(display_label(catalog, "run-one"), display_label(catalog, "run-two"))

    def test_new_run_is_visible_for_review(self):
        save_entries(self.root, {"run-one": {"visibility": "archive", "archive_reason": "준비"}}, 0)
        make_run(self.root, "run-two")
        self.assertEqual(active_ids(self.root), {"run-two"})
        self.assertEqual(entry_for(load_catalog(self.root), "run-two")["group"], "분류 대기")

    def test_catalog_overrides_legacy_allowlist(self):
        save_entries(self.root, {"run-one": {"visibility": "archive", "archive_reason": "준비"}}, 0)
        (self.root / "experiments/dashboard_active_runs.txt").write_text("run-one\n")
        self.assertEqual(find_lidar_registration_runs(self.root), [])


if __name__ == "__main__":
    unittest.main()
