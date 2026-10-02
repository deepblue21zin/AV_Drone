"""Real Streamlit form tests on temporary fixtures, never on user run labels."""
import json
import logging
import sys
import tempfile
import unittest
from pathlib import Path

logging.getLogger("streamlit").setLevel(logging.ERROR)
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from dashboard_catalog import load_catalog, save_entries

try:
    from streamlit.testing.v1 import AppTest
except ImportError:
    AppTest = None

logging.disable(logging.CRITICAL)


@unittest.skipIf(AppTest is None, "Streamlit testing dependency not installed")
class DashboardWorkspaceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        run = self.root / "artifacts/fixture"
        (run / "drone1").mkdir(parents=True)
        (run / "drone1/metadata.json").write_text('{"drone_name":"drone1"}')
        self.data_path = run / "drone1/metadata.json"
        self.raw_before = self.data_path.read_bytes()
        save_entries(self.root, {"fixture": {"label": "원래 이름", "purpose": "같은 bag 비교"}}, 0)
        self.source = (
            f"import sys\nfrom pathlib import Path\nsys.path.insert(0, {str(ROOT / 'scripts')!r})\n"
            f"from quant_dashboard import render_dashboard\nrender_dashboard(Path({str(self.root)!r}))\n"
        )
        self.app = AppTest.from_string(self.source, default_timeout=30).run()

    def tearDown(self):
        self.assertEqual(self.data_path.read_bytes(), self.raw_before)
        self.temp.cleanup()

    def button(self, label):
        return next(button for button in self.app.button if button.label == label)

    def test_home_has_no_mixed_experiment_averages(self):
        self.assertFalse(self.app.exception)
        self.assertIn("원래 이름", [title.value for title in self.app.title])
        self.assertFalse(any("Success rate" in metric.label for metric in self.app.metric))

    def install_guide(self, root=None):
        root = root or self.root
        path = root / "docs/slam_experiment_guide.md"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text((ROOT / "docs/slam_experiment_guide.md").read_text(encoding="utf-8"), encoding="utf-8")
        return path

    def test_guide_renders_markdown_source_without_changing_catalog(self):
        path = self.install_guide()
        catalog_path = self.root / "experiments/dashboard_catalog.json"
        before = catalog_path.read_bytes()
        self.app.radio(key="workspace_page").set_value("실험 해석 가이드").run()
        self.assertFalse(self.app.exception)
        self.assertIn(path.read_text(encoding="utf-8").strip(), [m.value.strip() for m in self.app.markdown])
        self.assertTrue(any(button.label == "가이드 Markdown 다운로드"
                            for button in self.app.get("download_button")))
        self.assertEqual(catalog_path.read_bytes(), before)

    def test_guide_is_available_without_experiment_data(self):
        empty = self.root / "empty_workspace"
        self.install_guide(empty)
        app = AppTest.from_string(self.source.replace(str(self.root), str(empty)), default_timeout=30).run()
        app.radio(key="workspace_page").set_value("실험 해석 가이드").run()
        self.assertFalse(app.exception)
        self.assertTrue(any("# 실험 해석 가이드" in m.value for m in app.markdown))

    def test_condition_help_shortcut_opens_guide(self):
        self.install_guide()
        analysis = self.root / "artifacts/fixture/lidar_registration"
        analysis.mkdir()
        (analysis / "metrics.json").write_text(json.dumps({"conditions": {"B0": {}, "B1": {}}}))
        (analysis / "manifest.json").write_text('{"gt_usage":"evaluation_only"}')
        self.app.run()
        self.button("조건·그래프 해석 가이드 열기").click().run()
        self.assertFalse(self.app.exception)
        self.assertEqual(self.app.radio(key="workspace_page").value, "실험 해석 가이드")

    def test_rename_persists_in_new_browser_session(self):
        self.app.text_input(key="catalog_edit_fixture_label").set_value("공통 관측 1구간")
        self.button("이름·설명 저장").click().run()
        self.assertFalse(self.app.exception)
        self.assertEqual(load_catalog(self.root)["runs"]["fixture"]["label"], "공통 관측 1구간")
        self.assertEqual(self.app.selectbox(key="workspace_run").options, ["공통 관측 1구간"])
        self.app = AppTest.from_string(self.source, default_timeout=30).run()
        self.assertIn("공통 관측 1구간", [title.value for title in self.app.title])

    def test_two_region_primary_reports_applied_not_just_accepted_factors(self):
        analysis = self.root / "artifacts/fixture/lidar_registration"
        analysis.mkdir()
        names = ["B0", "B1", "N-prior", "N-single-A", "N-single-B", "N-double"]
        (analysis / "metrics.json").write_text(json.dumps({"conditions": {c: {} for c in names}}))
        (analysis / "manifest.json").write_text(json.dumps({
            "gt_usage": "evaluation_only", "primary_condition": "N-double",
            "optimizer": {"N-double": {"accepted_factor_count": 2, "applied_factor_count": 0,
                                        "available": True, "success": False}}}))
        host = self.root / "runtime/sim/fixture/host.result.json"
        host.parent.mkdir(parents=True)
        host.write_text('{"reason":"mission_complete"}')
        self.app.run()
        self.assertFalse(self.app.exception)
        metric = next(m for m in self.app.metric if m.label == "적용한 LiDAR 제약")
        self.assertEqual(metric.value, "0개")
        self.assertTrue(any("N-single-A" in m.value for m in self.app.info))

    def test_two_factor_comparison_complete_is_distinct_from_accuracy_pass(self):
        analysis = self.root / "artifacts/fixture/lidar_registration"
        analysis.mkdir()
        counts = {"N-single-A": 1, "N-single-B": 1, "N-double": 2}
        (analysis / "metrics.json").write_text(json.dumps({
            "conditions": {name: {} for name in ["B0", *counts]},
            "phase2_gate": {"pass": False}}))
        (analysis / "manifest.json").write_text(json.dumps({
            "gt_usage": "evaluation_only", "primary_condition": "N-double",
            "optimizer": {name: dict(available=True, success=True, accepted_factor_count=n,
                                      applied_factor_count=n) for name, n in counts.items()}}))
        (analysis / "ablation_status.json").write_text(json.dumps({
            "complete": True, "actual_applied_counts": counts}))
        self.app.run()
        self.assertFalse(self.app.exception)
        self.assertTrue(any("1회·2회 비교 성립" in value.value for value in self.app.success))
        self.assertTrue(any("일부 기준 미달" in value.value for value in self.app.warning))

    def test_renaming_older_run_keeps_that_run_selected(self):
        second = self.root / "artifacts/newer/drone1"
        second.mkdir(parents=True)
        (second / "metadata.json").write_text("{}")
        self.app.run()
        self.app.selectbox(key="workspace_run").select("fixture").run()
        self.app.text_input(key="catalog_edit_fixture_label").set_value("예전 실험 새 이름")
        self.button("이름·설명 저장").click().run()
        self.assertFalse(self.app.exception)
        self.assertEqual(self.app.selectbox(key="workspace_run").value, "fixture")
        self.assertIn("예전 실험 새 이름", [title.value for title in self.app.title])

    def test_archive_empty_main_and_restore(self):
        self.app.radio(key="workspace_page").set_value("이름·보관 관리").run()
        self.app.text_input(key="catalog_edit_fixture_reason").set_value("완료된 준비 점검")
        self.app.checkbox(key="catalog_edit_fixture_confirm").check()
        self.button("기본 목록에서 삭제 → 보관함").click().run()
        self.assertFalse(self.app.exception)
        self.assertEqual(load_catalog(self.root)["runs"]["fixture"]["visibility"], "archive")
        self.assertFalse(any(select.label == "편집할 실험" for select in self.app.selectbox))
        self.app.radio(key="manage_scope").set_value("보관함").run()
        self.button("기본 목록으로 복원").click().run()
        self.assertFalse(self.app.exception)
        self.assertEqual(load_catalog(self.root)["runs"]["fixture"]["visibility"], "active")

    def test_other_browser_edit_is_not_overwritten(self):
        save_entries(self.root, {"fixture": {"label": "다른 창에서 저장"}}, 1)
        self.app.text_input(key="catalog_edit_fixture_label").set_value("오래된 창의 수정")
        self.button("이름·설명 저장").click().run()
        self.assertFalse(self.app.exception)
        self.assertTrue(any("다른 화면" in error.value for error in self.app.error))
        self.assertEqual(load_catalog(self.root)["runs"]["fixture"]["label"], "다른 창에서 저장")
        self.button("저장된 내용 다시 불러오기").click().run()
        self.assertEqual(self.app.text_input(key="catalog_edit_fixture_label").value, "다른 창에서 저장")
        self.app.text_input(key="catalog_edit_fixture_label").set_value("최신 내용으로 수정")
        self.button("이름·설명 저장").click().run()
        self.assertEqual(load_catalog(self.root)["runs"]["fixture"]["label"], "최신 내용으로 수정")


if __name__ == "__main__":
    unittest.main()
