import copy
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import runtime_guard as guard
from watch_mission_completion import CompletionTracker

FIXTURE = REPO / "test/runtime_guard_fixture.py"


class GuardTests(unittest.TestCase):
    def setUp(self):
        (REPO / "runtime").mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="guard_test_", dir=REPO / "runtime")
        self.project = Path(self.temp.name)
        self.run = self.project / "runtime/sim/test_session"
        self.run.mkdir(parents=True)
        guard.write_json(self.run / "session.json", {"run_dir": str(self.run)})
        self.limits = guard.load_limits()
        self.limits.update(max_runtime_sec=2, poll_sec=0.05, bag_grace_sec=2,
                           ros_grace_sec=0.3, sim_grace_sec=0.3, term_grace_sec=0.2)
        self.children = []

    def tearDown(self):
        for child in self.children:
            if child.poll() is None:
                guard.send_group(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        self.temp.cleanup()

    def start_guard(self, command, role="ros", mission=None):
        env = dict(os.environ, GUARD_TEST_PROJECT=str(self.project),
                   GUARD_TEST_RUN=str(self.run), GUARD_TEST_LIMITS=json.dumps(self.limits),
                   GUARD_TEST_ROLE=role)
        if mission is not None:
            env["GUARD_TEST_MISSION"] = str(mission)
            (self.project / "scripts").symlink_to(REPO / "scripts", target_is_directory=True)
        child = subprocess.Popen([sys.executable, str(FIXTURE), "supervise", *command],
                                 env=env, start_new_session=True, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
        self.children.append(child)
        return child

    def wait_file(self, path):
        deadline = time.monotonic() + 5
        while not path.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(path.exists(), str(path))

    def test_defaults_do_not_touch_px4_parameters(self):
        limits = guard.load_limits()
        self.assertEqual(limits["max_runtime_sec"], 1200)
        self.assertEqual(limits["max_log_gib"], 5)
        self.assertFalse(any("SDLOG" in key for key in limits))

    def test_invalid_limits_refused(self):
        path = self.project / "limits.json"
        for value in (0, -1, True, float("nan"), "100"):
            values = dict(self.limits, max_runtime_sec=value)
            guard.write_json(path, values)
            with self.assertRaises(ValueError):
                guard.load_limits(path)

    def test_unknown_limit_refused(self):
        path = self.project / "limits.json"
        guard.write_json(path, dict(self.limits, typo=123))
        with self.assertRaises(ValueError):
            guard.load_limits(path)

    def test_root_low_space(self):
        usage = lambda path: shutil._ntuple_diskusage(1000 * guard.GIB, 951 * guard.GIB, 49 * guard.GIB)
        self.assertIn("low_space:/:", guard.storage_problem(self.project, self.limits, usage))

    def test_project_low_space(self):
        def usage(path):
            free = (200 if Path(path) == Path("/") else 99) * guard.GIB
            return shutil._ntuple_diskusage(1000 * guard.GIB, 0, free)
        self.assertIn(str(self.project), guard.storage_problem(self.project, self.limits, usage))

    def test_threshold_boundary_allowed(self):
        def usage(path):
            free = (50 if Path(path) == Path("/") else 100) * guard.GIB
            return shutil._ntuple_diskusage(1000 * guard.GIB, 0, free)
        self.assertIsNone(guard.storage_problem(self.project, self.limits, usage))

    def test_tree_count_ignores_links(self):
        (self.run / "a").write_bytes(b"123")
        external = self.project / "large"
        external.write_bytes(b"x" * 100)
        (self.run / "link").symlink_to(external)
        before = len((self.run / "session.json").read_bytes())
        self.assertEqual(guard.tree_bytes(self.run), before + 3)

    def test_limit_time(self):
        with mock.patch.object(guard, "storage_problem", return_value=None):
            self.assertEqual(guard.limit_problem(self.project, self.run, self.limits, 2), "max_runtime")

    def test_limit_size(self):
        self.limits["max_log_gib"] = 1 / guard.GIB
        with mock.patch.object(guard, "storage_problem", return_value=None):
            self.assertEqual(guard.limit_problem(self.project, self.run, self.limits, 0), "max_log_size")

    def test_old_log_preserved_and_new_logs_redirected(self):
        old = self.project / "rootfs/2/log"
        old.mkdir(parents=True)
        (old / "original.ulg").write_bytes(b"keep")
        guard.prepare_px4(old.parent, 2, self.run)
        self.assertTrue(old.is_symlink())
        preserved = list(old.parent.glob("log.pre_guard_*"))
        self.assertEqual(len(preserved), 1)
        self.assertEqual((preserved[0] / "original.ulg").read_bytes(), b"keep")
        (old / "new.ulg").write_bytes(b"new")
        self.assertEqual((self.run / "px4/instance_2/log/new.ulg").read_bytes(), b"new")
        guard.prepare_px4(old.parent, 2, self.run)
        self.assertEqual(len(list(old.parent.glob("log.pre_guard_*"))), 1)

    def test_repoint_preserves_previous_run(self):
        old = self.project / "old_log"
        old.mkdir()
        (old / "keep.ulg").write_text("keep")
        source = self.project / "log"
        source.symlink_to(old)
        new = self.project / "new_log"
        new.mkdir()
        guard.redirect_path(source, new)
        self.assertEqual((old / "keep.ulg").read_text(), "keep")
        self.assertEqual(source.resolve(), new)

    def test_broken_link_preserved(self):
        source = self.project / "log"
        source.symlink_to(self.project / "missing")
        target = self.project / "new_log"
        target.mkdir()
        guard.redirect_path(source, target)
        self.assertEqual(len(list(self.project.glob("log.pre_guard_*"))), 1)

    def test_session_path_confinement(self):
        self.assertEqual(guard.validated_run(self.run, self.project), self.run)
        with self.assertRaises(ValueError):
            guard.validated_run(self.project, self.project)

    def test_normal_command_exit(self):
        child = self.start_guard([sys.executable, "-c", "pass"])
        self.assertEqual(child.wait(timeout=5), 0)
        report = guard.read_json(self.run / "ros.result.json")
        self.assertEqual(report["reason"], "command_exit:0")
        self.assertTrue(report["clean_shutdown"])

    def test_child_failure(self):
        child = self.start_guard([sys.executable, "-c", "raise SystemExit(7)"])
        self.assertEqual(child.wait(timeout=5), 1)
        self.assertEqual(guard.read_json(self.run / "ros.result.json")["reason"], "command_exit:7")

    def test_timeout_closes_bag_before_ros(self):
        bag = self.project / "bag"
        child = self.start_guard([sys.executable, str(FIXTURE), "launch", str(bag)])
        self.assertEqual(child.wait(timeout=7), 1)
        report = guard.read_json(self.run / "ros.result.json")
        self.assertEqual(report["reason"], "max_runtime")
        self.assertTrue(report["clean_shutdown"], report)
        self.assertTrue(report["bags"][0]["metadata_present"])
        self.assertLess(float((bag / "bag_stopped").read_text()), float((bag / "ros_stopped").read_text()))

    def test_stop_request(self):
        self.limits["max_runtime_sec"] = 10
        ready = self.project / "ready"
        child = self.start_guard([sys.executable, str(FIXTURE), "hold", str(ready)])
        self.wait_file(ready)
        guard.write_json(self.run / "stop.request.json", {"reason": "mission_complete"})
        self.assertEqual(child.wait(timeout=5), 0)

    def test_signal_closes_bag(self):
        self.limits["max_runtime_sec"] = 10
        bag = self.project / "bag"
        child = self.start_guard([sys.executable, str(FIXTURE), "launch", str(bag)])
        self.wait_file(bag / "bag_ready")
        child.send_signal(signal.SIGTERM)
        self.assertEqual(child.wait(timeout=5), 1)
        self.assertTrue(guard.read_json(self.run / "ros.result.json")["clean_shutdown"])

    def test_forced_shutdown_is_not_reported_clean(self):
        ready = self.project / "ready"
        child = self.start_guard([sys.executable, str(FIXTURE), "stubborn", str(ready)])
        self.assertEqual(child.wait(timeout=6), 1)
        report = guard.read_json(self.run / "ros.result.json")
        self.assertFalse(report["clean_shutdown"])
        self.assertTrue(report["forced"])
        self.assertEqual(report["remaining_pids"], [])

    def test_sim_guard_direct_entry(self):
        child = self.start_guard([sys.executable, "-c", "pass"], role="sim")
        self.assertEqual(child.wait(timeout=5), 0)
        descriptor = guard.read_json(self.project / "descriptor.json")
        run = Path(descriptor["run_dir"])
        self.assertTrue(guard.read_json(run / "sim.result.json")["clean_shutdown"])
        self.assertNotEqual(run, self.run)

    def test_runtime_size_limit(self):
        self.limits.update(max_runtime_sec=10, max_log_gib=10000 / guard.GIB)
        ready = self.project / "ready"
        child = self.start_guard([sys.executable, str(FIXTURE), "hold", str(ready)])
        self.wait_file(ready)
        (self.run / "new.ulg").write_bytes(b"x" * 11000)
        self.assertEqual(child.wait(timeout=5), 1)
        report = guard.read_json(self.run / "ros.result.json")
        self.assertEqual(report["reason"], "max_log_size")
        self.assertTrue(report["clean_shutdown"])
        self.assertTrue((self.run / "new.ulg").exists())

    def test_low_space_refuses_before_command(self):
        self.limits["root_min_free_gib"] = 10 ** 9
        ready = self.project / "ready"
        child = self.start_guard([sys.executable, str(FIXTURE), "hold", str(ready)])
        self.assertNotEqual(child.wait(timeout=5), 0)
        self.assertFalse(ready.exists())
        self.assertFalse((self.run / "ros.state.json").exists())

    @unittest.skipUnless(shutil.which("ros2"), "ROS container integration test")
    def test_real_mission_completion_and_rosbag(self):
        self.limits.update(max_runtime_sec=18, ros_grace_sec=3, bag_grace_sec=3)
        mission = self.run / "policy.json"
        guard.write_json(mission, {"targets": {"/guard_test/drone1/mission/phase": "HOVER_AT_GOAL",
                                              "/guard_test/drone2/mission/phase": "DONE"}})
        bag = self.project / "mission_bag"
        child = self.start_guard([sys.executable, str(FIXTURE), "mission_launch", str(bag)], mission=mission)
        self.assertEqual(child.wait(timeout=25), 0)
        report = guard.read_json(self.run / "ros.result.json")
        self.assertEqual(report["reason"], "mission_complete")
        self.assertTrue(report["clean_shutdown"], report)
        self.assertTrue(report["bags"][0]["metadata_present"], report)

    @unittest.skipUnless(shutil.which("ros2"), "ROS container integration test")
    def test_real_rosbag_metadata_flush(self):
        self.limits["max_runtime_sec"] = 4
        bag = self.project / "real_bag"
        child = self.start_guard(["ros2", "bag", "record", "--output", str(bag), "/runtime_guard_test"])
        self.assertEqual(child.wait(timeout=12), 1)
        report = guard.read_json(self.run / "ros.result.json")
        self.assertTrue(report["clean_shutdown"], report)
        self.assertTrue(report["bags"][0]["metadata_present"], report)


class MissionTests(unittest.TestCase):
    def test_all_drones_required_and_hold(self):
        tracker = CompletionTracker({"a": "HOVER_AT_GOAL", "b": "DONE"})
        tracker.update("a", "HOVER_AT_GOAL", 0)
        self.assertFalse(tracker.complete(0))
        for now in range(6):
            tracker.update("a", "HOVER_AT_GOAL", now)
            tracker.update("b", "DONE", now)
            self.assertEqual(tracker.complete(now), now == 5)

    def test_stale_phase_not_success(self):
        tracker = CompletionTracker({"a": "DONE"})
        tracker.update("a", "DONE", 0)
        self.assertFalse(tracker.complete(0))
        self.assertFalse(tracker.complete(6))

    def test_phase_regression_resets_hold(self):
        tracker = CompletionTracker({"a": "DONE"})
        tracker.update("a", "DONE", 0)
        tracker.complete(0)
        tracker.update("a", "RETURN_HOME_AVOID", 2)
        self.assertFalse(tracker.complete(2))
        tracker.update("a", "DONE", 4)
        self.assertFalse(tracker.complete(4))
        tracker.update("a", "DONE", 6)
        self.assertFalse(tracker.complete(6))

    def test_empty_policy_rejected(self):
        with self.assertRaises(ValueError):
            CompletionTracker({})


@unittest.skipUnless(importlib.util.find_spec("yaml"), "PyYAML needed for host runner")
class RunnerTests(unittest.TestCase):
    def test_running_or_unrelated_container_refused(self):
        from run_guarded_experiment import validate_container
        container = {"Name": "test", "State": {"Status": "exited"}, "HostConfig": {}, "Mounts": [
            {"Destination": "/workspace/AV_Drone", "Type": "bind", "RW": True, "Source": str(REPO)}]}
        validate_container(container)
        running = copy.deepcopy(container)
        running["State"]["Status"] = "running"
        with self.assertRaises(ValueError):
            validate_container(running)
        wrong = copy.deepcopy(container)
        wrong["Mounts"][0]["Source"] = "/tmp"
        with self.assertRaises(ValueError):
            validate_container(wrong)


if __name__ == "__main__":
    unittest.main()
