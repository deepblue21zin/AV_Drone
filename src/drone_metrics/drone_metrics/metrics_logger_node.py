#!/usr/bin/env python3

import csv
import json
import math
import os
import shutil
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("metrics_logger")

        self.declare_parameter("drone_name", "drone1")
        self.declare_parameter("scenario_name", "single_drone_obstacle_demo")
        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("scan_topic", "/drone1/scan")
        self.declare_parameter("planner_cmd_topic", "/drone1/autonomy/cmd_vel")
        self.declare_parameter("safe_cmd_topic", "/drone1/safety/cmd_vel")
        self.declare_parameter("nearest_obstacle_topic", "/drone1/perception/nearest_obstacle_distance")
        self.declare_parameter("safety_event_topic", "/drone1/safety/event")
        self.declare_parameter("goal_reached_topic", "/drone1/mission/goal_reached")
        self.declare_parameter("mission_phase_topic", "/drone1/mission/phase")
        self.declare_parameter("home_pose_topic", "/drone1/mission/home_pose")
        self.declare_parameter("active_goal_topic", "/drone1/mission/active_goal")
        self.declare_parameter(
            "planner_debug_score_terms_topic",
            "/drone1/planner/avoid/debug/score_terms",
        )
        self.declare_parameter(
            "planner_debug_escape_active_topic",
            "/drone1/planner/avoid/debug/escape_active",
        )
        self.declare_parameter("slam_map_ready_topic", "/drone1/slam/map_ready")
        self.declare_parameter("slam_localization_ok_topic", "/drone1/slam/localization_ok")
        self.declare_parameter("slam_coverage_topic", "/drone1/slam/coverage")
        self.declare_parameter("artifacts_root", "/workspace/AV_Drone/artifacts")
        self.declare_parameter("baseline_name", "single_drone_autonomy_baseline")
        self.declare_parameter("planner_name", "local_planner_lidar_reactive")
        self.declare_parameter("planner_version", "reactive_v1")
        self.declare_parameter("controller_version", "autonomy_manager_v1")
        self.declare_parameter("experiment_condition", "baseline_single_goal")
        self.declare_parameter("condition_id", "")
        self.declare_parameter("scenario_id", "")
        self.declare_parameter("world_name", "")
        self.declare_parameter("planner_family", "")
        self.declare_parameter("map_source", "")
        self.declare_parameter("planned_path_length_m", 0.0)
        self.declare_parameter("planning_time_ms_p50", -1.0)
        self.declare_parameter("planning_time_ms_p95", -1.0)
        self.declare_parameter("replan_count", 0)
        self.declare_parameter("paper_metrics_success_requires_return", False)
        self.declare_parameter("experiment_seed", 0)
        self.declare_parameter("scenario_manifest_path", "")
        self.declare_parameter("autonomy_config_path", "")
        self.declare_parameter("mavros_config_path", "")
        self.declare_parameter("mavros_pluginlists_path", "")
        self.declare_parameter("launch_file_path", "")

        drone_name = str(self.get_parameter("drone_name").value)
        self.scenario_name = str(self.get_parameter("scenario_name").value)
        self.baseline_name = str(self.get_parameter("baseline_name").value)
        self.planner_name = str(self.get_parameter("planner_name").value)
        self.planner_version = str(self.get_parameter("planner_version").value)
        self.controller_version = str(self.get_parameter("controller_version").value)
        self.experiment_condition = str(self.get_parameter("experiment_condition").value)
        self.condition_id = (
            str(self.get_parameter("condition_id").value).strip()
            or self.experiment_condition
        )
        self.scenario_id = (
            str(self.get_parameter("scenario_id").value).strip()
            or self.scenario_name
        )
        self.world_name = str(self.get_parameter("world_name").value).strip()
        self.planner_family = str(self.get_parameter("planner_family").value).strip()
        self.map_source = str(self.get_parameter("map_source").value).strip()
        self.success_requires_return = bool(
            self.get_parameter("paper_metrics_success_requires_return").value
        )
        self.experiment_seed = int(self.get_parameter("experiment_seed").value)

        artifacts_root = Path(str(self.get_parameter("artifacts_root").value))
        ts = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.run_id = f"{ts}_{drone_name}"
        self.run_dir = artifacts_root / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.run_dir / "metrics.csv"
        self.events_path = self.run_dir / "events.log"
        self.summary_path = self.run_dir / "summary.json"
        self.metadata_path = self.run_dir / "metadata.json"
        self.trajectory_path = self.run_dir / "trajectory.csv"
        self.phase_summary_path = self.run_dir / "phase_summary.json"
        self.planner_debug_path = self.run_dir / "planner_debug.jsonl"
        self.slam_summary_path = self.run_dir / "slam_summary.json"
        self.paper_metrics_path = self.run_dir / "paper_metrics.json"
        self.parameter_snapshot_path = self.run_dir / "parameter_snapshot.json"
        self.config_snapshot_dir = self.run_dir / "config_snapshots"
        self.config_snapshot_dir.mkdir(parents=True, exist_ok=True)

        git_context = self._git_context()
        self.git_commit = git_context["git_commit"]
        self.git_branch = git_context["git_branch"]
        self.git_dirty = git_context["git_dirty"]
        self.px4_gz_world = os.environ.get(
            "PX4_SITL_WORLD", os.environ.get("PX4_GZ_WORLD", "unknown")
        )
        if not self.world_name:
            self.world_name = self.px4_gz_world
        self.px4_gz_model_name = os.environ.get("PX4_GZ_MODEL_NAME", "unknown")
        self.px4_sim_target = os.environ.get("PX4_SIM_TARGET", "unknown")
        self.px4_sim_model = os.environ.get("PX4_SIM_MODEL", "unknown")

        self.start_time = time.time()
        self.started_at = time.strftime("%Y-%m-%dT%H:%M:%S")
        self.state_connected = False
        self.state_armed = False
        self.state_mode = ""
        self.pose_count = 0
        self.scan_count = 0
        self.planner_cmd_count = 0
        self.safe_cmd_count = 0
        self.safety_event_count = 0
        self.goal_reached = False
        self.current_obstacle = float("inf")
        self.closest_obstacle = float("inf")
        self.current_phase = "startup"
        self.last_pose_time = None
        self.last_scan_time = None
        self.last_pose_xyz = None
        self.last_pose_sample_time = None
        self.latest_pose_xyz = None
        self.total_path_length_m = 0.0
        self.outbound_path_length_m = 0.0
        self.return_path_length_m = 0.0
        self.last_pose_speed_mps = 0.0
        self.pose_periods = []
        self.scan_periods = []
        self.safety_reason_counts = {}
        self.last_safety_reason = ""
        self.phase_started_at = self.start_time
        self.phase_history = []
        self.phase_stats = {}
        self.seen_phases = {self.current_phase}
        self.outbound_time_s = None
        self.return_time_s = None
        self.escape_count = 0
        self.last_escape_active = False
        self.control_effort = 0.0
        self.last_safe_cmd_time = None
        self.planner_debug_count = 0
        self.home_pose = None
        self.active_goal = None
        self.slam_map_ready = False
        self.slam_localization_ok = False
        self.slam_coverage = None
        self.snapshot_files = {}
        self.snapshot_copy_errors = {}

        self._write_parameter_snapshot()
        self._copy_reference_files()
        self._write_metadata()
        self._init_csv()
        self._init_trajectory_csv()

        self.create_subscription(State, str(self.get_parameter("state_topic").value), self._on_state, 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("planner_cmd_topic").value),
            self._on_planner_cmd,
            10,
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("safe_cmd_topic").value),
            self._on_safe_cmd,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("nearest_obstacle_topic").value),
            self._on_obstacle,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("safety_event_topic").value),
            self._on_safety_event,
            10,
        )
        self.create_subscription(Bool, str(self.get_parameter("goal_reached_topic").value), self._on_goal, 10)
        self.create_subscription(String, str(self.get_parameter("mission_phase_topic").value), self._on_phase, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("home_pose_topic").value), self._on_home_pose, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("active_goal_topic").value), self._on_active_goal, 10)
        self.create_subscription(String, str(self.get_parameter("planner_debug_score_terms_topic").value), self._on_planner_debug, 10)
        self.create_subscription(Bool, str(self.get_parameter("planner_debug_escape_active_topic").value), self._on_escape_active, 10)
        self.create_subscription(Bool, str(self.get_parameter("slam_map_ready_topic").value), self._on_slam_map_ready, 10)
        self.create_subscription(Bool, str(self.get_parameter("slam_localization_ok_topic").value), self._on_slam_localization_ok, 10)
        self.create_subscription(Float32, str(self.get_parameter("slam_coverage_topic").value), self._on_slam_coverage, 10)

        self.create_timer(1.0, self._write_periodic_row)

        self.get_logger().info(f"Metrics logger writing artifacts to {self.run_dir}")

    def _git_context(self):
        git_prefix = ["git", "-c", "safe.directory=/workspace/AV_Drone"]
        try:
            commit = subprocess.run(
                git_prefix + ["rev-parse", "HEAD"],
                cwd="/workspace/AV_Drone",
                capture_output=True,
                text=True,
                check=True,
            )
            branch = subprocess.run(
                git_prefix + ["rev-parse", "--abbrev-ref", "HEAD"],
                cwd="/workspace/AV_Drone",
                capture_output=True,
                text=True,
                check=True,
            )
            dirty = subprocess.run(
                git_prefix + ["status", "--porcelain"],
                cwd="/workspace/AV_Drone",
                capture_output=True,
                text=True,
                check=True,
            )
            return {
                "git_commit": commit.stdout.strip(),
                "git_branch": branch.stdout.strip(),
                "git_dirty": bool(dirty.stdout.strip()),
            }
        except Exception:
            return {
                "git_commit": "unknown",
                "git_branch": "unknown",
                "git_dirty": True,
            }

    def _json_safe(self, value):
        if isinstance(value, float):
            if math.isfinite(value):
                return value
            return None
        if isinstance(value, list):
            return [self._json_safe(item) for item in value]
        if isinstance(value, tuple):
            return [self._json_safe(item) for item in value]
        if isinstance(value, dict):
            return {str(key): self._json_safe(item) for key, item in value.items()}
        return value

    def _selected_environment(self):
        keys = [
            "DISPLAY",
            "HEADLESS",
            "ROS_DOMAIN_ID",
            "ROS_LOCALHOST_ONLY",
            "PX4_SITL_WORLD",
            "PX4_GZ_WORLD",
            "PX4_SIM_TARGET",
            "PX4_SIM_MODEL",
            "PX4_GZ_MODEL_NAME",
        ]
        return {key: os.environ.get(key, "") for key in keys if key in os.environ}

    def _metric_param(self, name: str):
        value = float(self.get_parameter(name).value)
        return value if value >= 0.0 else None

    def _straight_line_distance_m(self):
        if self.home_pose is None or self.active_goal is None:
            return None

        dx = float(self.active_goal["x"]) - float(self.home_pose["x"])
        dy = float(self.active_goal["y"]) - float(self.home_pose["y"])
        return math.hypot(dx, dy)

    def _period_p99(self, values):
        if not values:
            return None
        ordered = sorted(values)
        return ordered[min(len(ordered) - 1, int(len(ordered) * 0.99))]

    def _write_parameter_snapshot(self):
        names = sorted(self._parameters.keys())
        snapshot = {
            "generated_at": self.started_at,
            "run_id": self.run_id,
            "parameters": {
                name: self._json_safe(self.get_parameter(name).value) for name in names
            },
            "environment": self._selected_environment(),
        }
        self.parameter_snapshot_path.write_text(json.dumps(snapshot, indent=2, sort_keys=True))

    def _copy_reference_file(self, label: str, parameter_name: str):
        raw_path = str(self.get_parameter(parameter_name).value).strip()
        if not raw_path:
            return
        src = Path(raw_path)
        if not src.exists() or not src.is_file():
            self.snapshot_copy_errors[label] = f"missing:{src}"
            return

        dest = self.config_snapshot_dir / f"{label}_{src.name}"
        try:
            shutil.copy2(src, dest)
            self.snapshot_files[label] = str(dest)
        except Exception as exc:
            self.snapshot_copy_errors[label] = str(exc)

    def _copy_reference_files(self):
        self._copy_reference_file("autonomy_config", "autonomy_config_path")
        self._copy_reference_file("mavros_config", "mavros_config_path")
        self._copy_reference_file("mavros_pluginlists", "mavros_pluginlists_path")
        self._copy_reference_file("launch_file", "launch_file_path")
        self._copy_reference_file("scenario_manifest", "scenario_manifest_path")

    def _write_metadata(self):
        metadata = {
            "started_at": self.started_at,
            "run_id": self.run_id,
            "artifact_dir": str(self.run_dir),
            "git_commit": self.git_commit,
            "git_branch": self.git_branch,
            "git_dirty": self.git_dirty,
            "drone_name": str(self.get_parameter("drone_name").value),
            "scenario_name": self.scenario_name,
            "scenario_id": self.scenario_id,
            "baseline_name": self.baseline_name,
            "planner_name": self.planner_name,
            "planner_family": self.planner_family,
            "planner_version": self.planner_version,
            "controller_version": self.controller_version,
            "experiment_condition": self.experiment_condition,
            "condition_id": self.condition_id,
            "world_name": self.world_name,
            "map_source": self.map_source,
            "experiment_seed": self.experiment_seed,
            "px4_gz_world": self.px4_gz_world,
            "px4_gz_model_name": self.px4_gz_model_name,
            "px4_sim_target": self.px4_sim_target,
            "px4_sim_model": self.px4_sim_model,
            "planned_path_length_m": self._metric_param("planned_path_length_m"),
            "planning_time_ms_p50": self._metric_param("planning_time_ms_p50"),
            "planning_time_ms_p95": self._metric_param("planning_time_ms_p95"),
            "replan_count": int(self.get_parameter("replan_count").value),
            "state_topic": str(self.get_parameter("state_topic").value),
            "pose_topic": str(self.get_parameter("pose_topic").value),
            "scan_topic": str(self.get_parameter("scan_topic").value),
            "planner_cmd_topic": str(self.get_parameter("planner_cmd_topic").value),
            "safe_cmd_topic": str(self.get_parameter("safe_cmd_topic").value),
            "nearest_obstacle_topic": str(self.get_parameter("nearest_obstacle_topic").value),
            "safety_event_topic": str(self.get_parameter("safety_event_topic").value),
            "goal_reached_topic": str(self.get_parameter("goal_reached_topic").value),
            "mission_phase_topic": str(self.get_parameter("mission_phase_topic").value),
            "home_pose_topic": str(self.get_parameter("home_pose_topic").value),
            "active_goal_topic": str(self.get_parameter("active_goal_topic").value),
            "planner_debug_score_terms_topic": str(
                self.get_parameter("planner_debug_score_terms_topic").value
            ),
            "planner_debug_escape_active_topic": str(
                self.get_parameter("planner_debug_escape_active_topic").value
            ),
            "slam_map_ready_topic": str(self.get_parameter("slam_map_ready_topic").value),
            "slam_localization_ok_topic": str(
                self.get_parameter("slam_localization_ok_topic").value
            ),
            "slam_coverage_topic": str(self.get_parameter("slam_coverage_topic").value),
            "trajectory_path": str(self.trajectory_path),
            "phase_summary_path": str(self.phase_summary_path),
            "planner_debug_path": str(self.planner_debug_path),
            "slam_summary_path": str(self.slam_summary_path),
            "paper_metrics_path": str(self.paper_metrics_path),
            "parameter_snapshot_path": str(self.parameter_snapshot_path),
            "config_snapshot_dir": str(self.config_snapshot_dir),
            "config_snapshot_files": self.snapshot_files,
            "config_snapshot_errors": self.snapshot_copy_errors,
            "scenario_manifest_path": str(self.get_parameter("scenario_manifest_path").value),
            "autonomy_config_path": str(self.get_parameter("autonomy_config_path").value),
            "mavros_config_path": str(self.get_parameter("mavros_config_path").value),
            "mavros_pluginlists_path": str(self.get_parameter("mavros_pluginlists_path").value),
            "launch_file_path": str(self.get_parameter("launch_file_path").value),
            "host_name": os.uname().nodename,
            "environment": self._selected_environment(),
        }
        self.metadata_path.write_text(json.dumps(self._json_safe(metadata), indent=2))

    def _init_csv(self):
        with self.csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t_sec",
                "connected",
                "armed",
                "mode",
                "pose_count",
                "scan_count",
                "planner_cmd_count",
                "safe_cmd_count",
                "current_obstacle_m",
                "nearest_obstacle_m",
                "safety_event_count",
                "mission_phase",
                "goal_reached",
            ])

    def _init_trajectory_csv(self):
        with self.trajectory_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t_sec",
                "mission_phase",
                "x",
                "y",
                "z",
                "speed_mps",
                "total_path_length_m",
                "nearest_obstacle_m",
                "goal_reached",
            ])

    def _phase_bucket(self, phase: str):
        if phase not in self.phase_stats:
            self.phase_stats[phase] = {
                "time_s": 0.0,
                "path_length_m": 0.0,
                "min_obstacle_distance_m": None,
                "safety_event_count": 0,
                "escape_count": 0,
                "control_effort": 0.0,
            }
        return self.phase_stats[phase]

    def _classify_phase(self, phase: str) -> str:
        if phase in {"MAPPING_TO_GOAL", "FOLLOW_PLAN"}:
            return "outbound"
        if phase in {"RETURN_HOME_AVOID", "RETURN_HOME_MPPI"}:
            return "return"
        return "other"

    def _append_trajectory_row(self):
        if self.latest_pose_xyz is None:
            return
        x, y, z = self.latest_pose_xyz
        with self.trajectory_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                round(time.time() - self.start_time, 3),
                self.current_phase,
                x,
                y,
                z,
                self.last_pose_speed_mps,
                self.total_path_length_m,
                self.current_obstacle,
                self.goal_reached,
            ])

    def _append_event(self, text: str):
        with self.events_path.open("a") as f:
            f.write(f"{time.time() - self.start_time:.3f}s {text}\n")

    def _on_state(self, msg: State):
        self.state_connected = bool(msg.connected)
        self.state_armed = bool(msg.armed)
        self.state_mode = str(msg.mode)

    def _on_pose(self, msg: PoseStamped):
        now = time.time()
        if self.last_pose_time is not None:
            self.pose_periods.append(now - self.last_pose_time)
        self.last_pose_time = now
        self.pose_count += 1
        p = msg.pose.position
        xyz = (float(p.x), float(p.y), float(p.z))
        self.latest_pose_xyz = xyz

        if self.last_pose_xyz is not None and self.last_pose_sample_time is not None:
            dt = max(now - self.last_pose_sample_time, 1e-6)
            dx = xyz[0] - self.last_pose_xyz[0]
            dy = xyz[1] - self.last_pose_xyz[1]
            dz = xyz[2] - self.last_pose_xyz[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if math.isfinite(dist) and dist < 5.0:
                self.last_pose_speed_mps = dist / dt
                self.total_path_length_m += dist
                bucket = self._phase_bucket(self.current_phase)
                bucket["path_length_m"] += dist
                phase_kind = self._classify_phase(self.current_phase)
                if phase_kind == "outbound":
                    self.outbound_path_length_m += dist
                elif phase_kind == "return":
                    self.return_path_length_m += dist

        self.last_pose_xyz = xyz
        self.last_pose_sample_time = now
        self._append_trajectory_row()

    def _on_scan(self, _msg: LaserScan):
        now = time.time()
        if self.last_scan_time is not None:
            self.scan_periods.append(now - self.last_scan_time)
        self.last_scan_time = now
        self.scan_count += 1

    def _on_planner_cmd(self, _msg: TwistStamped):
        self.planner_cmd_count += 1

    def _on_safe_cmd(self, _msg: TwistStamped):
        now = time.time()
        if self.last_safe_cmd_time is not None:
            dt = max(now - self.last_safe_cmd_time, 0.0)
            v = _msg.twist.linear
            w = _msg.twist.angular
            effort = (v.x * v.x + v.y * v.y + w.z * w.z) * dt
            self.control_effort += effort
            self._phase_bucket(self.current_phase)["control_effort"] += effort
        self.last_safe_cmd_time = now
        self.safe_cmd_count += 1

    def _on_obstacle(self, msg: Float32):
        self.current_obstacle = float(msg.data)
        self.closest_obstacle = min(self.closest_obstacle, float(msg.data))
        bucket = self._phase_bucket(self.current_phase)
        current = bucket["min_obstacle_distance_m"]
        if math.isfinite(self.current_obstacle):
            if current is None:
                bucket["min_obstacle_distance_m"] = self.current_obstacle
            else:
                bucket["min_obstacle_distance_m"] = min(current, self.current_obstacle)

    def _on_safety_event(self, msg: String):
        self.safety_event_count += 1
        self.last_safety_reason = str(msg.data)
        self.safety_reason_counts[msg.data] = self.safety_reason_counts.get(msg.data, 0) + 1
        self._phase_bucket(self.current_phase)["safety_event_count"] += 1
        self._append_event(msg.data)

    def _on_goal(self, msg: Bool):
        self.goal_reached = bool(msg.data)

    def _on_phase(self, msg: String):
        if msg.data != self.current_phase:
            now = time.time()
            elapsed = max(now - self.phase_started_at, 0.0)
            self._phase_bucket(self.current_phase)["time_s"] += elapsed
            self.phase_history.append({
                "phase": self.current_phase,
                "start_t_sec": round(self.phase_started_at - self.start_time, 3),
                "end_t_sec": round(now - self.start_time, 3),
                "duration_s": round(elapsed, 3),
            })
            if msg.data == "HOVER_AT_GOAL" and self.current_phase in {"MAPPING_TO_GOAL", "FOLLOW_PLAN"}:
                self.outbound_time_s = elapsed
            if msg.data == "HOVER_AT_HOME" and self.current_phase in {"RETURN_HOME_AVOID", "RETURN_HOME_MPPI"}:
                self.return_time_s = elapsed
            self.current_phase = msg.data
            self.seen_phases.add(self.current_phase)
            self.phase_started_at = now
            self._append_event(f"phase={msg.data}")

    def _on_home_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.home_pose = {"x": float(p.x), "y": float(p.y), "z": float(p.z)}

    def _on_active_goal(self, msg: PoseStamped):
        p = msg.pose.position
        self.active_goal = {"x": float(p.x), "y": float(p.y), "z": float(p.z)}

    def _on_planner_debug(self, msg: String):
        self.planner_debug_count += 1
        payload = {
            "t_sec": round(time.time() - self.start_time, 3),
            "phase": self.current_phase,
            "data": msg.data,
        }
        try:
            parsed = json.loads(msg.data)
            payload["parsed"] = parsed
        except Exception:
            pass
        with self.planner_debug_path.open("a") as f:
            f.write(json.dumps(self._json_safe(payload), sort_keys=True) + "\n")

    def _on_escape_active(self, msg: Bool):
        active = bool(msg.data)
        if active and not self.last_escape_active:
            self.escape_count += 1
            self._phase_bucket(self.current_phase)["escape_count"] += 1
        self.last_escape_active = active

    def _on_slam_map_ready(self, msg: Bool):
        self.slam_map_ready = bool(msg.data)

    def _on_slam_localization_ok(self, msg: Bool):
        self.slam_localization_ok = bool(msg.data)

    def _on_slam_coverage(self, msg: Float32):
        self.slam_coverage = float(msg.data)

    def _safety_intervention_count(self) -> int:
        benign = {"normal", "startup_grace"}
        return sum(
            count for reason, count in self.safety_reason_counts.items() if reason not in benign
        )

    def _infer_failure_code(self) -> str:
        if self.goal_reached and self.current_phase == "HOVER_AT_GOAL":
            return ""
        if self.current_phase in {"HOVER_AT_HOME", "DONE"}:
            return ""

        if self.safety_reason_counts.get("emergency_stop_obstacle"):
            return "EMERGENCY_STOP_OBSTACLE"
        if self.safety_reason_counts.get("pose_timeout"):
            return "POSE_TIMEOUT"
        if self.safety_reason_counts.get("scan_timeout"):
            return "SCAN_TIMEOUT"
        if self.safety_reason_counts.get("planner_cmd_timeout"):
            return "PLANNER_CMD_TIMEOUT"
        if not self.state_connected:
            return "FCU_DISCONNECT"
        if self.current_phase == "WAIT_STREAM":
            return "WAIT_STREAM_STALL"
        if self.current_phase == "OFFBOARD_ARM" and not self.state_armed:
            return "OFFBOARD_ARM_STALL"
        return ""

    def _phase_summary_payload(self):
        stats = json.loads(json.dumps(self._json_safe(self.phase_stats)))
        now = time.time()
        current_elapsed = max(now - self.phase_started_at, 0.0)
        current_bucket = stats.setdefault(self.current_phase, {
            "time_s": 0.0,
            "path_length_m": 0.0,
            "min_obstacle_distance_m": None,
            "safety_event_count": 0,
            "escape_count": 0,
            "control_effort": 0.0,
        })
        current_bucket["time_s"] = current_bucket.get("time_s", 0.0) + current_elapsed

        timeline = list(self.phase_history)
        timeline.append({
            "phase": self.current_phase,
            "start_t_sec": round(self.phase_started_at - self.start_time, 3),
            "end_t_sec": None,
            "duration_s": round(current_elapsed, 3),
        })

        return {
            "run_id": self.run_id,
            "condition": self.experiment_condition,
            "current_phase": self.current_phase,
            "phase_timeline": timeline,
            "phase_stats": stats,
            "outbound_time_s": self.outbound_time_s,
            "return_time_s": self.return_time_s,
            "outbound_path_length_m": self.outbound_path_length_m,
            "return_path_length_m": self.return_path_length_m,
            "total_path_length_m": self.total_path_length_m,
            "safety_event_count_by_phase": {
                phase: values.get("safety_event_count", 0)
                for phase, values in stats.items()
            },
        }

    def _slam_summary_payload(self):
        return {
            "run_id": self.run_id,
            "condition": self.experiment_condition,
            "map_ready": self.slam_map_ready,
            "localization_ok": self.slam_localization_ok,
            "coverage": self.slam_coverage,
        }

    def _paper_metrics_payload(self):
        failure_code = self._infer_failure_code()
        outbound_success = "HOVER_AT_GOAL" in self.seen_phases or self.current_phase in {
            "HOVER_AT_GOAL",
            "RETURN_HOME_AVOID",
            "RETURN_HOME_MPPI",
            "HOVER_AT_HOME",
            "DONE",
        }
        return_success = self.current_phase in {"HOVER_AT_HOME", "DONE"} or "HOVER_AT_HOME" in self.seen_phases
        success = return_success if self.success_requires_return else outbound_success

        if return_success:
            success_code = "return_home"
        elif outbound_success:
            success_code = "goal_only"
        elif failure_code:
            success_code = failure_code.lower()
        else:
            success_code = "in_progress"

        runtime_s = max(time.time() - self.start_time, 1e-6)
        mission_time_s = self.outbound_time_s if outbound_success else None
        actual_path_length_m = self.outbound_path_length_m if self.outbound_path_length_m > 0.0 else self.total_path_length_m
        straight_line_distance_m = self._straight_line_distance_m()
        path_efficiency = None
        if straight_line_distance_m is not None and actual_path_length_m > 1e-6:
            path_efficiency = straight_line_distance_m / actual_path_length_m
        planned_path_length_m = self._metric_param("planned_path_length_m")
        planning_time_ms_p50 = self._metric_param("planning_time_ms_p50")
        planning_time_ms_p95 = self._metric_param("planning_time_ms_p95")
        return {
            "run_id": self.run_id,
            "condition_id": self.condition_id,
            "condition": self.experiment_condition,
            "scenario_id": self.scenario_id,
            "scenario": self.scenario_name,
            "world_name": self.world_name,
            "planner_family": self.planner_family,
            "map_source": self.map_source,
            "success": bool(success),
            "success_code": success_code,
            "outbound_success": bool(outbound_success),
            "return_success": bool(return_success),
            "mission_time_s": mission_time_s,
            "actual_path_length_m": actual_path_length_m,
            "straight_line_distance_m": straight_line_distance_m,
            "planned_path_length_m": planned_path_length_m,
            "path_efficiency": path_efficiency,
            "outbound_time_s": self.outbound_time_s,
            "return_time_s": self.return_time_s,
            "runtime_s": round(runtime_s, 3),
            "total_path_length_m": self.total_path_length_m,
            "outbound_path_length_m": self.outbound_path_length_m,
            "return_path_length_m": self.return_path_length_m,
            "min_obstacle_distance_m": self.closest_obstacle,
            "safety_event_count": self.safety_event_count,
            "safety_intervention_count": self._safety_intervention_count(),
            "escape_count": self.escape_count,
            "mean_speed_mps": self.total_path_length_m / runtime_s,
            "control_effort": self.control_effort,
            "planning_time_ms_p50": planning_time_ms_p50,
            "planning_time_ms_p95": planning_time_ms_p95,
            "replan_count": int(self.get_parameter("replan_count").value),
            "pose_period_p99_s": self._period_p99(self.pose_periods),
            "scan_period_p99_s": self._period_p99(self.scan_periods),
            "planner_name": self.planner_name,
            "planner_version": self.planner_version,
            "controller_version": self.controller_version,
            "experiment_seed": self.experiment_seed,
            "mission_phase": self.current_phase,
            "goal_reached": self.goal_reached,
            "home_pose": self.home_pose,
            "active_goal": self.active_goal,
            "slam_map_ready": self.slam_map_ready,
            "slam_localization_ok": self.slam_localization_ok,
            "map_coverage": self.slam_coverage,
            "failure_code": failure_code,
        }

    def _write_periodic_row(self):
        with self.csv_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                round(time.time() - self.start_time, 3),
                self.state_connected,
                self.state_armed,
                self.state_mode,
                self.pose_count,
                self.scan_count,
                self.planner_cmd_count,
                self.safe_cmd_count,
                self.current_obstacle,
                self.closest_obstacle,
                self.safety_event_count,
                self.current_phase,
                self.goal_reached,
            ])

        if self.pose_periods:
            mean_period = sum(self.pose_periods) / len(self.pose_periods)
            p99_period = sorted(self.pose_periods)[
                min(len(self.pose_periods) - 1, int(len(self.pose_periods) * 0.99))
            ]
            worst_period = max(self.pose_periods)
        else:
            mean_period = None
            p99_period = None
            worst_period = None

        if self.scan_periods:
            scan_mean_period = sum(self.scan_periods) / len(self.scan_periods)
            scan_p99_period = sorted(self.scan_periods)[
                min(len(self.scan_periods) - 1, int(len(self.scan_periods) * 0.99))
            ]
            scan_worst_period = max(self.scan_periods)
        else:
            scan_mean_period = None
            scan_p99_period = None
            scan_worst_period = None

        phase_summary = self._phase_summary_payload()
        slam_summary = self._slam_summary_payload()
        paper_metrics = self._paper_metrics_payload()

        summary = {
            "runtime_s": round(time.time() - self.start_time, 3),
            "run_id": self.run_id,
            "artifact_dir": str(self.run_dir),
            "git_commit": self.git_commit,
            "git_branch": self.git_branch,
            "git_dirty": self.git_dirty,
            "scenario_name": self.scenario_name,
            "scenario_id": self.scenario_id,
            "baseline_name": self.baseline_name,
            "planner_name": self.planner_name,
            "planner_family": self.planner_family,
            "planner_version": self.planner_version,
            "controller_version": self.controller_version,
            "experiment_condition": self.experiment_condition,
            "condition_id": self.condition_id,
            "world_name": self.world_name,
            "map_source": self.map_source,
            "experiment_seed": self.experiment_seed,
            "scenario_manifest_path": str(self.get_parameter("scenario_manifest_path").value),
            "parameter_snapshot_path": str(self.parameter_snapshot_path),
            "config_snapshot_dir": str(self.config_snapshot_dir),
            "trajectory_path": str(self.trajectory_path),
            "phase_summary_path": str(self.phase_summary_path),
            "planner_debug_path": str(self.planner_debug_path),
            "slam_summary_path": str(self.slam_summary_path),
            "paper_metrics_path": str(self.paper_metrics_path),
            "px4_gz_world": self.px4_gz_world,
            "px4_gz_model_name": self.px4_gz_model_name,
            "px4_sim_target": self.px4_sim_target,
            "px4_sim_model": self.px4_sim_model,
            "connected": self.state_connected,
            "armed": self.state_armed,
            "mode": self.state_mode,
            "mission_phase": self.current_phase,
            "pose_count": self.pose_count,
            "scan_count": self.scan_count,
            "planner_cmd_count": self.planner_cmd_count,
            "safe_cmd_count": self.safe_cmd_count,
            "safety_event_count": self.safety_event_count,
            "safety_intervention_count": self._safety_intervention_count(),
            "safety_reason_counts": self.safety_reason_counts,
            "last_safety_reason": self.last_safety_reason,
            "failure_code": self._infer_failure_code(),
            "goal_reached": self.goal_reached,
            "home_pose": self.home_pose,
            "active_goal": self.active_goal,
            "current_obstacle_m": self.current_obstacle,
            "closest_obstacle_m": self.closest_obstacle,
            "total_path_length_m": self.total_path_length_m,
            "outbound_path_length_m": self.outbound_path_length_m,
            "return_path_length_m": self.return_path_length_m,
            "outbound_time_s": self.outbound_time_s,
            "return_time_s": self.return_time_s,
            "escape_count": self.escape_count,
            "planner_debug_count": self.planner_debug_count,
            "control_effort": self.control_effort,
            "slam_map_ready": self.slam_map_ready,
            "slam_localization_ok": self.slam_localization_ok,
            "slam_coverage": self.slam_coverage,
            "pose_period_mean_s": mean_period,
            "pose_period_p99_s": p99_period,
            "pose_period_worst_s": worst_period,
            "scan_period_mean_s": scan_mean_period,
            "scan_period_p99_s": scan_p99_period,
            "scan_period_worst_s": scan_worst_period,
        }
        self.summary_path.write_text(json.dumps(summary, indent=2))
        self.phase_summary_path.write_text(json.dumps(self._json_safe(phase_summary), indent=2))
        self.slam_summary_path.write_text(json.dumps(self._json_safe(slam_summary), indent=2))
        self.paper_metrics_path.write_text(json.dumps(self._json_safe(paper_metrics), indent=2))


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()
