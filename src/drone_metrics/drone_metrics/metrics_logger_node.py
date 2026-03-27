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
        self.declare_parameter("artifacts_root", "/workspace/AV_Drone/artifacts")
        self.declare_parameter("baseline_name", "single_drone_autonomy_baseline")
        self.declare_parameter("planner_name", "local_planner_lidar_reactive")
        self.declare_parameter("planner_version", "reactive_v1")
        self.declare_parameter("controller_version", "autonomy_manager_v1")
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
        self.pose_periods = []
        self.scan_periods = []
        self.safety_reason_counts = {}
        self.last_safety_reason = ""
        self.snapshot_files = {}
        self.snapshot_copy_errors = {}

        self._write_parameter_snapshot()
        self._copy_reference_files()
        self._write_metadata()
        self._init_csv()

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
            "baseline_name": self.baseline_name,
            "planner_name": self.planner_name,
            "planner_version": self.planner_version,
            "controller_version": self.controller_version,
            "experiment_seed": self.experiment_seed,
            "px4_gz_world": self.px4_gz_world,
            "px4_gz_model_name": self.px4_gz_model_name,
            "px4_sim_target": self.px4_sim_target,
            "px4_sim_model": self.px4_sim_model,
            "state_topic": str(self.get_parameter("state_topic").value),
            "pose_topic": str(self.get_parameter("pose_topic").value),
            "scan_topic": str(self.get_parameter("scan_topic").value),
            "planner_cmd_topic": str(self.get_parameter("planner_cmd_topic").value),
            "safe_cmd_topic": str(self.get_parameter("safe_cmd_topic").value),
            "nearest_obstacle_topic": str(self.get_parameter("nearest_obstacle_topic").value),
            "safety_event_topic": str(self.get_parameter("safety_event_topic").value),
            "goal_reached_topic": str(self.get_parameter("goal_reached_topic").value),
            "mission_phase_topic": str(self.get_parameter("mission_phase_topic").value),
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

    def _append_event(self, text: str):
        with self.events_path.open("a") as f:
            f.write(f"{time.time() - self.start_time:.3f}s {text}\n")

    def _on_state(self, msg: State):
        self.state_connected = bool(msg.connected)
        self.state_armed = bool(msg.armed)
        self.state_mode = str(msg.mode)

    def _on_pose(self, _msg: PoseStamped):
        now = time.time()
        if self.last_pose_time is not None:
            self.pose_periods.append(now - self.last_pose_time)
        self.last_pose_time = now
        self.pose_count += 1

    def _on_scan(self, _msg: LaserScan):
        now = time.time()
        if self.last_scan_time is not None:
            self.scan_periods.append(now - self.last_scan_time)
        self.last_scan_time = now
        self.scan_count += 1

    def _on_planner_cmd(self, _msg: TwistStamped):
        self.planner_cmd_count += 1

    def _on_safe_cmd(self, _msg: TwistStamped):
        self.safe_cmd_count += 1

    def _on_obstacle(self, msg: Float32):
        self.current_obstacle = float(msg.data)
        self.closest_obstacle = min(self.closest_obstacle, float(msg.data))

    def _on_safety_event(self, msg: String):
        self.safety_event_count += 1
        self.last_safety_reason = str(msg.data)
        self.safety_reason_counts[msg.data] = self.safety_reason_counts.get(msg.data, 0) + 1
        self._append_event(msg.data)

    def _on_goal(self, msg: Bool):
        self.goal_reached = bool(msg.data)

    def _on_phase(self, msg: String):
        if msg.data != self.current_phase:
            self.current_phase = msg.data
            self._append_event(f"phase={msg.data}")

    def _safety_intervention_count(self) -> int:
        benign = {"normal", "startup_grace"}
        return sum(
            count for reason, count in self.safety_reason_counts.items() if reason not in benign
        )

    def _infer_failure_code(self) -> str:
        if self.goal_reached and self.current_phase == "HOVER_AT_GOAL":
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

        summary = {
            "runtime_s": round(time.time() - self.start_time, 3),
            "run_id": self.run_id,
            "artifact_dir": str(self.run_dir),
            "git_commit": self.git_commit,
            "git_branch": self.git_branch,
            "git_dirty": self.git_dirty,
            "scenario_name": self.scenario_name,
            "baseline_name": self.baseline_name,
            "planner_name": self.planner_name,
            "planner_version": self.planner_version,
            "controller_version": self.controller_version,
            "experiment_seed": self.experiment_seed,
            "scenario_manifest_path": str(self.get_parameter("scenario_manifest_path").value),
            "parameter_snapshot_path": str(self.parameter_snapshot_path),
            "config_snapshot_dir": str(self.config_snapshot_dir),
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
            "current_obstacle_m": self.current_obstacle,
            "closest_obstacle_m": self.closest_obstacle,
            "pose_period_mean_s": mean_period,
            "pose_period_p99_s": p99_period,
            "pose_period_worst_s": worst_period,
            "scan_period_mean_s": scan_mean_period,
            "scan_period_p99_s": scan_p99_period,
            "scan_period_worst_s": scan_worst_period,
        }
        self.summary_path.write_text(json.dumps(summary, indent=2))


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()
