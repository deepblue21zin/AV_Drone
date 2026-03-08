#!/usr/bin/env python3

import csv
import json
import os
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("metrics_logger")

        self.declare_parameter("drone_name", "drone1")
        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("planner_cmd_topic", "/drone1/autonomy/cmd_vel")
        self.declare_parameter("safe_cmd_topic", "/drone1/safety/cmd_vel")
        self.declare_parameter("nearest_obstacle_topic", "/drone1/perception/nearest_obstacle_distance")
        self.declare_parameter("safety_event_topic", "/drone1/safety/event")
        self.declare_parameter("goal_reached_topic", "/drone1/mission/goal_reached")
        self.declare_parameter("artifacts_root", "/workspace/AV_Drone/artifacts")

        drone_name = str(self.get_parameter("drone_name").value)
        artifacts_root = Path(str(self.get_parameter("artifacts_root").value))
        ts = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.run_dir = artifacts_root / f"{ts}_{drone_name}"
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.run_dir / "metrics.csv"
        self.events_path = self.run_dir / "events.log"
        self.summary_path = self.run_dir / "summary.json"
        self.metadata_path = self.run_dir / "metadata.json"

        self.start_time = time.time()
        self.state_connected = False
        self.state_armed = False
        self.state_mode = ""
        self.pose_count = 0
        self.planner_cmd_count = 0
        self.safe_cmd_count = 0
        self.safety_event_count = 0
        self.goal_reached = False
        self.closest_obstacle = float("inf")
        self.last_pose_time = None
        self.pose_periods = []

        self._write_metadata()
        self._init_csv()

        self.create_subscription(State, str(self.get_parameter("state_topic").value), self._on_state, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self._on_pose, 10)
        self.create_subscription(TwistStamped, str(self.get_parameter("planner_cmd_topic").value), self._on_planner_cmd, 10)
        self.create_subscription(TwistStamped, str(self.get_parameter("safe_cmd_topic").value), self._on_safe_cmd, 10)
        self.create_subscription(Float32, str(self.get_parameter("nearest_obstacle_topic").value), self._on_obstacle, 10)
        self.create_subscription(String, str(self.get_parameter("safety_event_topic").value), self._on_safety_event, 10)
        self.create_subscription(Bool, str(self.get_parameter("goal_reached_topic").value), self._on_goal, 10)

        self.create_timer(1.0, self._write_periodic_row)

        self.get_logger().info(f"Metrics logger writing artifacts to {self.run_dir}")

    def _git_commit(self):
        try:
            result = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd="/workspace/AV_Drone",
                capture_output=True,
                text=True,
                check=True,
            )
            return result.stdout.strip()
        except Exception:
            return "unknown"

    def _write_metadata(self):
        metadata = {
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "git_commit": self._git_commit(),
            "drone_name": str(self.get_parameter("drone_name").value),
            "state_topic": str(self.get_parameter("state_topic").value),
            "pose_topic": str(self.get_parameter("pose_topic").value),
            "planner_cmd_topic": str(self.get_parameter("planner_cmd_topic").value),
            "safe_cmd_topic": str(self.get_parameter("safe_cmd_topic").value),
            "nearest_obstacle_topic": str(self.get_parameter("nearest_obstacle_topic").value),
            "safety_event_topic": str(self.get_parameter("safety_event_topic").value),
            "goal_reached_topic": str(self.get_parameter("goal_reached_topic").value),
        }
        self.metadata_path.write_text(json.dumps(metadata, indent=2))

    def _init_csv(self):
        with self.csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t_sec",
                "connected",
                "armed",
                "mode",
                "pose_count",
                "planner_cmd_count",
                "safe_cmd_count",
                "nearest_obstacle_m",
                "safety_event_count",
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

    def _on_planner_cmd(self, _msg: TwistStamped):
        self.planner_cmd_count += 1

    def _on_safe_cmd(self, _msg: TwistStamped):
        self.safe_cmd_count += 1

    def _on_obstacle(self, msg: Float32):
        self.closest_obstacle = min(self.closest_obstacle, float(msg.data))

    def _on_safety_event(self, msg: String):
        self.safety_event_count += 1
        self._append_event(msg.data)

    def _on_goal(self, msg: Bool):
        self.goal_reached = bool(msg.data)

    def _write_periodic_row(self):
        with self.csv_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                round(time.time() - self.start_time, 3),
                self.state_connected,
                self.state_armed,
                self.state_mode,
                self.pose_count,
                self.planner_cmd_count,
                self.safe_cmd_count,
                self.closest_obstacle,
                self.safety_event_count,
                self.goal_reached,
            ])

        if self.pose_periods:
            mean_period = sum(self.pose_periods) / len(self.pose_periods)
            p99_period = sorted(self.pose_periods)[min(len(self.pose_periods) - 1, int(len(self.pose_periods) * 0.99))]
            worst_period = max(self.pose_periods)
        else:
            mean_period = None
            p99_period = None
            worst_period = None

        summary = {
            "runtime_s": round(time.time() - self.start_time, 3),
            "connected": self.state_connected,
            "armed": self.state_armed,
            "mode": self.state_mode,
            "pose_count": self.pose_count,
            "planner_cmd_count": self.planner_cmd_count,
            "safe_cmd_count": self.safe_cmd_count,
            "safety_event_count": self.safety_event_count,
            "goal_reached": self.goal_reached,
            "closest_obstacle_m": self.closest_obstacle,
            "pose_period_mean_s": mean_period,
            "pose_period_p99_s": p99_period,
            "pose_period_worst_s": worst_period,
        }
        self.summary_path.write_text(json.dumps(summary, indent=2))


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()

