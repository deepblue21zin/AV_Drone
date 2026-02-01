# src/mppi/mppi/context.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from rclpy.node import Node

from mppi.config.gate_config import GateConfig
from mppi.io.mavros_io import MavrosIO
from mppi.io.takeoff_bridge import TakeoffBridge
from mppi.utils.mppi_controller import MPPIConfig


@dataclass
class GoalConfig:
    x: float = 10.0
    y: float = 0.0
    z: float = 3.0
    yaw: Optional[float] = None  # None이면 bearing-to-goal 사용


@dataclass
class RuntimeMemory:
    stable_start_sec: Optional[float] = None
    handover_start_sec: Optional[float] = None
    mppi_entered_sec: Optional[float] = None

    # --- takeoff enable/disable 전달 성공 여부 추적 ---
    takeoff_enable_sent: bool = False
    takeoff_disable_sent: bool = False


@dataclass
class Context:
    node: Node
    gate: GateConfig
    io: MavrosIO
    takeoff: TakeoffBridge

    goal: GoalConfig = field(default_factory=GoalConfig)
    mppi: MPPIConfig = field(default_factory=MPPIConfig)

    mem: RuntimeMemory = field(default_factory=RuntimeMemory)

    def now_sec(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9
