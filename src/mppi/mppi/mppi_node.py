#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from mppi.config.gate_config import GateConfig
from mppi.context import Context
from mppi.io.mavros_io import MavrosIO
from mppi.io.takeoff_bridge import TakeoffBridge
from mppi.states.init_state import InitState
from mppi.states.hold_state import HoldState


class MPPINode(Node):
    def __init__(self):
        super().__init__("mppi_node")

        self.get_logger().info(f"[BOOT] mppi_node file: {__file__}")


        # -------------------------
        # Gate / orchestration params
        # -------------------------
        self.declare_parameter("takeoff_z", 3.0)
        self.declare_parameter("z_tol", 0.3)
        self.declare_parameter("stable_sec", 1.0)
        self.declare_parameter("handover_sec", 3.0)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("ctrl_rate_hz", 30.0)

        # -------------------------
        # Goal params
        # -------------------------
        self.declare_parameter("goal_x", 12.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 3.0)
        # goal_yaw: nan이면 "제어하지 않음(None)"로 처리
        self.declare_parameter("goal_yaw", float("nan"))

        # -------------------------
        # Obstacle params (typed as DOUBLE_ARRAY)
        # 빈 리스트 []로 declare하면 BYTE_ARRAY로 잡혀서 크래시나는 케이스가 있으니,
        # Type 명시 선언 후, 런치에서 반드시 배열을 넣어주는 방식 권장.
        # -------------------------
        self.declare_parameter("obs_cx", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("obs_cy", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("obs_r",  Parameter.Type.DOUBLE_ARRAY)

        # -------------------------
        # MPPI tuning params
        # -------------------------
        self.declare_parameter("safe_margin", 1.0)
        self.declare_parameter("w_obs", 50.0)
        self.declare_parameter("obs_beta", 6.0)
        self.declare_parameter("w_collision", 1e6)

        self.declare_parameter("num_samples", 600)
        self.declare_parameter("lambda_", 1.0)
        self.declare_parameter("max_v_xy", 2.0)
        self.declare_parameter("max_yaw_rate", 1.2)

        gate = GateConfig(
            takeoff_z=float(self.get_parameter("takeoff_z").value),
            z_tol=float(self.get_parameter("z_tol").value),
            stable_sec=float(self.get_parameter("stable_sec").value),
            handover_sec=float(self.get_parameter("handover_sec").value),
            pose_timeout_sec=float(self.get_parameter("pose_timeout_sec").value),
            ctrl_rate_hz=float(self.get_parameter("ctrl_rate_hz").value),
        )

        io = MavrosIO(self)
        takeoff = TakeoffBridge(self)
        self.ctx = Context(node=self, gate=gate, io=io, takeoff=takeoff)

        # -------------------------
        # IMPORTANT: ROS 파라미터 -> Context 주입
        # (goal, obstacles, mppi weights/limits)
        # -------------------------
        self._load_params_into_context()

        self.state = InitState()
        self._enter(self.state)

        self.timer = self.create_timer(1.0 / gate.ctrl_rate_hz, self._tick)
        self.get_logger().info("mppi_node started")

    # -------------------------
    # Param -> Context injection
    # -------------------------
    def _load_params_into_context(self) -> None:
        # goal
        gx = float(self.get_parameter("goal_x").value)
        gy = float(self.get_parameter("goal_y").value)
        gz = float(self.get_parameter("goal_z").value)

        gyaw = float(self.get_parameter("goal_yaw").value)
        goal_yaw = None if math.isnan(gyaw) else gyaw

        # 장애물 배열 읽기 (None 방어)
        cx = self.get_parameter("obs_cx").value
        cy = self.get_parameter("obs_cy").value
        cr = self.get_parameter("obs_r").value
        cx = list(cx) if cx is not None else []
        cy = list(cy) if cy is not None else []
        cr = list(cr) if cr is not None else []

        if not (len(cx) == len(cy) == len(cr)):
            self.get_logger().error(
                f"[PARAM] obs arrays length mismatch: len(cx)={len(cx)}, len(cy)={len(cy)}, len(r)={len(cr)}. "
                "-> obstacles cleared"
            )
            obstacles = []
        else:
            obstacles = [(float(a), float(b), float(c)) for a, b, c in zip(cx, cy, cr)]

        # MPPI tuning
        safe_margin = float(self.get_parameter("safe_margin").value)
        w_obs = float(self.get_parameter("w_obs").value)
        obs_beta = float(self.get_parameter("obs_beta").value)
        w_collision = float(self.get_parameter("w_collision").value)

        num_samples = int(self.get_parameter("num_samples").value)
        lambda_ = float(self.get_parameter("lambda_").value)
        max_v_xy = float(self.get_parameter("max_v_xy").value)
        max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        # Context에 주입 (Context 구조가 바뀌어도 최소한 로그로 감지되게 방어)
        try:
            self.ctx.goal.x = gx
            self.ctx.goal.y = gy
            self.ctx.goal.z = gz
            self.ctx.goal.yaw = goal_yaw
        except Exception as e:
            self.get_logger().warn(f"[PARAM] failed to set ctx.goal.*: {e}")

        try:
            self.ctx.mppi.obstacles = obstacles
            self.ctx.mppi.safe_margin = safe_margin
            self.ctx.mppi.w_obs = w_obs
            self.ctx.mppi.obs_beta = obs_beta
            self.ctx.mppi.w_collision = w_collision

            self.ctx.mppi.num_samples = num_samples
            self.ctx.mppi.lambda_ = lambda_
            self.ctx.mppi.max_v_xy = max_v_xy
            self.ctx.mppi.max_yaw_rate = max_yaw_rate
        except Exception as e:
            self.get_logger().warn(f"[PARAM] failed to set ctx.mppi.*: {e}")

        self.get_logger().info(
            f"[PARAM] goal=({gx:.2f},{gy:.2f},{gz:.2f}, yaw={'None' if goal_yaw is None else f'{goal_yaw:.2f}'}) "
            f"obs={len(obstacles)} "
            f"safe_margin={safe_margin:.2f}, w_obs={w_obs:.1f}, beta={obs_beta:.1f}, w_col={w_collision:.1e} "
            f"samples={num_samples}, lambda={lambda_:.2f}, vmax_xy={max_v_xy:.2f}, ymax_rate={max_yaw_rate:.2f}"
        )

    # -------------------------
    # State machine
    # -------------------------
    def _enter(self, s):
        self.get_logger().info(f"[STATE] -> {s.name}")
        s.enter(self.ctx)

    def _tick(self):
        # failsafe: pose timeout이면 HOLD로 강제 전환
        if (self.ctx.io.pose() is not None) and (self.ctx.io.pose_age_sec() > self.ctx.gate.pose_timeout_sec):
            if not isinstance(self.state, HoldState):
                self.get_logger().warn("[FAILSAFE] pose timeout -> HOLD")
                self.state.exit(self.ctx)
                self.state = HoldState()
                self._enter(self.state)
            return

        nxt = self.state.tick(self.ctx)
        if nxt is None:
            return

        self.state.exit(self.ctx)
        self.state = nxt
        self._enter(self.state)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MPPINode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
