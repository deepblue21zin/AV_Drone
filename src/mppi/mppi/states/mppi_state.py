# src/mppi/mppi/states/mppi_state.py
from __future__ import annotations

from typing import Optional, Tuple
import math

import numpy as np

from mppi.states.base_state import BaseState
from mppi.states.hold_state import HoldState
from mppi.utils.mppi_controller import MPPIController


class MPPIState(BaseState):
    name = "MPPI"

    def __init__(self):
        self.ctrl: Optional[MPPIController] = None

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """
        geometry_msgs/Quaternion -> yaw (ENU)
        yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        """
        x = float(q.x)
        y = float(q.y)
        z = float(q.z)
        w = float(q.w)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(math.atan2(siny_cosp, cosy_cosp))

    def enter(self, ctx):
        # MPPI 컨트롤러 생성 (cfg는 이미 ctx.mppi에 주입됨)
        self.ctrl = MPPIController(ctx.mppi)
        self.ctrl.reset()

        ctx.mem.mppi_entered_sec = ctx.now_sec()

        ctx.node.get_logger().info(
            f"[MPPI] entered. goal_xy=({ctx.goal.x:.2f},{ctx.goal.y:.2f}), "
            f"obstacles={len(ctx.mppi.obstacles)}, max_v={ctx.mppi.max_v_xy:.2f}, max_r={ctx.mppi.max_yaw_rate:.2f}"
        )

    def tick(self, ctx) -> Optional[BaseState]:
        pose = ctx.io.pose()
        if pose is None or self.ctrl is None:
            # pose가 없거나 초기화 미완료 -> HOLD로 보내도 되지만, 여기서는 대기
            return None

        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        z = float(pose.pose.position.z)
        yaw = self._quat_to_yaw(pose.pose.orientation)

        # (선택) 고도 유지: z가 너무 내려가면 HOLD (원하면 삭제 가능)
        if z < (ctx.gate.takeoff_z - 2.0 * ctx.gate.z_tol):
            ctx.node.get_logger().warn(f"[MPPI] altitude dropped (z={z:.2f}) -> HOLD")
            ctx.io.publish_velocity(0.0, 0.0, 0.0)
            return HoldState()

        # goal check (xy)
        dx = ctx.goal.x - x
        dy = ctx.goal.y - y
        dist = float(math.hypot(dx, dy))
        if dist <= float(ctx.mppi.goal_radius):
            ctx.node.get_logger().info(f"[MPPI] goal reached (dist={dist:.2f}) -> HOLD")
            ctx.io.publish_velocity(0.0, 0.0, 0.0)
            return HoldState()

        # MPPI step
        vx, vy, yaw_rate, dbg = self.ctrl.step(
            state=(x, y, yaw),
            goal_xy=(ctx.goal.x, ctx.goal.y),
            goal_yaw=ctx.goal.yaw,  # None이면 controller 내부에서 bearing 사용
        )

        # publish
        ctx.io.publish_velocity(vx, vy, yaw_rate)

        # (디버그) 주기적으로 로그
        # 너무 시끄러우면 주석 처리
        ctx.node.get_logger().info(
            f"[MPPI] x={x:.2f},y={y:.2f},dist={dist:.2f} | "
            f"cmd(vx={vx:.2f},vy={vy:.2f},r={yaw_rate:.2f}) | "
            f"Jmin={dbg.get('J_min', 0.0):.2f}, wmax={dbg.get('w_max', 0.0):.3f}"
        )
        return None

    def exit(self, ctx):
        # 상태를 나갈 때 정지
        try:
            ctx.io.publish_velocity(0.0, 0.0, 0.0)
        except Exception:
            pass
