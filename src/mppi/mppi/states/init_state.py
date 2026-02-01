# src/mppi/mppi/states/init_state.py
from __future__ import annotations

from typing import Optional

from mppi.states.base_state import BaseState
from mppi.states.mppi_state import MPPIState


class InitState(BaseState):
    name = "INIT"

    def enter(self, ctx):
        ctx.mem.stable_start_sec = None
        ctx.mem.handover_start_sec = None
        ctx.mem.mppi_entered_sec = None

        # takeoff enable/disable 전송 상태 초기화
        ctx.mem.takeoff_enable_sent = False
        ctx.mem.takeoff_disable_sent = False

        ctx.node.get_logger().info(
            f"[INIT] enter. target_z={ctx.gate.takeoff_z:.2f}, z_tol={ctx.gate.z_tol:.2f}, "
            f"stable_sec={ctx.gate.stable_sec:.2f}, handover_sec={ctx.gate.handover_sec:.2f}"
        )

    def _ensure_takeoff_enabled(self, ctx) -> None:
        """
        서비스가 ready 될 때까지 기다렸다가 enable(True)를 '실제로 전송'한다.
        한 번 성공하면 다시 보내지 않음.
        """
        if ctx.mem.takeoff_enable_sent:
            return

        if not ctx.takeoff.ready():
            # 서비스가 아직 준비 안 됨: 다음 tick에서 재시도
            return

        sent = ctx.takeoff.set_enabled(True)
        if sent:
            ctx.mem.takeoff_enable_sent = True
            ctx.node.get_logger().info("[INIT] takeoff enable SENT (service ready)")
        else:
            # ready인데도 sent가 False면 중복/내부상태 문제일 수 있으나 다음 tick에서 재시도
            ctx.node.get_logger().warn("[INIT] takeoff enable NOT sent (will retry)")

    def _ensure_takeoff_disabled(self, ctx) -> None:
        """
        handover 전에 disable(False)를 '실제로 전송'한다.
        """
        if ctx.mem.takeoff_disable_sent:
            return

        if not ctx.takeoff.ready():
            return

        sent = ctx.takeoff.set_enabled(False)
        if sent:
            ctx.mem.takeoff_disable_sent = True
            ctx.node.get_logger().info("[INIT] takeoff disable SENT (handover prep)")
        else:
            ctx.node.get_logger().warn("[INIT] takeoff disable NOT sent (will retry)")

    def tick(self, ctx) -> Optional[BaseState]:
        # 0) takeoff enable을 반드시 보장
        self._ensure_takeoff_enabled(ctx)

        pose = ctx.io.pose()
        if pose is None:
            return None

        z = float(pose.pose.position.z)
        z_ok = (z >= (ctx.gate.takeoff_z - ctx.gate.z_tol))

        if not z_ok:
            # 아직 목표 고도 미달 -> takeoff 유지
            return None

        now = ctx.now_sec()

        # (1) 안정화 타이머 시작
        if ctx.mem.stable_start_sec is None:
            ctx.mem.stable_start_sec = now
            ctx.node.get_logger().info(f"[INIT] altitude reached (z={z:.2f}). start stable timer.")
            return None

        stable_elapsed = now - ctx.mem.stable_start_sec
        if stable_elapsed < ctx.gate.stable_sec:
            return None

        # (2) stable_sec 지난 뒤 takeoff disable (handover 준비)
        if ctx.mem.handover_start_sec is None:
            self._ensure_takeoff_disabled(ctx)
            # disable을 실제로 전송한 뒤에만 handover 타이머를 시작 (안전)
            if ctx.mem.takeoff_disable_sent:
                ctx.mem.handover_start_sec = now
                ctx.node.get_logger().info("[INIT] stable done. start handover timer.")
            return None

        # (3) handover_sec 지난 뒤 MPPI 진입
        handover_elapsed = now - ctx.mem.handover_start_sec
        if handover_elapsed < ctx.gate.handover_sec:
            return None

        ctx.node.get_logger().info("[INIT] handover done -> enter MPPI.")
        return MPPIState()

    def exit(self, ctx):
        pass
