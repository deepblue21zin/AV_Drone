from __future__ import annotations

from typing import Optional

from mppi.context import Context
from mppi.states.base_state import State


class HoldState(State):
    name = "HOLD"

    def enter(self, ctx: Context) -> None:
        ctx.takeoff.enable(False)
        ctx.mem.stable_start_sec = None
        ctx.mem.mppi_start_sec = None

    def tick(self, ctx: Context) -> Optional[State]:
        # Step1: HOLD 유지
        return None
