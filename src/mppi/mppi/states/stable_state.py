from __future__ import annotations

from typing import Optional

from mppi.context import Context
from mppi.states.base_state import State
from mppi.states.mppi_state import MPPIState


class StableState(State):
    name = "STABLE"

    def enter(self, ctx: Context) -> None:
        ctx.takeoff.enable(True)
        ctx.mem.stable_start_sec = None

    def tick(self, ctx: Context) -> Optional[State]:
        z = ctx.io.z()
        if z is None:
            return None

        at_alt = abs(z - ctx.gate.takeoff_z) < ctx.gate.z_tol
        if not at_alt:
            ctx.mem.stable_start_sec = None
            return None

        if ctx.mem.stable_start_sec is None:
            ctx.mem.stable_start_sec = ctx.now_sec()
            return None

        if (ctx.now_sec() - ctx.mem.stable_start_sec) >= ctx.gate.stable_sec:
            return MPPIState()

        return None
