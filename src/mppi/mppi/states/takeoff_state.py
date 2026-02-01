from __future__ import annotations

from typing import Optional

from mppi.context import Context
from mppi.states.base_state import State
from mppi.states.stable_state import StableState


class TakeoffState(State):
    name = "TAKEOFF"

    def enter(self, ctx: Context) -> None:
        ctx.takeoff.enable(True)
        ctx.mem.stable_start_sec = None
        ctx.mem.mppi_start_sec = None

    def tick(self, ctx: Context) -> Optional[State]:
        z = ctx.io.z()
        if z is None:
            return None

        if abs(z - ctx.gate.takeoff_z) < ctx.gate.z_tol:
            return StableState()
        return None
