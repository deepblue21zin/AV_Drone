from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from mppi.context import Context


class State(ABC):
    name: str = "STATE"

    def enter(self, ctx: Context) -> None:
        pass

    @abstractmethod
    def tick(self, ctx: Context) -> Optional["State"]:
        """
        다음 상태로 전환이 필요하면 State 인스턴스 반환.
        유지면 None 반환.
        """
        raise NotImplementedError

    def exit(self, ctx: Context) -> None:
        pass

BaseState = State
