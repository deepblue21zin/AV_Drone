from __future__ import annotations

from typing import Optional

from rclpy.node import Node
from std_srvs.srv import SetBool


class TakeoffBridge:
    """
    /offboard_takeoff/enable (std_srvs/SetBool) 래퍼.

    포인트:
      - service_is_ready()만 믿지 않고, 필요 시 wait_for_service()를 짧게 호출해서
        discovery/ready를 확실히 만든 뒤 call_async를 수행한다.
    """

    def __init__(self, node: Node, srv_name: str = "/offboard_takeoff/enable"):
        self._node = node
        self._srv_name = srv_name
        self._cli = node.create_client(SetBool, srv_name)
        self._last_sent: Optional[bool] = None

    def ready(self) -> bool:
        return self._cli.service_is_ready()

    def wait_ready(self, timeout_sec: float = 0.2) -> bool:
        """
        짧게 기다리면서 서비스 ready를 확보한다.
        - timeout_sec는 타이머 tick에서 너무 길게 잡지 말 것 (0.05~0.2 권장)
        """
        if self._cli.service_is_ready():
            return True
        return self._cli.wait_for_service(timeout_sec=timeout_sec)

    def enable(self, on: bool) -> bool:
        """
        요청을 보냈으면 True, 조건상(미준비/중복) 안 보냈으면 False.
        """
        on = bool(on)

        # 중복 호출 방지
        if self._last_sent == on:
            return False

        # 서비스 준비 확인: 그냥 포기하지 말고 짧게라도 기다려서 ready 확보 시도
        if not self._cli.service_is_ready():
            if not self._cli.wait_for_service(timeout_sec=0.2):
                return False

        req = SetBool.Request()
        req.data = on
        self._cli.call_async(req)

        self._last_sent = on
        self._node.get_logger().info(f"[takeoff_enable] -> {on}")
        return True

    # ---- backward-compatible API ----
    def set_enabled(self, on: bool) -> bool:
        return self.enable(on)
