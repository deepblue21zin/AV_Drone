#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
단일 노드 기반 PX4 Offboard 미션 (MAVROS, ROS2)

상태 머신 흐름:
WAIT_STREAM(셋포인트 선발행) ->
OFFBOARD_ARM(OFFBOARD 전환 + ARM) ->
TAKEOFF(이륙) ->
HOVER_AFTER_TAKEOFF(이륙 후 안정화) ->
MPPI_GO(MPPI로 목표점 이동 + 장애물 회피) ->
HOVER_AT_GOAL(목표점에서 호버) ->
LAND(AUTO.LAND) ->
WAIT_LANDED(착지 확인 후 DISARM) ->
DONE

사용 토픽/서비스:
- 구독: /mavros/state (PX4 연결/모드/ARM 상태), /mavros/local_position/pose (위치/자세)
- 발행: /mavros/setpoint_velocity/cmd_vel (속도 셋포인트)
- 서비스: /mavros/set_mode, /mavros/cmd/arming

중요:
- Gazebo 월드의 장애물(위치/반경)과, 이 노드의 obs_x/obs_y/obs_r 파라미터가
  반드시 동일해야 MPPI 장애물 비용이 제대로 작동함.
"""

import math
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional, Any

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


# ============================================================
# 1) 기본 유틸 함수들
# ============================================================
def clamp(x: float, lo: float, hi: float) -> float:
    """값 x를 [lo, hi] 범위로 제한."""
    return max(lo, min(hi, x))


def wrap_pi(a: float) -> float:
    """각도를 (-pi, pi] 범위로 래핑."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언(orientation) -> yaw(회전각) 변환."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ============================================================
# 2) 장애물 / MPPI 설정 데이터 구조
# ============================================================
@dataclass
class Obstacle2D:
    """2D 원형 장애물(중심 cx,cy / 반경 r)."""
    cx: float
    cy: float
    r: float


@dataclass
class MPPIConfig:
    """
    MPPI 알고리즘 튜닝 파라미터 모음.

    dt, horizon:
      - rollout(예측) 시간축 해상도
      - 총 예측 시간 = dt * horizon

    num_samples:
      - 매 tick마다 샘플링할 후보 경로 개수(N)

    lam:
      - soft-min(확률 가중 평균) 온도 파라미터(작을수록 최저 코스트에 더 집중)
    """
    dt: float = 0.05
    horizon: int = 100
    num_samples: int = 400
    lam: float = 1.0

    # 속도/요율 제한
    v_max: float = 2.0
    yaw_rate_max: float = 1.2

    # 샘플링 노이즈 표준편차(탐색 정도)
    sigma_v: float = 0.6
    sigma_yaw_rate: float = 0.6

    # 목표점 비용 가중치(평균 거리 + 터미널 거리)
    w_goal: float = 8.0
    w_goal_final: float = 30.0

    # 장애물/제어/부드러움 비용 가중치
    w_obst: float = 120.0
    w_ctrl: float = 0.2
    w_smooth: float = 0.4

    # 장애물 거리 기반 shaping 파라미터
    safety_margin: float = 0.9   # 장애물 반경에 더해지는 “안전 여유”
    near_buffer: float = 0.6     # 장애물 근처 “경고 구간” 두께

    # 장애물 비용의 강도(근접/관통)
    near_k: float = 18.0
    penetrate_k: float = 80.0
    penetrate_bias: float = 200.0

    # MPPI nominal 초기화 시 목표 방향으로 주는 기본 속도
    v_nom: float = 1.2


# ============================================================
# 3) MPPI 핵심 클래스
#    - 샘플링 -> 롤아웃 -> 코스트 계산 -> 가중 평균 업데이트
# ============================================================
class MPPIController:
    """
    MPPI (Model Predictive Path Integral) 컨트롤러.

    출력 제어 입력:
      u = [vx, vy, yaw_rate]
    (현재 코드는 XY는 속도로 직접 적분, yaw도 yaw_rate로 적분하는 단순 모델)

    내부 상태:
      u_nom: 길이 horizon(H)인 nominal control 시퀀스
    """
    def __init__(self, cfg: MPPIConfig, obstacles: List[Obstacle2D]):
        self.cfg = cfg
        self.obstacles = obstacles
        self.u_nom = np.zeros((cfg.horizon, 3), dtype=np.float32)
        self.rng = np.random.default_rng()

    def reset(self):
        """nominal control 시퀀스를 0으로 초기화."""
        self.u_nom[:] = 0.0

    def set_nominal_towards_goal(self, x: float, y: float, goal_x: float, goal_y: float):
        """
        MPPI 시작 직전에 nominal control을 목표 방향으로 초기화.
        (완전 랜덤 탐색만으로 시작하면 초기 수렴이 느려질 수 있어서 방향성을 부여)
        """
        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy) + 1e-6

        vx = self.cfg.v_nom * dx / dist
        vy = self.cfg.v_nom * dy / dist
        vx = clamp(vx, -self.cfg.v_max, self.cfg.v_max)
        vy = clamp(vy, -self.cfg.v_max, self.cfg.v_max)

        self.u_nom[:, 0] = vx
        self.u_nom[:, 1] = vy
        self.u_nom[:, 2] = 0.0

    def step(self, state: Tuple[float, float, float], goal: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        MPPI 한 번 수행하고, 최적화된 시퀀스의 첫 제어입력(u0)을 반환.

        입력:
          state = (x, y, yaw)
          goal  = (gx, gy, gyaw)

        출력:
          (vx, vy, yaw_rate)
        """
        cfg = self.cfg
        x0, y0, yaw0 = state
        gx, gy, gyaw = goal

        H = cfg.horizon
        N = cfg.num_samples
        dt = cfg.dt

        # (1) 후보 제어 시퀀스 샘플링: u = u_nom + noise
        noise = np.zeros((N, H, 3), dtype=np.float32)
        noise[:, :, 0] = self.rng.normal(0.0, cfg.sigma_v, size=(N, H))
        noise[:, :, 1] = self.rng.normal(0.0, cfg.sigma_v, size=(N, H))
        noise[:, :, 2] = self.rng.normal(0.0, cfg.sigma_yaw_rate, size=(N, H))

        u = self.u_nom[None, :, :] + noise

        # 제어 입력 제한(최대 속도/최대 yaw_rate)
        u[:, :, 0] = np.clip(u[:, :, 0], -cfg.v_max, cfg.v_max)
        u[:, :, 1] = np.clip(u[:, :, 1], -cfg.v_max, cfg.v_max)
        u[:, :, 2] = np.clip(u[:, :, 2], -cfg.yaw_rate_max, cfg.yaw_rate_max)

        # (2) 롤아웃(예측): 단순 적분 모델로 궤적 생성
        xs = np.zeros((N, H + 1), dtype=np.float32)
        ys = np.zeros((N, H + 1), dtype=np.float32)
        yaws = np.zeros((N, H + 1), dtype=np.float32)
        xs[:, 0] = x0
        ys[:, 0] = y0
        yaws[:, 0] = yaw0

        for k in range(H):
            xs[:, k + 1] = xs[:, k] + u[:, k, 0] * dt
            ys[:, k + 1] = ys[:, k] + u[:, k, 1] * dt
            yaws[:, k + 1] = yaws[:, k] + u[:, k, 2] * dt

        # (3) 코스트 계산
        costs = np.zeros((N,), dtype=np.float32)

        # (a) 목표점 추종: horizon 전체 평균 거리^2
        dx = xs[:, 1:] - gx
        dy = ys[:, 1:] - gy
        dist2 = dx * dx + dy * dy
        costs += cfg.w_goal * np.mean(dist2, axis=1)

        # (b) 최종(terminal) 목표 거리^2
        dxf = xs[:, -1] - gx
        dyf = ys[:, -1] - gy
        costs += cfg.w_goal_final * (dxf * dxf + dyf * dyf)

        # (c) 최종 yaw 정렬(soft)
        dyaw = (yaws[:, -1] - gyaw + np.pi) % (2 * np.pi) - np.pi
        costs += 0.5 * (dyaw * dyaw)

        # (d) 장애물 비용: 가장 가까운 거리 기반(관통/근접 페널티)
        if self.obstacles:
            obst_cost = np.zeros((N,), dtype=np.float32)
            for obs in self.obstacles:
                ox = xs[:, 1:] - obs.cx
                oy = ys[:, 1:] - obs.cy
                d = np.sqrt(ox * ox + oy * oy)

                # horizon 동안 최소 거리
                dmin = np.min(d, axis=1)

                # signed distance: (거리 - (반경 + safety_margin))
                sdmin = dmin - (obs.r + cfg.safety_margin)

                # 관통 페널티
                pen_depth = np.clip(-sdmin, 0.0, None)
                pen_cost = (
                    cfg.penetrate_bias * (pen_depth > 0.0).astype(np.float32)
                    + cfg.penetrate_k * (pen_depth ** 2)
                )

                # 근접 페널티
                near_depth = np.clip(cfg.near_buffer - sdmin, 0.0, cfg.near_buffer)
                near_cost = cfg.near_k * (near_depth ** 2)

                obst_cost += (pen_cost + near_cost)

            costs += cfg.w_obst * obst_cost

        # (e) 제어 입력 크기 페널티
        costs += cfg.w_ctrl * np.mean(
            u[:, :, 0] ** 2 + u[:, :, 1] ** 2 + 0.4 * u[:, :, 2] ** 2, axis=1
        )

        # (f) 제어 입력 변화율 페널티
        du = u[:, 1:, :] - u[:, :-1, :]
        costs += cfg.w_smooth * np.mean(
            du[:, :, 0] ** 2 + du[:, :, 1] ** 2 + 0.4 * du[:, :, 2] ** 2, axis=1
        )

        # (4) soft-min 가중치 계산 후 u_nom 업데이트
        cmin = float(np.min(costs))
        weights = np.exp(-(costs - cmin) / max(cfg.lam, 1e-6))
        wsum = float(np.sum(weights)) + 1e-9
        weights = (weights / wsum).astype(np.float32)

        self.u_nom = np.tensordot(weights, u, axes=(0, 0)).astype(np.float32)

        # (5) Receding horizon
        u0 = self.u_nom[0].copy()
        self.u_nom[:-1] = self.u_nom[1:]
        self.u_nom[-1] = self.u_nom[-2]

        return float(u0[0]), float(u0[1]), float(u0[2])


# ============================================================
# 4) ROS2 노드(오프보드 미션 + MAVROS I/O + 상태 머신)
# ============================================================
class MPPIOffboardNode(Node):
    def __init__(self):
        super().__init__("mppi")

        # -------------------------
        # 미션 관련 파라미터
        # -------------------------
        self.declare_parameter("takeoff_z", 3.0)
        self.declare_parameter("goal_x", 24.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 3.0)
        self.declare_parameter("goal_yaw", 0.0)

        self.declare_parameter("hover_sec_after_takeoff", 2.0)
        self.declare_parameter("hover_sec_at_goal", 3.0)
        self.declare_parameter("goal_tol_xy", 0.6)

        # -------------------------
        # 장애물 파라미터(월드와 동일해야 함)
        #  - 빈 리스트([])로 declare하면 BYTE_ARRAY로 잡히는 케이스가 있어서
        #    None으로 선언해 override 타입(DOUBLE_ARRAY)을 그대로 받도록 한다.
        # -------------------------
        self.declare_parameter("obs_x", None)
        self.declare_parameter("obs_y", None)
        self.declare_parameter("obs_r", None)

        # -------------------------
        # MPPI 파라미터
        # -------------------------
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("horizon", 100)
        self.declare_parameter("num_samples", 400)

        self.declare_parameter("v_max", 2.0)
        self.declare_parameter("yaw_rate_max", 1.2)

        self.declare_parameter("safety_margin", 0.9)
        self.declare_parameter("near_buffer", 0.6)
        self.declare_parameter("w_obst", 120.0)

        # -------------------------
        # Z축 제어(단순 P 제어)
        # -------------------------
        self.declare_parameter("kp_z", 1.2)
        self.declare_parameter("vz_max", 1.2)

        # -------------------------
        # setpoint 발행 주기
        # -------------------------
        self.declare_parameter("cmd_rate_hz", 20.0)

        # -------------------------
        # 내부 상태
        # -------------------------
        self.current_state = State()
        self.pose: Optional[PoseStamped] = None
        self.last_pose_t = 0.0

        self.phase = "WAIT_STREAM"
        self.phase_t0 = time.time()
        self.pre_stream_count = 0

        # -------------------------
        # 구독/발행
        # -------------------------
        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_pose, qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)

        # -------------------------
        # 서비스 클라이언트(모드/ARM)
        # -------------------------
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.mode_future = None
        self.arm_future = None
        self.last_mode_req_t = 0.0
        self.last_arm_req_t = 0.0

        # -------------------------
        # MPPI 초기화
        # -------------------------
        obs = self._load_obstacles()
        cfg = MPPIConfig(
            dt=float(self.get_parameter("dt").value),
            horizon=int(self.get_parameter("horizon").value),
            num_samples=int(self.get_parameter("num_samples").value),
            v_max=float(self.get_parameter("v_max").value),
            yaw_rate_max=float(self.get_parameter("yaw_rate_max").value),
            safety_margin=float(self.get_parameter("safety_margin").value),
            near_buffer=float(self.get_parameter("near_buffer").value),
            w_obst=float(self.get_parameter("w_obst").value),
        )
        self.mppi = MPPIController(cfg, obs)

        # -------------------------
        # 타이머
        # -------------------------
        rate = float(self.get_parameter("cmd_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        # 시작 로그
        self.get_logger().info(f"obstacles loaded: {[(o.cx, o.cy, o.r) for o in obs]}")
        self.get_logger().info(
            f"mppi cfg: dt={cfg.dt}, horizon={cfg.horizon}, T={cfg.dt*cfg.horizon:.2f}s, "
            f"w_obst={cfg.w_obst}, safety_margin={cfg.safety_margin}, near_buffer={cfg.near_buffer}"
        )

    # -------------------------
    # 구독 콜백
    # -------------------------
    def _on_state(self, msg: State):
        self.current_state = msg

    def _on_pose(self, msg: PoseStamped):
        self.pose = msg
        self.last_pose_t = time.time()

    # -------------------------
    # pose 유틸
    # -------------------------
    def _pose_age(self) -> float:
        if self.pose is None:
            return 1e9
        return time.time() - self.last_pose_t

    def _get_xyz_yaw(self) -> Tuple[float, float, float, float]:
        p = self.pose.pose.position
        q = self.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return float(p.x), float(p.y), float(p.z), float(yaw)

    # -------------------------
    # 속도 셋포인트 발행
    # -------------------------
    def _publish_cmd(self, vx: float, vy: float, vz: float, yaw_rate: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(yaw_rate)
        self.cmd_pub.publish(msg)

    # -------------------------
    # phase 관리
    # -------------------------
    def _enter_phase(self, name: str):
        if self.phase != name:
            self.phase = name
            self.phase_t0 = time.time()
            self.get_logger().info(f"PHASE => {name}")

    def _phase_elapsed(self) -> float:
        return time.time() - self.phase_t0

    # -------------------------
    # 장애물 파라미터 로드(타입 꼬임 방지 강화)
    # -------------------------
    @staticmethod
    def _as_float_list(v: Any) -> List[float]:
        """
        파라미터 값을 float 리스트로 정규화.
        - None -> []
        - 단일 숫자 -> [float]
        - 시퀀스 -> [float...]
        """
        if v is None:
            return []
        if isinstance(v, (float, int)):
            return [float(v)]
        try:
            return [float(x) for x in list(v)]
        except Exception:
            return []

    def _load_obstacles(self) -> List[Obstacle2D]:
        xs = self._as_float_list(self.get_parameter("obs_x").value)
        ys = self._as_float_list(self.get_parameter("obs_y").value)
        rs = self._as_float_list(self.get_parameter("obs_r").value)
        n = min(len(xs), len(ys), len(rs))
        return [Obstacle2D(xs[i], ys[i], rs[i]) for i in range(n)]

    # -------------------------
    # OFFBOARD 모드 / ARM 서비스 요청
    # -------------------------
    def _request_set_mode(self, mode: str):
        if not self.mode_cli.service_is_ready():
            return
        if self.mode_future is not None and not self.mode_future.done():
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_future = self.mode_cli.call_async(req)

    def _request_arm(self, arm: bool):
        if not self.arm_cli.service_is_ready():
            return
        if self.arm_future is not None and not self.arm_future.done():
            return
        req = CommandBool.Request()
        req.value = bool(arm)
        self.arm_future = self.arm_cli.call_async(req)

    # ============================================================
    # 메인 루프
    # ============================================================
    def _tick(self):
        # OFFBOARD 유지용 기본 셋포인트
        self._publish_cmd(0.0, 0.0, 0.0, 0.0)

        if not self.current_state.connected:
            return
        if self.pose is None or self._pose_age() > 0.5:
            return

        # 파라미터 로드
        takeoff_z = float(self.get_parameter("takeoff_z").value)
        gx = float(self.get_parameter("goal_x").value)
        gy = float(self.get_parameter("goal_y").value)
        gz = float(self.get_parameter("goal_z").value)
        gyaw = float(self.get_parameter("goal_yaw").value)

        hover_after_takeoff = float(self.get_parameter("hover_sec_after_takeoff").value)
        hover_at_goal = float(self.get_parameter("hover_sec_at_goal").value)
        goal_tol = float(self.get_parameter("goal_tol_xy").value)

        kp_z = float(self.get_parameter("kp_z").value)
        vz_max = float(self.get_parameter("vz_max").value)

        x, y, z, yaw = self._get_xyz_yaw()

        # WAIT_STREAM
        if self.phase == "WAIT_STREAM":
            self.pre_stream_count += 1
            if self.pre_stream_count >= 40:
                self._enter_phase("OFFBOARD_ARM")

        # OFFBOARD_ARM
        elif self.phase == "OFFBOARD_ARM":
            now = time.time()

            if self.current_state.mode != "OFFBOARD":
                if (now - self.last_mode_req_t) > 1.0:
                    self._request_set_mode("OFFBOARD")
                    self.last_mode_req_t = now
                return

            if not self.current_state.armed:
                if (now - self.last_arm_req_t) > 1.0:
                    self._request_arm(True)
                    self.last_arm_req_t = now

                if self.arm_future is not None and self.arm_future.done():
                    res = self.arm_future.result()
                    if res is not None and bool(res.success):
                        self.mppi.reset()
                        self._enter_phase("TAKEOFF")

        # TAKEOFF
        elif self.phase == "TAKEOFF":
            err_z = takeoff_z - z
            vz_cmd = clamp(kp_z * err_z, -vz_max, vz_max)
            vz_cmd = clamp(vz_cmd, 0.2, vz_max) if err_z > 0.2 else vz_cmd
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if z >= takeoff_z - 0.15:
                self._enter_phase("HOVER_AFTER_TAKEOFF")

        # HOVER_AFTER_TAKEOFF
        elif self.phase == "HOVER_AFTER_TAKEOFF":
            err_z = takeoff_z - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if self._phase_elapsed() >= hover_after_takeoff:
                self.mppi.set_nominal_towards_goal(x, y, gx, gy)
                self._enter_phase("MPPI_GO")

        # MPPI_GO
        elif self.phase == "MPPI_GO":
            err_z = gz - z
            vz_hold = clamp(kp_z * err_z, -vz_max, vz_max)

            vx, vy, yr = self.mppi.step(state=(x, y, yaw), goal=(gx, gy, gyaw))

            d_goal = math.hypot(gx - x, gy - y)
            if d_goal < 2.0:
                scale = clamp(d_goal / 2.0, 0.25, 1.0)
                vx *= scale
                vy *= scale

            self._publish_cmd(vx, vy, vz_hold, yr)

            if d_goal <= goal_tol:
                self._enter_phase("HOVER_AT_GOAL")

        # HOVER_AT_GOAL
        elif self.phase == "HOVER_AT_GOAL":
            err_z = gz - z
            vz_cmd = clamp(kp_z * err_z, -0.6, 0.6)
            self._publish_cmd(0.0, 0.0, vz_cmd, 0.0)

            if self._phase_elapsed() >= hover_at_goal:
                self._enter_phase("LAND")

        # LAND
        elif self.phase == "LAND":
            now = time.time()
            if (now - self.last_mode_req_t) > 1.0:
                self._request_set_mode("AUTO.LAND")
                self.last_mode_req_t = now

            if self.mode_future is not None and self.mode_future.done():
                res = self.mode_future.result()
                if res is not None and bool(res.mode_sent):
                    self._enter_phase("WAIT_LANDED")

        # WAIT_LANDED
        elif self.phase == "WAIT_LANDED":
            if z < 0.2:
                now = time.time()
                if (now - self.last_arm_req_t) > 1.0:
                    self._request_arm(False)
                    self.last_arm_req_t = now

                if self.arm_future is not None and self.arm_future.done():
                    res = self.arm_future.result()
                    if res is not None and bool(res.success):
                        self._enter_phase("DONE")

        elif self.phase == "DONE":
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MPPIOffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()