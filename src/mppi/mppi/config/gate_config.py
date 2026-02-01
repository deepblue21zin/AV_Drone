from dataclasses import dataclass

@dataclass(frozen=True)
class GateConfig:
    takeoff_z: float = 3.0
    z_tol: float = 0.15

    stable_sec: float = 3.0
    handover_sec: float = 3.0

    pose_timeout_sec: float = 0.5
    ctrl_rate_hz: float = 30.0
