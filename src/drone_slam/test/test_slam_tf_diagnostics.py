import math
from types import SimpleNamespace

from drone_slam.slam_tf_diagnostics_node import (
    compose_map_pose,
    quaternion_yaw,
    wrap_angle,
)


def test_compose_map_pose_rotates_odom_translation():
    x, y, yaw = compose_map_pose(
        (2.0, -1.0, math.pi / 2.0),
        (3.0, 1.0, -math.pi / 4.0),
    )

    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 2.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 4.0, abs_tol=1e-9)


def test_wrap_angle_handles_pi_boundary():
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi, abs_tol=1e-9)
    assert math.isclose(wrap_angle(-3.0 * math.pi), -math.pi, abs_tol=1e-9)


def test_quaternion_yaw_extracts_heading():
    expected = -0.65
    quaternion = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(0.5 * expected),
        w=math.cos(0.5 * expected),
    )

    assert math.isclose(quaternion_yaw(quaternion), expected, abs_tol=1e-9)
