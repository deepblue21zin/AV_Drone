import math

import numpy as np

from drone_cslam.se2 import between, compose, inverse, wrap_angle
from drone_cslam.time_series import (
    interpolate_pose_series,
    select_keyframe_indices,
)


def test_compose_inverse_and_between_round_trip():
    first = np.array([2.0, -1.0, math.radians(170.0)])
    second = np.array([-0.5, 3.0, math.radians(-175.0)])
    relative = between(first, second)
    np.testing.assert_allclose(compose(first, relative), second, atol=1.0e-9)
    np.testing.assert_allclose(compose(first, inverse(first)), np.zeros(3), atol=1.0e-9)


def test_interpolation_uses_shortest_yaw_and_rejects_large_gap():
    times = np.array([0.0, 1.0, 4.0])
    poses = np.array(
        [
            [0.0, 0.0, math.radians(170.0)],
            [1.0, 0.0, math.radians(-170.0)],
            [4.0, 0.0, 0.0],
        ]
    )
    output, valid = interpolate_pose_series(
        times, poses, np.array([0.5, 2.0]), max_gap_sec=1.1
    )
    assert valid.tolist() == [True, False]
    assert abs(abs(math.degrees(output[0, 2])) - 180.0) < 1.0e-6


def test_keyframe_selection_keeps_first_threshold_crossing_and_last():
    poses = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [2.1, 0.0, 0.0],
            [2.2, 0.0, math.radians(6.0)],
            [2.3, 0.0, math.radians(6.0)],
        ]
    )
    selected = select_keyframe_indices(poses, 2.0, math.radians(5.0))
    assert selected.tolist() == [0, 2, 3, 4]
    assert -math.pi <= float(wrap_angle(math.pi)) < math.pi
