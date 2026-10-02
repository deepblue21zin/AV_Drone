import math
import warnings

import numpy as np

from drone_cslam.registration import register_submaps
from drone_cslam.se2 import inverse, transform_points, wrap_angle
from drone_cslam.submap import voxel_downsample


def _shape():
    horizontal = np.column_stack((np.linspace(-4.0, 4.0, 100), np.zeros(100)))
    vertical = np.column_stack((np.full(70, -2.0), np.linspace(-1.0, 3.0, 70)))
    diagonal_x = np.linspace(0.5, 3.5, 60)
    diagonal = np.column_stack((diagonal_x, 0.45 * diagonal_x + 1.0))
    return np.vstack((horizontal, vertical, diagonal))


def test_voxel_downsample_uses_centroids():
    points = np.array([[0.01, 0.01], [0.09, 0.05], [0.21, 0.01]])
    result = voxel_downsample(points, 0.2)
    assert result.shape == (2, 2)
    assert np.allclose(result[0], [0.05, 0.03])


def test_registration_recovers_relative_pose_without_truth_input():
    source = _shape()
    truth = np.array([1.15, -0.65, math.radians(8.0)])
    target = transform_points(inverse(truth), source)
    initial = np.array([0.75, -0.30, math.radians(3.0)])
    config = {
        "registration": {
            "minimum_submap_points": 50,
            "search_translation_m": 1.0,
            "search_yaw_deg": 12.0,
            "coarse_translation_step_m": 0.25,
            "coarse_yaw_step_deg": 2.0,
            "correspondence_distance_m": 0.35,
            "coarse_max_points": 400,
            "top_k": 4,
        },
        "gate": {
            "minimum_overlap_ratio": 0.7,
            "maximum_inlier_rmse_m": 0.12,
            "minimum_score_margin_ratio": 0.0,
            "maximum_correction_translation_m": 1.0,
            "maximum_correction_yaw_deg": 12.0,
        },
    }
    result = register_submaps(source, target, initial, config)
    assert result.accepted
    assert np.linalg.norm(result.pose[:2] - truth[:2]) < 0.08
    yaw_error = abs(float(wrap_angle(result.pose[2] - truth[2])))
    assert yaw_error < math.radians(1.0)
    assert result.overlap_ratio > 0.9


def test_registration_wrap_boundary_seed_stays_inside_optimizer_bounds():
    from scipy.optimize import OptimizeWarning
    source = _shape()
    truth = np.array([.3, -.2, math.radians(-178)])
    target = transform_points(inverse(truth), source)
    config = {"registration": {"minimum_submap_points": 50, "search_translation_m": .5,
              "search_yaw_deg": 12, "coarse_translation_step_m": .25,
              "coarse_yaw_step_deg": 3, "coarse_max_points": 200, "top_k": 4},
              "gate": {"minimum_score_margin_ratio": 0, "maximum_correction_yaw_deg": 15}}
    with warnings.catch_warnings():
        warnings.simplefilter("error", OptimizeWarning)
        result = register_submaps(source, target, np.array([.2, -.1, math.radians(176)]), config)
    assert np.linalg.norm(result.pose[:2]-truth[:2]) < .08
    assert abs(wrap_angle(result.pose[2]-truth[2])) < math.radians(1)


def test_refinement_cannot_discard_a_better_coarse_solution(monkeypatch):
    from types import SimpleNamespace
    from drone_cslam import registration
    source = _shape()
    calls = []
    def worsening_optimizer(objective, seed, **kwargs):
        bounds = kwargs["bounds"]
        calls.append(bounds)
        # Deliberately simulate a numerical refinement returning a worse pose.
        return SimpleNamespace(x=np.array([hi for lo, hi in bounds]))
    monkeypatch.setattr(registration, "minimize", worsening_optimizer)
    config = {"registration": {"minimum_submap_points": 50, "search_translation_m": 3.,
              "search_yaw_deg": 30, "coarse_translation_step_m": .5,
              "coarse_yaw_step_deg": 5, "top_k": 4},
              "gate": {"minimum_overlap_ratio": .9, "maximum_inlier_rmse_m": .1,
                       "minimum_score_margin_ratio": 0}}
    result = register_submaps(source, source.copy(), np.zeros(3), config)
    assert result.accepted
    assert result.cost < 1e-10
    np.testing.assert_allclose(result.pose, np.zeros(3), atol=1e-10)
    for bounds in calls:
        assert bounds[0][1]-bounds[0][0] <= 1.+1e-10
        assert bounds[1][1]-bounds[1][0] <= 1.+1e-10
        assert bounds[2][1]-bounds[2][0] <= math.radians(10)+1e-10
