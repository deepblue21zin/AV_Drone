import math

import numpy as np

from drone_cslam.pose_graph import PoseFactor, PosePrior, optimize_pose_graph
from drone_cslam.se2 import between


def test_periodic_inter_uav_factors_reduce_differential_endpoint_error():
    count = 8
    truth_first = np.array([[2.0 * i, -2.0, 0.0] for i in range(count)])
    truth_second = np.array([[2.0 * i, 2.0, 0.0] for i in range(count)])
    raw_first = truth_first.copy()
    raw_second = truth_second.copy()
    raw_first[:, 1] += np.linspace(0.0, 1.4, count)
    raw_second[:, 1] -= np.linspace(0.0, 1.4, count)
    raw_first[:, 2] += np.linspace(0.0, math.radians(4.0), count)
    raw_second[:, 2] -= np.linspace(0.0, math.radians(4.0), count)
    initial = np.vstack((raw_first, raw_second))

    intra_sigma = np.array([0.20, 0.20, math.radians(2.0)])
    oracle_sigma = np.array([0.02, 0.02, math.radians(0.2)])
    factors = []
    for offset, raw in ((0, raw_first), (count, raw_second)):
        for index in range(count - 1):
            factors.append(
                PoseFactor(
                    source=offset + index,
                    target=offset + index + 1,
                    measurement=between(raw[index], raw[index + 1]),
                    sigma=intra_sigma,
                    kind="intra",
                )
            )
    for index in (2, 4, 6, 7):
        factors.append(
            PoseFactor(
                source=index,
                target=count + index,
                measurement=between(truth_first[index], truth_second[index]),
                sigma=oracle_sigma,
                kind="oracle",
            )
        )
    priors = [
        PosePrior(0, truth_first[0], np.array([0.01, 0.01, math.radians(0.1)])),
        PosePrior(
            count,
            truth_second[0],
            np.array([0.01, 0.01, math.radians(0.1)]),
        ),
    ]
    before_relative = between(raw_first[-1], raw_second[-1])
    truth_relative = between(truth_first[-1], truth_second[-1])
    before_error = np.linalg.norm(between(truth_relative, before_relative)[:2])

    result = optimize_pose_graph(initial, factors, priors, max_function_evaluations=200)
    assert result.success, result.message
    after_relative = between(result.poses[count - 1], result.poses[-1])
    after_error = np.linalg.norm(between(truth_relative, after_relative)[:2])
    assert after_error < 0.25 * before_error


def test_optimizer_requires_a_gauge_prior():
    with np.testing.assert_raises(ValueError):
        optimize_pose_graph(np.zeros((2, 3)), [], [])
