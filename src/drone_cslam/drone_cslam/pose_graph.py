"""A compact SciPy SE(2) pose-graph backend for the phase-1 oracle study."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from .se2 import between, wrap_angle


@dataclass(frozen=True)
class PoseFactor:
    source: int
    target: int
    measurement: np.ndarray
    sigma: np.ndarray
    kind: str


@dataclass(frozen=True)
class PosePrior:
    node: int
    measurement: np.ndarray
    sigma: np.ndarray


@dataclass(frozen=True)
class OptimizationResult:
    poses: np.ndarray
    success: bool
    cost: float
    optimality: float
    evaluations: int
    message: str


def factor_residual(
    poses: np.ndarray, factor: PoseFactor
) -> np.ndarray:
    predicted = between(poses[factor.source], poses[factor.target])
    error = between(factor.measurement, predicted)
    error[2] = float(wrap_angle(error[2]))
    return error / factor.sigma


def prior_residual(poses: np.ndarray, prior: PosePrior) -> np.ndarray:
    error = between(prior.measurement, poses[prior.node])
    error[2] = float(wrap_angle(error[2]))
    return error / prior.sigma


def graph_residuals(
    flat_poses: np.ndarray,
    node_count: int,
    factors: Sequence[PoseFactor],
    priors: Sequence[PosePrior],
) -> np.ndarray:
    poses = np.asarray(flat_poses, dtype=float).reshape(node_count, 3)
    residuals = [factor_residual(poses, factor) for factor in factors]
    residuals.extend(prior_residual(poses, prior) for prior in priors)
    if not residuals:
        return np.empty(0, dtype=float)
    return np.concatenate(residuals)


def _jacobian_sparsity(node_count, factors, priors):
    from scipy.sparse import lil_matrix

    row_count = 3 * (len(factors) + len(priors))
    sparsity = lil_matrix((row_count, 3 * node_count), dtype=int)
    row = 0
    for factor in factors:
        sparsity[row:row + 3, 3 * factor.source:3 * factor.source + 3] = 1
        sparsity[row:row + 3, 3 * factor.target:3 * factor.target + 3] = 1
        row += 3
    for prior in priors:
        sparsity[row:row + 3, 3 * prior.node:3 * prior.node + 3] = 1
        row += 3
    return sparsity.tocsr()


def optimize_pose_graph(
    initial_poses: np.ndarray,
    factors: Iterable[PoseFactor],
    priors: Iterable[PosePrior],
    *,
    loss: str = "huber",
    robust_scale: float = 1.5,
    max_function_evaluations: int = 300,
) -> OptimizationResult:
    """Optimize an SE(2) graph while keeping the public graph format backend-neutral."""
    from scipy.optimize import least_squares

    initial_poses = np.asarray(initial_poses, dtype=float)
    factors = tuple(factors)
    priors = tuple(priors)
    if initial_poses.ndim != 2 or initial_poses.shape[1] != 3:
        raise ValueError("initial_poses must have shape (N, 3)")
    if not len(priors):
        raise ValueError("pose graph needs at least one prior to fix its gauge")
    result = least_squares(
        graph_residuals,
        initial_poses.reshape(-1),
        args=(len(initial_poses), factors, priors),
        jac_sparsity=_jacobian_sparsity(len(initial_poses), factors, priors),
        loss=loss,
        f_scale=float(robust_scale),
        max_nfev=int(max_function_evaluations),
        x_scale="jac",
    )
    poses = result.x.reshape(-1, 3)
    poses[:, 2] = wrap_angle(poses[:, 2])
    return OptimizationResult(
        poses=poses,
        success=bool(result.success),
        cost=float(result.cost),
        optimality=float(result.optimality),
        evaluations=int(result.nfev),
        message=str(result.message),
    )
