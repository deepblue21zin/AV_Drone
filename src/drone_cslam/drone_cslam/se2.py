"""Small, explicit SE(2) helpers used by the offline experiment."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def wrap_angle(angle):
    """Wrap an angle or numpy array to [-pi, pi)."""
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi


def compose(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Return ``first * second`` for poses represented by [x, y, yaw]."""
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    cosine = math.cos(float(first[2]))
    sine = math.sin(float(first[2]))
    return np.array(
        [
            first[0] + cosine * second[0] - sine * second[1],
            first[1] + sine * second[0] + cosine * second[1],
            float(wrap_angle(first[2] + second[2])),
        ],
        dtype=float,
    )


def inverse(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float)
    cosine = math.cos(float(pose[2]))
    sine = math.sin(float(pose[2]))
    return np.array(
        [
            -cosine * pose[0] - sine * pose[1],
            sine * pose[0] - cosine * pose[1],
            float(wrap_angle(-pose[2])),
        ],
        dtype=float,
    )


def between(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Return the pose of ``second`` expressed in ``first``."""
    return compose(inverse(first), second)


def transform_points(pose: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Transform an N x 2 point array by an SE(2) pose."""
    points = np.asarray(points, dtype=float)
    if not len(points):
        return np.empty((0, 2), dtype=float)
    cosine = math.cos(float(pose[2]))
    sine = math.sin(float(pose[2]))
    rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=float)
    return points @ rotation.T + np.asarray(pose[:2], dtype=float)


def interpolate_pose(
    first: np.ndarray, second: np.ndarray, fraction: float
) -> np.ndarray:
    fraction = float(np.clip(fraction, 0.0, 1.0))
    delta_yaw = float(wrap_angle(second[2] - first[2]))
    return np.array(
        [
            first[0] + fraction * (second[0] - first[0]),
            first[1] + fraction * (second[1] - first[1]),
            float(wrap_angle(first[2] + fraction * delta_yaw)),
        ],
        dtype=float,
    )


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)
