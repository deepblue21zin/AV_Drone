import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    @property
    def width(self) -> int:
        return int(math.ceil((self.max_x - self.min_x) / self.resolution))

    @property
    def height(self) -> int:
        return int(math.ceil((self.max_y - self.min_y) / self.resolution))

    @property
    def size(self) -> int:
        return self.width * self.height


@dataclass(frozen=True)
class ProjectedGrid:
    indices: np.ndarray
    log_odds: np.ndarray
    free_indices: np.ndarray
    occupied_indices: np.ndarray


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def probability_to_log_odds(probability: np.ndarray) -> np.ndarray:
    probability = np.clip(probability, 0.05, 0.95)
    return np.log(probability / (1.0 - probability))


def project_occupancy_grid(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin: Tuple[float, float, float],
    transform: Tuple[float, float, float],
    target: GridSpec,
    free_threshold: int = 25,
    occupied_threshold: int = 65,
) -> ProjectedGrid:
    """Project one local OccupancyGrid into flat target-grid contributions."""
    array = np.asarray(data, dtype=np.int16)
    if width <= 0 or height <= 0 or array.size != width * height:
        raise ValueError("occupancy data length does not match width * height")
    if resolution <= 0.0:
        raise ValueError("source resolution must be positive")

    observed = np.flatnonzero(array >= 0)
    if observed.size == 0:
        empty_i = np.empty((0,), dtype=np.int64)
        empty_f = np.empty((0,), dtype=np.float64)
        return ProjectedGrid(empty_i, empty_f, empty_i, empty_i)

    rows = observed // width
    cols = observed % width
    local_x = (cols.astype(np.float64) + 0.5) * resolution
    local_y = (rows.astype(np.float64) + 0.5) * resolution

    origin_x, origin_y, origin_yaw = origin
    cos_o, sin_o = math.cos(origin_yaw), math.sin(origin_yaw)
    frame_x = origin_x + cos_o * local_x - sin_o * local_y
    frame_y = origin_y + sin_o * local_x + cos_o * local_y

    tx, ty, yaw = transform
    cos_t, sin_t = math.cos(yaw), math.sin(yaw)
    world_x = tx + cos_t * frame_x - sin_t * frame_y
    world_y = ty + sin_t * frame_x + cos_t * frame_y
    target_x = np.floor((world_x - target.min_x) / target.resolution).astype(np.int64)
    target_y = np.floor((world_y - target.min_y) / target.resolution).astype(np.int64)
    valid = (
        (target_x >= 0)
        & (target_x < target.width)
        & (target_y >= 0)
        & (target_y < target.height)
    )
    target_indices = target_y[valid] * target.width + target_x[valid]
    values = array[observed[valid]]
    probabilities = np.clip(values.astype(np.float64) / 100.0, 0.05, 0.95)
    log_odds = probability_to_log_odds(probabilities)

    if target_indices.size:
        sums = np.bincount(target_indices, weights=log_odds, minlength=target.size)
        counts = np.bincount(target_indices, minlength=target.size)
        unique = np.flatnonzero(counts)
        log_odds = sums[unique] / counts[unique]
        value_sums = np.bincount(target_indices, weights=values, minlength=target.size)
        averaged_values = value_sums[unique] / counts[unique]
        target_indices = unique.astype(np.int64)
    else:
        averaged_values = np.empty((0,), dtype=np.float64)

    free_indices = target_indices[averaged_values <= free_threshold]
    occupied_indices = target_indices[averaged_values >= occupied_threshold]
    return ProjectedGrid(target_indices, log_odds, free_indices, occupied_indices)


def fuse_projected_grids(
    projected: Iterable[ProjectedGrid],
    weights: Sequence[float],
    target: GridSpec,
) -> Tuple[np.ndarray, int, int]:
    """Fuse current source snapshots and return grid, conflict count, observed count."""
    projected = list(projected)
    if len(projected) != len(weights):
        raise ValueError("projected grids and weights must have the same length")

    log_odds_sum = np.zeros(target.size, dtype=np.float64)
    contribution_count = np.zeros(target.size, dtype=np.int16)
    free_votes = np.zeros(target.size, dtype=np.int16)
    occupied_votes = np.zeros(target.size, dtype=np.int16)

    for source, weight in zip(projected, weights):
        if weight <= 0.0:
            continue
        log_odds_sum[source.indices] += source.log_odds * float(weight)
        contribution_count[source.indices] += 1
        free_votes[source.free_indices] += 1
        occupied_votes[source.occupied_indices] += 1

    observed = contribution_count > 0
    result = np.full(target.size, -1, dtype=np.int8)
    probabilities = 1.0 / (1.0 + np.exp(-np.clip(log_odds_sum[observed], -8.0, 8.0)))
    result[observed] = np.clip(np.rint(probabilities * 100.0), 0, 100).astype(np.int8)
    conflicts = (free_votes > 0) & (occupied_votes > 0)
    return result.reshape((target.height, target.width)), int(np.count_nonzero(conflicts)), int(np.count_nonzero(observed))
