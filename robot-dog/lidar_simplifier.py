
from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

Point3 = Tuple[float, float, float]


def _distance_to_segment(point: Point3, start: Point3, end: Point3) -> float:
    direction = (
        end[0] - start[0],
        end[1] - start[1],
        end[2] - start[2],
    )
    length_squared = sum(value * value for value in direction)
    if length_squared == 0:
        return math.dist(point, start)

    projection = sum(
        (point[index] - start[index]) * direction[index] for index in range(3)
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest = tuple(
        start[index] + projection * direction[index] for index in range(3)
    )
    return math.dist(point, closest)


def simplify_points(points: Sequence[Point3], epsilon: float) -> list[Point3]:
    """Simplify an ordered 3D point sequence with Ramer-Douglas-Peucker.

    Endpoints are retained. ``epsilon`` is the maximum perpendicular distance
    from a point to the segment between retained points, in meters.
    """
    if not math.isfinite(epsilon) or epsilon < 0:
        raise ValueError("epsilon must be a finite non-negative number")
    if len(points) < 3 or epsilon == 0:
        return list(points)

    retained = {0, len(points) - 1}
    pending = [(0, len(points) - 1)]
    while pending:
        start_index, end_index = pending.pop()
        if end_index - start_index < 2:
            continue

        maximum_distance = epsilon
        split_index = None
        start = points[start_index]
        end = points[end_index]
        for index in range(start_index + 1, end_index):
            distance = _distance_to_segment(points[index], start, end)
            if distance > maximum_distance:
                maximum_distance = distance
                split_index = index

        if split_index is not None:
            retained.add(split_index)
            pending.append((start_index, split_index))
            pending.append((split_index, end_index))

    return [point for index, point in enumerate(points) if index in retained]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Keep the first point in each ``voxel_size`` (meters) grid cell.

    ``points`` is an (N, 3) array. Input order is preserved among the kept
    points. A ``voxel_size`` of 0 disables downsampling.
    """
    if not math.isfinite(voxel_size) or voxel_size < 0:
        raise ValueError("voxel_size must be a finite non-negative number")
    if voxel_size == 0 or len(points) < 2:
        return points

    cells = np.floor(points / voxel_size).astype(np.int64)
    cells -= cells.min(axis=0)
    dims = cells.max(axis=0) + 1
    if int(dims[0]) * int(dims[1]) * int(dims[2]) < 2**62:
        flat = (cells[:, 0] * dims[1] + cells[:, 1]) * dims[2] + cells[:, 2]
        _, first = np.unique(flat, return_index=True)
    else:  # extremely sparse/huge extent; fall back to the slower row-wise unique
        _, first = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(first)]