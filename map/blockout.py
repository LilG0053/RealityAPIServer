"""Closeness-to-wall grid built from the measured room blockout.

The blockout (dashboard/blockout.json) is the fixed map the robot localizes
against: the room outline plus measured obstacles such as the pillars. It is
stored in dashboard coordinates; this module shifts it into the server frame
by removing DISPLAY_OFFSET, the same way map/room.py does.

WallGrid holds, for every cell, the exact distance in meters from the cell
center to the nearest blockout wall, and a likelihood

    likelihood = exp(-distance^2 / (2 * sigma^2))

which is 1.0 on a wall and falls toward 0 away from walls. A scan placed at
the right pose has its points on walls, so its mean likelihood is high.

Indexing matches map/occupancy.py: grid[i, j] with i along server x and j
along server z. Cell (i, j) covers
[origin_x + i*res, origin_x + (i+1)*res) x [origin_z + j*res, origin_z + (j+1)*res).
"""

from __future__ import annotations

from dataclasses import dataclass
import json

import numpy as np

from map.room import DISPLAY_OFFSET

BLOCKOUT_PATH = "dashboard/blockout.json"

# 5 cm cells. Finer than the 10 cm occupancy grid: localization wants precision.
WALL_GRID_RESOLUTION = 0.05
# How soft the walls are. Larger converges from further away but is less precise.
WALL_SIGMA = 0.10
# Extra space around the room so points just outside it still get a distance.
WALL_GRID_MARGIN = 1.0


def load_blockout_polygons(
    path: str = BLOCKOUT_PATH,
    offset: tuple[float, float, float] = DISPLAY_OFFSET,
) -> list[np.ndarray]:
    """Room outline first, then each obstacle, as K x 2 (x, z) arrays in the server frame."""
    with open(path) as file:
        blockout = json.load(file)
    shift = np.array([offset[0], offset[2]])
    polygons = [np.asarray(blockout["room"], dtype=float) - shift]
    for obstacle in blockout.get("obstacles", []):
        polygons.append(np.asarray(obstacle["polygon"], dtype=float) - shift)
    return polygons


def polygon_segments(polygons: list[np.ndarray]) -> np.ndarray:
    """Every closed polygon edge as one row (x0, z0, x1, z1)."""
    rows = [np.hstack((polygon, np.roll(polygon, -1, axis=0))) for polygon in polygons]
    return np.vstack(rows)


def distance_to_segments(points: np.ndarray, segments: np.ndarray) -> np.ndarray:
    """Exact distance from each point (N x 2) to its nearest segment (M x 4)."""
    start = segments[:, :2]
    direction = segments[:, 2:] - start
    length_sq = np.maximum((direction ** 2).sum(axis=1), 1e-12)
    best = np.full(len(points), np.inf)
    # One segment at a time keeps memory at N floats instead of N x M.
    for a, d, l2 in zip(start, direction, length_sq):
        t = np.clip(((points - a) @ d) / l2, 0.0, 1.0)
        nearest = a + t[:, None] * d
        np.minimum(best, np.hypot(*(points - nearest).T), out=best)
    return best


@dataclass
class WallGrid:
    origin: np.ndarray        # (x, z) of the corner of cell (0, 0)
    resolution: float
    sigma: float
    distance: np.ndarray      # meters to the nearest wall, shape (nx, nz)
    likelihood: np.ndarray    # exp(-distance^2 / 2 sigma^2), same shape

    @classmethod
    def from_blockout(
        cls,
        path: str = BLOCKOUT_PATH,
        resolution: float = WALL_GRID_RESOLUTION,
        sigma: float = WALL_SIGMA,
        margin: float = WALL_GRID_MARGIN,
    ) -> "WallGrid":
        segments = polygon_segments(load_blockout_polygons(path))
        coords = segments.reshape(-1, 2)
        low = coords.min(axis=0) - margin
        high = coords.max(axis=0) + margin
        shape = np.ceil((high - low) / resolution).astype(int)

        i, j = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), indexing="ij")
        centers = low + (np.stack((i, j), axis=-1).reshape(-1, 2) + 0.5) * resolution
        distance = distance_to_segments(centers, segments).reshape(tuple(shape))
        likelihood = np.exp(-distance ** 2 / (2.0 * sigma ** 2))
        return cls(low, resolution, sigma, distance, likelihood)

    def world_to_index(self, points: np.ndarray) -> np.ndarray:
        """Integer (i, j) cell of each server-frame (x, z) point."""
        return np.floor((np.asarray(points, dtype=float) - self.origin) / self.resolution).astype(int)

    def inside(self, index: np.ndarray) -> np.ndarray:
        return (index >= 0).all(axis=1) & (index < self.likelihood.shape).all(axis=1)

    def lookup(self, grid: np.ndarray, points: np.ndarray, outside: float) -> np.ndarray:
        index = self.world_to_index(points).reshape(-1, 2)
        values = np.full(len(index), outside, dtype=float)
        ok = self.inside(index)
        values[ok] = grid[index[ok, 0], index[ok, 1]]
        return values

    def likelihood_at(self, points: np.ndarray) -> np.ndarray:
        """Per-point likelihood; points off the grid count as 0."""
        return self.lookup(self.likelihood, points, 0.0)

    def distance_at(self, points: np.ndarray) -> np.ndarray:
        """Per-point distance to the nearest wall; points off the grid are inf."""
        return self.lookup(self.distance, points, np.inf)

    def score(self, points: np.ndarray) -> float:
        """Mean likelihood of server-frame (x, z) points. 1.0 = every point on a wall."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(points) == 0:
            return 0.0
        return float(self.likelihood_at(points).mean())
