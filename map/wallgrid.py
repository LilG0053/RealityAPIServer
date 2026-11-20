"""Closeness-to-wall grid built from a room (map/room.py).

The room is the fixed map a robot localizes against: its outline plus measured
obstacles such as the pillars, in the server frame.

WallGrid holds, for every cell, the exact distance in meters from the cell
center to the nearest wall of the room, and a wall likelihood

    wall_likelihood = exp(-distance^2 / (2 * sigma^2))

which is 1.0 on a wall and falls toward 0 away from walls.

A WallGrid may also be given a live OccupancyGrid. It then looks up, for every
cell, the probability that the cell is filled (0.5 where the occupancy grid has
no observation), and the likelihood branch and bound and Monte Carlo score
against is the two added and normalized back to [0, 1]:

    likelihood = (wall_likelihood + w * occupancy) / (1 + w)

where w is the tunable WallGrid.OCCUPANCY_WEIGHT (0 = walls only, 1 = equal
say). 1.0 is a cell on a room wall that the robot has also seen filled. A
scan placed at the right pose has its points on walls and on things it has
seen before, so its mean likelihood is high. Without an occupancy grid the
likelihood is the wall likelihood alone.

Indexing matches map/occupancy.py: grid[i, j] with i along server x and j
along server z. Cell (i, j) covers
[origin_x + i*res, origin_x + (i+1)*res) x [origin_z + j*res, origin_z + (j+1)*res).
"""

from __future__ import annotations

import time

import numpy as np

from map.algorithms.wallgridinterface import (
    WallGridInterface,
    distance_to_segments,
    polygon_segments,
)
from map.occupancy import OccupancyGrid
from map.room import Room, polygon_array
from map.tunable import TunableParameter

# 5 cm cells. Finer than the 10 cm occupancy grid: localization wants precision.
WALL_GRID_RESOLUTION = 0.05
# How soft the walls are. Larger converges from further away but is less precise.
WALL_SIGMA = 0.10
# Extra space around the room so points just outside it still get a distance.
WALL_GRID_MARGIN = 1.0

class WallGrid(WallGridInterface):
    # How much the occupancy grid counts next to the blockout walls. The
    # occupancy grid is built from localized poses, so a high weight lets a
    # past localization error reinforce itself.
    OCCUPANCY_WEIGHT = TunableParameter(
        1.0, minimum=0.0, maximum=1.0, step=0.05, label="Localization occupancy weight",
    )
    # A changed occupancy grid is blended in at most this often (seconds).
    # It changes with every scan, and blending takes a few milliseconds.
    REFRESH_INTERVAL = 10.0

    def __init__(
        self,
        room: Room,
        occupancy: OccupancyGrid | None = None,
        resolution: float = WALL_GRID_RESOLUTION,
        sigma: float = WALL_SIGMA,
        margin: float = WALL_GRID_MARGIN,
    ) -> None:
        """room: what the grid measures distances to (outline and obstacles).
        occupancy: a live grid to blend into the likelihood, or None for walls only.
        """
        self.room = room
        self.occupancy = occupancy
        self._resolution = resolution
        self.sigma = sigma

        segments = polygon_segments([polygon_array(polygon) for polygon in room.polygons()])
        coords = segments.reshape(-1, 2)
        low = coords.min(axis=0) - margin
        high = coords.max(axis=0) + margin
        shape = tuple(np.ceil((high - low) / resolution).astype(int))
        self._origin = low                 # (x, z) of the corner of cell (0, 0)

        i, j = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), indexing="ij")
        self._centers = low + (np.stack((i, j), axis=-1).reshape(-1, 2) + 0.5) * resolution
        # Meters to the nearest wall, and 1.0 on a wall falling toward 0 away from walls.
        self.distance = distance_to_segments(self._centers, segments).reshape(shape)
        self.wall_likelihood = np.exp(-self.distance ** 2 / (2.0 * sigma ** 2))

        # (occupancy revision, weight) the likelihood was blended from, and when.
        self._blend_key: tuple[int, float] | None = None
        self._blended_at = -float("inf")
        self._likelihood = self.wall_likelihood
        # Bumped whenever the likelihood changes, so callers can rebuild caches.
        self.version = 0

    @property
    def resolution(self) -> float:
        return self._resolution
    @property
    def origin(self) -> np.ndarray:
        return self._origin
    @property
    def likelihood(self) -> np.ndarray:
        """(wall_likelihood + w * occupancy) / (1 + w), what scans score against.

        The wall likelihood alone without an occupancy grid. It is as of the
        last refresh(); call that first to pick up occupancy or weight changes.
        """
        return self._likelihood

    def refresh(self) -> bool:
        """Blend in the occupancy grid if it or the weight changed. Returns whether it did.

        A changed occupancy grid is picked up at most every REFRESH_INTERVAL; a
        changed weight at once. Each blend bumps version.
        """
        if self.occupancy is None:
            return False
        weight = self.OCCUPANCY_WEIGHT
        key = (self.occupancy.revision, weight)
        if key == self._blend_key:
            return False
        now = time.monotonic()
        weight_changed = self._blend_key is None or self._blend_key[1] != weight
        if not weight_changed and now - self._blended_at < self.REFRESH_INTERVAL:
            return False
        filled = self.occupancy.probabilities_at(self._centers).reshape(self.distance.shape)
        # Both terms are in [0, 1], so dividing by 1 + w keeps the weight in [0, 1].
        self._likelihood = (self.wall_likelihood + weight * filled) / (1.0 + weight)
        self._blend_key = key
        self._blended_at = now
        self.version += 1
        return True

    def world_to_index(self, points: np.ndarray) -> np.ndarray:
        """Integer (i, j) cell of each server-frame (x, z) point."""
        return np.floor((np.asarray(points, dtype=float) - self.origin) / self.resolution).astype(int)

    def inside(self, index: np.ndarray) -> np.ndarray:
        return (index >= 0).all(axis=1) & (index < self.distance.shape).all(axis=1)

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
        """Mean likelihood of server-frame (x, z) points. 1.0 = every point on a filled wall."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(points) == 0:
            return 0.0
        return float(self.likelihood_at(points).mean())
