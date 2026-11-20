"""Closeness-to-wall grid built from the measured room blockout.

The blockout (assets/blockout.json, loaded by map/room.py) is the fixed map
the robot localizes against: the room outline plus measured obstacles such as
the pillars. It is stored in dashboard coordinates; this module shifts it into
the server frame by removing DISPLAY_OFFSET, the same way map/room.py does.

WallGrid holds, for every cell, the exact distance in meters from the cell
center to the nearest blockout wall, and a wall likelihood

    wall_likelihood = exp(-distance^2 / (2 * sigma^2))

which is 1.0 on a wall and falls toward 0 away from walls.

It also loads the saved occupancy grid (assets/occupancy_grid_turtlebot.json)
and looks up, for every cell, the probability that the cell is filled (0.5
where the occupancy grid has no observation). The weight branch and bound
and Monte Carlo score against is the two added and normalized back to [0, 1]:

    likelihood = (wall_likelihood + w * occupancy) / (1 + w)

where w is the tunable WallGrid.OCCUPANCY_WEIGHT (0 = walls only, 1 = equal
say). 1.0 is a cell on a blockout wall that the robot has also seen filled. A scan
placed at the right pose has its points on walls and on things it has seen
before, so its mean likelihood is high.

Indexing matches map/occupancy.py: grid[i, j] with i along server x and j
along server z. Cell (i, j) covers
[origin_x + i*res, origin_x + (i+1)*res) x [origin_z + j*res, origin_z + (j+1)*res).
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from map.occupancy import INITIAL_PROBABILITY, OCCUPANCY_SAVE_PATH
from map.room import BLOCKOUT_PATH, DISPLAY_OFFSET, load_blockout
from map.tunable import TunableParameter

# 5 cm cells. Finer than the 10 cm occupancy grid: localization wants precision.
WALL_GRID_RESOLUTION = 0.05
# How soft the walls are. Larger converges from further away but is less precise.
WALL_SIGMA = 0.10
# Extra space around the room so points just outside it still get a distance.
WALL_GRID_MARGIN = 1.0


def load_blockout_polygons(
    path: str | Path = BLOCKOUT_PATH,
    offset: tuple[float, float, float] = DISPLAY_OFFSET,
) -> list[np.ndarray]:
    """Room outline first, then each obstacle, as K x 2 (x, z) arrays in the server frame."""
    blockout = load_blockout(path)
    shift = np.array([offset[0], offset[2]])
    polygons = [np.asarray(blockout.room, dtype=float) - shift]
    for obstacle in blockout.obstacles:
        polygons.append(np.asarray(obstacle.polygon, dtype=float) - shift)
    return polygons


def load_occupancy(path: str | Path = OCCUPANCY_SAVE_PATH) -> tuple[float, dict[tuple[int, int], float]]:
    """Cell size and {(i, j): probability filled} from a saved occupancy grid.

    Cell (i, j) covers [i*size, (i+1)*size) x [j*size, (j+1)*size) in the server
    frame. A missing or unreadable file gives no cells (everything unknown).
    """
    try:
        with open(path) as file:
            payload = json.load(file)
        cells = {(int(cell["x"]), int(cell["z"])): float(cell["p"]) for cell in payload.get("cells", [])}
        return float(payload["cell_size"]), cells
    except FileNotFoundError:
        return 0.0, {}
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"[!] could not load occupancy grid from {path}: {error}")
        return 0.0, {}


def occupancy_at(
    points: np.ndarray,
    cell_size: float,
    cells: dict[tuple[int, int], float],
) -> np.ndarray:
    """Probability each server-frame (x, z) point is filled; INITIAL_PROBABILITY if unknown."""
    values = np.full(len(points), INITIAL_PROBABILITY)
    if not cells or cell_size <= 0.0:
        return values
    keys = np.array(list(cells.keys()))
    low = keys.min(axis=0)
    dense = np.full(keys.max(axis=0) - low + 1, INITIAL_PROBABILITY)
    dense[keys[:, 0] - low[0], keys[:, 1] - low[1]] = list(cells.values())

    index = np.floor(points / cell_size).astype(int) - low
    ok = (index >= 0).all(axis=1) & (index < dense.shape).all(axis=1)
    values[ok] = dense[index[ok, 0], index[ok, 1]]
    return values


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
    # How much the saved occupancy grid counts next to the blockout walls.
    # The occupancy grid is built from localized poses, so a high weight lets a
    # past localization error reinforce itself.
    OCCUPANCY_WEIGHT = TunableParameter(
        1.0, minimum=0.0, maximum=1.0, step=0.05, label="Localization occupancy weight",
    )

    origin: np.ndarray        # (x, z) of the corner of cell (0, 0)
    resolution: float
    sigma: float
    distance: np.ndarray        # meters to the nearest wall, shape (nx, nz)
    wall_likelihood: np.ndarray  # exp(-distance^2 / 2 sigma^2), same shape
    occupancy: np.ndarray       # probability the cell is filled, same shape
    likelihood: np.ndarray      # (wall_likelihood + w * occupancy) / (1 + w), what scans score against

    @classmethod
    def from_blockout(
        cls,
        path: str | Path = BLOCKOUT_PATH,
        resolution: float = WALL_GRID_RESOLUTION,
        sigma: float = WALL_SIGMA,
        margin: float = WALL_GRID_MARGIN,
        occupancy_path: str | Path | None = OCCUPANCY_SAVE_PATH,
    ) -> "WallGrid":
        """occupancy_path=None scores against the blockout walls alone."""
        segments = polygon_segments(load_blockout_polygons(path))
        coords = segments.reshape(-1, 2)
        low = coords.min(axis=0) - margin
        high = coords.max(axis=0) + margin
        shape = np.ceil((high - low) / resolution).astype(int)

        i, j = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), indexing="ij")
        centers = low + (np.stack((i, j), axis=-1).reshape(-1, 2) + 0.5) * resolution
        distance = distance_to_segments(centers, segments).reshape(tuple(shape))
        wall_likelihood = np.exp(-distance ** 2 / (2.0 * sigma ** 2))

        if occupancy_path is None:
            occupancy = np.full(tuple(shape), INITIAL_PROBABILITY)
            likelihood = wall_likelihood
        else:
            occupancy = occupancy_at(centers, *load_occupancy(occupancy_path)).reshape(tuple(shape))
            # Both terms are in [0, 1], so dividing by 1 + w keeps the weight in [0, 1].
            weight = cls.OCCUPANCY_WEIGHT
            likelihood = (wall_likelihood + weight * occupancy) / (1.0 + weight)
        return cls(low, resolution, sigma, distance, wall_likelihood, occupancy, likelihood)

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
        """Mean likelihood of server-frame (x, z) points. 1.0 = every point on a filled wall."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(points) == 0:
            return 0.0
        return float(self.likelihood_at(points).mean())
