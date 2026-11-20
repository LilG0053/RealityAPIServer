# interface for WallGrid

from abc import ABC, abstractmethod

import numpy as np


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


# interface for wallgrid so that bnb doesnt actually reference the project
class WallGridInterface(ABC):
    @property
    @abstractmethod
    def resolution(self) -> float:
        pass

    @property
    @abstractmethod
    def origin(self) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def likelihood(self) -> np.ndarray:
        pass

    @abstractmethod
    def refresh(self) -> bool:
        pass

    @abstractmethod
    def world_to_index(self, points: np.ndarray) -> np.ndarray:
        pass

    @abstractmethod
    def inside(self, index: np.ndarray) -> np.ndarray:
        pass

    @abstractmethod
    def lookup(self, grid: np.ndarray, points: np.ndarray, outside: float) -> np.ndarray:
        pass

    @abstractmethod
    def likelihood_at(self, points: np.ndarray) -> np.ndarray:
        pass

    @abstractmethod
    def distance_at(self, points: np.ndarray) -> np.ndarray:
        pass

    @abstractmethod
    def score(self, points: np.ndarray) -> float:
        pass
