"""Place a robot in the room by matching one scan against the blockout.

Runs map/branch_and_bound.py over the whole room and every heading. The
result is the robot's pose in the server frame; a TurtleBot turns that into
the origin of its odometry frame and starts tracking it with Monte Carlo
localization (see TurtleBot.observe_scan).
"""

from __future__ import annotations

import math
import os
import time

import numpy as np

from map.blockout import WallGrid
from map.branch_and_bound import BranchAndBound
from map.occupancy import OCCUPANCY_SAVE_PATH
from map.room import point_in_polygon, server_room_corners

# Lowest mean likelihood accepted as a match. Likelihood is
# (wall + w * occupancy) / (1 + w), so with w = 1 this is the old walls-only
# 0.5 wherever occupancy is unknown (0.5).
MIN_SCORE = 0.5

# A newer saved occupancy grid is picked up at most this often (seconds).
# The server saves every second while mapping; rebuilding takes ~0.2 s.
GRID_REFRESH_INTERVAL = 10.0

# (occupancy file time, occupancy weight), build time, grid, branch and bound or None
_cache: tuple[tuple[float | None, float], float, WallGrid, BranchAndBound | None] | None = None


def _occupancy_stamp() -> float | None:
    """Modification time of the saved occupancy grid; None when there is none."""
    try:
        return os.path.getmtime(OCCUPANCY_SAVE_PATH)
    except OSError:
        return None


def wall_grid() -> WallGrid:
    """The blockout plus saved occupancy grid that scans are scored against."""
    global _cache
    key = (_occupancy_stamp(), WallGrid.OCCUPANCY_WEIGHT)
    if _cache is not None:
        cached_key, built, grid, _ = _cache
        weight_changed = cached_key[1] != key[1]
        stale = cached_key != key and time.monotonic() - built >= GRID_REFRESH_INTERVAL
        if not (weight_changed or stale):
            return grid
    grid = WallGrid.from_blockout()
    _cache = (key, time.monotonic(), grid, None)
    return grid


def _matcher() -> BranchAndBound:
    """Branch and bound over the current wall grid, built once per grid."""
    global _cache
    grid = wall_grid()
    key, built, _, matcher = _cache
    if matcher is None:
        matcher = BranchAndBound(grid)
        _cache = (key, built, grid, matcher)
    return matcher


def _window() -> tuple[tuple[float, float, float], float]:
    """Search center and half-width covering the whole room."""
    corners = np.array(server_room_corners())
    low, high = corners.min(axis=0), corners.max(axis=0)
    return (*((low + high) / 2), 0.0), float((high - low).max() / 2)


def localize_in_room(scan_rel: np.ndarray) -> tuple[float, float, float, float] | None:
    """Find the robot in the room from one scan.

    scan_rel: (N, 2) scan points minus the robot's position, server-frame X/Z
    axes, still in the robot's odometry orientation.

    Returns (x, z, theta, score): the robot's server-frame position and the
    rotation (from X toward Z) that turns the odometry frame onto the room.
    None when no pose scores at least MIN_SCORE inside the room.
    """
    if len(scan_rel) == 0:
        return None
    center, window = _window()
    result = _matcher().search(scan_rel, center, window, math.pi, MIN_SCORE)
    if result is None:
        return None
    x, z, theta, score = result
    if not point_in_polygon(x, z, server_room_corners()):
        return None
    return float(x), float(z), float(theta), float(score)
