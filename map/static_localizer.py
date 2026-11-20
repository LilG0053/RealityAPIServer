"""Absolute 2D localization of a robot against the measured room blockout.

The accumulated-scan localizer (map/localization.py) only matches a scan
against earlier scans, so it can drift and it needs a hand-measured start pose.
This module matches the scan against the fixed room instead (walls and pillars
from assets/blockout.json, through map.blockout.WallGrid) with the
branch-and-bound search in map.branch_and_bound.

It works in 2D on purpose. A horizontal slice of a vertical wall looks the same
at any height, so the dog's ring and the TurtleBot's scan can both be matched
against the same blockout. The 3D map is not involved: it only receives the
correction this module produces.

What it returns is a *delta*: a 3x3 transform that moves the robot's currently
corrected scan onto the blockout. The caller multiplies it into
ScanLocalizer.correction, exactly like the transform icp() produces.

Two modes:
  global   Not located yet (or lost). Search every heading within
           GLOBAL_XY_WINDOW of where the robot is believed to be (the start guess
           at first, the last good pose after being lost). Rate limited.
  tracking Located. Search a small window around the current corrected pose.
           Fast enough to run on every scan.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from time import monotonic

import numpy as np

from map.blockout import WallGrid
from map.branch_and_bound import BranchAndBound

# Points farther than this are ignored. BranchAndBound pads the wall grid by
# 300 cells (15 m at 5 cm), so a scan reaching farther than ~10 m plus the
# search window could index past the padding.
MATCH_RANGE_MAX = 10.0
MATCH_RANGE_MIN = 0.3
# A scan with fewer usable points than this is not matched.
MATCH_MIN_POINTS = 20

# Mean wall likelihood (0..1) a pose must reach to be accepted. Clutter that is
# not in the blockout (people, tables) lowers the score, so these are well
# below 1. Global is stricter: it accepts a pose from nothing.
GLOBAL_MIN_SCORE = 0.45
TRACK_MIN_SCORE = 0.30

# Tracking window around the current corrected pose.
TRACK_XY_WINDOW = 0.40            # meters, each way
TRACK_THETA_WINDOW = math.radians(8.0)
# Run tracking at most this often (seconds). 0 = on every scan.
TRACK_INTERVAL = 0.2
# This many tracking failures in a row and the robot counts as lost.
LOST_AFTER_MISSES = 15

# Seconds between global searches while not located.
GLOBAL_RETRY = 3.0
# How far from its believed position a global search looks (meters, each way),
# with every heading allowed. The room is close to point-symmetric and a scan
# partly hidden by people or furniture can fit the far side of the room better
# than the true spot, so the search is kept near where the robot is believed to
# be. Set to None to search the whole room (only safe with an unobstructed view).
GLOBAL_XY_WINDOW: float | None = 3.0
# How far from its believed heading a global search turns (radians, each way).
# pi allows any heading. If DOG_START_YAW in server.py is accurate to a few
# degrees, a smaller window (for example math.radians(45)) rules out the
# mirrored poses a symmetric room allows.
GLOBAL_THETA_WINDOW = math.pi


def _translation(x: float, z: float) -> np.ndarray:
    matrix = np.eye(3)
    matrix[0, 2], matrix[1, 2] = x, z
    return matrix


def _pose_matrix(x: float, z: float, theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, x], [s, c, z], [0.0, 0.0, 1.0]])


@dataclass
class StaticFix:
    delta: np.ndarray            # 3x3: current corrected frame -> blockout frame
    score: float                 # mean wall likelihood of the matched scan
    shift: float                 # meters the robot moved to reach the fix
    turn: float                  # radians the fix rotated the scan
    global_search: bool          # True for a whole-room search


class StaticLocalizer:
    def __init__(self, wall_grid: WallGrid | None = None) -> None:
        self.wall_grid = wall_grid or WallGrid.from_blockout()
        self.matcher = BranchAndBound(self.wall_grid)
        self.located = False
        self.last_score: float | None = None
        self._misses = 0
        self._last_global = -math.inf
        self._last_track = -math.inf

        grid = self.wall_grid
        extent = np.array(grid.likelihood.shape) * grid.resolution
        self._room_center = grid.origin + extent / 2.0
        self._room_radius = float(extent.max()) / 2.0
        self._searches = 0

    def reset(self) -> None:
        """Forget the fix. The next scan does a whole-room search."""
        self.located = False
        self.last_score = None
        self._misses = 0
        self._last_global = -math.inf
        self._last_track = -math.inf

    def locate(self, ring: np.ndarray, position: np.ndarray, now: float | None = None) -> StaticFix | None:
        """Match one scan against the blockout.

        ring      N x 2 scan points (server x, z), already corrected.
        position  (x, z) of the robot in the same corrected frame.
        Returns the fix, or None when no search ran or none was good enough.
        """
        now = monotonic() if now is None else now
        global_search = not self.located
        if global_search:
            if now - self._last_global < GLOBAL_RETRY:
                return None
            self._last_global = now
        else:
            if now - self._last_track < TRACK_INTERVAL:
                return None
            self._last_track = now

        # The matcher takes points relative to the robot, in the robot's frame.
        # Using the corrected world axes as that frame (heading 0) is enough: the
        # search then reports how far to rotate about the robot and where it ends
        # up, which is exactly the correction we want.
        position = np.asarray(position, dtype=float).reshape(2)
        rel = np.asarray(ring, dtype=float).reshape(-1, 2) - position
        distance = np.hypot(rel[:, 0], rel[:, 1])
        rel = rel[(distance >= MATCH_RANGE_MIN) & (distance <= MATCH_RANGE_MAX)]
        if len(rel) < MATCH_MIN_POINTS:
            return self._miss(global_search)

        if global_search:
            if GLOBAL_XY_WINDOW is None:
                center, window = (self._room_center[0], self._room_center[1], 0.0), self._room_radius
            else:
                center, window = (position[0], position[1], 0.0), GLOBAL_XY_WINDOW
            # Headings within GLOBAL_THETA_WINDOW. The scan is relative to the robot, so
            # the center is where the robot is searched for, not where it was.
            found = self.matcher.search(rel, center, window, GLOBAL_THETA_WINDOW, GLOBAL_MIN_SCORE)
        else:
            center = (position[0], position[1], 0.0)
            found = self.matcher.search(
                rel, center, TRACK_XY_WINDOW, TRACK_THETA_WINDOW, TRACK_MIN_SCORE
            )
        if found is None:
            if global_search:
                print(f"[static] no fix within {GLOBAL_XY_WINDOW} m of the start guess "
                      f"(needs a mean wall score of {GLOBAL_MIN_SCORE}); will retry")
            return self._miss(global_search)

        x, z, theta, score = found
        # World point w = position + rel moves to R(theta) @ rel + (x, z).
        delta = _pose_matrix(x, z, theta) @ _translation(-position[0], -position[1])
        self.located = True
        self._misses = 0
        self.last_score = float(score)
        return StaticFix(
            delta=delta,
            score=float(score),
            shift=float(math.hypot(x - position[0], z - position[1])),
            turn=float(theta),
            global_search=global_search,
        )

    def _miss(self, global_search: bool) -> None:
        if not global_search:
            self._misses += 1
            if self._misses >= LOST_AFTER_MISSES:
                print(f"[static] lost after {self._misses} failed fixes; searching the whole room")
                self.located = False
                self._last_global = -math.inf
        return None