"""Absolute 2D localization of a robot against the measured room blockout.

The accumulated-scan localizer (map/localization.py) only matches a scan
against earlier scans, so it can drift and it needs a hand-measured start pose.
This module matches the scan against the fixed room instead (walls and pillars
from assets/blockout.json, through map.blockout.WallGrid).

It works in 2D on purpose. A horizontal slice of a vertical wall looks the same
at any height, so the dog's ring and the TurtleBot's scan can both be matched
against the same blockout. The 3D map is not involved: it only receives the
correction this module produces.

What it returns is a *delta*: a 3x3 transform that moves the robot's currently
corrected scan onto the blockout. The caller multiplies it into
ScanLocalizer.correction, exactly like the transform icp() produces.

Two modes:
  global   Not located yet (or lost). BranchAndBound searches the whole room and
           every heading, one window per call so the server is never blocked for
           long, all against one saved scan. The best pose is accepted only if no
           far-away pose scores nearly as well (a long room looks the same from
           both ends) and the score drops when the pose is nudged (a corridor
           does not pin the position). Otherwise it waits and tries again with a
           new scan from where the robot has walked to.
  tracking Located. Monte Carlo localization (map/montecarlo.py) follows the pose.
           It is seeded with the global fix, moved by the robot's odometry and
           weighted by every scan.

Frames. Tracking keeps one transform that does not depend on the corrections the
caller applies in between: raw frame -> room. The raw frame is the one the ring
arrived in before ScanLocalizer.correction (ICP and earlier fixes) was applied, so
locate() takes that correction and undoes it. Each fix is then reported as
(estimate) @ inverse(current correction), which is what the caller multiplies in.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from time import monotonic

import numpy as np

from map.blockout import WallGrid, load_blockout_polygons
from map.branch_and_bound import BranchAndBound
from map.montecarlo import MonteCarlo, wrap_angle
from map.room import DISPLAY_OFFSET

FRAME_SHIFT = np.array([DISPLAY_OFFSET[0], DISPLAY_OFFSET[2]])

# Points farther than this are ignored. BranchAndBound pads the wall grid by
# 300 cells (15 m at 5 cm), so a scan reaching 10 m plus a search window of up to
# about 5 m stays inside the padding. Keep TILE_RADIUS at or below 4.
MATCH_RANGE_MAX = 10.0
MATCH_RANGE_MIN = 0.3
# A scan with fewer usable points than this is not matched.
MATCH_MIN_POINTS = 20

# Mean wall likelihood (0..1) a pose must reach to be accepted. Clutter that is
# not in the blockout (people, tables) lowers the score, so these are well
# below 1. Global is stricter: it accepts a pose from nothing.
GLOBAL_MIN_SCORE = 0.50
TRACK_MIN_SCORE = 0.30

# ---------------------------------------------------------------------------
# Global localization (BranchAndBound over the whole room)
# ---------------------------------------------------------------------------

# Optional prior: search only this many meters each way around where the robot is
# believed to be (one window, at most TILE_RADIUS). None = the whole room.
GLOBAL_XY_WINDOW: float | None = None
# The room is covered by square windows about TILE_STEP apart, each searched at
# every heading. A window of 3 m took 0.39 s in testing; one is searched per call.
TILE_RADIUS = 3.0
TILE_STEP = 4.0
# Seconds between two windows of the same search.
TILE_INTERVAL = 0.3
# Seconds between finishing a search without a fix and starting the next one.
GLOBAL_RETRY = 3.0
# Windows are searched down to GLOBAL_MIN_SCORE minus this, so runners-up show up.
CANDIDATE_MARGIN = 0.10
# Another pose at least this far (meters) or this far turned from the best one,
# scoring within AMBIG_MARGIN of it, means the answer is ambiguous: wait.
AMBIG_MARGIN = 0.05
AMBIG_DIST = 1.5
AMBIG_TURN = math.radians(30.0)
# The best pose must beat the same scan moved 0.3 m or turned 6 degrees by this much.
FIRM_MARGIN = 0.08
FIRM_SHIFT = 0.3
FIRM_TURN = math.radians(6.0)

# ---------------------------------------------------------------------------
# Tracking (Monte Carlo localization)
# ---------------------------------------------------------------------------

# Run tracking at most this often (seconds). 0 = on every scan.
TRACK_INTERVAL = 0.2
# This many tracking failures in a row and the robot counts as lost.
LOST_AFTER_MISSES = 15
TRACK_PARTICLES = 1000
TRACK_MIN_PARTICLES = 200
# Spread of the cloud around the global fix: meters, radians.
TRACK_INIT_STD = (0.10, math.radians(3.0))
# Odometry noise for MonteCarlo.predict. The ring is in fixed world axes, so the
# robot's own turning never enters and the direction of travel is not a rotation:
# the two alphas that multiply rotations are 0. Order: rot-from-rot,
# rot-from-trans, trans-from-trans, trans-from-rot.
TRACK_ALPHAS = (0.0, 0.002, 0.01, 0.0)
TRACK_BEAMS = 60
TRACK_Z_HIT = 0.8
# The summed log-likelihood of a scan is divided by this before it becomes a
# weight. Above 1 it keeps the cloud from collapsing onto one lucky particle.
TRACK_TEMPERATURE = 3.0
# Seed for the particles' random numbers (None = different every run).
TRACK_SEED: int | None = None


def _translation(x: float, z: float) -> np.ndarray:
    matrix = np.eye(3)
    matrix[0, 2], matrix[1, 2] = x, z
    return matrix


def _pose_matrix(x: float, z: float, theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, x], [s, c, z], [0.0, 0.0, 1.0]])


def _inside(polygon: np.ndarray, x: float, z: float) -> bool:
    """Even-odd point-in-polygon test; polygon is K x 2 (x, z)."""
    px, pz = polygon[:, 0], polygon[:, 1]
    qx, qz = np.roll(px, 1), np.roll(pz, 1)
    crosses = (pz > z) != (qz > z)
    with np.errstate(divide="ignore", invalid="ignore"):
        x_at = (qx - px) * (z - pz) / (qz - pz) + px
    return bool(np.count_nonzero(crosses & (x < x_at)) % 2)


class _TemperedMonteCarlo(MonteCarlo):
    """MonteCarlo.update with the scan's log-likelihood divided by `temperature`.

    Identical to MonteCarlo.update otherwise; montecarlo.py is left untouched.
    """

    temperature = 1.0

    def update(self, scan):
        scan = np.asarray(scan, dtype=float).reshape(-1, 2)
        if len(scan) == 0 or len(self.particles) == 0:
            return self.pose

        beams = scan[np.linspace(0, len(scan) - 1, min(self.max_beams, len(scan))).astype(int)]
        log_weights = np.log(
            self.z_hit * self._likelihood(self._place(beams, self.particles)) + self.z_rand
        ).sum(axis=1) / self.temperature
        weights = self.weights * np.exp(log_weights - log_weights.max())
        total = weights.sum()
        self.weights = weights / total if total > 0 else np.full(len(weights), 1.0 / len(weights))

        self.pose = self._best_cluster_pose()
        self.fit = float(self._likelihood(self._place(scan, np.array([self.pose])))[0].mean())
        self._resample()
        return self.pose


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

        # Server-frame room and obstacle outlines, to reject a fix that puts the
        # robot outside the room or inside a pillar or cabinet.
        polygons = load_blockout_polygons()
        self._room_polygon = polygons[0]
        self._obstacle_polygons = polygons[1:]

        self._job: dict | None = None            # the global search in progress
        self._mc: MonteCarlo | None = None       # the tracker
        self._prev_raw: np.ndarray | None = None  # raw position at the last tracker step

    def reset(self) -> None:
        """Forget the fix. The next scan starts a new whole-room search."""
        self.located = False
        self.last_score = None
        self._misses = 0
        self._last_global = -math.inf
        self._last_track = -math.inf
        self._clear()

    def _clear(self) -> None:
        self._job = None
        self._mc = None
        self._prev_raw = None

    def locate(
        self,
        ring: np.ndarray,
        position: np.ndarray,
        *,
        correction: np.ndarray,
        now: float | None = None,
    ) -> StaticFix | None:
        """Match one scan against the blockout.

        ring        N x 2 scan points (drawn x, z), already corrected.
        position    (x, z) of the robot in the same corrected frame.
        correction  the 3x3 ScanLocalizer.correction that produced them.
        Returns the fix, or None when no search ran or none was good enough.
        """
        now = monotonic() if now is None else now
        if self.located:
            if now - self._last_track < TRACK_INTERVAL:
                return None
            self._last_track = now
        elif self._job is None:
            if now - self._last_global < GLOBAL_RETRY:
                return None
        elif now - self._job["t"] < TILE_INTERVAL:
            return None

        # Undo the correction: the filter's state must not move when the caller
        # folds a fix or an ICP step into it. The matcher takes points relative to
        # the robot in fixed world axes (heading 0), so the scan is made relative
        # to the position and turned back to the raw axes.
        correction = np.asarray(correction, dtype=float)
        position = np.asarray(position, dtype=float).reshape(2)
        inverse = np.linalg.inv(correction)
        raw = inverse[:2, :2] @ position + inverse[:2, 2]
        rel = (np.asarray(ring, dtype=float).reshape(-1, 2) - position) @ correction[:2, :2]
        distance = np.hypot(rel[:, 0], rel[:, 1])
        rel = rel[(distance >= MATCH_RANGE_MIN) & (distance <= MATCH_RANGE_MAX)]

        if not self.located:
            if self._job is None and len(rel) < MATCH_MIN_POINTS:
                return None
            return self._global_step(rel, raw, position, correction, now)
        if len(rel) < MATCH_MIN_POINTS:
            return self._miss(False)
        return self._track(rel, raw, position, correction)

    # ------------------------------------------------------------------ global

    def _tiles(self, hint: np.ndarray) -> tuple[list[tuple[float, float]], float]:
        """Window centers (server frame) covering the room, and their half-width."""
        if GLOBAL_XY_WINDOW is not None:
            return [(float(hint[0]), float(hint[1]))], min(GLOBAL_XY_WINDOW, TILE_RADIUS)
        low, high = self._room_polygon.min(axis=0), self._room_polygon.max(axis=0)
        n = np.maximum(np.ceil((high - low) / TILE_STEP).astype(int), 1)
        xs = low[0] + (np.arange(n[0]) + 0.5) * (high[0] - low[0]) / n[0]
        zs = low[1] + (np.arange(n[1]) + 0.5) * (high[1] - low[1]) / n[1]
        return [(float(x), float(z)) for x in xs for z in zs], TILE_RADIUS

    def _global_step(self, rel, raw, position, correction, now) -> StaticFix | None:
        job = self._job
        if job is None:
            tiles, radius = self._tiles(position - FRAME_SHIFT)
            job = self._job = {"rel": rel, "raw": raw.copy(), "tiles": tiles, "radius": radius,
                               "i": 0, "found": [], "t": now}
        job["t"] = now
        cx, cz = job["tiles"][job["i"]]
        found = self.matcher.search(
            job["rel"], (cx, cz, 0.0), job["radius"], math.pi, GLOBAL_MIN_SCORE - CANDIDATE_MARGIN
        )
        if found is not None:
            job["found"].append(found)
        job["i"] += 1
        if job["i"] < len(job["tiles"]):
            return None
        self._job = None
        self._last_global = now
        return self._decide(job, position, correction)

    def _decide(self, job: dict, position: np.ndarray, correction: np.ndarray) -> StaticFix | None:
        found = job["found"]
        if not found:
            print(f"[static] no pose scores {GLOBAL_MIN_SCORE - CANDIDATE_MARGIN:.2f} or better "
                  f"in {len(job['tiles'])} windows; will retry")
            return None
        best = max(found, key=lambda f: f[3])
        x, z, theta, score = best
        theta = float(wrap_angle(theta))
        if score < GLOBAL_MIN_SCORE:
            print(f"[static] best pose ({x:.2f}, {z:.2f}) scores {score:.2f}, below {GLOBAL_MIN_SCORE}; will retry")
            return None

        rivals = [f for f in found if f is not best and f[3] >= score - AMBIG_MARGIN and self._apart(best, f)]
        if rivals:
            r = max(rivals, key=lambda f: f[3])
            print(f"[static] ambiguous: ({x:.2f}, {z:.2f}) scores {score:.2f} but ({r[0]:.2f}, {r[1]:.2f}) "
                  f"scores {r[3]:.2f}; waiting for a view that tells them apart")
            return None
        if not self._firm(job["rel"], x, z, theta):
            print(f"[static] ({x:.2f}, {z:.2f}) scores {score:.2f} but the scan does not pin it down "
                  "(long wall or corridor); waiting")
            return None
        if not self._pose_is_valid(x, z):
            print(f"[static] rejected a fix at ({x:.2f}, {z:.2f}) (score {score:.2f}): "
                  "outside the room or inside an obstacle")
            return None

        self._start_tracker(x, z, theta, job["raw"])
        return self._fix(x, z, theta, score, job["raw"], position, correction, global_search=True)

    @staticmethod
    def _apart(a, b) -> bool:
        turn = abs(float(wrap_angle(a[2] - b[2])))
        return math.hypot(a[0] - b[0], a[1] - b[1]) > AMBIG_DIST or turn > AMBIG_TURN

    def _firm(self, rel: np.ndarray, x: float, z: float, theta: float) -> bool:
        """True when nudging the pose clearly lowers the scan's wall score."""
        def score(px: float, pz: float, angle: float) -> float:
            c, s = math.cos(angle), math.sin(angle)
            return self.wall_grid.score(rel @ np.array([[c, -s], [s, c]]).T + (px, pz))

        d, a = FIRM_SHIFT, FIRM_TURN
        nudged = max(
            score(x + d, z, theta), score(x - d, z, theta),
            score(x, z + d, theta), score(x, z - d, theta),
            score(x, z, theta + a), score(x, z, theta - a),
        )
        return score(x, z, theta) - nudged >= FIRM_MARGIN

    def _pose_is_valid(self, x: float, z: float) -> bool:
        """True when a server-frame position is inside the room and outside every obstacle."""
        if not _inside(self._room_polygon, x, z):
            return False
        return not any(_inside(polygon, x, z) for polygon in self._obstacle_polygons)

    # ---------------------------------------------------------------- tracking

    def _start_tracker(self, x: float, z: float, theta: float, raw: np.ndarray) -> None:
        mc = _TemperedMonteCarlo(
            self.wall_grid,
            min_particles=TRACK_MIN_PARTICLES,
            max_particles=TRACK_PARTICLES,
            initial_particles=TRACK_PARTICLES,
            initial_std=TRACK_INIT_STD,
            alphas=TRACK_ALPHAS,
            max_beams=TRACK_BEAMS,
            z_hit=TRACK_Z_HIT,
            z_rand=1.0 - TRACK_Z_HIT,
            seed=TRACK_SEED,
        )
        mc.temperature = TRACK_TEMPERATURE
        mc.initialize((x, z, theta))
        self._mc = mc
        self._prev_raw = raw.copy()

    def _track(self, rel, raw, position, correction) -> StaticFix | None:
        mc, before = self._mc, self._prev_raw
        # Heading 0 on both ends: the scan is in fixed raw axes, so the robot's own
        # turning never enters, only where it went.
        mc.predict((before[0], before[1], 0.0), (raw[0], raw[1], 0.0))
        self._prev_raw = raw.copy()
        pose = mc.update(rel)
        if pose is None or mc.fit is None or mc.fit < TRACK_MIN_SCORE:
            return self._miss(False)
        return self._fix(pose[0], pose[1], pose[2], mc.fit, raw, position, correction, global_search=False)

    # ------------------------------------------------------------------ shared

    def _fix(self, x, z, theta, score, raw, position, correction, global_search: bool) -> StaticFix:
        """(x, z, theta) is the robot's server-frame pose when it was at `raw`."""
        # raw frame -> drawn room frame: R(theta) @ (w - raw) + (x, z) + FRAME_SHIFT.
        target = _pose_matrix(x + FRAME_SHIFT[0], z + FRAME_SHIFT[1], theta) @ _translation(-raw[0], -raw[1])
        # The caller multiplies delta into the correction, so make that product the target.
        delta = target @ np.linalg.inv(correction)
        moved = delta[:2, :2] @ position + delta[:2, 2]
        self.located = True
        self._misses = 0
        self.last_score = float(score)
        return StaticFix(
            delta=delta,
            score=float(score),
            shift=float(math.hypot(*(moved - position))),
            turn=float(math.atan2(delta[1, 0], delta[0, 0])),
            global_search=global_search,
        )

    def _miss(self, global_search: bool) -> None:
        if not global_search:
            self._misses += 1
            if self._misses >= LOST_AFTER_MISSES:
                print(f"[static] lost after {self._misses} failed fixes; searching the whole room")
                self.located = False
                self._last_global = -math.inf
                self._clear()
        return None