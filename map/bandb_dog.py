"""Absolute 2D localization of a flat scan against the room walls.

Global: BranchAndBound over the whole room, one window per call, accepted only
if the answer is unambiguous and firm. Tracking: MonteCarlo seeded from the
global fix and moved by odometry. locate() returns a StaticFix whose delta the
caller multiplies into ScanLocalizer.correction.

Everything is in the server frame (the room polygons come from Room.polygons()).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from time import monotonic

import numpy as np

from common.transform2 import make_transform2
from map.algorithms.branch_and_bound import BranchAndBound
from map.algorithms.montecarlo import MonteCarlo, wrap_angle
from map.algorithms.wallgridinterface import WallGridInterface

# BranchAndBound pads the grid by 300 cells (15 m at 5 cm): scan range plus
# search window must stay inside. Keep TILE_RADIUS at or below 4.
MATCH_RANGE_MAX = 10.0
MATCH_RANGE_MIN = 0.3
MATCH_MIN_POINTS = 20

GLOBAL_MIN_SCORE = 0.50
TRACK_MIN_SCORE = 0.30

# Where the dog is in the room when it starts, as an (x, z) server-frame spot in
# meters. It is not a pose: every heading is still searched, but only within
# GLOBAL_XY_WINDOW of this spot, which rules out look-alike poses elsewhere in
# the room (the old behavior). None searches the whole room, which is ambiguous
# with a weak ring. Server frame = the dashboard's drawn position minus (0.1, 2.0).
GLOBAL_HINT: tuple[float, float] | None = (0.9, 0.0)
# Search radius around GLOBAL_HINT, at most TILE_RADIUS.
GLOBAL_XY_WINDOW = 3.0
TILE_RADIUS = 3.0
TILE_STEP = 4.0
TILE_INTERVAL = 0.3
GLOBAL_RETRY = 3.0
CANDIDATE_MARGIN = 0.15
AMBIG_MARGIN = 0.15
AMBIG_DIST = 1.5
AMBIG_TURN = math.radians(30.0)
FIRM_MARGIN = 0.08
FIRM_SHIFT = 0.3
FIRM_TURN = math.radians(6.0)

TRACK_INTERVAL = 0.2
LOST_AFTER_MISSES = 15
TRACK_PARTICLES = 1000
TRACK_MIN_PARTICLES = 200
TRACK_INIT_STD = (0.10, math.radians(3.0))
# The ring is in fixed world axes, so rotation-driven noise terms are 0.
TRACK_ALPHAS = (0.0, 0.002, 0.01, 0.0)
TRACK_BEAMS = 60
TRACK_Z_HIT = 0.8
TRACK_TEMPERATURE = 3.0
TRACK_SEED: int | None = None


def _trans(x: float, z: float) -> np.ndarray:
    m = np.eye(3)
    m[0, 2], m[1, 2] = x, z
    return m


def _pose_mat(x: float, z: float, th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, z], [0.0, 0.0, 1.0]])


def _inside(poly: np.ndarray, x: float, z: float) -> bool:
    px, pz = poly[:, 0], poly[:, 1]
    qx, qz = np.roll(px, 1), np.roll(pz, 1)
    cross = (pz > z) != (qz > z)
    with np.errstate(divide="ignore", invalid="ignore"):
        xi = (qx - px) * (z - pz) / (qz - pz) + px
    return bool(np.count_nonzero(cross & (x < xi)) % 2)


@dataclass
class StaticFix:
    delta: np.ndarray
    score: float
    shift: float
    turn: float
    global_search: bool


class _TemperedMonteCarlo(MonteCarlo):
    """MonteCarlo.update with the scan's log-likelihood divided by `temperature`."""

    temperature = 1.0

    def update(self, scan):
        scan = np.asarray(scan, dtype=float).reshape(-1, 2)
        if len(scan) == 0 or len(self.particles) == 0:
            return self.pose
        beams = scan[np.linspace(0, len(scan) - 1, min(self.max_beams, len(scan))).astype(int)]
        lw = np.log(
            self.z_hit * self._likelihood(self._place(beams, self.particles)) + self.z_rand
        ).sum(axis=1) / self.temperature
        w = self.weights * np.exp(lw - lw.max())
        total = w.sum()
        self.weights = w / total if total > 0 else np.full(len(w), 1.0 / len(w))
        self.pose = self._best_cluster_pose()
        p = self.pose
        at = np.array([[p.position.x, p.position.y, p.rotation]])
        self.fit = float(self._likelihood(self._place(scan, at))[0].mean())
        self._resample()
        return self.pose


class StaticLocalizer:
    def __init__(self, grid: WallGridInterface, room: np.ndarray, obstacles: list[np.ndarray]) -> None:
        """room and obstacles are K x 2 (x, z) polygons in the server frame."""
        self.grid = grid
        self.room = room
        self.obstacles = list(obstacles)
        self.matcher = BranchAndBound(grid)
        self.located = False
        self.last_score: float | None = None
        self._misses = 0
        self._last_global = -math.inf
        self._last_track = -math.inf
        self._warned_at = -math.inf
        self._job: dict | None = None
        self._mc: MonteCarlo | None = None
        self._prev_raw: np.ndarray | None = None

    def reset(self) -> None:
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
        """Match one corrected scan (N x 2) taken at corrected position (x, z).

        correction is the ScanLocalizer.correction that produced both. Returns
        None when no search ran or none was good enough.
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

        corr = np.asarray(correction, dtype=float)
        pos = np.asarray(position, dtype=float).reshape(2)
        inv = np.linalg.inv(corr)
        raw = inv[:2, :2] @ pos + inv[:2, 2]
        rel = (np.asarray(ring, dtype=float).reshape(-1, 2) - pos) @ corr[:2, :2]
        dist = np.hypot(rel[:, 0], rel[:, 1])
        rel = rel[(dist >= MATCH_RANGE_MIN) & (dist <= MATCH_RANGE_MAX)]

        if not self.located:
            if self._job is None and len(rel) < MATCH_MIN_POINTS:
                self._warn(f"[static] only {len(rel)} ring points within {MATCH_RANGE_MIN}-{MATCH_RANGE_MAX} m "
                           f"(need {MATCH_MIN_POINTS}); not searching", now)
                return None
            return self._global_step(rel, raw, pos, corr, now)
        if len(rel) < MATCH_MIN_POINTS:
            return self._miss(False)
        return self._track(rel, raw, pos, corr)

    def _warn(self, msg: str, now: float) -> None:
        if now - self._warned_at >= 3.0:
            self._warned_at = now
            print(msg)

    def _tiles(self) -> tuple[list[tuple[float, float]], float]:
        if GLOBAL_HINT is not None:
            return [(float(GLOBAL_HINT[0]), float(GLOBAL_HINT[1]))], min(GLOBAL_XY_WINDOW, TILE_RADIUS)
        lo, hi = self.room.min(axis=0), self.room.max(axis=0)
        n = np.maximum(np.ceil((hi - lo) / TILE_STEP).astype(int), 1)
        xs = lo[0] + (np.arange(n[0]) + 0.5) * (hi[0] - lo[0]) / n[0]
        zs = lo[1] + (np.arange(n[1]) + 0.5) * (hi[1] - lo[1]) / n[1]
        return [(float(x), float(z)) for x in xs for z in zs], TILE_RADIUS

    def _global_step(self, rel, raw, pos, corr, now) -> StaticFix | None:
        job = self._job
        if job is None:
            tiles, radius = self._tiles()
            job = self._job = {"rel": rel, "raw": raw.copy(), "tiles": tiles, "radius": radius,
                               "i": 0, "found": [], "t": now}
            print(f"[static] searching the room with {len(rel)} ring points in {len(tiles)} windows")
        job["t"] = now
        cx, cz = job["tiles"][job["i"]]
        res = self.matcher.search(
            job["rel"], make_transform2(cx, cz, 0.0), job["radius"], math.pi,
            GLOBAL_MIN_SCORE - CANDIDATE_MARGIN,
        )
        if res is not None:
            pose, s = res
            job["found"].append((float(pose.position.x), float(pose.position.y), float(pose.rotation), s))
        job["i"] += 1
        if job["i"] < len(job["tiles"]):
            return None
        self._job = None
        self._last_global = now
        return self._decide(job, pos, corr)

    def _decide(self, job: dict, pos: np.ndarray, corr: np.ndarray) -> StaticFix | None:
        found = job["found"]
        if not found:
            print(f"[static] no pose scores {GLOBAL_MIN_SCORE - CANDIDATE_MARGIN:.2f} or better "
                  f"in {len(job['tiles'])} windows; will retry")
            return None
        best = max(found, key=lambda f: f[3])
        x, z, th, score = best
        th = float(wrap_angle(th))
        if score < GLOBAL_MIN_SCORE:
            print(f"[static] best pose ({x:.2f}, {z:.2f}) scores {score:.2f}, below {GLOBAL_MIN_SCORE}; will retry")
            return None

        rivals = [f for f in found if f is not best and f[3] >= score - AMBIG_MARGIN and self._apart(best, f)]
        if rivals:
            r = max(rivals, key=lambda f: f[3])
            print(f"[static] ambiguous: ({x:.2f}, {z:.2f}, {math.degrees(th):.0f} deg) scores {score:.2f} but "
                  f"({r[0]:.2f}, {r[1]:.2f}, {math.degrees(float(wrap_angle(r[2]))):.0f} deg) scores {r[3]:.2f}; "
                  "waiting for a view that tells them apart")
            return None
        firm = self._firm(job["rel"], x, z, th)
        if firm < FIRM_MARGIN:
            print(f"[static] ({x:.2f}, {z:.2f}) scores {score:.2f} but nudging it costs only {firm:.2f} "
                  f"(needs {FIRM_MARGIN}): the scan does not pin it down (long wall); waiting")
            return None
        if not self._valid(x, z):
            print(f"[static] rejected a fix at ({x:.2f}, {z:.2f}) (score {score:.2f}): "
                  "outside the room or inside an obstacle")
            return None

        rival = max((f[3] for f in found if f is not best and self._apart(best, f)), default=None)
        print(f"[static] placed at ({x:.2f}, {z:.2f}) heading {math.degrees(th):.0f} deg, score {score:.2f}, "
              f"best distinct rival {'none' if rival is None else f'{rival:.2f}'}")
        self._start_tracker(x, z, th, job["raw"])
        return self._fix(x, z, th, score, job["raw"], pos, corr, global_search=True)

    @staticmethod
    def _apart(a, b) -> bool:
        turn = abs(float(wrap_angle(a[2] - b[2])))
        return math.hypot(a[0] - b[0], a[1] - b[1]) > AMBIG_DIST or turn > AMBIG_TURN

    def _firm(self, rel: np.ndarray, x: float, z: float, th: float) -> float:
        def score(px: float, pz: float, a: float) -> float:
            c, s = math.cos(a), math.sin(a)
            return self.grid.score(rel @ np.array([[c, -s], [s, c]]).T + (px, pz))

        d, a = FIRM_SHIFT, FIRM_TURN
        nudged = max(
            score(x + d, z, th), score(x - d, z, th),
            score(x, z + d, th), score(x, z - d, th),
            score(x, z, th + a), score(x, z, th - a),
        )
        return score(x, z, th) - nudged

    def _valid(self, x: float, z: float) -> bool:
        if not _inside(self.room, x, z):
            return False
        return not any(_inside(p, x, z) for p in self.obstacles)

    def _start_tracker(self, x: float, z: float, th: float, raw: np.ndarray) -> None:
        mc = _TemperedMonteCarlo(
            self.grid,
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
        mc.initialize(make_transform2(x, z, th))
        self._mc = mc
        self._prev_raw = raw.copy()

    def _track(self, rel, raw, pos, corr) -> StaticFix | None:
        mc, before = self._mc, self._prev_raw
        mc.predict(
            make_transform2(float(before[0]), float(before[1]), 0.0),
            make_transform2(float(raw[0]), float(raw[1]), 0.0),
        )
        self._prev_raw = raw.copy()
        pose = mc.update(rel)
        if pose is None or mc.fit is None or mc.fit < TRACK_MIN_SCORE:
            return self._miss(False)
        return self._fix(pose.position.x, pose.position.y, pose.rotation, mc.fit, raw, pos, corr,
                         global_search=False)

    def _fix(self, x, z, th, score, raw, pos, corr, global_search: bool) -> StaticFix:
        """(x, z, th) is the robot's server-frame pose when it was at `raw`."""
        target = _pose_mat(x, z, th) @ _trans(-raw[0], -raw[1])
        delta = target @ np.linalg.inv(corr)
        moved = delta[:2, :2] @ pos + delta[:2, 2]
        self.located = True
        self._misses = 0
        self.last_score = float(score)
        return StaticFix(
            delta=delta,
            score=float(score),
            shift=float(math.hypot(*(moved - pos))),
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