"""Absolute 2D localization of a flat scan against the room walls.

Global: BranchAndBound over the whole room, one window per call, accepted only
if the answer is unambiguous and firm. Tracking: MonteCarlo seeded from the
global fix and moved by odometry. locate() returns a StaticFix whose delta the
caller multiplies into ScanLocalizer.correction.

Everything is in the server frame (the room polygons come from Room.polygons()).
"""

from __future__ import annotations

import math
import os
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
# the room (the old behavior). None (default) searches the whole room, which the
# multi-hypothesis placer handles by waiting until one pose is unambiguous. If you set
# it, remember server frame = the dashboard's drawn position minus (0.1, 2.0): drawn
# (1.0, 14.0) is server (0.9, 12.0), NOT (0.9, 0.0).
GLOBAL_HINT: tuple[float, float] | None = None
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

# --- Multi-hypothesis global localization (AMCL-style initialization) ---------------------
# Instead of committing to the single best branch-and-bound pose, score a lattice of
# (x, z, heading) poses over the whole room, keep every plausible one as a particle, and let
# odometry plus later scans thin them out. A pose is only handed to the caller once one
# cluster holds nearly all particles AND passes the same firmness (nudge) test as before, so a
# flat wall that cannot fix z keeps the dog "unplaced" until it moves and sees a corner.
# False falls back to the old tile search (GLOBAL_HINT / _decide above).
MULTI_HYPOTHESIS = True
SEED_POS_STEP = 0.20              # lattice spacing in x and z (m)
SEED_HEADING_STEP = math.radians(5.0)
SEED_BEAMS = 60                   # scan points scored per pose
SEED_MIN_SCORE = 0.35             # a seed pose must at least score this ...
SEED_MARGIN = 0.30                # ... and be within this of the best pose
SEED_PARTICLES = 4000
HYP_TEMPERATURE = 2.0             # lower than TRACK_TEMPERATURE: sharpen faster while placing
HYP_COMMIT_FRACTION = 0.90        # share of particles within the radius/turn below of the estimate
HYP_COMMIT_RADIUS = 0.5
HYP_COMMIT_TURN = math.radians(20.0)
HYP_COMMIT_UPDATES = 3            # consecutive updates the commit test must hold
RIVAL_REFINE = 3                 # distinct rival basins refined exactly before committing
RIVAL_RETRY = 1.5                 # seconds between rival checks (each re-scores the whole room, ~1 s)
HYP_LOST_FIT = 0.30               # a hypothesis cloud fitting worse than this for LOST_AFTER_MISSES updates is reseeded
# Directory to write a .npz of every seeding and commit/decision (see testing/diagnose_dog_ring.py).
# Set DOG_DUMP_DIR in the environment, or assign this.
DUMP_DIR: str | None = os.environ.get("DOG_DUMP_DIR")

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

    def seed(self, particles: np.ndarray) -> None:
        """Start from an arbitrary (N, 3) cloud of (x, z, heading) hypotheses instead of one pose."""
        self.particles = np.asarray(particles, dtype=float).copy()
        self.weights = np.full(len(self.particles), 1.0 / len(self.particles))
        p = self.particles[0]
        self.pose = make_transform2(float(p[0]), float(p[1]), float(p[2]))
        self.fit = None

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
        # Multi-hypothesis placement: the cloud of candidate poses, and how long it has held a verdict.
        self._hyp: _TemperedMonteCarlo | None = None
        self._hyp_ok = 0
        self._hyp_bad = 0
        self._lattice: np.ndarray | None = None
        self._rival_after = -math.inf
        self._dump_n = 0
        self.status = "idle"   # idle | seeded | placing | tracking, for the dashboard / logs

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
        self._hyp = None
        self._hyp_ok = 0
        self._hyp_bad = 0
        self._rival_after = -math.inf
        self.status = "idle"

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
        elif MULTI_HYPOTHESIS:
            # Seeding is rate-limited like the old global search; once seeded the cloud is
            # stepped at the tracking rate.
            limit = TRACK_INTERVAL if self._hyp is not None else GLOBAL_RETRY
            if now - self._last_global < limit:
                return None
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

        if not self.located and MULTI_HYPOTHESIS:
            if len(rel) < MATCH_MIN_POINTS:
                self._warn(f"[static] only {len(rel)} ring points within {MATCH_RANGE_MIN}-{MATCH_RANGE_MAX} m "
                           f"(need {MATCH_MIN_POINTS}); not placing", now)
                return None
            return self._hypotheses(rel, raw, pos, corr, now)
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

    # ---- multi-hypothesis placement -----------------------------------------------------------

    def _positions(self) -> np.ndarray:
        """Lattice of free (x, z) spots in the room, once. Honors GLOBAL_HINT as an optional prior."""
        if self._lattice is None:
            lo, hi = self.room.min(axis=0), self.room.max(axis=0)
            xs = np.arange(lo[0] + SEED_POS_STEP / 2, hi[0], SEED_POS_STEP)
            zs = np.arange(lo[1] + SEED_POS_STEP / 2, hi[1], SEED_POS_STEP)
            pts = np.array([(x, z) for x in xs for z in zs if self._valid(float(x), float(z))])
            self._lattice = pts.reshape(-1, 2)
        pts = self._lattice
        if GLOBAL_HINT is not None:
            near = np.hypot(pts[:, 0] - GLOBAL_HINT[0], pts[:, 1] - GLOBAL_HINT[1]) <= GLOBAL_XY_WINDOW
            pts = pts[near]
        return pts

    def score_poses(self, rel: np.ndarray, positions: np.ndarray, headings: np.ndarray) -> np.ndarray:
        """Mean wall likelihood of scan `rel` at every (position, heading): shape (P, H).

        Public so testing/diagnose_dog_ring.py scores exactly what the seeder scores.
        """
        rel = np.asarray(rel, dtype=float).reshape(-1, 2)
        beams = rel[np.linspace(0, len(rel) - 1, min(SEED_BEAMS, len(rel))).astype(int)]
        lik, origin, res = self.grid.likelihood, np.asarray(self.grid.origin), self.grid.resolution
        out = np.empty((len(positions), len(headings)))
        for j, h in enumerate(headings):
            c, s = math.cos(h), math.sin(h)
            placed = positions[:, None, :] + (beams @ np.array([[c, -s], [s, c]]).T)[None]
            idx = np.floor((placed - origin) / res).astype(int)
            ok = (idx[..., 0] >= 0) & (idx[..., 0] < lik.shape[0]) & (idx[..., 1] >= 0) & (idx[..., 1] < lik.shape[1])
            vals = np.zeros(ok.shape)
            vals[ok] = lik[idx[..., 0][ok], idx[..., 1][ok]]
            out[:, j] = vals.mean(axis=1)
        return out

    def _lattice_scores(self, rel: np.ndarray):
        """(positions, headings, scores) of the scan at every lattice pose in the room."""
        pts = self._positions()
        heads = np.arange(-math.pi, math.pi, SEED_HEADING_STEP)
        if len(pts) == 0:
            return pts, heads, np.zeros((0, len(heads)))
        return pts, heads, self.score_poses(rel, pts, heads)

    def _seed_particles(self, rel: np.ndarray, rng: np.random.Generator):
        """(particles, best score, poses kept) for every plausible (x, z, heading); particles None if nothing fits."""
        pts, heads, score = self._lattice_scores(rel)
        if score.size == 0:
            return None, 0.0, 0
        best = float(score.max())
        floor = max(SEED_MIN_SCORE, best - SEED_MARGIN)
        pi, hi = np.nonzero(score >= floor)
        if len(pi) == 0:
            return None, best, 0
        w = score[pi, hi] ** 4
        take = rng.choice(len(pi), size=SEED_PARTICLES, p=w / w.sum())
        jitter = rng.uniform(-0.5, 0.5, (SEED_PARTICLES, 3)) * (SEED_POS_STEP, SEED_POS_STEP, SEED_HEADING_STEP)
        parts = np.column_stack((pts[pi[take], 0], pts[pi[take], 1], heads[hi[take]])) + jitter
        parts[:, 2] = wrap_angle(parts[:, 2])
        return parts, best, len(pi)

    def _rival(self, rel: np.ndarray, x: float, z: float, th: float):
        """Best pose in the whole room that is distinct from (x, z, th), as (x, z, heading, score), or None.

        The lattice only finds where to look (it is too coarse to score a basin's peak), so the top
        few distinct lattice maxima are refined with an exact branch-and-bound in a small window.
        """
        pts, heads, score = self._lattice_scores(rel)
        if score.size == 0:
            return None
        far = np.hypot(pts[:, 0] - x, pts[:, 1] - z) > AMBIG_DIST
        turned = np.abs(wrap_angle(heads - th)) > AMBIG_TURN
        distinct = far[:, None] | turned[None, :]
        s = np.where(distinct, score, -1.0)
        found: list[tuple[float, float, float, float]] = []
        for _ in range(RIVAL_REFINE):
            i, j = np.unravel_index(int(np.argmax(s)), s.shape)
            if s[i, j] < SEED_MIN_SCORE:
                break
            cx, cz, ch = float(pts[i, 0]), float(pts[i, 1]), float(heads[j])
            res = self.matcher.search(rel, make_transform2(cx, cz, ch), SEED_POS_STEP, SEED_HEADING_STEP, 0.0)
            if res is not None:
                pose, sc = res
                found.append((float(pose.position.x), float(pose.position.y), float(wrap_angle(pose.rotation)), sc))
            # Suppress this basin so the next pick is a different place.
            near = (np.hypot(pts[:, 0] - cx, pts[:, 1] - cz) <= AMBIG_DIST)[:, None] & (
                np.abs(wrap_angle(heads - ch)) <= AMBIG_TURN)[None, :]
            s = np.where(near, -1.0, s)
        found = [f for f in found if math.hypot(f[0] - x, f[1] - z) > AMBIG_DIST
                 or abs(float(wrap_angle(f[2] - th))) > AMBIG_TURN]
        return max(found, key=lambda f: f[3]) if found else None

    def _new_hypotheses(self) -> _TemperedMonteCarlo:
        mc = _TemperedMonteCarlo(
            self.grid,
            min_particles=TRACK_MIN_PARTICLES,
            max_particles=SEED_PARTICLES,
            initial_particles=SEED_PARTICLES,
            initial_std=TRACK_INIT_STD,
            alphas=TRACK_ALPHAS,
            max_beams=TRACK_BEAMS,
            z_hit=TRACK_Z_HIT,
            z_rand=1.0 - TRACK_Z_HIT,
            seed=TRACK_SEED,
        )
        mc.temperature = HYP_TEMPERATURE
        return mc

    @staticmethod
    def _share(mc: MonteCarlo) -> float:
        """Share of particles within the commit radius/turn of the estimate (weights are uniform after resampling)."""
        p, est = mc.particles, mc.pose
        near = np.hypot(p[:, 0] - est.position.x, p[:, 1] - est.position.y) <= HYP_COMMIT_RADIUS
        turn = np.abs(wrap_angle(p[:, 2] - est.rotation)) <= HYP_COMMIT_TURN
        return float(np.mean(near & turn))

    def _hypotheses(self, rel, raw, pos, corr, now) -> StaticFix | None:
        self._last_global = now
        if self._hyp is None:
            mc = self._new_hypotheses()
            res = self._seed_particles(rel, mc.rng)
            if res is None or res[0] is None:
                best = None if res is None else res[1]
                print("[static] no pose in the room scores "
                      f"{SEED_MIN_SCORE:.2f} or better" + ("" if best is None else f" (best {best:.2f})") + "; will retry")
                self._dump("noseed", rel, raw, pos, corr)
                return None
            parts, best, kept = res
            mc.seed(parts)
            self._hyp, self._prev_raw, self._hyp_ok, self._hyp_bad = mc, raw.copy(), 0, 0
            self.status = "seeded"
            print(f"[static] seeded {len(parts)} hypotheses from {kept} poses scoring >= "
                  f"{max(SEED_MIN_SCORE, best - SEED_MARGIN):.2f} (best {best:.2f}) over {len(self._positions())} spots")
            self._dump("seed", rel, raw, pos, corr, particles=parts)
            return None

        mc = self._hyp
        mc.predict(make_transform2(float(self._prev_raw[0]), float(self._prev_raw[1]), 0.0),
                   make_transform2(float(raw[0]), float(raw[1]), 0.0))
        self._prev_raw = raw.copy()
        pose = mc.update(rel)
        if pose is None or mc.fit is None:
            return None
        x, z, th = pose.position.x, pose.position.y, float(wrap_angle(pose.rotation))
        share = self._share(mc)

        if mc.fit < HYP_LOST_FIT:
            self._hyp_bad += 1
            if self._hyp_bad >= LOST_AFTER_MISSES:
                print(f"[static] hypotheses stopped fitting (fit {mc.fit:.2f}); reseeding")
                self._hyp, self._hyp_ok, self._last_global = None, 0, -math.inf
                return None
        else:
            self._hyp_bad = 0

        holds = share >= HYP_COMMIT_FRACTION and mc.fit >= GLOBAL_MIN_SCORE and self._valid(x, z)
        self._hyp_ok = self._hyp_ok + 1 if holds else 0
        self.status = "placing"
        self._warn(f"[static] placing: {len(mc.particles)} particles, {share:.0%} agree on "
                   f"({x:.2f}, {z:.2f}, {math.degrees(th):.0f} deg), fit {mc.fit:.2f}", now)
        if self._hyp_ok < HYP_COMMIT_UPDATES:
            return None

        firm = self._firm(rel, x, z, th)
        if firm < FIRM_MARGIN:
            self._warn(f"[static] ({x:.2f}, {z:.2f}) fits {mc.fit:.2f} but nudging it costs only {firm:.2f} "
                       f"(needs {FIRM_MARGIN}): the view does not pin it down (long wall); keep moving", now)
            return None
        if now < self._rival_after:
            return None
        self._rival_after = now + RIVAL_RETRY
        rival = self._rival(rel, x, z, th)
        here = self.grid.score(rel @ _pose_mat(0.0, 0.0, th)[:2, :2].T + (x, z))
        if rival is not None and rival[3] >= here - AMBIG_MARGIN:
            rx, rz, rh, rs = rival
            print(f"[static] ambiguous: ({x:.2f}, {z:.2f}, {math.degrees(th):.0f} deg) scores {here:.2f} but "
                  f"({rx:.2f}, {rz:.2f}, {math.degrees(rh):.0f} deg) scores {rs:.2f}; keep moving for a view that tells them apart")
            self._dump("ambiguous", rel, raw, pos, corr, particles=mc.particles, pose=np.array([x, z, th]),
                       rival=np.array(rival))
            if rs > here:   # the cloud collapsed onto the worse of the two: start over from the richer scan
                self._hyp, self._hyp_ok, self._last_global = None, 0, -math.inf
            return None
        rival_txt = "none" if rival is None else f"{rival[3]:.2f}"
        print(f"[static] placed at ({x:.2f}, {z:.2f}) heading {math.degrees(th):.0f} deg, fit {mc.fit:.2f}, "
              f"{share:.0%} of {len(mc.particles)} hypotheses agree, firmness {firm:.2f}, best distinct rival {rival_txt}")
        self._dump("placed", rel, raw, pos, corr, particles=mc.particles, pose=np.array([x, z, th]))
        self._hyp = None
        self._start_tracker(x, z, th, raw)
        return self._fix(x, z, th, mc.fit, raw, pos, corr, global_search=True)

    def _dump(self, tag: str, rel, raw, pos, corr, **extra) -> None:
        """Write one decision's inputs to DUMP_DIR for testing/diagnose_dog_ring.py."""
        if not DUMP_DIR:
            return
        try:
            os.makedirs(DUMP_DIR, exist_ok=True)
            self._dump_n += 1
            parts = extra.pop("particles", None)
            if parts is not None and len(parts) > 600:
                parts = parts[:: -(-len(parts) // 600)]
            np.savez_compressed(
                os.path.join(DUMP_DIR, f"static_{self._dump_n:04d}_{tag}.npz"),
                rel=np.asarray(rel), raw=np.asarray(raw), pos=np.asarray(pos), corr=np.asarray(corr),
                particles=np.zeros((0, 3)) if parts is None else parts,
                **{k: np.asarray(v) for k, v in extra.items()},
            )
        except OSError as e:  # a full disk must never take localization down
            print(f"[static] could not write dump: {e}")

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
        self.status = "tracking"
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