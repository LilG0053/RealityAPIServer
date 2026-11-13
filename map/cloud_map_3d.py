"""The dog's 3D accumulated map.

Kept apart from ScanLocalizer so the 2D localizer (what the TurtleBot uses) knows
nothing about 3D. The 3D map is not matched against anything: it borrows the 2D
correction the localizer found from the flat ring, applies it to x and z, and
leaves the height (y) alone.
"""

import math

import numpy as np

# One point is kept per cell of this size, and the oldest points are dropped once
# the map passes the cap.
CLOUD_VOXEL = 0.08
CLOUD_MAX_POINTS = 150_000

# Erasing old points that a new cloud sees through (the 3D version of the 2D
# map's _drop_enclosed). FREE_SPACE_MISSES = 0 turns it off.
FREE_SPACE_MISSES = 3          # times in a row a point must be seen through
FREE_SPACE_MARGIN = 0.20       # m; the return must be at least this much farther
FREE_SPACE_RANGE = 8.0         # m; only points this close to the dog are tested
FREE_SPACE_MIN_HEIGHT = -0.15  # m above the dog's pose; keeps floor points out of the test
FREE_AZ_BINS = 180             # 2 degree bins around the dog
FREE_EL_BINS = 90              # 2 degree bins from straight down to straight up


def voxel_keys(points: np.ndarray, size: float) -> np.ndarray:
    """One integer per voxel, so np.unique runs on a 1-D array instead of rows.

    Each axis index gets 21 bits (about +-1 million cells), packed side by side.
    """
    cells = np.floor(points / size).astype(np.int64) + (1 << 20)
    cells = np.clip(cells, 0, (1 << 21) - 1)
    return (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]


def _range_bins(rel: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Direction bin and distance of points given relative to the dog (x, y up, z)."""
    dist = np.maximum(np.linalg.norm(rel, axis=1), 1e-9)
    azimuth = np.arctan2(rel[:, 2], rel[:, 0])
    elevation = np.arcsin(np.clip(rel[:, 1] / dist, -1.0, 1.0))
    az_bin = np.minimum(((azimuth + math.pi) / (2 * math.pi) * FREE_AZ_BINS).astype(int), FREE_AZ_BINS - 1)
    el_bin = np.minimum(((elevation + math.pi / 2) / math.pi * FREE_EL_BINS).astype(int), FREE_EL_BINS - 1)
    return az_bin * FREE_EL_BINS + el_bin, dist


def _apply_xz(correction: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 3x3 2D correction to the x and z columns of (N, 3) points (x, y up, z)."""
    xz = points[:, [0, 2]] @ correction[:2, :2].T + correction[:2, 2]
    return np.column_stack((xz[:, 0], points[:, 1], xz[:, 1]))


class CloudMap:
    def __init__(self) -> None:
        # Columns x, y (up), z in the server frame.
        self.points: np.ndarray | None = None
        # One voxel key and one "seen through" counter per point.
        self.keys = np.empty(0, dtype=np.int64)
        self.misses = np.empty(0, dtype=np.uint8)
        # Bumped whenever the points change, so callers can cache what they send.
        self.version = 0

    def clear(self) -> None:
        self.points = None
        self.keys = np.empty(0, dtype=np.int64)
        self.misses = np.empty(0, dtype=np.uint8)
        self.version += 1

    def add(
        self,
        points: np.ndarray,
        correction: np.ndarray,
        origin: np.ndarray | None = None,
        evidence: np.ndarray | None = None,
    ) -> None:
        """Add a 3D cloud (N x 3: x, y up, z) to the map.

        correction is the localizer's 2D correction for the scan this cloud came
        with. origin is the dog's position in the same uncorrected frame as the
        cloud; with it, old points this cloud sees straight through are erased.
        evidence is the same scan without the height crop (ceiling, far walls); the
        free-space check looks at it instead of points when it is given.
        """
        if len(points) == 0:
            return
        corrected = _apply_xz(correction, points)
        new_keys = voxel_keys(corrected, CLOUD_VOXEL)
        before = 0 if self.points is None else len(self.points)
        changed = False

        if self.points is not None:
            if origin is not None and FREE_SPACE_MISSES > 0:
                seen = corrected if evidence is None or len(evidence) == 0 else _apply_xz(correction, evidence)
                corrected_origin = _apply_xz(correction, origin.reshape(1, 3))[0]
                changed = self._erase_seen_through(seen, new_keys, corrected_origin)
            merged = np.vstack((self.points, corrected))
            keys = np.concatenate((self.keys, new_keys))
            misses = np.concatenate((self.misses, np.zeros(len(corrected), dtype=np.uint8)))
        else:
            merged, keys = corrected, new_keys
            misses = np.zeros(len(corrected), dtype=np.uint8)

        # np.unique returns the first point of every voxel; sorting the indices
        # keeps the points in the order they arrived (oldest first).
        _, keep = np.unique(keys, return_index=True)
        keep = np.sort(keep)[-CLOUD_MAX_POINTS:]
        self.points = merged[keep]
        self.keys = keys[keep]
        self.misses = misses[keep]
        if changed or len(self.points) != before:
            self.version += 1

    def _erase_seen_through(self, seen_points: np.ndarray, new_keys: np.ndarray, origin: np.ndarray) -> bool:
        """Erase old points that the new cloud sees straight through. True if any went.

        Looking out from the dog, the new cloud gives the nearest return in each
        direction. An old point that sits clearly in front of the return in its own
        direction has nothing there any more (a person who walked away, say). One
        sparse cloud is not proof, so a point must be seen through
        FREE_SPACE_MISSES times in a row; any sign it is still there resets it.
        """
        cloud, misses = self.points, self.misses

        # Seen again: a new point landed in the same voxel.
        order = np.sort(new_keys)
        at = np.minimum(np.searchsorted(order, self.keys), len(order) - 1)
        misses[order[at] == self.keys] = 0

        # Only points near the dog and above the floor are tested. The floor and
        # walls seen at a grazing angle have a huge range spread inside one bin,
        # which would look like free space when it is not.
        rel = cloud - origin
        flat = np.hypot(rel[:, 0], rel[:, 2])
        test = np.flatnonzero((flat <= FREE_SPACE_RANGE) & (rel[:, 1] >= FREE_SPACE_MIN_HEIGHT))
        if len(test):
            # Range image of the new cloud: nearest return per (azimuth, elevation) bin.
            new_bin, new_range = _range_bins(seen_points - origin)
            nearest = np.full(FREE_AZ_BINS * FREE_EL_BINS, np.inf)
            np.minimum.at(nearest, new_bin, new_range)

            old_bin, old_range = _range_bins(rel[test])
            seen = nearest[old_bin]
            # A direction with no return says nothing either way, so it is skipped.
            known = np.isfinite(seen)
            test, old_range, seen = test[known], old_range[known], seen[known]
            margin = np.maximum(FREE_SPACE_MARGIN, 0.05 * seen)
            through = old_range < seen - margin                       # return is farther behind it
            present = np.abs(old_range - seen) <= margin              # return is right on it
            misses[test[present]] = 0
            hit = test[through]
            misses[hit] = np.minimum(misses[hit], 254) + 1

        alive = misses < FREE_SPACE_MISSES
        if alive.all():
            return False
        self.points = cloud[alive]
        self.keys = self.keys[alive]
        self.misses = misses[alive]
        return True