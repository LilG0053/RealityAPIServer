"""Accumulated maps of 2D scans: plain (AccumulatedMap) and ICP-aligned (ScanLocalizer)."""

import math
from dataclasses import dataclass, field

import numpy as np
from sklearn.neighbors import NearestNeighbors

from common.transform2 import make_transform2
from map.icp import closest_point_rmse, icp
from map.lidar import LIDAR_EPSILON, inside_contour, shrink_contour, simplified_contour
from realityapi_pb2 import Transform2

# Shrink the current scan's contour about its centroid before treating the
# interior as free space. The outer band stays, so points near the walls are
# not deleted by a contour that only approximately matches them.
CONTOUR_SHRINK = 0.95

VOXEL_SIZE = 0.05
# Scan points further than this from the map are not paired for RMSE; the
# same rule (and value) as icp()'s distance_threshold.
PAIR_DISTANCE = 0.3


def apply(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a homogeneous 2D transform to (N, 2) row-vector points."""
    return points @ matrix[:2, :2].T + matrix[:2, 2]


def _drop_enclosed(accumulated: np.ndarray, scan: np.ndarray) -> np.ndarray:
    """Remove accumulated points inside the shrunk contour of this scan.

    A point the current scan encloses used to be a hit and is now free space:
    a transient occlusion the new scan does not see.
    """
    if len(scan) < 3 or len(accumulated) == 0:
        return accumulated
    contour = simplified_contour(scan, LIDAR_EPSILON)
    if len(contour) < 3:
        return accumulated
    shrunk = shrink_contour(contour, CONTOUR_SHRINK)
    return accumulated[~inside_contour(accumulated, shrunk)]


def _merge(accumulated: np.ndarray, scan: np.ndarray) -> np.ndarray:
    """Add a placed scan to the map, keeping one point per VOXEL_SIZE cell."""
    merged = np.vstack((_drop_enclosed(accumulated, scan), scan))
    _, keep = np.unique(np.floor(merged / VOXEL_SIZE).astype(int), axis=0, return_index=True)
    return merged[keep]


def _within_limit(rmse: float | None, rmse_limit: float) -> bool:
    """Log a scan's closest-point RMSE; False when it exceeds rmse_limit."""
    if rmse is None:
        print("lidar scan RMSE: no closest point pairs")
        return True
    print(f"lidar scan RMSE: {rmse:.4f} m")
    if rmse > rmse_limit:
        print(f"lidar scan discarded: RMSE {rmse:.4f} m exceeds {rmse_limit:.1f} m")
        return False
    return True


@dataclass
class AccumulatedMap:
    """Scans that are already placed (e.g. by Monte Carlo), merged as they are.

    The closest-point RMSE of each scan against the map is measured, so a scan
    that disagrees with the map by more than rmse_limit is not merged.
    """

    points: np.ndarray | None = None
    # Closest-point RMSE of the scan most recently compared with the map.
    last_rmse: float | None = None

    def update(self, points: np.ndarray, rmse_limit: float = math.inf) -> bool:
        """Merge an (N, 2) server-frame X/Z scan. False when it was not compared or not merged."""
        self.last_rmse = None
        scan = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(scan) == 0:
            return False
        if self.points is None:
            self.points = scan
            return False

        nbrs = NearestNeighbors(n_neighbors=1).fit(self.points)
        self.last_rmse = closest_point_rmse(scan, nbrs, PAIR_DISTANCE)
        if not _within_limit(self.last_rmse, rmse_limit):
            return False
        self.points = _merge(self.points, scan)
        return True

    def clear(self) -> None:
        self.points = None
        self.last_rmse = None


@dataclass
class ScanLocalizer:
    """Accumulated map that also aligns each scan to it with ICP.

    correction is the homogeneous 2D transform that maps where a scan was
    reported onto where ICP says it belongs.
    """

    accumulated_scan: np.ndarray | None = None
    correction: np.ndarray = field(default_factory=lambda: np.eye(3))
    # Closest-point RMSE of the scan most recently compared with the map.
    last_rmse: float | None = None

    def update(self, points: np.ndarray, rmse_limit: float = math.inf) -> bool:
        """Align an (N, 2) server-frame X/Z scan to the map and merge it.

        False means there was no usable alignment (first scan, empty scan, too
        few matches, or a closest-point RMSE above rmse_limit).
        """
        self.last_rmse = None
        scan = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(scan) == 0:
            return False
        scan = apply(self.correction, scan)
        if self.accumulated_scan is None:
            self.accumulated_scan = scan
            return False

        transformation_history, aligned, rmse = icp(self.accumulated_scan, scan)
        self.last_rmse = rmse
        if not _within_limit(rmse, rmse_limit) or not transformation_history:
            return False

        # Chain icp's per-iteration [R | T] steps into one transform. The error
        # is expressed in world space, so it goes in front of the correction.
        total = np.eye(3)
        for step in transformation_history:
            total = np.vstack((step, [0, 0, 1])) @ total
        self.correction = total @ self.correction
        self.accumulated_scan = _merge(self.accumulated_scan, aligned)
        return True

    def clear_scan(self) -> None:
        self.accumulated_scan = None
        self.correction = np.eye(3)

    def correct_points(self, points: np.ndarray) -> np.ndarray:
        """Apply the correction to (N, 2) server-frame X/Z points."""
        return apply(self.correction, np.asarray(points, dtype=float).reshape(-1, 2))

    def correct_pose(self, pose: Transform2) -> Transform2:
        """Apply the correction to a server-frame ground-plane pose."""
        x, z = apply(self.correction, np.array([[pose.position.x, pose.position.y]]))[0]
        yaw = math.atan2(self.correction[1, 0], self.correction[0, 0])
        return make_transform2(float(x), float(z), pose.rotation + yaw)
