from realityapi_pb2 import Lidar, Transform, Vector3
import math
from dataclasses import dataclass, field
from map.icp import icp
import numpy as np

@dataclass
class ScanLocalizer:
    prev_scan: np.ndarray | None = None

    def align_to_prev_scan(self, lidar: Lidar) -> tuple[float, float, float]:
        """
        Wraps around icp for 2D, returning delta angle, x, and z in that order.
        The delta is the transform that aligns this scan onto the previous one.
        All zeros means there was no usable delta (first scan, empty scan, or too few matches).
        """
        if not lidar.points:
            return (0.0, 0.0, 0.0)
        # Only get X and Z columns
        scan = np.array([[point.x, point.z] for point in lidar.points])
        if self.prev_scan is None:
            self.prev_scan = scan
            return (0.0, 0.0, 0.0)
        transformation_history, points = icp(self.prev_scan, scan, max_iterations=100, distance_threshold=0.3, convergence_translation_threshold=1e-3,
        convergence_rotation_threshold=1e-4, point_pairs_threshold=10, verbose=False)
        self.prev_scan = scan

        # Chain icp's per-iteration [R | T] steps into one transform
        total = np.eye(3)
        for step in transformation_history:
            total = np.vstack((step, [0, 0, 1])) @ total

        return (float(math.atan2(total[1, 0], total[0, 0])), float(total[0, 2]), float(total[1, 2]))
