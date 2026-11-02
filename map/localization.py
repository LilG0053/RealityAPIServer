from realityapi_pb2 import Lidar, Transform, Vector3
import math
from dataclasses import dataclass, field
from map.icp import icp
import numpy as np


VOXEL_SIZE = 0.03
# Currently only 2D enabled
@dataclass
class ScanLocalizer:
    accumulated_scan: np.ndarray | None = None
    # Homogeneous transformation matrix
    correction: np.ndarray = field(default_factory=lambda: np.eye(3))
    def update(self, lidar: Lidar) -> bool:
        """
        Wraps around icp for 2D, returning bool value if successful.
        The delta is the transform that aligns this scan onto the previous one.
        All zeros means there was no usable delta (first scan, empty scan, or too few matches).
        """
        if not lidar.points:
            return False
        # Only get X and Z columns
        scan = np.array([[point.x, point.z] for point in lidar.points])
        # From the homogeneous matrix, multiply first by linear transformation portion and then translation. Transpose because we use row vectors for points instead of column vectors
        scan = scan @ self.correction[:2, :2].T + self.correction[:2, 2]
        if self.accumulated_scan is None:
            self.accumulated_scan = scan
            return False
        
        transformation_history, aligned = icp(self.accumulated_scan, scan)

        if not transformation_history:
            return False


        # Chain icp's per-iteration [R | T] steps into one transform
        total = np.eye(3)
        for step in transformation_history:
            total = np.vstack((step, [0, 0, 1])) @ total
        # error is expressed in world space, so we do M @ delta
        self.correction = total @ self.correction

        # Downsample points to voxel
        merged = np.vstack((self.accumulated_scan, aligned))
        _, keep = np.unique(np.floor(merged / VOXEL_SIZE).astype(int), axis=0, return_index=True)
        self.accumulated_scan = merged[keep]
        return True

