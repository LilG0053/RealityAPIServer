from realityapi_pb2 import Lidar, Transform, Vector3
import math
from dataclasses import dataclass, field
from map.icp import icp
import numpy as np

def apply(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
        return points @ matrix[:2, :2].T + matrix[:2, 2]

VOXEL_SIZE = 0.05
# Currently only 2D enabled
@dataclass
class ScanLocalizer:
    accumulated_scan: np.ndarray | None = None
    # Homogeneous transformation matrix
    correction: np.ndarray = field(default_factory=lambda: np.eye(3))
    def update(self, lidar: Lidar) -> bool:
        """
        Wraps around icp for 2D, returning bool value if successful.
        The delta is the transform that aligns this scan onto the accumulated map.
        False means there was no usable delta (first scan, empty scan, or too few matches).
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
        # error is expressed in world space, so we do delta @ M
        self.correction = total @ self.correction

        # Downsample points to voxel
        merged = np.vstack((self.accumulated_scan, aligned))
        _, keep = np.unique(np.floor(merged / VOXEL_SIZE).astype(int), axis=0, return_index=True)
        self.accumulated_scan = merged[keep]
        return True

    def correct_lidar(self, lidar: Lidar) -> Lidar:
        # Correction lives in the server's X/Z ground plane; Y is up and stays put
        points = np.array([[p.x, p.z] for p in lidar.points]).reshape(-1, 2)
        corrected = apply(self.correction, points)
        return Lidar(points=[
            Vector3(x=float(c[0]), y=p.y, z=float(c[1]))
            for c, p in zip(corrected, lidar.points)
        ])

    def correct_transform(self, transform: Transform) -> Transform:
        p = transform.position
        x, z = apply(self.correction, np.array([[p.x, p.z]]))[0]
        yaw = math.atan2(self.correction[1, 0], self.correction[0, 0])
        r = transform.rotation
        # Yaw is a rotation about the server's UP axis, which is rotation.y
        return Transform(
            position=Vector3(x=float(x), y=p.y, z=float(z)),
            rotation=Vector3(x=r.x, y=r.y + yaw, z=r.z),
        )

