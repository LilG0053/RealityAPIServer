"""Planar LiDAR simplification."""

import cv2
import numpy as np

from realityapi_pb2 import Lidar, Vector3


def simplify_lidar_2d(
    lidar: Lidar,
    epsilon: float = 0.05,
    *,
    closed: bool = True,
    axis_tolerance: float = 1e-4,
) -> Lidar:
    """Simplify a planar LiDAR scan with cv2.approxPolyDP.

    A 2D scan leaves one of x, y, or z unchanged across every point. That axis
    is dropped, the other two are reshaped to (N, 1, 2) and approximated, and
    the constant value is written back onto each simplified point.
    """
    if not lidar.points:
        return Lidar()

    coords = np.array([[point.x, point.y, point.z] for point in lidar.points], dtype=np.float32)
    spans = np.ptp(coords, axis=0)
    flat_axis = int(np.argmin(spans))
    if spans[flat_axis] > axis_tolerance:
        raise ValueError(
            "simplify_lidar_2d needs one constant axis, "
            f"but x/y/z spans are {spans.tolist()}"
        )

    kept_axes = [axis for axis in range(3) if axis != flat_axis]
    curve = np.ascontiguousarray(coords[:, kept_axes]).reshape(-1, 1, 2)
    approx = cv2.approxPolyDP(curve, epsilon, closed).reshape(-1, 2)

    simplified = np.empty((len(approx), 3), dtype=np.float32)
    simplified[:, kept_axes] = approx
    simplified[:, flat_axis] = coords[0, flat_axis]
    return Lidar(
        points=[Vector3(x=float(point[0]), y=float(point[1]), z=float(point[2])) for point in simplified]
    )

def scan_to_world_points(scan: RawLidarScan) -> Lidar:
    # loop through scan.ranges
    # calculate theta from angle_min + i * angle_increment
    # add the sensor mounting offset and robot yaw
    # use scan.transform position to make world coordinates
    # simplify the result
    return Lidar(points=world_points)
