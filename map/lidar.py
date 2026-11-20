"""Planar LiDAR: raw scans to points, and scan simplification."""

import cv2
import numpy as np
from realityapi_pb2 import RawLidar2Scan, Transform2

# Contour tolerance, in meters, applied to each scan before it is posted.
LIDAR_EPSILON = 0.05

def simplify_points(points: np.ndarray, epsilon: float = LIDAR_EPSILON, *, closed: bool = True) -> np.ndarray:
    """Simplify an (N, 2) planar scan with cv2.approxPolyDP."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(points) == 0:
        return points.astype(float)
    curve = np.ascontiguousarray(points).reshape(-1, 1, 2)
    return cv2.approxPolyDP(curve, epsilon, closed).reshape(-1, 2).astype(float)


def shrink_contour(contour: np.ndarray, scale: float) -> np.ndarray:
    """Scale a contour about its centroid.

    scale < 1 shrinks it. scale > 1 expands it. Coordinates stay in meters;
    the centroid is not snapped to an integer, which would quantize a metric
    contour down to whole meters.
    """
    moments = cv2.moments(contour.astype(np.float32))
    if moments["m00"] == 0:
        return contour
    center = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]])
    return (contour - center) * scale + center


def simplified_contour(points: np.ndarray, epsilon: float = LIDAR_EPSILON) -> np.ndarray:
    """Closed approxPolyDP contour of a 2D scan, shape (N, 1, 2)."""
    curve = np.ascontiguousarray(points, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.approxPolyDP(curve, epsilon, True)

def inside_contour(points: np.ndarray, contour: np.ndarray) -> np.ndarray:
    """True for points strictly inside the contour."""
    if len(points) == 0 or len(contour) < 3:
        return np.zeros(len(points), dtype=bool)
    contour = np.ascontiguousarray(contour, dtype=np.float32).reshape(-1, 1, 2)
    inside = np.empty(len(points), dtype=bool)
    for index, (x, y) in enumerate(points):
        inside[index] = cv2.pointPolygonTest(contour, (float(x), float(y)), False) > 0
    return inside


def scan_to_device_points(raw_scan: RawLidar2Scan, lidar_offset: Transform2) -> np.ndarray:
    """Convert a LaserScan into (N, 2) points relative to the device.

    The points are in the device's own frame on the server's ground plane: the
    device faces +X and +Z is its left (counterclockwise from above, the same
    turn as ROS +y), so a device at room pose p sees them at
    common.transform2.apply_transform2(p, points).

    Beam i is at angle angle_min + i * angle_increment, counterclockwise from
    the sensor's 0-radian axis. lidar_offset is the sensor's pose in the
    device's frame: where it sits (position, Vector2.y being Z) and how far its
    0-radian axis is turned from the device's forward axis (rotation).
    """
    ranges = np.asarray(raw_scan.ranges, dtype=float)
    beams = np.arange(len(ranges))
    valid = (ranges > raw_scan.range_min) & (ranges < raw_scan.range_max)
    r = ranges[valid]
    theta = beams[valid] * raw_scan.angle_increment + raw_scan.angle_min + lidar_offset.rotation
    return np.column_stack((
        lidar_offset.position.x + r * np.cos(theta),
        lidar_offset.position.y + r * np.sin(theta),
    ))
