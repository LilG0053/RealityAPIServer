"""Planar LiDAR simplification."""

import cv2
import numpy as np
import math
from realityapi_pb2 import Lidar, Vector3, RawLidarScan

# Contour tolerance, in meters, applied to each scan before it is posted.
LIDAR_EPSILON = 0.05

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


def contour_iou(a: np.ndarray, b: np.ndarray, resolution: float = 0.05) -> float:
    """Rasterized intersection-over-union of two closed planar contours."""
    first = np.asarray(a, dtype=np.float32).reshape(-1, 2)
    second = np.asarray(b, dtype=np.float32).reshape(-1, 2)
    if len(first) < 3 or len(second) < 3:
        return 0.0

    first_min, first_max = first.min(axis=0), first.max(axis=0)
    second_min, second_max = second.min(axis=0), second.max(axis=0)
    if np.any(first_max < second_min) or np.any(second_max < first_min):
        return 0.0

    origin = np.minimum(first_min, second_min)
    extent = np.maximum(first_max, second_max) - origin
    width = int(np.ceil(extent[0] / resolution)) + 1
    height = int(np.ceil(extent[1] / resolution)) + 1
    if width > 4000 or height > 4000:
        return 0.0

    def mask(poly: np.ndarray) -> np.ndarray:
        pixels = np.round((poly - origin) / resolution).astype(np.int32)
        pixels[:, 0] = np.clip(pixels[:, 0], 0, width - 1)
        pixels[:, 1] = np.clip(pixels[:, 1], 0, height - 1)
        image = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(image, [pixels], 1)
        return image

    filled_first = mask(first)
    filled_second = mask(second)
    intersection = np.count_nonzero(filled_first & filled_second)
    union = np.count_nonzero(filled_first | filled_second)
    if union == 0:
        return 0.0
    return float(intersection) / float(union)


def inside_contour(points: np.ndarray, contour: np.ndarray) -> np.ndarray:
    """True for points strictly inside the contour."""
    if len(points) == 0 or len(contour) < 3:
        return np.zeros(len(points), dtype=bool)
    contour = np.ascontiguousarray(contour, dtype=np.float32).reshape(-1, 1, 2)
    inside = np.empty(len(points), dtype=bool)
    for index, (x, y) in enumerate(points):
        inside[index] = cv2.pointPolygonTest(contour, (float(x), float(y)), False) > 0
    return inside


def scan_to_world_points(raw_scan: RawLidarScan) -> Lidar:
    # Convert each LiDAR range reading into a point in the robot's world-fixed odom frame.
    #
    # Computing the ray angle `theta` in the odom XY plane requires chaining three angles:
    #   1. `(i * delta) + angle_min`:
    #      The angle of beam `i` in the LiDAR sensor's own frame (`rplidar_link`),
    #      measured counterclockwise from the sensor's 0-radian axis.
    #   2. `+ (math.pi / 2)`:
    #       transforms from `rplidar_link` into the robot body frame (`base_link`).
    #      On the TurtleBot 4, the RPLIDAR is physically mounted rotated +90 degrees
    #      counterclockwise (rpy="0 0 pi/2" in turtlebot4.urdf.xacro), so the sensor's
    #      0-radian axis points out the left side (+y) of `base_link` rather than the
    #      front (+x). Adding pi/2 aligns 0 radians with the robot's forward +x axis.
    #   3. `+ transform.rotation.z`:
    #      Transforms from the rotating `base_link` frame into the world-fixed `odom`
    #      frame by adding the robot's current yaw relative to its starting orientation.
    points = []
    for i, r in enumerate(raw_scan.ranges):
        if raw_scan.range_min < r < raw_scan.range_max:
            theta = (i * raw_scan.angle_increment) + raw_scan.angle_min + (math.pi / 2) + raw_scan.transform.rotation.z
            points.append(
                Vector3(
                    x=raw_scan.transform.position.x + (r * math.cos(theta)),
                    y=raw_scan.transform.position.y + (r * math.sin(theta)),
                    z=raw_scan.transform.position.z,
                )
            )
    return Lidar(points=points)

