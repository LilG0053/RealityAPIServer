"""Planar LiDAR simplification."""

import cv2
import numpy as np
import math
from realityapi_pb2 import Lidar, Vector3, RawLidarScan

# Contour tolerance, in meters, applied to each scan before it is posted.
LIDAR_EPSILON = 0.05

# LiDAR position relative to base_link, in meters.
lidar_offset_x = -0.040
lidar_offset_y = 0.000
lidar_offset_z = 0.193

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

def points_to_lidar(points: np.ndarray) -> Lidar:
    """Convert an (N, 3) array into a protobuf Lidar message."""
    return Lidar(
        points=[
            Vector3(x=float(x), y=float(y), z=float(z))
            for x, y, z in points
        ]
    )

def scan_to_world_points(raw_scan: RawLidarScan) -> np.ndarray:
    """Return valid scan endpoints in ROS odometry coordinates, shape (N, 3)."""

    # Convert the protobuf range sequence into a NumPy array.
    ranges = np.asarray(raw_scan.ranges, dtype=float)
    # Each index corresponds to a LiDAR beam.
    beam_indices = np.arange(len(ranges))

    valid = (
        np.isfinite(ranges)
        & (ranges > raw_scan.range_min)
        & (ranges < raw_scan.range_max)
    )

    valid_ranges = ranges[valid]
    valid_indices = beam_indices[valid]

    angles = (
        raw_scan.angle_min
        + valid_indices * raw_scan.angle_increment
        + math.pi / 2 # accounts for mounting
        + raw_scan.transform.rotation.z # adjuts for yaw
    )

    robot_yaw = raw_scan.transform.rotation.z

    # Rotate the LiDAR's offset along with the robot.
    offset_world_x = (
        lidar_offset_x * math.cos(robot_yaw)
        - lidar_offset_y * math.sin(robot_yaw)
    )
    offset_world_y = (
        lidar_offset_x * math.sin(robot_yaw)
        + lidar_offset_y * math.cos(robot_yaw)
    )

    # Find the LiDAR origin in the odometry frame.
    lidar_world_x = raw_scan.transform.position.x + offset_world_x
    lidar_world_y = raw_scan.transform.position.y + offset_world_y

    # Place each measured endpoint relative to the LiDAR origin.
    x = lidar_world_x + valid_ranges * np.cos(angles)
    y = lidar_world_y + valid_ranges * np.sin(angles)
    z = np.full_like(x, raw_scan.transform.position.z + lidar_offset_z)

    # # in ROS coordinates, x, y represents the 2D, and Z represents height so we keep Z the same
    # x = raw_scan.transform.position.x + valid_ranges * np.cos(angles)
    # y = raw_scan.transform.position.y + valid_ranges * np.sin(angles)

    # # Every point in this planar scan has the same ROS z coordinate.
    # z = np.full_like(x, raw_scan.transform.position.z)

    return np.column_stack((x, y, z))

