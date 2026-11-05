"""Small, independently testable parts of a 2D KISS-ICP-style pipeline."""

import math

import numpy as np

from realityapi_pb2 import RawLidarScan


def scan_to_robot_points(raw_scan: RawLidarScan) -> np.ndarray:
    """Return valid LiDAR endpoints in the robot's base_link XY frame.

    The TurtleBot's LiDAR zero ray points along base_link +Y, so its mounting
    angle is added here. The odometry pose is deliberately not applied: it
    will be the initial pose guess for registration against the local map.

    Returns an (N, 2) float array, including an (0, 2) array for no valid rays.
    """
    points = []
    for beam_index, distance in enumerate(raw_scan.ranges):
        if not math.isfinite(distance) or not raw_scan.range_min < distance < raw_scan.range_max:
            continue

        beam_angle = raw_scan.angle_min + beam_index * raw_scan.angle_increment
        robot_angle = beam_angle + math.pi / 2
        points.append((distance * math.cos(robot_angle), distance * math.sin(robot_angle)))

    return np.asarray(points, dtype=float).reshape(-1, 2)
