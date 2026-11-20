"""Simulated robots in the lab room, shared by the tests.

Poses are (x, z, heading) tuples in the server frame (heading from +X toward
+Z), like map/montecarlo.py. Scans are ray-cast against the room's walls and
obstacles from assets/blockout.json.
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.transform2 import compose_transform2, make_transform2
from map.algorithms.montecarlo import wrap_angle
from map.room import BLOCKOUT_PATH, Room, polygon_array
from realityapi_pb2 import RawLidar2Scan, Transform3, Vector3

ROOM = Room.from_json(BLOCKOUT_PATH)
POLYGONS = [polygon_array(polygon) for polygon in ROOM.polygons()]  # server frame
SEGMENTS = [(p[i], p[(i + 1) % len(p)]) for p in POLYGONS for i in range(len(p))]
MAX_RANGE = 12.0

# A loop through open floor, in the server frame (meters).
WAYPOINTS = [(5.5, 3.0), (5.5, 8.0), (1.5, 8.0), (1.5, 3.0), (5.5, 3.0)]
STEP = 0.1


def ranges_from(x, z, angles):
    """Distance to the nearest wall along each room-frame angle from (x, z); inf past MAX_RANGE."""
    o = np.array([x, z])
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    ranges = np.full(len(d), np.inf)
    for a, b in SEGMENTS:
        e = b - a
        denom = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide="ignore", invalid="ignore"):
            w = a - o
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / denom
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / denom
        hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1) & (t < ranges)
        ranges = np.where(hit, t, ranges)
    ranges[ranges >= MAX_RANGE] = np.inf
    return ranges


def body_scan(pose, beams=720):
    """Points a robot at pose sees, in its own frame (it faces +X), with the sensor at its center."""
    angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    r = ranges_from(pose[0], pose[1], angles + pose[2])
    ok = np.isfinite(r)
    return np.column_stack((r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])))


def raw_scan(pose, lidar_offset, beams=720):
    """The RawLidar2Scan a lidar mounted at lidar_offset (a Transform2) reports at a room pose."""
    sensor = compose_transform2(make_transform2(*pose), lidar_offset)
    angle_min, increment = -math.pi, 2 * math.pi / beams
    angles = angle_min + np.arange(beams) * increment
    ranges = ranges_from(sensor.position.x, sensor.position.y, angles + sensor.rotation)
    return RawLidar2Scan(
        ranges=[float(r) for r in ranges],
        range_min=0.15, range_max=MAX_RANGE,
        angle_min=angle_min, angle_increment=increment,
    )


def true_path():
    """Room poses along the loop: drive each leg, then turn in place."""
    poses = []
    for (x0, z0), (x1, z1) in zip(WAYPOINTS, WAYPOINTS[1:]):
        heading = math.atan2(z1 - z0, x1 - x0)
        if poses:
            # Turn toward the next leg in 10 degree steps.
            current = poses[-1][2]
            turn = float(wrap_angle(heading - current))
            for k in range(1, int(abs(turn) / math.radians(10)) + 1):
                poses.append((x0, z0, current + math.copysign(math.radians(10) * k, turn)))
        steps = int(math.hypot(x1 - x0, z1 - z0) / STEP)
        for k in range(steps + 1):
            poses.append((x0 + (x1 - x0) * k / steps, z0 + (z1 - z0) * k / steps, heading))
    return poses


def compose(origin, pose):
    """Pose in a frame placed at origin (x, z, heading)."""
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c * pose[0] - s * pose[1] + origin[0], s * pose[0] + c * pose[1] + origin[1], pose[2] + origin[2])


def drifting_odometry(path, odom_origin, rng):
    """Odometry (server axes) for a path: starts at odom_origin, drifts 3% in distance and 2 deg/m in heading."""
    odom = [compose(odom_origin, (0.0, 0.0, 0.0))]
    for before, after in zip(path, path[1:]):
        c, s = math.cos(-before[2]), math.sin(-before[2])
        dx, dz = after[0] - before[0], after[1] - before[1]
        forward = c * dx - s * dz
        side = s * dx + c * dz
        turn = float(wrap_angle(after[2] - before[2]))
        distance = math.hypot(forward, side)
        forward *= 1.03 + rng.normal(0, 0.01)
        turn += math.radians(2.0) * distance + rng.normal(0, math.radians(0.2))
        h = odom[-1][2]
        odom.append((
            odom[-1][0] + math.cos(h) * forward - math.sin(h) * side,
            odom[-1][1] + math.sin(h) * forward + math.cos(h) * side,
            h + turn,
        ))
    return odom


def ros_odometry(odom):
    """A server-axes odometry pose as the ROS odometry a TurtleBot reports (x north, y west, yaw about z).

    Server X is south and Z is east, so ROS x = -X, y = -Z, and the ROS yaw is
    the server heading less pi.
    """
    x, z, heading = odom
    return Transform3(position=Vector3(x=-x, y=-z), rotation=Vector3(z=heading - math.pi))


def heading_error(a, b):
    """Absolute difference of two headings, in degrees."""
    return math.degrees(abs(float(wrap_angle(a - b))))
