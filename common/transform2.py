"""Rigid poses on the server's ground plane, as protobuf Transform2.

A Transform2 position is Vector2(x=X, y=Z) in server coordinates (Vector2.y
always carries the server's Z) and its rotation is the heading in radians,
measured from +X toward +Z. This is pose arithmetic only; converting between
coordinate systems is common/coordinate.py's job and runs on the devices.
"""

from __future__ import annotations

import math

import numpy as np

from realityapi_pb2 import Transform2, Transform3, Vector2


def make_transform2(x: float, z: float, rotation: float) -> Transform2:
    return Transform2(position=Vector2(x=x, y=z), rotation=rotation)


def as_tuple(transform: Transform2) -> tuple:
    """(x, z, heading), for code that works on plain numbers (e.g. map/montecarlo.py)."""
    return (transform.position.x, transform.position.y, transform.rotation)


def transform2_from_transform3(transform: Transform3) -> Transform2:
    """Ground-plane part of a Transform3 already in server coordinates (heading is rotation.y)."""
    return make_transform2(transform.position.x, transform.position.z, transform.rotation.y)


def compose_transform2(a: Transform2, b: Transform2) -> Transform2:
    """Pose b, given in the frame of pose a, expressed in a's parent frame."""
    c, s = math.cos(a.rotation), math.sin(a.rotation)
    bx, bz = b.position.x, b.position.y
    return make_transform2(
        c * bx - s * bz + a.position.x,
        s * bx + c * bz + a.position.y,
        a.rotation + b.rotation,
    )


def invert_transform2(transform: Transform2) -> Transform2:
    """The pose that undoes this one: compose_transform2(t, invert_transform2(t)) is the identity."""
    c, s = math.cos(transform.rotation), math.sin(transform.rotation)
    x, z = transform.position.x, transform.position.y
    return make_transform2(-(c * x + s * z), -(-s * x + c * z), -transform.rotation)


def apply_transform2(transform: Transform2, points: np.ndarray) -> np.ndarray:
    """Place (N, 2) X/Z points given relative to a pose into the pose's parent frame."""
    c, s = math.cos(transform.rotation), math.sin(transform.rotation)
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return points @ np.array([[c, -s], [s, c]]).T + np.array([transform.position.x, transform.position.y])


def pose_change(before: Transform2, after: Transform2) -> tuple[float, float]:
    """(distance moved, absolute heading change in radians) between two poses."""
    distance = math.hypot(after.position.x - before.position.x, after.position.y - before.position.y)
    turn = abs(math.remainder(after.rotation - before.rotation, math.tau))
    return distance, turn
