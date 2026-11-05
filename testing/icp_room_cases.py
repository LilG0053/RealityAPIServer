"""Synthetic 2D LiDAR scenes and repeatable scan perturbations.

Noise level is a unitless severity from 0 to 1. At level 1, both scans
receive 3 cm coordinate jitter (Gaussian standard deviation), and 15% of
moving observations are replaced by unrelated points. Intermediate levels
scale each effect linearly. Replacement keeps the source count fixed.
"""

import math
import numpy as np

# Segments are wall and obstacle boundaries in meters: (start, end).
ROOM = [((-3, -2.5), (3, -2.5)), ((3, -2.5), (3, 2.5)),
        ((3, 2.5), (-3, 2.5)), ((-3, 2.5), (-3, -2.5))]
FURNITURE = [((1.0, 0.5), (1.8, 0.5)), ((1.8, 0.5), (1.8, 1.3)),
             ((1.8, 1.3), (1.0, 1.3)), ((1.0, 1.3), (1.0, 0.5)),
             ((-1.8, -1.4), (-1.2, -1.4)), ((-1.2, -1.4), (-1.2, -0.7)),
             ((-1.2, -0.7), (-1.8, -0.7)), ((-1.8, -0.7), (-1.8, -1.4))]
CORRIDOR = [((-8, -1.5), (8, -1.5)), ((8, -1.5), (8, 1.5)),
            ((8, 1.5), (-8, 1.5)), ((-8, 1.5), (-8, -1.5))]
CASES = {"room": ROOM, "furnished_room": ROOM + FURNITURE, "corridor": CORRIDOR}


def cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def scan_scene(segments, point_count):
    """Cast evenly spaced rays from the origin and return their nearest hits."""
    angles = np.linspace(-math.pi, math.pi, point_count, endpoint=False)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    closest = np.full(point_count, np.inf)
    for start, end in segments:
        start = np.asarray(start, dtype=float)
        edge = np.asarray(end, dtype=float) - start
        denominator = cross(directions, edge)
        valid = np.abs(denominator) > 1e-12
        ray_distance = np.full(point_count, np.inf)
        segment_position = np.full(point_count, np.inf)
        ray_distance[valid] = cross(start, edge) / denominator[valid]
        segment_position[valid] = cross(start, directions[valid]) / denominator[valid]
        hit = valid & (ray_distance >= 0) & (segment_position >= 0) & (segment_position <= 1)
        closest[hit] = np.minimum(closest[hit], ray_distance[hit])
    visible = np.isfinite(closest)
    return directions[visible] * closest[visible, None]


def make_room_scans(case, noise, seed=42, point_count=500):
    """Return reference, moving scan, and true moving-to-reference transform.

    The second scan is a transformed copy with measurement noise and simulated
    missing/extra returns; it is not a full simulation of viewpoint occlusion.
    """
    if case not in CASES:
        raise ValueError(f"Unknown case: {case}")
    if not 0 <= noise <= 1:
        raise ValueError("noise must be between 0 and 1")
    if point_count < 20:
        raise ValueError("point_count must be at least 20")
    rng = np.random.default_rng(seed)
    ideal = scan_scene(CASES[case], point_count)
    angle = math.radians(3)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    translation = np.array([0.08, 0.04])
    reference = ideal + rng.normal(0, 0.03 * noise, ideal.shape)
    moving = ideal @ rotation.T + translation
    moving += rng.normal(0, 0.03 * noise, moving.shape)

    # A contiguous gap approximates an occluded section; random drops add sparsity.
    missing = round(len(moving) * 0.15 * noise)
    if missing:
        gap = missing // 2
        start = rng.integers(len(moving))
        drop = (start + np.arange(gap)) % len(moving)
        keep = np.ones(len(moving), dtype=bool)
        keep[drop] = False
        remaining = np.flatnonzero(keep)
        keep[rng.choice(remaining, size=missing - gap, replace=False)] = False
        moving = moving[keep]

    # These unrelated returns model outliers or newly observed objects.
    extra_count = missing
    if extra_count:
        extra = rng.uniform([-3, -2.5], [3, 2.5], size=(extra_count, 2))
        moving = np.vstack((moving, extra))

    expected = np.eye(3)
    expected[:2, :2] = rotation.T
    expected[:2, 2] = -rotation.T @ translation
    return reference, moving, expected
