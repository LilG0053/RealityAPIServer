"""Server-side flattening of the dog's raw 3D point cloud into a 2D scan.

numpy only (no ROS, OpenCV or extra protobuf classes), so it can be tested
without the robot.

The server's localizer and dashboard work on a flat ring of points, like the
TurtleBot's. The dog sends a full 3D cloud, already in server coordinates (the
dog converts it on-device), so we keep a horizontal slice around the dog's
body, keep the nearest return per bearing, and hand back at most DOG_SCAN_BINS
points at a constant height.
"""

import math

import numpy as np

from realityapi_pb2 import RawPointCloud

# Slice of the cloud kept, in meters relative to the dog's height. The floor
# sits below the low edge and the ceiling above the high one.
DOG_SCAN_Z_MIN = -0.15
DOG_SCAN_Z_MAX = 0.25
# Horizontal range kept around the dog (meters).
DOG_SCAN_RANGE_MIN = 0.2
DOG_SCAN_RANGE_MAX = 10.0
# Angular bins around the dog; one point (the nearest) survives per bin. This
# also caps the scan size, which keeps the server's 2D ICP fast.
DOG_SCAN_BINS = 180

_BYTES_PER_POINT = 12  # three little-endian float32


def decode_xyz(data: bytes) -> np.ndarray:
    """Decode packed little-endian float32 x/y/z triples into an (N, 3) array."""
    if len(data) % _BYTES_PER_POINT:
        raise ValueError(
            f"xyz length {len(data)} is not a multiple of {_BYTES_PER_POINT} bytes"
        )
    return np.frombuffer(data, dtype="<f4").reshape(-1, 3).astype(np.float64)


def flatten_to_scan(
    points: np.ndarray,
    origin: np.ndarray,
    z_min: float = DOG_SCAN_Z_MIN,
    z_max: float = DOG_SCAN_Z_MAX,
    range_min: float = DOG_SCAN_RANGE_MIN,
    range_max: float = DOG_SCAN_RANGE_MAX,
    bins: int = DOG_SCAN_BINS,
) -> np.ndarray:
    """Return an (M, 2) array of points on the first two axes: nearest return per bearing bin.

    The third axis is height; points and origin are (plane, plane, height).
    """
    points = points[np.isfinite(points).all(axis=1)]
    rel = points - origin
    rel = rel[(rel[:, 2] >= z_min) & (rel[:, 2] <= z_max)]
    dist = np.hypot(rel[:, 0], rel[:, 1])
    keep = (dist >= range_min) & (dist <= range_max)
    rel, dist = rel[keep], dist[keep]
    if len(rel) == 0:
        return np.empty((0, 2))

    angle = np.arctan2(rel[:, 1], rel[:, 0])
    bin_index = np.minimum(((angle + math.pi) / (2 * math.pi) * bins).astype(int), bins - 1)
    # Sort by (bin, distance); the first row of each bin is its nearest point.
    order = np.lexsort((dist, bin_index))
    _, first = np.unique(bin_index[order], return_index=True)
    nearest = order[first]
    return rel[nearest, :2] + origin[:2]


def raw_cloud_to_ring(raw: RawPointCloud) -> tuple[np.ndarray, float]:
    """Flatten a dog cloud into (ring, height).

    The cloud and ``raw.transform`` are in server coordinates (Y up). ring is an
    (M, 2) array of server-frame X/Z points and height is the dog's Y. An empty
    ring means nothing was in range.
    """
    p = raw.transform.position
    # Reorder to (X, Z, Y) so the ground plane comes first and height last.
    origin = np.array([p.x, p.z, p.y], dtype=np.float64)
    if not np.isfinite(origin).all():
        raise ValueError("point cloud transform has a non-finite position")

    ring = flatten_to_scan(decode_xyz(raw.xyz)[:, [0, 2, 1]], origin)
    return ring, float(p.y)
