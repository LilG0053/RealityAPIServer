"""Server-side flattening of the dog's raw 3D point cloud into a 2D scan.

numpy only (no ROS, OpenCV or extra protobuf classes), so it can be tested
without the robot.

The server's localizer and dashboard work on a flat ring of points, like the
TurtleBot's. The dog sends a full 3D cloud, so we keep a horizontal slice around
the dog's body, keep the nearest return per bearing, and hand back at most
DOG_SCAN_BINS points at a constant height.
"""

import math
from collections import deque
from time import monotonic

import numpy as np

from realityapi_pb2 import Lidar, RawPointCloud, Vector3

# Slice of the cloud kept, in meters relative to the dog's pose z. The floor
# sits below the low edge and the ceiling above the high one.
DOG_SCAN_Z_MIN = -0.15
DOG_SCAN_Z_MAX = 0.25
# Horizontal range kept around the dog (meters).
DOG_SCAN_RANGE_MIN = 0.1
DOG_SCAN_RANGE_MAX = 20.0
# Angular bins around the dog; one point (the nearest) survives per bin. This
# also caps the scan size, which keeps the server's 2D ICP fast.
DOG_SCAN_BINS = 180

# Ring smoothing (see RingAccumulator). The dog's lidar returns only a few points
# inside the height slice per cloud, so one cloud's ring is patchy and changes
# every frame. The ring is built from the slices of the last few clouds instead.
DOG_RING_FRAMES = 8        # clouds combined; 1 turns smoothing off
DOG_RING_MAX_AGE = 2.0     # seconds; older slices are dropped (odom may have reset)
DOG_RING_MIN_POINTS = 2    # a bearing bin needs this many returns, else it is left empty

_BYTES_PER_POINT = 12  # three little-endian float32

# 3D accumulated map (see cloud_to_points). Heights are relative to the dog's pose z.
DOG_CLOUD_RANGE_MAX = 8.0   # horizontal range kept (meters)
DOG_CLOUD_Z_MIN = -0.5      # just below the floor (floor is about 0.3 m under the pose)
DOG_CLOUD_Z_MAX = 2.0       # about 2 m above the dog
DOG_CLOUD_VOXEL = 0.08      # one point kept per 8 cm cell
DOG_CLOUD_MAX_POINTS = 20000


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
    """Return an (M, 2) array of x/y points: nearest return per bearing bin."""
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


def slice_cloud(
    points: np.ndarray,
    origin: np.ndarray,
    z_min: float = DOG_SCAN_Z_MIN,
    z_max: float = DOG_SCAN_Z_MAX,
) -> np.ndarray:
    """Keep the points in the height slice around the dog. Returns world (M, 2) x/y."""
    points = points[np.isfinite(points).all(axis=1)]
    rel_z = points[:, 2] - origin[2]
    return points[(rel_z >= z_min) & (rel_z <= z_max), :2]


def ring_from_points(
    xy: np.ndarray,
    origin: np.ndarray,
    range_min: float = DOG_SCAN_RANGE_MIN,
    range_max: float = DOG_SCAN_RANGE_MAX,
    bins: int = DOG_SCAN_BINS,
    min_points: int = 1,
) -> np.ndarray:
    """One point per bearing bin: the MEDIAN-distance return, not the nearest.

    The nearest return is whatever stray point (a leg, a speck of noise) happens
    to be closest in that bin, so it jumps around. The median sits on the wall.
    Bins with fewer than min_points returns are dropped.
    """
    if len(xy) == 0:
        return np.empty((0, 2))
    rel = xy - origin[:2]
    dist = np.hypot(rel[:, 0], rel[:, 1])
    keep = (dist >= range_min) & (dist <= range_max)
    xy, rel, dist = xy[keep], rel[keep], dist[keep]
    if len(xy) == 0:
        return np.empty((0, 2))

    angle = np.arctan2(rel[:, 1], rel[:, 0])
    bin_index = np.minimum(((angle + math.pi) / (2 * math.pi) * bins).astype(int), bins - 1)
    order = np.lexsort((dist, bin_index))          # grouped by bin, nearest first
    _, first, count = np.unique(bin_index[order], return_index=True, return_counts=True)
    enough = count >= min_points
    median = order[first + count // 2][enough]     # middle element of each bin
    return xy[median]


class RingAccumulator:
    """Builds the dog's ring from the slices of the last few clouds.

    Slices are stored in the sender's world frame, so they stay valid while the
    dog moves; the ring is cut around the dog's *current* position each time.
    """

    def __init__(self, frames: int = DOG_RING_FRAMES, max_age: float = DOG_RING_MAX_AGE) -> None:
        self._slices: deque = deque(maxlen=max(1, frames))
        self._max_age = max_age

    def update(self, points: np.ndarray, origin: np.ndarray, now: float | None = None) -> np.ndarray:
        now = monotonic() if now is None else now
        self._slices.append((now, slice_cloud(points, origin)))
        while self._slices and now - self._slices[0][0] > self._max_age:
            self._slices.popleft()
        xy = np.vstack([s for _, s in self._slices])
        min_points = DOG_RING_MIN_POINTS if len(self._slices) > 1 else 1
        return ring_from_points(xy, origin, min_points=min_points)


def raw_cloud_to_scan(raw: RawPointCloud, accumulator: "RingAccumulator | None" = None) -> Lidar:
    """Flatten a dog cloud into a constant-height ring in the sender's axes.

    The cloud and ``raw.transform.position`` must share a frame and axes (the
    dog sends raw ROS odom axes for both). Output points stay in that world
    frame at z = the dog's z, so the caller converts them with
    ``convert_lidar(scan, DOG_COORDS, SERVER_COORDS)``. An empty Lidar means
    nothing was in range.
    """
    p = raw.transform.position
    origin = np.array([p.x, p.y, p.z], dtype=np.float64)
    if not np.isfinite(origin).all():
        raise ValueError("point cloud transform has a non-finite position")

    if accumulator is None:
        ring = flatten_to_scan(decode_xyz(raw.xyz), origin)
    else:
        ring = accumulator.update(decode_xyz(raw.xyz), origin)
    return Lidar(points=[Vector3(x=x, y=y, z=float(origin[2])) for x, y in ring.tolist()])


def cloud_to_points(
    points: np.ndarray,
    origin: np.ndarray,
    range_max: float = DOG_CLOUD_RANGE_MAX,
    z_min: float = DOG_CLOUD_Z_MIN,
    z_max: float = DOG_CLOUD_Z_MAX,
    voxel: float = DOG_CLOUD_VOXEL,
    max_points: int = DOG_CLOUD_MAX_POINTS,
    range_min: float = DOG_SCAN_RANGE_MIN,
) -> np.ndarray:
    """Crop a 3D cloud around the dog and thin it. Returns an (M, 3) array.

    ``points`` and ``origin`` share a frame (the sender's world frame, ROS axes
    for the dog). Heights and ranges are measured from ``origin``, but the points
    that come back are still in the world frame, not relative to the dog.
    """
    # 1. Drop NaN / infinite rows; they would break floor() and astype(int) below.
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        return np.empty((0, 3))

    # 2. Height and range are measured from the dog, so look at positions
    #    relative to it. Boolean masks are cheap, so filter before the voxel step.
    rel = points - origin
    horizontal = np.hypot(rel[:, 0], rel[:, 1])  # ignores height
    keep = (
        (rel[:, 2] >= z_min)
        & (rel[:, 2] <= z_max)
        & (horizontal >= range_min)
        & (horizontal <= range_max)
    )
    points = points[keep]
    if len(points) == 0:
        return np.empty((0, 3))

    # 3. One point per voxel. floor() before astype(int): astype truncates toward
    #    zero, which would merge the two cells either side of 0.
    cells = np.floor(points / voxel).astype(np.int64)
    cells -= cells.min(axis=0)          # make every index >= 0
    dims = cells.max(axis=0) + 1        # cells per axis
    if int(dims[0]) * int(dims[1]) * int(dims[2]) < 2**62:
        # Pack (x, y, z) into one integer so unique() runs on a 1-D array (fast).
        flat = (cells[:, 0] * dims[1] + cells[:, 1]) * dims[2] + cells[:, 2]
        _, first = np.unique(flat, return_index=True)
    else:  # absurdly sparse extent: the packed index would overflow
        _, first = np.unique(cells, axis=0, return_index=True)
    points = points[np.sort(first)]     # sorted = keeps the original point order

    # 4. Hard cap last, with an even stride so the whole room stays covered.
    if len(points) > max_points:
        points = points[:: -(-len(points) // max_points)]  # ceil division

    return points


def raw_cloud_to_points(raw: RawPointCloud) -> np.ndarray:
    """Decode a dog cloud and run cloud_to_points around the dog's position."""
    p = raw.transform.position
    origin = np.array([p.x, p.y, p.z], dtype=np.float64)
    if not np.isfinite(origin).all():
        raise ValueError("point cloud transform has a non-finite position")
    return cloud_to_points(decode_xyz(raw.xyz), origin)