"""Server-side processing of a robot's raw 3D point cloud (the dog).

 The dog sends a full 3D cloud, so instead of
flattening it we keep it 3D, and as a first cut keep only the points within
DOG_LIDAR_RADIUS of the dog and thin them with a voxel grid.
"""

import numpy as np

from realityapi_pb2 import Lidar, RawPointCloud, Vector3

# Keep points within this distance (meters, straight-line 3D) of the dog.
DOG_LIDAR_RADIUS = 1.0
# Voxel edge (meters): one point is kept per cell. 0 disables downsampling.
DOG_LIDAR_VOXEL = 0.03
# Hard cap on points per update, applied after downsampling.
DOG_LIDAR_MAX_POINTS = 5000

_BYTES_PER_POINT = 12  # three little-endian float32


def decode_xyz(data: bytes) -> np.ndarray:
    """Decode packed little-endian float32 x/y/z triples into an (N, 3) array."""
    if len(data) % _BYTES_PER_POINT:
        raise ValueError(
            f"xyz length {len(data)} is not a multiple of {_BYTES_PER_POINT} bytes"
        )
    return np.frombuffer(data, dtype="<f4").reshape(-1, 3).astype(np.float64)


def crop_to_radius(points: np.ndarray, origin: np.ndarray, radius: float) -> np.ndarray:
    """Keep finite points within ``radius`` meters of ``origin``."""
    points = points[np.isfinite(points).all(axis=1)]
    return points[np.linalg.norm(points - origin, axis=1) <= radius]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Keep the first point in each ``voxel_size`` (meters) grid cell."""
    if voxel_size <= 0 or len(points) < 2:
        return points
    cells = np.floor(points / voxel_size).astype(np.int64)
    cells -= cells.min(axis=0)
    dims = cells.max(axis=0) + 1
    if int(dims[0]) * int(dims[1]) * int(dims[2]) < 2**62:
        flat = (cells[:, 0] * dims[1] + cells[:, 1]) * dims[2] + cells[:, 2]
        _, first = np.unique(flat, return_index=True)
    else:  # huge sparse extent: fall back to the slower row-wise unique
        _, first = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(first)]


def raw_cloud_to_world_points(
    raw: RawPointCloud,
    radius: float = DOG_LIDAR_RADIUS,
    voxel_size: float = DOG_LIDAR_VOXEL,
    max_points: int = DOG_LIDAR_MAX_POINTS,
) -> Lidar:
    """Crop, downsample and convert a dog cloud to server-frame lidar points.

    The cloud and ``raw.transform.position`` must share a frame and axes (the
    dog sends ROS odom axes for both).
    """
    p = raw.transform.position
    origin = np.array([p.x, p.y, p.z], dtype=np.float64)
    if not np.isfinite(origin).all():
        raise ValueError("point cloud transform has a non-finite position")

    points = crop_to_radius(decode_xyz(raw.xyz), origin, radius)
    points = voxel_downsample(points, voxel_size)
    if len(points) > max_points:
        points = points[:: -(-len(points) // max_points)]  # even stride, ceil

    return Lidar(
        points=[Vector3(x=x, y=z, z=y) for x, y, z in points.tolist()]
    )