from __future__ import annotations

import math
import os
from time import time
from typing import Any
from uuid import uuid4

import numpy as np

from realityapi_pb2 import DeviceInfo, Lidar, ServerRequest, Transform, Vector3
from lidar_simplifier import simplify_points, voxel_downsample


DEVICE_ID = "robot-dog"
ROS_TOPIC = "/utlidar/robot_pose"
LIDAR_TOPIC = "/utlidar/cloud_deskewed"
# Keep the wire value compatible with older generated protobuf modules.
DOG_DEVICE_TYPE = 3
POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8
_NUMPY_CODES = {POINT_FIELD_FLOAT32: "f4", POINT_FIELD_FLOAT64: "f8"}

# RDP tolerance (m) for organized clouds, applied per scan row.
LIDAR_EPSILON = float(os.getenv("REALITYAPI_LIDAR_EPSILON", "0.05"))
# Voxel size (m) applied to every cloud; 0 disables.
LIDAR_VOXEL_SIZE = float(os.getenv("REALITYAPI_LIDAR_VOXEL", "0.05"))
# Hard cap on points per dashboard update.
LIDAR_MAX_POINTS = int(os.getenv("REALITYAPI_LIDAR_MAX_POINTS", "20000"))


def _coordinate(position: dict[str, Any], name: str) -> float:
    value = position.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"invalid {name} coordinate: {value!r}")
    return float(value)


def _find_position(payload: Any) -> dict[str, Any] | None:
    """Find position in Pose, PoseStamped, or Odometry-shaped ROS payloads."""
    current = payload
    while isinstance(current, dict) and "position" not in current:
        current = current.get("pose")
    return current.get("position") if isinstance(current, dict) else None


def position_request(message: dict[str, Any]) -> ServerRequest:
    payload = message.get("msg")
    if not isinstance(payload, dict):
        raise ValueError("payload is not an object")

    position = _find_position(payload)
    if not isinstance(position, dict):
        raise ValueError("no position object in message")

    # Dog sensor axes map ROS y/z to Reality z/y.
    transform = Transform(
        position=Vector3(
            x=_coordinate(position, "x"),
            y=_coordinate(position, "z"),
            z=_coordinate(position, "y"),
        ),
        rotation=Vector3(),
    )
    return ServerRequest(
        req_uuid=str(uuid4()),
        post_device_info=DeviceInfo(
            id=DEVICE_ID,
            device_type=DOG_DEVICE_TYPE,
            timestamp=time(),
            transform=transform,
        ),
    )


def pose_request(message: Any) -> ServerRequest:
    """Build a request from a native geometry_msgs/msg/PoseStamped message."""
    position = message.pose.position
    transform = Transform(
        position=Vector3(
            x=_coordinate({"x": position.x}, "x"),
            y=_coordinate({"z": position.z}, "z"),
            z=_coordinate({"y": position.y}, "y"),
        ),
        rotation=Vector3(),
    )
    return ServerRequest(
        req_uuid=str(uuid4()),
        post_device_info=DeviceInfo(
            id=DEVICE_ID,
            device_type=DOG_DEVICE_TYPE,
            timestamp=time(),
            transform=transform,
        ),
    )


def _point_field(message: Any, name: str) -> Any:
    for field in message.fields:
        if field.name == name and field.count >= 1:
            return field
    raise ValueError(f"point cloud has no {name} field")


def _cloud_xyz(message: Any) -> np.ndarray:
    """Decode a PointCloud2 into a (height, width, 3) float64 array (ROS axes)."""
    fields = [_point_field(message, name) for name in "xyz"]
    for field in fields:
        if field.datatype not in _NUMPY_CODES:
            raise ValueError("x, y, and z fields must be float32 or float64")

    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if width < 1 or height < 1 or point_step < 1 or row_step < width * point_step:
        raise ValueError("point cloud has invalid dimensions")

    data = bytes(message.data)
    if len(data) < (height - 1) * row_step + width * point_step:
        raise ValueError("point cloud data is shorter than its header claims")

    endian = ">" if message.is_bigendian else "<"
    try:
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [endian + _NUMPY_CODES[field.datatype] for field in fields],
                "offsets": [int(field.offset) for field in fields],
                "itemsize": point_step,
            }
        )
        if row_step == width * point_step:
            cloud = np.frombuffer(data, dtype=dtype, count=width * height)
            xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1)
            return xyz.reshape(height, width, 3).astype(np.float64)

        rows = []
        for row in range(height):
            cloud = np.frombuffer(data, dtype=dtype, count=width, offset=row * row_step)
            rows.append(np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1))
        return np.stack(rows).astype(np.float64)
    except ValueError as error:
        raise ValueError(f"invalid point cloud data: {error}") from error


def lidar_request(
    message: Any,
    epsilon: float = LIDAR_EPSILON,
    voxel_size: float = LIDAR_VOXEL_SIZE,
    max_points: int = LIDAR_MAX_POINTS,
) -> ServerRequest:
    """Build a request from a native sensor_msgs/msg/PointCloud2 message.

    Pipeline: vectorised decode -> drop non-finite points -> per-row
    Ramer-Douglas-Peucker (organized clouds only, where rows are ordered) ->
    voxel downsample -> cap at ``max_points``.
    """
    xyz = _cloud_xyz(message)

    if xyz.shape[0] == 1:
        # Unorganized cloud: point order carries no spatial meaning, so RDP
        # would be both slow and ineffective. Voxel downsampling handles it.
        points = xyz[0]
        points = points[np.isfinite(points).all(axis=1)]
    else:
        # Bound the pure-Python RDP cost on dense scans before simplifying.
        stride = max(1, math.ceil(xyz.shape[0] * xyz.shape[1] / (max_points * 4)))
        kept = []
        for row in xyz[:, ::stride]:
            row = row[np.isfinite(row).all(axis=1)]
            if epsilon > 0 and len(row) >= 3:
                row = np.asarray(simplify_points(row.tolist(), epsilon), dtype=np.float64)
            if len(row):
                kept.append(row)
        points = np.concatenate(kept) if kept else np.empty((0, 3))

    points = voxel_downsample(points, voxel_size)
    if len(points) > max_points:
        points = points[:: math.ceil(len(points) / max_points)]
    if len(points) == 0:
        raise ValueError("point cloud has no finite points")

    # Match the pose conversion: ROS x/y/z -> Reality x/z/y.
    points = points[:, [0, 2, 1]]
    vectors = [Vector3(x=x, y=y, z=z) for x, y, z in points.tolist()]

    return ServerRequest(
        req_uuid=str(uuid4()),
        post_device_info=DeviceInfo(
            id=DEVICE_ID,
            device_type=DOG_DEVICE_TYPE,
            timestamp=time(),
            lidar=Lidar(points=vectors),
        ),
    )


def heartbeat_request() -> ServerRequest:
    return ServerRequest(req_uuid=str(uuid4()), post_heartbeat=DEVICE_ID)