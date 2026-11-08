"""Build Reality API requests from the dog's ROS 2 messages."""

# The dog runs an older Python, where `tuple[float, ...]` in a signature fails
# at import time unless annotations are deferred.
from __future__ import annotations

import math
from time import time
from typing import Any
from uuid import uuid4

import numpy as np

from realityapi_pb2 import (
    DeviceInfo,
    DeviceType,
    RawPointCloud,
    ServerRequest,
    Transform,
    Vector3,
)

DEVICE_ID = "robot-dog"
ROS_TOPIC = "/utlidar/robot_pose"
LIDAR_TOPIC = "/utlidar/cloud_deskewed"
# Old protobuf runtimes (like the dog's) do not allow DeviceType.DOG attribute
# access, but Value("DOG") works on every version.
DOG_DEVICE_TYPE = DeviceType.Value("DOG")

POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8
_NUMPY_CODES = {POINT_FIELD_FLOAT32: "f4", POINT_FIELD_FLOAT64: "f8"}


def _finite(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"invalid {name} coordinate: {value!r}")
    return float(value)


def _device_request(**fields: Any) -> ServerRequest:
    return ServerRequest(
        req_uuid=str(uuid4()),
        post_device_info=DeviceInfo(
            id=DEVICE_ID,
            device_type=DOG_DEVICE_TYPE,
            timestamp=time(),
            **fields,
        ),
    )


def _yaw(orientation: Any) -> float:
    """Heading in radians (counterclockwise about ROS z, up) from a quaternion.

    An unset orientation (all zeros) gives 0.
    """
    x, y, z, w = (_finite(getattr(orientation, name), name) for name in "xyzw")
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_request(message: Any) -> ServerRequest:
    """Build a pose-only update from a geometry_msgs/msg/PoseStamped.

    Position and heading stay in raw ROS axes (x north, y west, z up, yaw about
    z). The server converts them to its own axes (map.coordinate.DOG_COORDS).
    """
    p = message.pose.position
    position = Vector3(x=_finite(p.x, "x"), y=_finite(p.y, "y"), z=_finite(p.z, "z"))
    rotation = Vector3(z=_yaw(message.pose.orientation))
    return _device_request(transform=Transform(position=position, rotation=rotation))


def _point_field(message: Any, name: str) -> Any:
    for field in message.fields:
        if field.name == name and field.count >= 1:
            return field
    raise ValueError(f"point cloud has no {name} field")


def _cloud_xyz(message: Any) -> np.ndarray:
    """Decode a sensor_msgs/msg/PointCloud2 into an (N, 3) float64 array."""
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
            clouds = [np.frombuffer(data, dtype=dtype, count=width * height)]
        else:  # rows are padded
            clouds = [
                np.frombuffer(data, dtype=dtype, count=width, offset=row * row_step)
                for row in range(height)
            ]
    except ValueError as error:
        raise ValueError(f"invalid point cloud data: {error}") from error

    return np.concatenate(
        [np.stack([c["x"], c["y"], c["z"]], axis=1) for c in clouds]
    ).astype(np.float64)


def cloud_request(message: Any, pose: tuple[float, float, float, float]) -> ServerRequest:
    """Pack a native sensor_msgs/msg/PointCloud2 for the server to process.

    The client does no filtering: it decodes the cloud to packed float32 x/y/z
    and sends it with the dog's pose ``(x, y, z, yaw)``. The server flattens it
    to a 2D scan and converts the axes (map/pointcloud.py). Cloud and pose stay
    in raw ROS axes, so they must share a frame (the client logs a warning if
    the frame_ids differ).
    """
    ox, oy, oz, yaw = (_finite(v, name) for v, name in zip(pose, ("x", "y", "z", "yaw")))
    xyz = _cloud_xyz(message)
    return ServerRequest(
        req_uuid=str(uuid4()),
        post_raw_point_cloud=RawPointCloud(
            id=DEVICE_ID,
            transform=Transform(position=Vector3(x=ox, y=oy, z=oz), rotation=Vector3(z=yaw)),
            xyz=xyz.astype("<f4").tobytes(),
        ),
    )


def heartbeat_request() -> ServerRequest:
    return ServerRequest(req_uuid=str(uuid4()), post_heartbeat=DEVICE_ID)