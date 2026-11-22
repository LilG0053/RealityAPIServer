"""Build Reality API requests from the dog's ROS 2 messages."""

# The dog runs an older Python, where `tuple[float, ...]` in a signature fails
# at import time unless annotations are deferred.
from __future__ import annotations

import math
from time import time
from typing import Any
from uuid import uuid4

import numpy as np

from common.coordinate import DOG_COORDS, SERVER_COORDS, convert_points, convert_transform
from realityapi_pb2 import (
    DeviceInfo,
    DeviceType,
    RawPointCloud,
    ServerRequest,
    Transform2,
    Transform3,
    Vector2,
    Vector3,
)

DEVICE_ID = "robot-dog"
ROS_TOPIC = "/utlidar/robot_pose"
LIDAR_TOPIC = "/utlidar/cloud_deskewed"
# Old protobuf runtimes (like the dog's) do not allow DeviceType.DOG attribute
# access, but Value("DOG") works on every version.
DOG_DEVICE_TYPE = DeviceType.Value("DOG")

# Where the dog's odometry starts in the room, on the server's ground plane:
# position Vector2(x=X, y=Z) in meters, and rotation the way it faces there
# (radians: 0 = toward +X, pi/2 = toward +Z). The dog's own odometry always
# starts at (0, 0) facing "forward", so this decides where that start is.
# Change it to match where you set the dog down.
DOG_START = Transform2(position=Vector2(x=0.9, y=0.0), rotation=math.pi)

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


def _place(points: np.ndarray) -> np.ndarray:
    """Turn server-axis (N, 3) points about the dog's start heading and move them to its start."""
    c, s = math.cos(DOG_START.rotation), math.sin(DOG_START.rotation)
    placed = points.copy()
    placed[:, 0] = c * points[:, 0] - s * points[:, 2] + DOG_START.position.x
    placed[:, 2] = s * points[:, 0] + c * points[:, 2] + DOG_START.position.y
    return placed


def server_transform(pose: Transform3) -> Transform3:
    """A raw ROS pose (see ros_pose) as a Transform3 in server coordinates."""
    converted = convert_transform(pose, DOG_COORDS, SERVER_COORDS)
    p, r = converted.position, converted.rotation
    placed = _place(np.array([[p.x, p.y, p.z]]))[0]
    return Transform3(
        position=Vector3(x=float(placed[0]), y=float(placed[1]), z=float(placed[2])),
        # Heading is a rotation about the server's UP axis, which is rotation.y.
        rotation=Vector3(x=r.x, y=r.y + DOG_START.rotation, z=r.z),
    )


def server_points(xyz: np.ndarray) -> np.ndarray:
    """Raw ROS (N, 3) cloud points in server coordinates."""
    return _place(convert_points(xyz, DOG_COORDS, SERVER_COORDS))


def _yaw(orientation: Any) -> float:
    """Heading in radians (counterclockwise about ROS z, up) from a quaternion.

    An unset orientation (all zeros) gives 0.
    """
    x, y, z, w = (_finite(getattr(orientation, name), name) for name in "xyzw")
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def ros_pose(message: Any) -> Transform3:
    """A geometry_msgs/msg/PoseStamped as a Transform3 in raw ROS axes (yaw about z)."""
    p = message.pose.position
    return Transform3(
        position=Vector3(x=_finite(p.x, "x"), y=_finite(p.y, "y"), z=_finite(p.z, "z")),
        rotation=Vector3(z=_yaw(message.pose.orientation)),
    )


def pose_request(pose: Transform3) -> ServerRequest:
    """Build a pose-only update from a raw ROS pose (see ros_pose), in server coordinates."""
    return _device_request(transform3=server_transform(pose))


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


def cloud_request(message: Any, pose: Transform3) -> ServerRequest:
    """Pack a native sensor_msgs/msg/PointCloud2 for the server to process.

    The client does no filtering: it decodes the cloud, converts it and the
    dog's raw ROS pose (see ros_pose) to server coordinates, and sends both in
    one device update. The server flattens the cloud to a 2D scan
    (map/pointcloud.py). Cloud and pose must share a ROS frame (the client logs
    a warning if the frame_ids differ).
    """
    transform = server_transform(pose)
    xyz = server_points(_cloud_xyz(message))
    return _device_request(
        transform3=transform,
        raw_point_cloud=RawPointCloud(
            id=DEVICE_ID,
            transform=transform,
            xyz=xyz.astype("<f4").tobytes(),
        ),
    )


def heartbeat_request() -> ServerRequest:
    return ServerRequest(req_uuid=str(uuid4()), post_heartbeat=DEVICE_ID)