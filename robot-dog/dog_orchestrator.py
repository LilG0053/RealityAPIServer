import math
from time import time
from typing import Any
from uuid import uuid4

from realityapi_pb2 import DeviceInfo, DeviceType, ServerRequest, Transform, Vector3


DEVICE_ID = "robot"
ROS_TOPIC = "/utlidar/robot_pose"


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
            device_type=DeviceType.DOG,
            timestamp=time(),
            transform=transform,
        ),
    )


def heartbeat_request() -> ServerRequest:
    return ServerRequest(req_uuid=str(uuid4()), post_heartbeat=DEVICE_ID)
