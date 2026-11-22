"""The base class for every device the server tracks.

Everything a device receives and stores is in server coordinates: clients
convert their own poses and scans before sending, so devices convert no
coordinate systems.
"""

from __future__ import annotations

from time import time
from typing import Any, ClassVar

from google.protobuf.json_format import MessageToDict

from realityapi_pb2 import DeviceInfo, DeviceType, Response


def _message_dict(message) -> dict[str, Any]:
    """A message as a dict for to_dict, zero-valued fields included so a coordinate of exactly 0 is kept."""
    return MessageToDict(message, always_print_fields_with_no_presence=True)


class Device:
    # Set by each concrete subclass.
    DEVICE_TYPE: ClassVar[int] = DeviceType.NONE

    def __init__(self, id: str, timestamp: float = 0.0) -> None:
        self.id = id
        self.timestamp = timestamp

    @property
    def device_type(self) -> int:
        return self.DEVICE_TYPE

    @classmethod
    def from_packet(cls, device_info: DeviceInfo, **kwargs: Any) -> Device:
        """A new device for the first DeviceInfo from a sender. handle_device_info takes the rest.

        kwargs go to the constructor (e.g. a 2D device's wall_grid).
        """
        return cls(id=device_info.id, timestamp=time(), **kwargs)

    @classmethod
    def from_dict(cls, data: dict[str, Any], **kwargs: Any) -> Device:
        """A device saved by to_dict. kwargs go to the constructor."""
        return cls(id=data["id"], timestamp=time(), **kwargs)

    def handle_device_info(self, device_info: DeviceInfo) -> Response:
        """Take one post_device_info from this device and answer with reply().

        Subclasses call heartbeat() rather than this, and reply() once they
        have taken the post in.
        """
        self.heartbeat()
        return self.reply()

    def reply(self) -> Response:
        """The answer to a post: the server's view of this device.

        The device takes on what it is sent (its id, and for a 2D device its
        room pose), so this is how the server renames devices and corrects
        their poses and scans.
        """
        return Response(device_info=self.get_device_info())

    def get_device_info(self) -> DeviceInfo:
        """What other clients (and the dashboard) see of this device."""
        return DeviceInfo(id=self.id, device_type=self.device_type, timestamp=self.timestamp)

    def heartbeat(self) -> None:
        # Server clock: the float32 wire timestamp is only accurate to ~128 s.
        self.timestamp = time()

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "device_type": DeviceType.Name(self.device_type),
        }
