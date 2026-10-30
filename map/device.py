"""Devices tracked by the server map."""

from dataclasses import InitVar, dataclass, field
from time import time
from typing import Any, ClassVar

from google.protobuf.json_format import MessageToDict, ParseDict

from map.coordinate import CoordinateSystem, Direction, convert_lidar, convert_transform
from realityapi_pb2 import DeviceInfo, DeviceType, Lidar, Transform, Vector3


def zero_transform() -> Transform:
    return Transform(position=Vector3(), rotation=Vector3())


def _message_dict(message) -> dict[str, Any]:
    # Print zero-valued fields too, so a coordinate of exactly 0 still reaches
    # the dashboard JSON instead of being dropped.
    return MessageToDict(message, always_print_fields_with_no_presence=True)


@dataclass
class Device:
    id: str
    device_type: int
    timestamp: float
    coordinate_system: CoordinateSystem | None = field(default=None, kw_only=True)

    def handle_device_info(self, device_info: DeviceInfo) -> None:
        # id and device_type are fixed when the device is created. Partial
        # updates (a raw lidar scan, for example) leave them unset, so copying
        # them here would silently turn a TurtleBot into device_type NONE.
        self.timestamp = device_info.timestamp

    def get_device_info(self, target_coords: CoordinateSystem | None = None) -> DeviceInfo:
        return DeviceInfo(id=self.id, device_type=self.device_type, timestamp=self.timestamp)

    def heartbeat(self) -> None:
        self.timestamp = time()

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "device_type": DeviceType.Name(self.device_type),
        }


@dataclass
class TransformDevice(Device):
    transform: Transform
    lidar: Lidar
    initial_heading: InitVar[Direction] = Direction.NORTH

    # Set by each concrete subclass.
    DEVICE_TYPE: ClassVar[int] = DeviceType.NONE

    def __post_init__(self, initial_heading: Direction) -> None:
        if self.coordinate_system is None:
            self.coordinate_system = CoordinateSystem(
                x=initial_heading,
                y=initial_heading.counterclockwise90(),
                z=Direction.UP,
            )

    @classmethod
    def from_packet(cls, device_info: DeviceInfo) -> "TransformDevice":
        device = cls(
            id=device_info.id,
            device_type=cls.DEVICE_TYPE,
            timestamp=device_info.timestamp,
            transform=zero_transform(),
            lidar=Lidar(),
        )
        device.handle_device_info(device_info)
        return device

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TransformDevice":
        return cls(
            id=data["id"],
            device_type=cls.DEVICE_TYPE,
            timestamp=time(),
            transform=ParseDict(data.get("transform", {}), zero_transform()),
            lidar=ParseDict(data.get("lidar", {}), Lidar()),
        )

    def handle_device_info(self, device_info: DeviceInfo) -> None:
        super().handle_device_info(device_info)
        if device_info.HasField("transform"):
            self.transform.CopyFrom(device_info.transform)
        if device_info.HasField("lidar"):
            self.lidar.CopyFrom(device_info.lidar)

    def get_device_info(
        self,
        target_coords: CoordinateSystem | None = None,
        include_lidar: bool = True,
    ) -> DeviceInfo:
        info = super().get_device_info(target_coords)
        convert = target_coords is not None and self.coordinate_system is not None

        transform = self.transform
        if convert:
            transform = convert_transform(transform, self.coordinate_system, target_coords)
        info.transform.CopyFrom(transform)

        # Leave lidar unset so a pose-only post does not replace the stored scan.
        if include_lidar:
            lidar = self.lidar
            if convert:
                lidar = convert_lidar(lidar, self.coordinate_system, target_coords)
            info.lidar.CopyFrom(lidar)
        return info

    def to_dict(self) -> dict[str, Any]:
        ret = super().to_dict()
        ret["transform"] = _message_dict(self.transform)
        ret["lidar"] = _message_dict(self.lidar)
        return ret


class TurtleBot(TransformDevice):
    DEVICE_TYPE = DeviceType.TURTLE


class Dog(TransformDevice):
    DEVICE_TYPE = DeviceType.DOG


_DEVICE_CLASSES: dict[int, type[TransformDevice]] = {
    DeviceType.TURTLE: TurtleBot,
    DeviceType.DOG: Dog,
}


def device_from_info(device_info: DeviceInfo) -> Device | None:
    cls = _DEVICE_CLASSES.get(device_info.device_type)
    return cls.from_packet(device_info) if cls else None


def device_from_dict(data: dict[str, Any]) -> Device | None:
    device_type = data.get("device_type")
    if isinstance(device_type, str):
        try:
            device_type = DeviceType.Value(device_type)
        except ValueError:
            return None
    cls = _DEVICE_CLASSES.get(device_type)
    return cls.from_dict(data) if cls else None