from dataclasses import dataclass
from time import time
from typing import Any

from google.protobuf.json_format import MessageToDict, ParseDict
from realityapi_pb2 import DeviceInfo, DeviceType, Transform, Vector3


def zero_transform() -> Transform:
    return Transform(position=Vector3(), rotation=Vector3())


@dataclass
class Device:
    id: str
    device_type: int
    timestamp: float

    def handle_device_info(self, device_info: DeviceInfo) -> None:
        self.id = device_info.id
        self.device_type = device_info.device_type
        self.timestamp = device_info.timestamp

    def get_device_info(self) -> DeviceInfo:
        return DeviceInfo(
            id=self.id,
            device_type=self.device_type,
            timestamp=self.timestamp,
        )

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

    def handle_device_info(self, device_info: DeviceInfo) -> None:
        super().handle_device_info(device_info)
        if device_info.HasField("transform"):
            self.transform.CopyFrom(device_info.transform)

    def get_device_info(self) -> DeviceInfo:
        return DeviceInfo(
            id=self.id,
            device_type=self.device_type,
            timestamp=self.timestamp,
            transform=self.transform,
        )

    def to_dict(self) -> dict[str, Any]:
        result = super().to_dict()
        result["transform"] = MessageToDict(
            self.transform,
            always_print_fields_with_no_presence=True,
        )
        return result


@dataclass
class TurtleBot(TransformDevice):
    @classmethod
    def from_packet(cls, device_info: DeviceInfo) -> "TurtleBot":
        return cls(
            id=device_info.id,
            device_type=device_info.device_type,
            timestamp=device_info.timestamp,
            transform=Transform(),
        )._copy_transform(device_info)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TurtleBot":
        return cls(
            id=data["id"],
            device_type=DeviceType.TURTLE,
            timestamp=time(),
            transform=ParseDict(data.get("transform", {}), zero_transform()),
        )

    def _copy_transform(self, device_info: DeviceInfo) -> "TurtleBot":
        if device_info.HasField("transform"):
            self.transform.CopyFrom(device_info.transform)
        return self


@dataclass
class Dog(TransformDevice):
    @classmethod
    def from_packet(cls, device_info: DeviceInfo) -> "Dog":
        dog = cls(
            id=device_info.id,
            device_type=device_info.device_type,
            timestamp=device_info.timestamp,
            transform=Transform(),
        )
        return dog._copy_transform(device_info)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Dog":
        return cls(
            id=data["id"],
            device_type=DeviceType.DOG,
            timestamp=time(),
            transform=ParseDict(data.get("transform", {}), zero_transform()),
        )

    def _copy_transform(self, device_info: DeviceInfo) -> "Dog":
        if device_info.HasField("transform"):
            self.transform.CopyFrom(device_info.transform)
        return self


def device_from_info(device_info: DeviceInfo) -> Device | None:
    if device_info.device_type == DeviceType.TURTLE:
        return TurtleBot.from_packet(device_info)
    if device_info.device_type == DeviceType.DOG:
        return Dog.from_packet(device_info)
    return None


def device_from_dict(data: dict[str, Any]) -> Device | None:
    device_type = data.get("device_type")
    if device_type in (DeviceType.TURTLE, "TURTLE"):
        return TurtleBot.from_dict(data)
    if device_type in (DeviceType.DOG, "DOG"):
        return Dog.from_dict(data)
    return None
