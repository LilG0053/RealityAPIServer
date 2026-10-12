from ast import NodeTransformer
from time import time
from enum import Enum, auto
from dataclasses import dataclass
from realityapi_pb2 import Vector3, ServerBoundPacket
from google.protobuf.json_format import MessageToDict, ParseDict

class DeviceType(Enum):
    NONE = auto()
    VR = auto()
    AR = auto()
    DOG = auto()
    ARM = auto()
    TURTLE = auto()

@dataclass
class Device:
    id: str
    timestamp: float

    def handle_packet(self, packet: ServerBoundPacket):
        self.id = packet.id
        self.timestamp = packet.timestamp

    def to_dict(self) -> dict:
        ret = {
            "id": self.id,
            "device_type": self.device_type().name
        }
        return ret

    def device_type(self) -> DeviceType:
        return DeviceType.NONE

@dataclass
class TurtleBot(Device):
    position: Vector3
    rotation: Vector3

    @classmethod
    def from_packet(cls, packet: ServerBoundPacket):
        return cls(
            id=packet.id,
            timestamp=packet.timestamp,
            position=packet.turtle.position,
            rotation=packet.turtle.rotation
        )

    @classmethod
    def from_dict(cls, d: dict):
        return cls(
            id=d["id"],
            timestamp=time(),
            position=d["position"] if "position" in d else Vector3(x=0, y=0, z=0),
            rotation=d["rotation"] if "rotation" in d else Vector3(x=0, y=0, z=0),
        )

    def handle_packet(self, packet: ServerBoundPacket):
        super().handle_packet(packet)
        if not packet.HasField("turtle"):
            return
        self.position = packet.turtle.position
        self.rotation = packet.turtle.rotation

    def to_dict(self) -> dict:
        ret = super().to_dict()
        ret["position"] = MessageToDict(self.position)
        ret["rotation"] = MessageToDict(self.rotation)
        return ret

    def device_type(self) -> DeviceType:
        return DeviceType.TURTLE

def device_from_packet(packet: ServerBoundPacket) -> Device | None:
    if packet.HasField("turtle"):
        return TurtleBot.from_packet(packet)
    return None

def device_from_dict(d: dict) -> Device | None:
    if not "device_type" in d:
        return None
    match d["device_type"]:
        case DeviceType.TURTLE.name:
            return TurtleBot.from_dict(d)
        case _:
            return None
