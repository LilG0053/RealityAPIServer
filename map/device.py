from enum import Enum, auto
from dataclasses import dataclass
from realityapi_pb2 import Vector3, ServerBoundPacket


class DeviceType(Enum):
    VR = auto()
    AR = auto()
    DOG = auto()
    ARM = auto()
    TURTLE = auto()

class Device:
    id: str
    last_timestamp: float

    def __init__(self, packet: ServerBoundPacket):
        self.id = packet.id
        self.last_timestamp = packet.timestamp

    def handle_packet(self, packet: ServerBoundPacket):
        self.id = packet.id
        self.last_timestamp = packet.timestamp

class TurtleBot(Device):
    position: Vector3
    rotation: Vector3

    def __init__(self, packet: ServerBoundPacket):
        super().__init__(packet)
        self.position = packet.turtle.position
        self.rotation = packet.turtle.rotation

    def handle_packet(self, packet: ServerBoundPacket):
        super().handle_packet(packet)
        self.position = packet.turtle.position
        self.rotation = packet.turtle.rotation

def create_new_device(packet: ServerBoundPacket) -> Device | None:
    if packet.HasField("turtle"):
        return TurtleBot(packet)
    return None
