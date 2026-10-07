from enum import Enum, auto
from dataclasses import dataclass
from realityapi_pb2 import DeviceType as ProtoDeviceType
from realityapi_pb2 import Heartbeat, Packet, Vector3


class DeviceType(Enum):
    VR = auto()
    AR = auto()
    DOG = auto()
    ARM = auto()
    TURTLE = auto()

@dataclass
class Device:
    id : str
    device_type: DeviceType
    pos : Vector3
    last_heartbeat: float

    # position updated, heartbeat not provided
    def get_position_packet(self) -> Packet:
        return Packet(
            position=self.pos,
            devicetype=ProtoDeviceType.Value(self.device_type.name),
            id=self.id,
        )

    # position not updated, heartbeat provided
    def get_heartbeat_packet(self) -> Packet:
        return Packet(
            devicetype=ProtoDeviceType.Value(self.device_type.name),
            id=self.id,
            heartbeat=Heartbeat(current_time=self.last_heartbeat),
        )


class TurtleBot(Device):
    def __init__(self, id: str, pos: Vector3, last_heartbeat: float):
        super().__init__(id, DeviceType.TURTLE, pos, last_heartbeat)
    
    def update_position(self, pos: Vector3):
        self.pos = pos
