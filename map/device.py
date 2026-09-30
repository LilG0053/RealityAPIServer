from enum import Enum, auto
from dataclasses import dataclass
from realityapi_pb2 import Vector3


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
