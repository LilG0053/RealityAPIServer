from time import time
from dataclasses import dataclass
from realityapi_pb2 import DeviceType, Transform, Vector3, DeviceInfo, Lidar
from google.protobuf.json_format import MessageToDict, ParseDict

@dataclass
class Device:
    id: str
    device_type: DeviceType
    timestamp: float

    def handle_device_info(self, device_info: DeviceInfo):
        self.id = device_info.id
        self.timestamp = device_info.timestamp

    def get_device_info(self) -> DeviceInfo:
        return DeviceInfo(id=self.id, device_type=self.device_type, timestamp=self.timestamp)

    def heartbeat(self):
        self.timestamp = time()

    def to_dict(self) -> dict:
        ret = {
            "id": self.id,
            "device_type": self.device_type
        }
        return ret

@dataclass
class TurtleBot(Device):
    transform: Transform
    lidar: Lidar
    @classmethod
    def from_packet(cls, device_info: DeviceInfo):
        return cls(
            id=device_info.id,
            device_type=DeviceType.TURTLE,
            timestamp=device_info.timestamp,
            transform=device_info.transform,
            lidar = device_info.lidar
        )

    @classmethod
    def from_dict(cls, d: dict):
        return cls(
            id=d["id"],
            device_type=d["device_type"],
            timestamp=time(),
            transform=ParseDict(d["transform"], Transform()) if "transform" in d else Transform(
                position=Vector3(x=0,y=0,z=0),
                rotation=Vector3(x=0,y=0,z=0)
            ),
            lidar=ParseDict(d["lidar"], Lidar()) if "lidar" in d else Lidar()
        )

    def handle_device_info(self, device_info: DeviceInfo):
        super().handle_device_info(device_info)
        if device_info.HasField("transform"):
            self.transform = device_info.transform
        if device_info.HasField("lidar"):
            self.lidar = device_info.lidar

    def get_device_info(self) -> DeviceInfo:
        di = super().get_device_info()
        return DeviceInfo(id=di.id,device_type=di.device_type,timestamp=di.timestamp,transform=self.transform, lidar=self.lidar)

    def to_dict(self) -> dict:
        ret = super().to_dict()
        ret["transform"] = MessageToDict(self.transform)
        ret["lidar"] = MessageToDict(self.lidar)
        return ret

def device_from_info(packet: DeviceInfo) -> Device | None:
    if packet.device_type == DeviceType.TURTLE:
        return TurtleBot.from_packet(packet)
    return None

def device_from_dict(d: dict) -> Device | None:
    if not "device_type" in d:
        return None
    match d["device_type"]:
        case DeviceType.TURTLE:
            return TurtleBot.from_dict(d)
        case _:
            return None
