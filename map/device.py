from dataclasses import dataclass, field
from time import time
from dataclasses import InitVar, dataclass, field
from realityapi_pb2 import DeviceType, Transform, Vector3, DeviceInfo, Lidar
from google.protobuf.json_format import MessageToDict, ParseDict
from map.coordinate import CoordinateSystem, Direction, convert_lidar, convert_transform


def zero_transform() -> Transform:
    return Transform(position=Vector3(), rotation=Vector3())


@dataclass
class Device:
    id: str
    device_type: int
    timestamp: float
    coordinate_system: CoordinateSystem | None = field(default=None, kw_only=True)

    def handle_device_info(self, device_info: DeviceInfo) -> None:
        self.id = device_info.id
        self.device_type = device_info.device_type
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

    def __post_init__(self, initial_heading: Direction):
        self.coordinate_system = CoordinateSystem(
            x=initial_heading,
            y=initial_heading.counterclockwise90(),
            z=Direction.UP,
        )

    @classmethod
    def from_packet(cls, device_info: DeviceInfo):
        return cls(
            id=device_info.id,
            device_type=DeviceType.TURTLE,
            timestamp=device_info.timestamp,
            # should have transform
            transform=device_info.transform,
            lidar=device_info.lidar if device_info.HasField("lidar") else Lidar(points=[])
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
            lidar=ParseDict(d["lidar"], Lidar()) if "lidar" in d else Lidar(points=[])
        )

    def handle_device_info(self, device_info: DeviceInfo):
        super().handle_device_info(device_info)
        if device_info.HasField("transform"):
            self.transform = device_info.transform
        if device_info.HasField("lidar"):
            self.lidar = device_info.lidar

    def get_device_info(
        self,
        target_coords: CoordinateSystem | None = None,
        include_lidar: bool = True,
    ) -> DeviceInfo:
        di = super().get_device_info(target_coords)
        transform = self.transform
        if target_coords is not None and self.coordinate_system is not None:
            transform = convert_transform(transform, self.coordinate_system, target_coords)
        info = DeviceInfo(
            id=di.id,
            device_type=di.device_type,
            timestamp=di.timestamp,
            transform=transform,
        )
        # Leave lidar unset so a pose-only post does not replace the stored scan.
        if include_lidar:
            lidar = self.lidar
            if target_coords is not None and self.coordinate_system is not None:
                lidar = convert_lidar(lidar, self.coordinate_system, target_coords)
            info.lidar.CopyFrom(lidar)
        return info

    def to_dict(self) -> dict:
        ret = super().to_dict()
        ret["transform"] = MessageToDict(self.transform)
        ret["lidar"] = MessageToDict(self.lidar)
        return ret
    
    def device_type(self) -> DeviceType:
            return DeviceType.TURTLE
    

def device_from_info(packet: DeviceInfo) -> Device | None:
    if packet.device_type == DeviceType.TURTLE:
        return TurtleBot.from_info(packet)
    if packet.device_type == DeviceType.DOG:
        return Dog.from_info(packet)
    return None

@dataclass
class Dog(Device):
    position: Vector3
    rotation: Vector3

    @classmethod
    def from_packet(cls, packet: ServerBoundPacket):
        return cls(
            id=packet.id,
            timestamp=packet.timestamp,
            position=packet.dog.position,
            rotation=packet.dog.rotation,
        )
    def handle_device_info(self, device_info: DeviceInfo) -> None:
        super().handle_device_info(device_info)
        if device_info.HasField("transform"):
            self.transform.CopyFrom(device_info.transform)
        if device_info.HasField("lidar"):
            self.lidar.CopyFrom(device_info.lidar)

    def get_device_info(self) -> DeviceInfo:
        info = DeviceInfo(
            id=self.id,
            device_type=self.device_type,
            timestamp=self.timestamp,
            transform=self.transform,
        )
        if self.lidar.points:
            info.lidar.CopyFrom(self.lidar)
        return info

    def to_dict(self) -> dict[str, Any]:
        result = super().to_dict()
        result["transform"] = MessageToDict(
            self.transform,
            always_print_fields_with_no_presence=True,
        )
        if self.lidar.points:
            result["lidar"] = MessageToDict(self.lidar)
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
            lidar=Lidar(),
        )._copy_transform(device_info)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TurtleBot":
        return cls(
            id=data["id"],
            device_type=DeviceType.TURTLE,
            timestamp=time(),
            transform=ParseDict(data.get("transform", {}), zero_transform()),
            lidar=ParseDict(data.get("lidar", {}), Lidar()),
        )

    def _copy_transform(self, device_info: DeviceInfo) -> "TurtleBot":
        if device_info.HasField("transform"):
            self.transform.CopyFrom(device_info.transform)
        if device_info.HasField("lidar"):
            self.lidar.CopyFrom(device_info.lidar)
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
            lidar=Lidar(),
        )
        return dog._copy_transform(device_info)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Dog":
        return cls(
            id=data["id"],
            device_type=DeviceType.DOG,
            timestamp=time(),
            transform=ParseDict(data.get("transform", {}), zero_transform()),
            lidar=ParseDict(data.get("lidar", {}), Lidar()),
        )

    def _copy_transform(self, device_info: DeviceInfo) -> "Dog":
        if device_info.HasField("transform"):
            self.transform.CopyFrom(device_info.transform)
        if device_info.HasField("lidar"):
            self.lidar.CopyFrom(device_info.lidar)
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
