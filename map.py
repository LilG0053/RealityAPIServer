from typing import Iterator
from uuid import UUID
from device import Device, DeviceType
from realityapi_pb2 import Vector3


class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}

    def add_device(self, id: str, device_type: DeviceType, pos: Vector3):
        self._devices[id] = Device(id=id, device_type=device_type, pos=pos)
        print(f"Added device: {self._devices.id} to dict")


    def set_device_position(self, id: str, pos: Vector3) -> bool:
        if id in self._devices:
            device = self._devices[id]
            device.pos = pos
            print(f"Set position for device: {device.id}")
            return True
        print(f"Device {device.id} not found")
        return False

    # ... same methods as before