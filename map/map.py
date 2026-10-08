from typing import Iterator
from uuid import UUID
from map.device import Device, DeviceType
from realityapi_pb2 import Vector3, Heartbeat
from time import time


class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}

    def add_device(self, id: str, device_type: DeviceType, pos: Vector3, last_heartbeat: Heartbeat):
        if id not in self._devices:
            self._devices[id] = Device(id=id, device_type=device_type, pos=pos, last_heartbeat = last_heartbeat)
            print(f"Added device: {id} to dict")
        else:
            return False
        
    def exists(self,name: str):
        if name in self._devices:
            return True
        else:
            return False
        
    def set_device_position(self, id: str, pos: Vector3) -> bool:
        if id in self._devices:
            device = self._devices[id]
            device.pos = pos
            print(f"Set position for device: {device.id}")
            return True
        print(f"Device {id} not found")
        return False

    def get_device_(self, id: str) -> Vector3:
        self._device 

    def del_device(self, id:str):
        return self._devices.pop(id)

    def get_positions(self) -> dict[str, Device]:
        pass

    def heartbeat_check(self, id:str, TIMEOUT:int):
        name = self._devices[id]
        current_time = time()
        last_time = name.last_heartbeat

        if (current_time-last_time) >= TIMEOUT:
            print(f"[-] {id} disconnected")
            self.del_device(id)
    def update_last_time(self, id:str, new_time: int):
        self._devices[id].last_heartbeat = new_time

    def check_all(self, timeout):
        for id in list(self._devices):
            self.heartbeat_check(id, timeout)
    


    # ... same methods as before