from typing import Iterator
from uuid import UUID
from time import time
from map.device import Device, DeviceType
from realityapi_pb2 import Vector3


class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}

    def add_device(self, id: str, device_type: DeviceType, pos: Vector3, last_heartbeat: float = None):
        if last_heartbeat is None:
            last_heartbeat = time()

        self._devices[id] = Device(
            id=id,
            device_type=device_type,
            pos=pos,
            last_heartbeat=last_heartbeat,
        )
        print(f"Added device: {self._devices[id]} to dict")

    def exists(self, id: str) -> bool:
        return id in self._devices

    # function that defines what values are sent to updated_pos.json
    def export_to_json(self, filepath: str):
        devices_data = []
        for device in self._devices.values():
            devices_data.append({
                "id": device.id,
                "device_type": device.device_type.name,
                "pos": {
                    "x": device.pos.x,
                    "y": device.pos.y,
                    "z": device.pos.z
                }
            })

        with open(filepath, 'w') as f:
            import json
            json.dump(devices_data, f, indent=2)

    # function called every time new device/changing existing device position
    def set_device_position(self, id: str, pos: Vector3, device_type: DeviceType):
        if id in self._devices: # if id of robot exists, set new position
            device = self._devices[id]
            device.pos = pos
            # A position report is also proof the device is still alive.
            device.last_heartbeat = time()
            print(f"Set position for device: {device.id}")
        else: # if robot id does not exist, add it as a device
            self.add_device(id, device_type=device_type, pos=pos)

        # exports file to dashboard folder which updates the visual dashboard
        self.export_to_json("dashboard/updated_pos.json")

    def get_device_pos(self, id: str) -> Vector3:
        if id in self._devices:
            return self._devices[id].pos
        
        print(f"Device with id {id} does not exist")
        return Vector3(0, 0, 0)

    def del_device(self, id: str) -> Device:
        return self._devices.pop(id)

    def update_last_time(self, id: str, new_time: float):
        self._devices[id].last_heartbeat = new_time

    def heartbeat_check(self, id: str, timeout: float) -> bool:
        """Drop a device that has not been heard from within timeout seconds."""
        device = self._devices[id]

        if (time() - device.last_heartbeat) >= timeout:
            print(f"Device {id} is inactive")
            self.del_device(id)
            return True

        return False

    def check_all(self, timeout: float):
        dropped = False
        for id in list(self._devices):
            dropped = self.heartbeat_check(id, timeout) or dropped

        # Re-export so the dashboard stops drawing a device that timed out.
        if dropped:
            self.export_to_json("dashboard/updated_pos.json")
