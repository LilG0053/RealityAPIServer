from typing import Iterator
from uuid import UUID
from time import time
from map.device import Device, DeviceType, DeviceStatus
from realityapi_pb2 import Vector3


class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}

    def add_device(self, id: str, device_type: DeviceType, pos: Vector3, device_status: DeviceStatus, last_heartbeat: float = None):
        if last_heartbeat is None:
            last_heartbeat = time()

        self._devices[id] = Device(
            id=id,
            device_type=device_type,
            pos=pos,
            device_status=device_status,
            last_heartbeat=last_heartbeat,
        )
        print(f"Added device: {self._devices[id]} to dict")

    def exists(self, id: str) -> bool:
        return id in self._devices
    
    # helper method that creates dictionary with device info
    def create_device_data(self, device: Device):
        return {
            "id": device.id,
            "device_type": device.device_type.name,
            "pos": {
                "x": round(device.pos.x, 2),
                "y": round(device.pos.y, 2),
                "z": round(device.pos.z, 2)
            },
            "status": device.device_status.value
        }
        
    # function that defines what values are sent to updated_pos.json
    def export_to_json(self, filepath: str):
        devices_data = []

        # seperate online and offline devices so ONLINE appears first
        online_devices = []
        offline_devices = []
        for device in self._devices.values():
            if (device.device_status == DeviceStatus.ONLINE): 
                online_devices.append(self.create_device_data(device))
            else:
                offline_devices.append(self.create_device_data(device))
        devices_data = online_devices + offline_devices

        with open(filepath, 'w') as f:
            import json
            json.dump(devices_data, f, indent=2)

    def load_from_json(self, filepath: str):
        import json
        try:
            with open(filepath, 'r') as f:
                devices_data = json.load(f)
            
            for device_data in devices_data:
                device_type = DeviceType[device_data["device_type"]]
                pos = Vector3(
                    x=device_data["pos"]["x"],
                    y=device_data["pos"]["y"],
                    z=device_data["pos"]["z"]
                )
                device_status = DeviceStatus(device_data["status"])

                self.add_device(device_data["id"], device_type, pos, device_status)
            print(f"Loaded {len(devices_data)} devices from {filepath}")
        except FileNotFoundError:
            print(f"No existing JSON file at {filepath}, starting with empty map")
        except Exception as e:
            print(f"Error loading from JSON: {e}")

    def set_device_position(self, id: str, pos: Vector3, device_type: DeviceType):
        if id in self._devices: # if id of robot exists, set new position
            device = self._devices[id]
            device.pos = pos
            # A position report is also proof the device is still alive.
            device.last_heartbeat = time()
            device.device_status = DeviceStatus.ONLINE
            print(f"Set position for device: {device.id}")
        else: # if robot id does not exist, add it as a device
            self.add_device(id, device_type=device_type, pos=pos, device_status=DeviceStatus.ONLINE)

        # exports file to dashboard folder which updates the visual dashboard
        self.export_to_json("dashboard/updated_pos.json")

    def get_device_pos(self, id: str) -> Vector3:
        if id in self._devices:
            return self._devices[id].pos
        
        print(f"Device with id {id} does not exist")
        return Vector3(0, 0, 0)

    # updates hearbeat and turns device status online
    def update_last_time(self, id: str, new_time: float):
        self._devices[id].last_heartbeat = new_time

        # if device comes ONLINE, rewrite json so dashboard reflects
        if (self._devices[id].device_status == DeviceStatus.OFFLINE):
            self._devices[id].device_status = DeviceStatus.ONLINE
            self.export_to_json("dashboard/updated_pos.json")

    def heartbeat_check(self, id: str, timeout: float) -> bool:
        """Turn device offline if not heard from within timeout seconds."""
        device = self._devices[id]

        if (time() - device.last_heartbeat) >= timeout:
            if (device.device_status == DeviceStatus.ONLINE): # only set inactive when device previously active
                print(f"Device {id} is inactive")
                device.device_status = DeviceStatus.OFFLINE
                return True

        return False

    def check_all(self, timeout: float):
        dropped = False
        for id in list(self._devices):
            dropped = self.heartbeat_check(id, timeout) or dropped

        # Re-export so the dashboard stops drawing a device that timed out.
        if dropped:
            self.export_to_json("dashboard/updated_pos.json")
