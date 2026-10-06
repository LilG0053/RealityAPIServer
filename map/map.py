from typing import Iterator
from uuid import UUID
from map.device import Device, DeviceType
from realityapi_pb2 import Vector3


class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}

    def add_device(self, id: str, device_type: DeviceType, pos: Vector3):
        self._devices[id] = Device(id=id, device_type=device_type, pos=pos)
        print(f"Added device: {self._devices[id]} to dict")

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
                self._devices[device_data["id"]] = Device(
                    id=device_data["id"],
                    device_type=device_type,
                    pos=pos
                )
            print(f"Loaded {len(devices_data)} devices from {filepath}")
        except FileNotFoundError:
            print(f"No existing JSON file at {filepath}, starting with empty map")
        except Exception as e:
            print(f"Error loading from JSON: {e}")

    def set_device_position(self, id: str, pos: Vector3, device_type: DeviceType):
        if id in self._devices: # if id of robot exists, set new position
            device = self._devices[id]
            device.pos = pos
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

        