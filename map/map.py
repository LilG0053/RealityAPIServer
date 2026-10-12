from typing import Iterator
from uuid import UUID
from time import time
from map.device import Device, DeviceType, TurtleBot, device_from_packet, device_from_dict
from realityapi_pb2 import ServerBoundPacket, Vector3


class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}

    def add_device(self, device: Device):
        self._devices[device.id] = device
        print(f"Added device: {device} to dict")

    def exists(self, id: str) -> bool:
        return id in self._devices

    # function that defines what values are sent to updated_pos.json
    def export_to_json(self, filepath: str):
        devices_data = []

        for device in self._devices.values():
            print(f"{device}")
            devices_data.append(device.to_dict())

        with open(filepath, 'w') as f:
            import json
            json.dump(devices_data, f, indent=2)

    def load_from_json(self, filepath: str):
        import json
        try:
            with open(filepath, 'r') as f:
                devices_data = json.load(f)

            devices_loaded = 0
            for device_data in devices_data:
                device = device_from_dict(device_data)
                if device is None:
                    print(f"Failed to load device {device_data["id"]} (may be in wrong format)")
                    continue
                self.add_device(device)
                devices_loaded += 1
            print(f"Loaded {devices_loaded} devices from {filepath}")
        except FileNotFoundError:
            print(f"No existing JSON file at {filepath}, starting with empty map")
        except Exception as e:
            print(f"Error loading from JSON: {e}")

    def handle_info_packet(self, packet: ServerBoundPacket):
        if packet.id in self._devices: # if id of robot exists, set new position
            device = self._devices[packet.id]
            device.handle_packet(packet)
            # A position report is also proof the device is still alive.
            device.timestamp = time()
            print(f"Set position for device: {device.id}")
        else: # if robot id does not exist, add it as a device
            device = device_from_packet(packet)
            if device is None:
                return
            self.add_device(device)

        # exports file to dashboard folder which updates the visual dashboard
        self.export_to_json("dashboard/updated_pos.json")

    def get_device_type(self, id: str) -> DeviceType | None:
        if not id in self._devices:
            return None
        return self._devices[id].device_type()

    def get_turtlebot_pos(self, id: str) -> Vector3 | None:
        if not id in self._devices:
            print(f"Device with id {id} does not exist")
            return None

        dev = self._devices[id]
        if type(dev) is not TurtleBot:
            print(f"Device with id {id} is not a turtlebot")
            return None

        return dev.position

    def del_device(self, id: str) -> Device:
        return self._devices.pop(id)

    def update_last_time(self, id: str, new_time: float):
        self._devices[id].timestamp = new_time

    def heartbeat_check(self, id: str, timeout: float) -> bool:
        """Drop a device that has not been heard from within timeout seconds."""
        device = self._devices[id]

        if device.timestamp is not None and time() - device.timestamp >= timeout:
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
