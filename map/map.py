from uuid import UUID
from time import time
from map.device import Device, device_from_info, device_from_dict
from realityapi_pb2 import AllDeviceInfo, DeviceInfo, Lidar, Response
import json, os

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

        # Write to a temp file, then swap it in, so the dashboard never
        # reads a half-written file.
        tmp_path = filepath + ".tmp"
        with open(tmp_path, 'w') as f:
            json.dump(devices_data, f, indent=2)
        os.replace(tmp_path, filepath)

    def load_from_json(self, filepath: str):
        try:
            with open(filepath, 'r') as f:
                devices_data = json.load(f)

            devices_loaded = 0
            for device_data in devices_data:
                device = device_from_dict(device_data)
                if device is None:
                    print(f"Failed to load device {device_data.get('id')} (may be in wrong format)")
                    continue
                self.add_device(device)
                devices_loaded += 1
            print(f"Loaded {devices_loaded} devices from {filepath}")
        except FileNotFoundError:
            print(f"No existing JSON file at {filepath}, starting with empty map")
        except Exception as e:
            print(f"Error loading from JSON: {e}")

    def handle_device_info(self, device_info: DeviceInfo) -> Response:
        if device_info.id in self._devices: # if id of robot exists, set new position
            device = self._devices[device_info.id]
            device.handle_device_info(device_info)
            print(f"Set position for device: {device.id}")
        else: # if robot id does not exist, add it as a device
            device = device_from_info(device_info)
            if device is None:
                return Response(err_not_supported={})
            self.add_device(device)

        # A position report is also proof the device is still alive. Use the
        # server clock: the float32 wire timestamp is only accurate to ~128s.
        device.timestamp = time()

        # exports file to dashboard folder which updates the visual dashboard
        self.export_to_json("dashboard/updated_pos.json")

        return Response(ok={})

    def handle_heartbeat(self, device_id: str) -> Response:
        if not device_id in self._devices:
            return Response(err_id_not_found={})

        self._devices[device_id].heartbeat()

        return Response(ok={})

    def get_device_info(self, id: str) -> Response:
        if not id in self._devices:
            print(f"Device with id {id} does not exist")
            return Response(err_id_not_found={})

        dev = self._devices[id]
        return Response(device_info=dev.get_device_info())

    def get_all_device_info(self) -> Response:
        devices = [dev.get_device_info() for dev in self._devices.values()]
        return Response(all_device_info=AllDeviceInfo(devices=devices))

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

    def get_lidar_data(self) -> Response:
        points = []
        for device in self._devices.values():
            if hasattr(device, "lidar"):
                points.extend(device.lidar.points)
        return Response(lidar_data=Lidar(points=points))
