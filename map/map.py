import json
import os
from time import time
from map.device import Device, device_from_info, device_from_dict
from realityapi_pb2 import AllDeviceInfo, DeviceInfo, Lidar, Response, Transform, Vector3
import json, os
from map.localization import ScanLocalizer
from map.lidar import simplify_lidar_2d, LIDAR_EPSILON

# File the visual dashboard polls.
DASHBOARD_PATH = "dashboard/updated_pos.json"

EXPORT_INTERVAL = 1.0

class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}
        self.export_interval = EXPORT_INTERVAL
        self._last_export = 0.0
        self._localizers: dict[str, ScanLocalizer] = {}
        self._last_map_size = -1

    def add_device(self, device: Device) -> None:
        self._devices[device.id] = device
        print(f"Added device: {device.id}")

    def exists(self, id: str) -> bool:
        return id in self._devices

    # function that defines what values are sent to updated_pos.json
    def export_to_json(self, filepath: str = DASHBOARD_PATH) -> None:
        devices_data = [device.to_dict() for device in self._devices.values()]

        # Write to a temp file, then swap it in, so the dashboard never
        # reads a half-written file.
        # json.dumps without indent uses the C encoder; switched from Python's json.dump to increase speed.
        tmp_path = filepath + ".tmp"
        try:
            with open(tmp_path, "w") as f:
                f.write(json.dumps(devices_data))
            os.replace(tmp_path, filepath)
        except OSError as e:
            # Windows/OneDrive can briefly lock the file. Skip this write (the
            # next one rewrites it) instead of crashing the connection.
            print(f"[!] could not write {filepath}: {e}")
        self._last_export = time()

    def _export_if_due(self) -> None:
        if time() - self._last_export >= self.export_interval:
            self.export_to_json()

    def load_from_json(self, filepath: str = DASHBOARD_PATH) -> None:
        try:
            with open(filepath, "r") as f:
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
        except Exception as error:
            print(f"Error loading from JSON: {error}")

    def handle_lidar_scan(self, device_id: str, transform: Transform, lidar: Lidar) -> Response:
        # A scan can update an existing device but cannot create one by itself.
        if not self.exists(device_id):
            return Response(err_id_not_found={})

        localizer = self._localizers.setdefault(device_id, ScanLocalizer())
        localizer.update(lidar)
        return self.handle_device_info(DeviceInfo(
            id=device_id,
            transform=transform,
            lidar=simplify_lidar_2d(localizer.correct_lidar(lidar), epsilon=LIDAR_EPSILON),
        ))
    
    def handle_device_info(self, device_info: DeviceInfo) -> Response:
        # Every pose, from /odom or a scan, gets the correction exactly once here
        localizer = self._localizers.get(device_info.id)
        if localizer is not None and device_info.HasField("transform"):
            device_info.transform.CopyFrom(localizer.correct_transform(device_info.transform))
        if device_info.id in self._devices: # if id of robot exists, set new position
            device = self._devices[device_info.id]
            device.handle_device_info(device_info)
        else:
            device = device_from_info(device_info)
            if device is None:
                return Response(err_not_supported={})
            self.add_device(device)

        device.timestamp = time()

        # The dashboard gets its data over the websocket; this file is only the
        # saved state reloaded at startup, so it does not need every update.
        self._export_if_due()

        return Response(ok={})

    def handle_heartbeat(self, device_id: str) -> Response:
        if device_id not in self._devices:
            return Response(err_id_not_found={})

        self._devices[device_id].heartbeat()
        return Response(ok={})

    def get_device_info(self, id: str) -> Response:
        if id not in self._devices:
            print(f"Device with id {id} does not exist")
            return Response(err_id_not_found={})

        return Response(device_info=self._devices[id].get_device_info())

    def get_all_device_info(self) -> Response:
        devices = [dev.get_device_info() for dev in self._devices.values()]
        return Response(all_device_info=AllDeviceInfo(devices=devices))

    def del_device(self, id: str) -> Device:
        return self._devices.pop(id)

    def update_last_time(self, id: str, new_time: float) -> None:
        self._devices[id].timestamp = new_time

    def heartbeat_check(self, id: str, timeout: float) -> bool:
        """Drop a device that has not been heard from within timeout seconds."""
        if time() - self._devices[id].timestamp >= timeout:
            print(f"Device {id} is inactive")
            self.del_device(id)
            return True
        return False

    def check_all(self, timeout: float) -> None:
        dropped = False
        for id in list(self._devices):
            dropped = self.heartbeat_check(id, timeout) or dropped

        # Re-export so the dashboard stops drawing a device that timed out.
        if dropped:
            self.export_to_json()

    def clear_accumulated_scan(self) -> Response:
        for localizer in self._localizers.values():
            localizer.clear_scan()
            
        return Response(ok={})

    def get_lidar_data(self) -> Response:
        # Every localizer's accumulated map. Those points only track the X/Z
        # ground plane, so they are sent at height 0.
        points = []
        for localizer in self._localizers.values():
            if localizer.accumulated_scan is not None:
                points.extend(
                    Vector3(x=float(x), y=0.0, z=float(z))
                    for x, z in localizer.accumulated_scan
                )
        # Log only when the size changes, so an empty map is visible in the terminal.
        if len(points) != self._last_map_size:
            self._last_map_size = len(points)
            print(f"accumulated map: {len(points)} points from {len(self._localizers)} localizer(s)")
        return Response(lidar_data=Lidar(points=points))