import json
import math
import os
from collections import deque
from time import time
from map.device import Device, device_from_info, device_from_dict
from realityapi_pb2 import AllDeviceInfo, DeviceInfo, DeviceType, Lidar, Response, Transform, Vector3
import json, os
from map.localization import ScanLocalizer
from map.lidar import simplify_lidar_2d, LIDAR_EPSILON

# File the visual dashboard polls.
DASHBOARD_PATH = "dashboard/updated_pos.json"

EXPORT_INTERVAL = 1.0

# Starting RMSE cutoff for the dog (metres). The dashboard slider changes it at
# runtime. A scan whose closest-point RMSE is above the cutoff is not merged, so
# it cannot move the correction. TurtleBot keeps TurtleBot.RMSE_LIMIT.
DOG_RMSE_LIMIT = 0.10
# Per-device RMSE history used to print stats when the limit changes.
RECENT_RMSE_COUNT = 200

class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}
        self.export_interval = EXPORT_INTERVAL
        self._last_export = 0.0
        self._localizers: dict[str, ScanLocalizer] = {}
        self._last_map_size = -1
        # Runtime cutoffs by device type; they win over the class RMSE_LIMIT.
        self._rmse_overrides: dict[int, float] = {DeviceType.DOG: DOG_RMSE_LIMIT}
        self._recent_rmse: dict[str, deque] = {}

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
        with open(tmp_path, "w") as f:
            f.write(json.dumps(devices_data))
        os.replace(tmp_path, filepath)
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
        localizer.update(lidar, rmse_limit=self.rmse_limit_for(device_id))
        if localizer.last_rmse is not None:
            self._recent_rmse.setdefault(
                device_id, deque(maxlen=RECENT_RMSE_COUNT)
            ).append(localizer.last_rmse)
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

    def rmse_limit_for(self, device_id: str) -> float:
        """The runtime cutoff for the device's type, else the device class default."""
        device = self._devices[device_id]
        return self._rmse_overrides.get(device.device_type, device.RMSE_LIMIT)

    def set_rmse_limit(self, device_type: int, limit: float) -> Response:
        """Change the cutoff for one device type. limit <= 0 or inf turns it off."""
        if not math.isfinite(limit) or limit <= 0:
            limit = math.inf
        self._rmse_overrides[device_type] = limit

        # Show how the new cutoff would have treated the latest scans, so the
        # slider can be tuned against numbers rather than by eye.
        samples = sorted(
            r
            for device_id, values in self._recent_rmse.items()
            if device_id in self._devices and self._devices[device_id].device_type == device_type
            for r in values
        )
        name = DeviceType.Name(device_type)
        if samples:
            median = samples[len(samples) // 2]
            p90 = samples[int(len(samples) * 0.9)]
            rejected = sum(r > limit for r in samples) / len(samples)
            print(
                f"[rmse] {name} limit={limit:.3f} m | last {len(samples)} scans: "
                f"median={median:.3f} p90={p90:.3f} max={samples[-1]:.3f} | "
                f"{rejected:.0%} would be rejected"
            )
        else:
            print(f"[rmse] {name} limit={limit:.3f} m (no scans recorded yet)")
        return Response(ok={})

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