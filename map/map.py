from uuid import UUID
from time import time
from map.device import Device, device_from_info, device_from_dict
from realityapi_pb2 import AllDeviceInfo, DeviceInfo, Lidar, Response, Transform, Vector3
import json, os
from map.localization import ScanLocalizer
import numpy as np
from map.lidar import LIDAR_EPSILON, contour_iou, simplified_contour, simplify_lidar_2d

# A new keyframe this close to any saved contour is treated as a loop closure.
LOOP_IOU_THRESHOLD = 0.9

class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}
        self._localizers: dict[str, ScanLocalizer] = {}
        # Latest scan before the localizer's correction, kept for the test keyframe button.
        self._unadjusted_scans: dict[str, Lidar] = {}
        self._latest_unadjusted_id: str | None = None

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

    def handle_lidar_scan(self, device_id: str, transform: Transform, lidar: Lidar) -> Response:
        # A scan can update an existing device but cannot create one by itself.
        if not self.exists(device_id):
            return Response(err_id_not_found={})

        # Snapshot the scan before update() applies the localization correction.
        snapshot = Lidar()
        snapshot.CopyFrom(lidar)
        self._unadjusted_scans[device_id] = snapshot
        self._latest_unadjusted_id = device_id

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

    def save_lidar_keyframe(self, directory: str = "dashboard/keyframes") -> Response:
        """Write the simplified contour of the latest scan before correction."""
        device_id = self._latest_unadjusted_id
        lidar = self._unadjusted_scans.get(device_id) if device_id else None
        if lidar is None or len(lidar.points) < 3:
            return Response(err_cannot_execute={})

        points = np.array([[point.x, point.z] for point in lidar.points], dtype=np.float32)
        contour = simplified_contour(points, LIDAR_EPSILON).reshape(-1, 2)
        if len(contour) < 3:
            return Response(err_cannot_execute={})

        # Compare before writing so the new file is not matched against itself.
        ious = self.iou_with_saved_keyframes(contour, directory)
        loop_enclosed = any(iou > LOOP_IOU_THRESHOLD for iou in ious)

        os.makedirs(directory, exist_ok=True)
        safe_id = device_id.replace("/", "_")
        filename = f"{safe_id}-{time():.3f}.json"
        payload = {
            "id": device_id,
            "contour": [{"x": float(x), "z": float(z)} for x, z in contour],
        }
        tmp_path = os.path.join(directory, filename + ".tmp")
        path = os.path.join(directory, filename)
        with open(tmp_path, "w") as f:
            json.dump(payload, f, indent=2)
        os.replace(tmp_path, path)
        print(f"saved lidar keyframe {path}")
        if loop_enclosed:
            print(f"loop enclosed (max iou {max(ious):.3f})")
            return Response(loop_enclosed={})
        return Response(ok={})

    def iou_with_saved_keyframes(
        self,
        contour: np.ndarray,
        directory: str = "dashboard/keyframes",
    ) -> list[float]:
        """IoU of one contour against every keyframe already saved in directory."""
        keyframe_dir = os.path.join(directory)
        if not os.path.isdir(keyframe_dir):
            return []

        ious = []
        for filename in sorted(os.listdir(keyframe_dir)):
            if not filename.endswith(".json"):
                continue
            try:
                with open(os.path.join(keyframe_dir, filename)) as f:
                    saved = json.load(f)
                other = np.array(
                    [[point["x"], point["z"]] for point in saved["contour"]],
                    dtype=np.float32,
                )
            except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError):
                continue
            ious.append(contour_iou(contour, other))
        return ious

    def clear_accumulated_scans(self, directory: str = "dashboard/keyframes") -> Response:
        for localizer in self._localizers.values():
            localizer.clear_scan()
        if os.path.isdir(directory):
            for filename in os.listdir(directory):
                if filename.endswith(".json") or filename.endswith(".tmp"):
                    os.remove(os.path.join(directory, filename))
        return Response(ok={})

    def get_lidar_data(self) -> Response:
        # Every localizer's accumulated map. Those points only track the X/Z
        # ground plane, so they are sent at height 0.
        points = []
        for localizer in self._localizers.values():
            if localizer.accumulated_scan is not None:
                points.extend(Vector3(x=float(x), y=0.0, z=float(z)) for x, z in localizer.accumulated_scan)
        return Response(lidar_data=Lidar(points=points))
