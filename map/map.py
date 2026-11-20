import json
import math
import os
from collections import deque
from time import time

import numpy as np

from map.cloud_map_3d import CloudMap
from map.device import Device, Dog, TurtleBot, device_from_info, device_from_dict
from map.occupancy import OccupancyGrid  # <- use the exact import line main has for OccupancyGrid
from realityapi_pb2 import AllDeviceInfo, DeviceInfo, DeviceType, Lidar, Response, Transform, Vector3
import json, os
from map.localization import ScanLocalizer, apply
from map.static_localizer import StaticLocalizer
from map.lidar import simplify_lidar_2d, LIDAR_EPSILON

# File the visual dashboard polls.
DASHBOARD_PATH = "dashboard/updated_pos.json"

EXPORT_INTERVAL = 1.0

# Histogram samples kept in arrival order. Older values are dropped past this.
SCAN_RMSE_LIMIT = 2000
# Starting RMSE cutoff for the dog (metres). The dashboard slider changes it at
# runtime. A scan whose closest-point RMSE is above the cutoff is not merged, so
# it cannot move the correction. TurtleBot keeps TurtleBot.RMSE_LIMIT.
DOG_RMSE_LIMIT = 0.10
# Per-device RMSE history used to print stats when the limit changes.
RECENT_RMSE_COUNT = 200

# The most map points sent to the dashboard in one reply. A bigger 3D map is
# thinned with an even stride so the whole room stays covered.
MAX_MAP_POINTS_SENT = 30000

# Match the dog's ring against the room blockout (map/static_localizer.py). The
# correction then comes from the walls instead of from earlier scans, and no map
# is built until the dog has been located. False keeps the old behaviour.
DOG_STATIC_LOCALIZATION = True
# A whole-room fix that moves the dog farther than this (or turns it more) means
# the maps built so far are in the wrong place, so they are cleared.
RELOCATE_CLEAR_DISTANCE = 0.5
RELOCATE_CLEAR_TURN = math.radians(8.0)

class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}
        self.export_interval = EXPORT_INTERVAL
        self._last_export = 0.0
        self._localizers: dict[str, ScanLocalizer] = {}
        self._last_map_size = -1
        # Finite closest-point RMSE of each scan after the first, oldest first.
        self.scan_rmses: list[float] = []
        self.occupancy = OccupancyGrid()
        # Runtime cutoffs by device type; they win over the class RMSE_LIMIT.
        self._rmse_overrides: dict[int, float] = {DeviceType.DOG: DOG_RMSE_LIMIT}
        self._recent_rmse: dict[str, deque] = {}
        # 3D maps (the dog), kept apart from the 2D localizers above. A device only
        # has one if it sends point clouds. The Vector3 form of each is cached by
        # version so an unchanged map is not rebuilt on every dashboard poll.
        self._cloud_maps: dict[str, CloudMap] = {}
        self._cloud_points_cache: dict[str, tuple[int, list[Vector3]]] = {}
        # Blockout matchers (the dog), built on first use. None = unavailable.
        self._static_localizers: dict[str, StaticLocalizer] = {}
        self._static_failed = not DOG_STATIC_LOCALIZATION

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
            self.occupancy.export_pending_updates()

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

    def handle_lidar_scan(
        self,
        device_id: str,
        transform: Transform,
        lidar: Lidar,
        cloud: np.ndarray | None = None,
        evidence: np.ndarray | None = None,
    ) -> Response:
        # A scan can update an existing device but cannot create one by itself.
        if not self.exists(device_id):
            return Response(err_id_not_found={})

        device = self._devices[device_id]
        # TurtleBot only keeps the first N scans while parked; past that, pose
        # still updates but ICP / occupancy / live lidar are left alone.
        if isinstance(device, TurtleBot) and not device.allow_lidar_scan(transform):
            return self.handle_device_info(DeviceInfo(id=device_id, transform=transform))

        localizer = self._localizers.setdefault(device_id, ScanLocalizer())
        static = self._static_localizer_for(device)
        if static is not None and not self._anchor_to_room(device_id, localizer, static, transform, lidar):
            # Not located in the room yet. The pose and ring are still reported,
            # but nothing is added to the maps until the dog knows where it is.
            return self.handle_device_info(DeviceInfo(
                id=device_id,
                transform=transform,
                lidar=simplify_lidar_2d(localizer.correct_lidar(lidar), epsilon=LIDAR_EPSILON),
            ))
        had_map = localizer.accumulated_scan is not None
        accepted = localizer.update(lidar, rmse_limit=self.rmse_limit_for(device_id))
        if localizer.last_rmse is not None:
            self.record_scan_rmse(localizer.last_rmse)
            self._recent_rmse.setdefault(
                device_id, deque(maxlen=RECENT_RMSE_COUNT)
            ).append(localizer.last_rmse)
        corrected = localizer.correct_lidar(lidar)
        # The first scan is stored with no RMSE. Later scans enter only when
        # update() accepts them. A rejected scan stays at the predicted pose
        # and must not change the grid.
        kept = accepted or (not had_map and localizer.accumulated_scan is not None)
        if kept and corrected.points:
            pose = localizer.correct_transform(transform)
            points = np.array([[point.x, point.y, point.z] for point in corrected.points])
            self.occupancy.update_dashboard(pose, points)
        # The optional 3D cloud (N x 3, server axes) shares this scan's correction,
        # so it is only added when the scan itself was kept.
        if cloud is not None and kept:
            p = transform.position
            self._cloud_maps.setdefault(device_id, CloudMap()).add(
                cloud, localizer.correction, np.array([p.x, p.y, p.z], dtype=float), evidence
            )
        return self.handle_device_info(DeviceInfo(
            id=device_id,
            transform=transform,
            lidar=simplify_lidar_2d(corrected, epsilon=LIDAR_EPSILON),
        ))
    
    def _static_localizer_for(self, device: Device) -> StaticLocalizer | None:
        """The blockout matcher for this device, or None (not a dog, or unavailable)."""
        if self._static_failed or not isinstance(device, Dog):
            return None
        static = self._static_localizers.get(device.id)
        if static is None:
            try:
                shared = next(iter(self._static_localizers.values()), None)
                static = StaticLocalizer(shared.wall_grid if shared else None)
            except Exception as error:  # missing blockout.json, bad import, ...
                print(f"[static] blockout localization unavailable, using scan matching only: {error}")
                self._static_failed = True
                return None
            self._static_localizers[device.id] = static
        return static

    def _anchor_to_room(
        self,
        device_id: str,
        localizer: ScanLocalizer,
        static: StaticLocalizer,
        transform: Transform,
        lidar: Lidar,
    ) -> bool:
        """Fold a blockout fix into the correction. True once the device is located."""
        if lidar.points:
            ring = apply(localizer.correction, np.array([[p.x, p.z] for p in lidar.points]))
            position = apply(
                localizer.correction,
                np.array([[transform.position.x, transform.position.z]]),
            )[0]
            fix = static.locate(ring, position)
            if fix is not None:
                # Maps built before this fix sit in the old frame. After a big
                # relocation they would smear, so start them over. (The very first
                # fix needs no clearing: nothing is built before it.)
                if fix.global_search and (
                    fix.shift > RELOCATE_CLEAR_DISTANCE or abs(fix.turn) > RELOCATE_CLEAR_TURN
                ):
                    localizer.accumulated_scan = None
                    cloud_map = self._cloud_maps.get(device_id)
                    if cloud_map is not None:
                        cloud_map.clear()
                localizer.correction = fix.delta @ localizer.correction
                kind = "global" if fix.global_search else "track"
                if fix.global_search:
                    print(
                        f"[static] {device_id} located ({kind}): score {fix.score:.2f}, "
                        f"moved {fix.shift:.2f} m, turned {math.degrees(fix.turn):.1f} deg"
                    )
        return static.located

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

    def record_scan_rmse(self, rmse: float) -> None:
        """Keep at most SCAN_RMSE_LIMIT samples, dropping the oldest first."""
        self.scan_rmses.append(rmse)
        overflow = len(self.scan_rmses) - SCAN_RMSE_LIMIT
        if overflow > 0:
            del self.scan_rmses[:overflow]

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
        # The correction was just reset, so the dog has to find itself in the room again.
        for static in self._static_localizers.values():
            static.reset()
        for cloud_map in self._cloud_maps.values():
            cloud_map.clear()
        self.scan_rmses.clear()
        self.occupancy.clear()
        for device in self._devices.values():
            if isinstance(device, TurtleBot):
                device.reset_stationary_scans()

        return Response(ok={})

    def _cloud_map_points(self, device_id: str, cloud_map: CloudMap) -> list[Vector3]:
        cached = self._cloud_points_cache.get(device_id)
        if cached is None or cached[0] != cloud_map.version:
            cloud = cloud_map.points
            if len(cloud) > MAX_MAP_POINTS_SENT:
                cloud = cloud[:: -(-len(cloud) // MAX_MAP_POINTS_SENT)]  # ceil division
            points = [Vector3(x=float(x), y=float(y), z=float(z)) for x, y, z in cloud]
            cached = self._cloud_points_cache[device_id] = (cloud_map.version, points)
        return cached[1]

    def get_lidar_data(self) -> Response:
        # Every device's accumulated map. A device with a 3D map (the dog) sends it
        # with real heights; the others send their flat 2D map at height 0, as before.
        points = []
        for device_id, localizer in self._localizers.items():
            cloud_map = self._cloud_maps.get(device_id)
            if cloud_map is not None and cloud_map.points is not None:
                points.extend(self._cloud_map_points(device_id, cloud_map))
            elif localizer.accumulated_scan is not None:
                points.extend(
                    Vector3(x=float(x), y=0.0, z=float(z))
                    for x, z in localizer.accumulated_scan
                )
        # Log only when the size changes, so an empty map is visible in the terminal.
        if len(points) != self._last_map_size:
            self._last_map_size = len(points)
            print(f"accumulated map: {len(points)} points from {len(self._localizers)} localizer(s)")
        return Response(lidar_data=Lidar(points=points))