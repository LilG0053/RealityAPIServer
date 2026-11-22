from pathlib import Path
from time import time
from typing import Any

import numpy as np

from map.device.device import Device
from map.device.device2d import Device2D
from map.device.dog import Dog
from map.device.turtlebot import TurtleBot
from map.cloud_map_3d import CloudMap
from map.algorithms.pointcloud import raw_cloud_to_evidence, raw_cloud_to_points
from map.occupancy import OccupancyGrid
from map.room import BLOCKOUT_PATH, Room
from map.wallgrid import WallGrid
from realityapi_pb2 import (
    AllDeviceInfo,
    DeviceInfo,
    DeviceType,
    Lidar3,
    Response,
    Vector3,
)

# Where the TurtleBots' shared occupancy grid is kept between server runs.
TURTLEBOT_OCCUPANCY_PATH = Path(__file__).resolve().parents[1] / "assets" / "occupancy_grid_turtlebot.json"
# The dog's occupancy grid.
DOG_OCCUPANCY_PATH = Path(__file__).resolve().parents[1] / "assets" / "occupancy_grid_dog.json"

# The class for each device type a client can register as.
_DEVICE_CLASSES: dict[int, type[Device]] = {
    DeviceType.TURTLE: TurtleBot,
    DeviceType.DOG: Dog,
}


class Map:
    def __init__(self, blockout_path: str | Path = BLOCKOUT_PATH) -> None:
        self._devices: dict[str, Device] = {}
        self._last_map_size = -1
        self._cloud_maps: dict[str, CloudMap] = {}
        self._cloud_points_cache: dict[str, tuple[int, list[Vector3]]] = {}
        self._last_map_size3d = -1
        self._room = Room.from_json(blockout_path)
        # One occupancy grid per device type; devices of a type share theirs and
        # dashboards fetch them per type.
        self.occupancy: dict[int, OccupancyGrid] = {
            DeviceType.TURTLE: OccupancyGrid(self._room, TURTLEBOT_OCCUPANCY_PATH),
            DeviceType.DOG: OccupancyGrid(self._room, DOG_OCCUPANCY_PATH),
        }
        # The wall grid each device type localizes against. The TurtleBots blend
        # their occupancy grid in; the dog localizes against the room walls only,
        # so its occupancy grid is drawn but never fed back into its fixes.
        self.wall_grids: dict[int, WallGrid] = {
            DeviceType.TURTLE: WallGrid(self._room, self.occupancy[DeviceType.TURTLE]),
            DeviceType.DOG: WallGrid(self._room),
        }

    @property
    def room(self) -> Room:
        """The room loaded from the blockout."""
        return self._room

    def _device_kwargs(self, cls: type[Device], device_type: int) -> dict[str, Any]:
        """What a device class's constructor needs beyond its id."""
        if issubclass(cls, Device2D):
            return {"wall_grid": self.wall_grids[device_type]}
        if issubclass(cls, Dog):
            return {"wall_grid": self.wall_grids[device_type], "occupancy": self.occupancy[device_type]}
        return {}

    def _new_device(self, device_info: DeviceInfo) -> Device | None:
        cls = _DEVICE_CLASSES.get(device_info.device_type)
        if cls is None:
            return None
        return cls.from_packet(device_info, **self._device_kwargs(cls, device_info.device_type))

    def occupancy_grids(self) -> list[OccupancyGrid]:
        """Every device type's occupancy grid, once each."""
        return list(self.occupancy.values())

    def get_room_bounds(self) -> Response:
        """The room every device shares."""
        return Response(room_bounds=self.room.bounds())

    def get_occupancy_grid(self, device_type: int, since_revision: int) -> Response:
        """A device type's occupancy grid: cells changed after since_revision (see OccupancyGrid.snapshot)."""
        grid = self.occupancy.get(device_type)
        if grid is None:
            return Response(err_not_supported={})
        return Response(occupancy_grid=grid.snapshot(since_revision))

    @property
    def scan_rmses(self) -> list[float]:
        """Every 2D device's recorded scan RMSEs, for the dashboard histogram."""
        return [rmse for device in self._devices_2d() for rmse in device.scan_rmses]

    def _devices_2d(self) -> list[Device2D]:
        return [device for device in self._devices.values() if isinstance(device, Device2D)]

    def _mapping_devices(self) -> list[Device2D | Dog]:
        """Devices that keep an accumulated map."""
        return [device for device in self._devices.values() if isinstance(device, (Device2D, Dog))]

    def add_device(self, device: Device) -> None:
        self._devices[device.id] = device
        print(f"Added device: {device.id}")

    def exists(self, id: str) -> bool:
        return id in self._devices

    def handle_device_info(self, info: DeviceInfo) -> Response:
        """Take a post_device_info. Everything in it is in server coordinates.

        The reply is the device's DeviceInfo as the server sees it, which the
        device takes on.
        """
        device = self._devices.get(info.id)
        if device is None:
            device = self._new_device(info)
            if device is None:
                return Response(err_not_supported={})
            self.add_device(device)

        response = device.handle_device_info(info)
        if isinstance(device, Dog) and info.HasField("raw_point_cloud"):
            raw = info.raw_point_cloud
            p = raw.transform.position
            cloud_map = self._cloud_maps.setdefault(info.id, CloudMap())
            if device.reset_cloud:
                # A whole-room fix moved the dog's frame; older points no longer line up.
                device.reset_cloud = False
                cloud_map.clear()
            # Server axes (x, y up, z) throughout, which is what CloudMap takes.
            cloud_map.add(
                raw_cloud_to_points(raw),
                device.localizer.correction,
                np.array([p.x, p.y, p.z], dtype=np.float64),
                raw_cloud_to_evidence(raw),
            )
        if device.id != info.id:
            # Renamed: the reply tells the device its new id.
            self._devices[device.id] = self._devices.pop(info.id)
        return response

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
        devices = [device.get_device_info() for device in self._devices.values()]
        return Response(all_device_info=AllDeviceInfo(devices=devices))

    def del_device(self, id: str) -> Device:
        return self._devices.pop(id)

    def heartbeat_check(self, id: str, timeout: float) -> bool:
        """Drop a device that has not been heard from within timeout seconds."""
        if time() - self._devices[id].timestamp >= timeout:
            print(f"Device {id} is inactive")
            self.del_device(id)
            return True
        return False

    def check_all(self, timeout: float) -> None:
        for id in list(self._devices):
            self.heartbeat_check(id, timeout)

    def clear_accumulated_scan(self) -> Response:
        for device in self._mapping_devices():
            device.clear_scan()
        for grid in self.occupancy_grids():
            grid.clear()
        for cloud_map in self._cloud_maps.values():
            cloud_map.clear()
        self._cloud_points_cache.clear()
        return Response(ok={})

    def get_lidar_data(self) -> Response:
        # 3D-capable devices send real heights; 2D devices remain at y=0.
        devices = self._mapping_devices()
        points3d = []
        for device in devices:
            cloud_map = self._cloud_maps.get(device.id)
            if cloud_map is not None and cloud_map.points is not None:
                cached = self._cloud_points_cache.get(device.id)
                if cached is None or cached[0] != cloud_map.version:
                    cloud = cloud_map.points
                    if len(cloud) > 30000:
                        cloud = cloud[:: -(-len(cloud) // 30000)]
                    cloud_points = [Vector3(x=float(x), y=float(y), z=float(z)) for x, y, z in cloud]
                    points3d.extend(cloud_points)
                    self._cloud_points_cache[device.id] = (cloud_map.version, cloud_points)
                else:
                    points3d.extend(cached[1])
                continue
            accumulated = device.accumulated_points()
            if accumulated is not None:
                points3d.extend(Vector3(x=float(x), y=0.0, z=float(z)) for x, z in accumulated)
        # Log only when the size changes, so an empty map is visible in the terminal.
        if len(points3d) != self._last_map_size3d:
            self._last_map_size3d = len(points3d)
            print(f"accumulated map: {len(points3d)} points from {len(devices)} device(s)")
        return Response(lidar_data=Lidar3(points=points3d))