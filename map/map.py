from pathlib import Path
from time import time
from typing import Any

from map.device.device import Device
from map.device.device2d import Device2D
from map.device.dog import Dog
from map.device.turtlebot import TurtleBot
from map.occupancy import OccupancyGrid
from map.room import BLOCKOUT_PATH, Room
from map.wallgrid import WallGrid
from realityapi_pb2 import AllDeviceInfo, DeviceInfo, DeviceType, Lidar2, Response, Vector2

# Where the TurtleBots' shared occupancy grid is kept between server runs.
TURTLEBOT_OCCUPANCY_PATH = Path(__file__).resolve().parents[1] / "assets" / "occupancy_grid_turtlebot.json"

# The class for each device type a client can register as.
_DEVICE_CLASSES: dict[int, type[Device]] = {
    DeviceType.TURTLE: TurtleBot,
    DeviceType.DOG: Dog,
}


class Map:
    def __init__(self, blockout_path: str | Path = BLOCKOUT_PATH) -> None:
        self._devices: dict[str, Device] = {}
        self._last_map_size = -1
        self._room = Room.from_json(blockout_path)
        # The wall grid each device type localizes against: the shared room and,
        # through it, the occupancy grid that type maps into. Devices of a type
        # share theirs; dashboards fetch the occupancy grid per type.
        self.wall_grids: dict[int, WallGrid] = {
            DeviceType.TURTLE: WallGrid(self._room, OccupancyGrid(self._room, TURTLEBOT_OCCUPANCY_PATH)),
        }

    @property
    def room(self) -> Room:
        """The room loaded from the blockout."""
        return self._room

    def _device_kwargs(self, cls: type[Device], device_type: int) -> dict[str, Any]:
        """What a device class's constructor needs beyond its id."""
        # 2D devices localize against their type's shared wall grid.
        if issubclass(cls, Device2D):
            return {"wall_grid": self.wall_grids[device_type]}
        return {}

    def _new_device(self, device_info: DeviceInfo) -> Device | None:
        cls = _DEVICE_CLASSES.get(device_info.device_type)
        if cls is None:
            return None
        return cls.from_packet(device_info, **self._device_kwargs(cls, device_info.device_type))

    def occupancy_grids(self) -> list[OccupancyGrid]:
        """Every device type's occupancy grid, once each."""
        grids = []
        for wall_grid in self.wall_grids.values():
            if wall_grid.occupancy is not None and wall_grid.occupancy not in grids:
                grids.append(wall_grid.occupancy)
        return grids

    def get_room_bounds(self) -> Response:
        """The room every device shares."""
        return Response(room_bounds=self.room.bounds())

    def get_occupancy_grid(self, device_type: int, since_revision: int) -> Response:
        """A device type's occupancy grid: cells changed after since_revision (see OccupancyGrid.snapshot)."""
        wall_grid = self.wall_grids.get(device_type)
        if wall_grid is None or wall_grid.occupancy is None:
            return Response(err_not_supported={})
        return Response(occupancy_grid=wall_grid.occupancy.snapshot(since_revision))

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
        """Take a post_device_info. Everything in it is in server coordinates."""
        device = self._devices.get(info.id)
        if device is None:
            device = self._new_device(info)
            if device is None:
                return Response(err_not_supported={})
            self.add_device(device)

        return device.handle_device_info(info)

    def handle_heartbeat(self, device_id: str) -> Response:
        if device_id not in self._devices:
            return Response(err_id_not_found={})

        self._devices[device_id].heartbeat()
        return Response(ok={})

    def get_device_info(self, id: str) -> Response:
        if id not in self._devices:
            print(f"Device with id {id} does not exist")
            return Response(err_id_not_found={})

        device = self._devices[id]
        response = Response(device_info=device.get_device_info())
        # A 2D device asks for its own info to take on its room pose; from its
        # next post on, it reports in that frame.
        if isinstance(device, Device2D):
            device.handover()
        return response

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
        return Response(ok={})

    def get_lidar_data(self) -> Response:
        # Every device's accumulated map: server-frame X/Z points, with
        # Vector2.y carrying Z.
        devices = self._mapping_devices()
        points = []
        for device in devices:
            accumulated = device.accumulated_points()
            if accumulated is not None:
                points.extend(Vector2(x=float(x), y=float(z)) for x, z in accumulated)
        # Log only when the size changes, so an empty map is visible in the terminal.
        if len(points) != self._last_map_size:
            self._last_map_size = len(points)
            print(f"accumulated map: {len(points)} points from {len(devices)} device(s)")
        return Response(lidar_data=Lidar2(points=points))
