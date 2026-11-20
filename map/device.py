"""Devices tracked by the server map."""

import math
from dataclasses import InitVar, dataclass, field
from time import time
from typing import Any, ClassVar

import numpy as np
from google.protobuf.json_format import MessageToDict, ParseDict

from map.coordinate import CoordinateSystem, Direction, convert_lidar, convert_transform
from map.tunable import TunableParameter
from realityapi_pb2 import DeviceInfo, DeviceType, Lidar, Transform3, Vector3


def zero_transform() -> Transform3:
    return Transform3(position=Vector3(), rotation=Vector3())


def _message_dict(message) -> dict[str, Any]:
    # Print zero-valued fields too, so a coordinate of exactly 0 still reaches
    # the dashboard JSON instead of being dropped.
    return MessageToDict(message, always_print_fields_with_no_presence=True)


@dataclass
class Device:
    # Scans whose closest-point RMSE exceeds this are not merged. Subclasses
    # that want a real cutoff override it; infinity accepts every finite RMSE.
    RMSE_LIMIT: ClassVar[float] = math.inf

    id: str
    device_type: int
    timestamp: float
    coordinate_system: CoordinateSystem | None = field(default=None, kw_only=True)

    def handle_device_info(self, device_info: DeviceInfo) -> None:
        # id and device_type are fixed when the device is created. Partial
        # updates (a raw lidar scan, for example) leave them unset, so copying
        # them here would silently turn a TurtleBot into device_type NONE.
        self.timestamp = device_info.timestamp

    def get_device_info(self, target_coords: CoordinateSystem | None = None) -> DeviceInfo:
        return DeviceInfo(id=self.id, device_type=self.device_type, timestamp=self.timestamp)

    def heartbeat(self) -> None:
        self.timestamp = time()

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "device_type": DeviceType.Name(self.device_type),
        }


@dataclass
class TransformDevice(Device):
    transform: Transform3
    lidar: Lidar
    initial_heading: InitVar[Direction] = Direction.NORTH

    # Set by each concrete subclass.
    DEVICE_TYPE: ClassVar[int] = DeviceType.NONE

    def __post_init__(self, initial_heading: Direction) -> None:
        if self.coordinate_system is None:
            self.coordinate_system = CoordinateSystem(
                x=initial_heading,
                y=initial_heading.counterclockwise90(),
                z=Direction.UP,
            )

    @classmethod
    def from_packet(cls, device_info: DeviceInfo) -> "TransformDevice":
        device = cls(
            id=device_info.id,
            device_type=cls.DEVICE_TYPE,
            timestamp=device_info.timestamp,
            transform=zero_transform(),
            lidar=Lidar(),
        )
        device.handle_device_info(device_info)
        return device

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TransformDevice":
        return cls(
            id=data["id"],
            device_type=cls.DEVICE_TYPE,
            timestamp=time(),
            transform=ParseDict(data.get("transform", {}), zero_transform()),
            lidar=ParseDict(data.get("lidar", {}), Lidar()),
        )

    def handle_device_info(self, device_info: DeviceInfo) -> None:
        super().handle_device_info(device_info)
        if device_info.HasField("transform"):
            self.transform.CopyFrom(device_info.transform)
        if device_info.HasField("lidar"):
            self.lidar.CopyFrom(device_info.lidar)

    def get_device_info(
        self,
        target_coords: CoordinateSystem | None = None,
        include_lidar: bool = True,
    ) -> DeviceInfo:
        info = super().get_device_info(target_coords)
        convert = target_coords is not None and self.coordinate_system is not None

        transform = self.transform
        if convert:
            transform = convert_transform(transform, self.coordinate_system, target_coords)
        info.transform.CopyFrom(transform)

        # Leave lidar unset so a pose-only post does not replace the stored scan.
        if include_lidar:
            lidar = self.lidar
            if convert:
                lidar = convert_lidar(lidar, self.coordinate_system, target_coords)
            info.lidar.CopyFrom(lidar)
        return info

    def to_dict(self) -> dict[str, Any]:
        ret = super().to_dict()
        ret["transform"] = _message_dict(self.transform)
        ret["lidar"] = _message_dict(self.lidar)
        return ret


class TurtleBot(TransformDevice):
    DEVICE_TYPE = DeviceType.TURTLE
    RMSE_LIMIT = TunableParameter(
        0.2, minimum=0.0, maximum=1.0, step=0.1, label="TurtleBot RMSE limit", unit="m",
    )
    # While parked, only this many lidar posts update the map / occupancy / live scan.
    stationary_scan_limit: ClassVar[int] = 5
    _STATIONARY_TRANSLATION_M: ClassVar[float] = 0.05
    _STATIONARY_YAW_RAD: ClassVar[float] = math.radians(5.0)

    # Seconds between branch-and-bound attempts while the robot is not placed.
    LOCALIZE_RETRY: ClassVar[float] = 1.0
    # Monte Carlo runs on a scan once odometry has moved this far since the last run.
    MC_MIN_TRANSLATION: ClassVar[float] = 0.05
    MC_MIN_ROTATION: ClassVar[float] = math.radians(5.0)
    # Fit (mean grid likelihood of the scan at the estimated pose) below this
    # on MC_POOR_FIT_UPDATES updates in a row reruns branch and bound.
    MC_POOR_FIT = TunableParameter(
        0.4, minimum=0.0, maximum=1.0, step=0.05, label="Relocalize below Monte Carlo fit",
    )
    MC_POOR_FIT_UPDATES: ClassVar[int] = 5
    # Odometry cannot move this far between two scans; it was reset or jumped.
    ODOM_JUMP_TRANSLATION: ClassVar[float] = 0.5
    ODOM_JUMP_ROTATION: ClassVar[float] = math.radians(60.0)
    # A relocalization that moves the pose further than this starts a new map
    # epoch, since scans merged at the old pose do not line up with the new one.
    RELOCALIZE_JUMP_TRANSLATION: ClassVar[float] = 0.5
    RELOCALIZE_JUMP_ROTATION: ClassVar[float] = math.radians(20.0)

    def __post_init__(self, initial_heading: Direction = Direction.NORTH) -> None:
        super().__post_init__(initial_heading)
        self.reset_stationary_scans()
        # Where this robot's odometry origin sits in the room: (x, z, yaw) in the
        # server frame. Branch and bound sets it from the first usable scan;
        # Monte Carlo updates it on every scan after the robot moves. None until then.
        self.room_origin: tuple[float, float, float] | None = None
        self.last_localize_attempt = -math.inf
        # Monte Carlo particle map, seeded from the branch-and-bound pose.
        self.particles = None
        self._last_mc_odom: tuple[float, float, float] | None = None
        self._last_scan_odom: tuple[float, float, float] | None = None
        self._poor_fits = 0
        self.needs_relocalization = False
        # Bumped when a relocalization moves the pose; the map drops scans
        # merged in an earlier epoch.
        self.map_epoch = 0

    @property
    def localized(self) -> bool:
        return self.room_origin is not None

    @staticmethod
    def odom_pose(transform: Transform3) -> tuple[float, float, float]:
        """(x, z, heading) of an odometry pose in server axes; heading from +X toward +Z."""
        return transform.position.x, transform.position.z, transform.rotation.y

    def room_pose(self, odom: tuple[float, float, float]) -> tuple[float, float, float]:
        """Where an odometry pose is in the room, using room_origin."""
        x0, z0, yaw = self.room_origin
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * odom[0] - s * odom[1] + x0, s * odom[0] + c * odom[1] + z0, odom[2] + yaw)

    def _set_origin(self, room: tuple[float, float, float], odom: tuple[float, float, float]) -> None:
        """Pick room_origin so that this odometry pose lands on this room pose."""
        yaw = room[2] - odom[2]
        c, s = math.cos(yaw), math.sin(yaw)
        self.room_origin = (
            room[0] - (c * odom[0] - s * odom[1]),
            room[1] - (s * odom[0] + c * odom[1]),
            yaw,
        )

    def observe_scan(self, odom_transform: Transform3, odom_lidar: Lidar) -> bool:
        """Localize or track the robot with one scan. Returns whether it is placed in the room.

        Both arguments are odometry (the robot's own frame) in server axes.
        """
        odom = self.odom_pose(odom_transform)
        if self._last_scan_odom is not None and self._odom_jumped(self._last_scan_odom, odom):
            print(f"{self.id}: odometry jumped between scans; rerunning branch and bound")
            self.needs_relocalization = True
            # The robot did not move with it (e.g. /reset_pose): keep the
            # particles where they are and re-anchor odometry to them.
            if self.particles is not None and self.particles.pose is not None:
                self._set_origin(self.particles.pose, odom)
                self._last_mc_odom = odom
        self._last_scan_odom = odom

        if not self.localized or self.needs_relocalization:
            if self.localize(odom_transform, odom_lidar):
                return True
            # Keep tracking with the particle map while relocalization retries.
            if not self.localized:
                return False
        self._track(odom, odom_lidar)
        return True

    def localize(self, odom_transform: Transform3, odom_lidar: Lidar) -> bool:
        """Branch and bound over the whole room; seed the particle map on success."""
        from map.montecarlo import MonteCarlo
        from map.room_localization import localize_in_room, wall_grid

        now = time()
        if now - self.last_localize_attempt < self.LOCALIZE_RETRY:
            return False
        self.last_localize_attempt = now

        odom = self.odom_pose(odom_transform)
        points = np.array([[point.x, point.z] for point in odom_lidar.points]).reshape(-1, 2)
        result = localize_in_room(points - np.array(odom[:2]))
        if result is None:
            print(f"{self.id}: branch and bound found no match in the room; retrying")
            return False

        x, z, yaw, score = result
        room = (x, z, odom[2] + yaw)
        if self.localized and self._pose_jumped(self.room_pose(odom), room):
            self.map_epoch += 1
        self._set_origin(room, odom)
        self.particles = MonteCarlo(wall_grid())
        self.particles.initialize(room)
        self._last_mc_odom = odom
        self._poor_fits = 0
        self.needs_relocalization = False
        print(
            f"{self.id}: branch and bound placed it at ({x:.2f}, {z:.2f}), heading offset "
            f"{math.degrees(yaw):.1f} deg, score {score:.3f}"
        )
        return True

    def _track(self, odom: tuple[float, float, float], odom_lidar: Lidar) -> None:
        """Run Monte Carlo on this scan if the robot moved since the last run."""
        from map.room_localization import wall_grid

        if self.particles is None or not self._moved(self._last_mc_odom, odom):
            return
        # Robot-frame scan: subtract the position, undo the odometry heading.
        points = np.array([[point.x, point.z] for point in odom_lidar.points]).reshape(-1, 2)
        c, s = math.cos(-odom[2]), math.sin(-odom[2])
        scan = (points - np.array(odom[:2])) @ np.array([[c, -s], [s, c]]).T

        self.particles.grid = wall_grid()
        self.particles.predict(self._last_mc_odom, odom)
        pose = self.particles.update(scan)
        self._last_mc_odom = odom
        if pose is None:
            return
        self._set_origin(pose, odom)

        if self.particles.fit is not None and self.particles.fit < self.MC_POOR_FIT:
            self._poor_fits += 1
            if self._poor_fits >= self.MC_POOR_FIT_UPDATES and not self.needs_relocalization:
                print(f"{self.id}: Monte Carlo fit {self.particles.fit:.2f} stayed poor; rerunning branch and bound")
                self.needs_relocalization = True
        else:
            self._poor_fits = 0

    def _moved(self, before: tuple[float, float, float], now: tuple[float, float, float]) -> bool:
        turn = abs(math.remainder(now[2] - before[2], math.tau))
        return math.dist(before[:2], now[:2]) >= self.MC_MIN_TRANSLATION or turn >= self.MC_MIN_ROTATION

    def _odom_jumped(self, before: tuple[float, float, float], now: tuple[float, float, float]) -> bool:
        turn = abs(math.remainder(now[2] - before[2], math.tau))
        return math.dist(before[:2], now[:2]) > self.ODOM_JUMP_TRANSLATION or turn > self.ODOM_JUMP_ROTATION

    def _pose_jumped(self, before: tuple[float, float, float], now: tuple[float, float, float]) -> bool:
        turn = abs(math.remainder(now[2] - before[2], math.tau))
        return (
            math.dist(before[:2], now[:2]) > self.RELOCALIZE_JUMP_TRANSLATION
            or turn > self.RELOCALIZE_JUMP_ROTATION
        )

    def _place(self, vec: Vector3) -> Vector3:
        x0, z0, yaw = self.room_origin
        c, s = math.cos(yaw), math.sin(yaw)
        return Vector3(x=c * vec.x - s * vec.z + x0, y=vec.y, z=s * vec.x + c * vec.z + z0)

    def place_transform(self, transform: Transform3) -> Transform3:
        """Odometry pose in server axes -> pose in the room (server frame)."""
        r = transform.rotation
        return Transform3(
            position=self._place(transform.position),
            rotation=Vector3(x=r.x, y=r.y + self.room_origin[2], z=r.z),
        )

    def place_lidar(self, lidar: Lidar) -> Lidar:
        return Lidar(points=[self._place(point) for point in lidar.points])

    def reset_stationary_scans(self) -> None:
        """Clear the parked-scan budget (e.g. after clear accumulated scan)."""
        self.stationary_scans = 0
        self._station_pose: tuple[float, float, float, float] | None = None

    def allow_lidar_scan(self, transform: Transform3) -> bool:
        """True for the first stationary_scan_limit scans at the current pose."""
        pose = (
            transform.position.x,
            transform.position.z,
            transform.rotation.y,
            transform.rotation.z,
        )
        if self._station_pose is None or self._station_pose_moved(self._station_pose, pose):
            self._station_pose = pose
            self.stationary_scans = 1
            return True

        self.stationary_scans += 1
        return self.stationary_scans <= self.stationary_scan_limit

    def _station_pose_moved(
        self,
        previous: tuple[float, float, float, float],
        current: tuple[float, float, float, float],
    ) -> bool:
        if math.hypot(current[0] - previous[0], current[1] - previous[1]) > self._STATIONARY_TRANSLATION_M:
            return True
        dyaw = max(abs(current[2] - previous[2]), abs(current[3] - previous[3]))
        return dyaw > self._STATIONARY_YAW_RAD


class Dog(TransformDevice):
    DEVICE_TYPE = DeviceType.DOG


_DEVICE_CLASSES: dict[int, type[TransformDevice]] = {
    DeviceType.TURTLE: TurtleBot,
    DeviceType.DOG: Dog,
}


def device_from_info(device_info: DeviceInfo) -> Device | None:
    cls = _DEVICE_CLASSES.get(device_info.device_type)
    return cls.from_packet(device_info) if cls else None


def device_from_dict(data: dict[str, Any]) -> Device | None:
    device_type = data.get("device_type")
    if isinstance(device_type, str):
        try:
            device_type = DeviceType.Value(device_type)
        except ValueError:
            return None
    cls = _DEVICE_CLASSES.get(device_type)
    return cls.from_dict(data) if cls else None