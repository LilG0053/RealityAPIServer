"""The robot dog: 3D poses and point clouds, in server coordinates.

Its point cloud is flattened into a smoothed ring and matched with ICP against
the dog's own accumulated map. StaticLocalizer then fixes that frame against
the room walls (branch and bound, then Monte Carlo); its delta is folded into
the ICP correction, which is applied to the ring and to the pose.
"""

from __future__ import annotations

from time import monotonic
from typing import Any, ClassVar

import numpy as np
from google.protobuf.json_format import ParseDict

from common.transform2 import make_transform2, transform2_from_transform3
from map.bandb_dog import StaticLocalizer
from map.algorithms.lidar import simplify_points
from map.algorithms.localization import ScanLocalizer, apply
from map.algorithms.pointcloud import RingAccumulator, decode_xyz, raw_cloud_to_ring
from map.device.device import Device, _message_dict
from map.occupancy import OccupancyGrid
from map.room import polygon_array
from map.wallgrid import WallGrid
from realityapi_pb2 import DeviceInfo, DeviceType, Lidar3, Response, Transform3, Vector3

# Seconds between ring updates to the occupancy grid (each one walks every ray in Python).
OCCUPANCY_INTERVAL = 1.0
# The whole-room search matches the accumulated ICP map within this range of the dog,
# thinned to one point per voxel and at most GLOBAL_SCAN_MAX points.
GLOBAL_SCAN_RANGE = 10.0
GLOBAL_SCAN_VOXEL = 0.2
GLOBAL_SCAN_MAX = 150
# Seconds between [dog] diagnostic lines, and the height edges (m vs the pose) they count points in.
DIAG_INTERVAL = 5.0
DIAG_EDGES = (-0.5, -0.25, -0.15, 0.05, 0.25, 0.5, 1.0, 2.0)


class Dog(Device):
    DEVICE_TYPE: ClassVar[int] = DeviceType.DOG

    def __init__(self, id: str, wall_grid: WallGrid, occupancy: OccupancyGrid, timestamp: float = 0.0) -> None:
        super().__init__(id, timestamp)
        self.transform3 = Transform3()
        self.lidar3 = Lidar3()
        self.localizer = ScanLocalizer()
        self.ring = RingAccumulator()
        room, *obstacles = (polygon_array(p) for p in wall_grid.room.polygons())
        self.static = StaticLocalizer(wall_grid, room, obstacles)
        # Set after a whole-room fix; Map clears the 3D cloud map once and resets it.
        self.reset_cloud = False
        # Display-only grid of the placed ring; never blended into localization.
        self.occupancy = occupancy
        self._mapped_at = -OCCUPANCY_INTERVAL
        self._diag_at = -DIAG_INTERVAL

    @classmethod
    def from_dict(cls, data: dict[str, Any], **kwargs: Any) -> Dog:
        dog = super().from_dict(data, **kwargs)
        dog.transform3 = ParseDict(data.get("transform3", {}), Transform3())
        dog.lidar3 = ParseDict(data.get("lidar3", {}), Lidar3())
        return dog

    def handle_device_info(self, device_info: DeviceInfo) -> Response:
        self.heartbeat()
        if device_info.HasField("raw_point_cloud"):
            raw = device_info.raw_point_cloud
            ring, height = raw_cloud_to_ring(raw, self.ring)
            self._diag(raw, ring)
            if len(ring):
                self.localizer.update(ring)
                self._locate(ring, raw.transform)
                self._map_ring(ring, raw.transform)
                pts = simplify_points(self.localizer.correct_points(ring))
                self.lidar3 = Lidar3(points=[Vector3(x=float(x), y=height, z=float(z)) for x, z in pts])
        elif device_info.HasField("lidar3"):
            self.lidar3.CopyFrom(device_info.lidar3)

        if device_info.HasField("transform3"):
            self.transform3 = self._corrected(device_info.transform3)
        elif device_info.HasField("raw_point_cloud"):
            self.transform3 = self._corrected(device_info.raw_point_cloud.transform)
        return self.reply()

    def _locate(self, ring: np.ndarray, transform: Transform3) -> None:
        loc = self.localizer
        p = transform.position
        pos = loc.correct_points(np.array([[p.x, p.z]]))[0]
        was = self.static.located
        scan = loc.correct_points(ring) if was else self._global_scan(ring, pos)
        fix = self.static.locate(scan, pos, correction=loc.correction)
        if was and not self.static.located:
            # Lost: the old map and 3D cloud belong to a place the dog may have left.
            loc.accumulated_scan = None
            self.ring.clear()
            self.reset_cloud = True
        if fix is None:
            return
        loc.correction = fix.delta @ loc.correction
        if loc.accumulated_scan is not None:
            loc.accumulated_scan = apply(fix.delta, loc.accumulated_scan)
        if fix.global_search:
            self.reset_cloud = True

    def _diag(self, raw, ring: np.ndarray) -> None:
        now = monotonic()
        if now - self._diag_at < DIAG_INTERVAL:
            return
        self._diag_at = now
        z = decode_xyz(raw.xyz)[:, 1] - raw.transform.position.y
        z = z[np.isfinite(z)]
        n = np.histogram(z, bins=DIAG_EDGES)[0]
        bands = " ".join(f"{lo:+.2f}..{hi:+.2f}:{c}" for lo, hi, c in zip(DIAG_EDGES, DIAG_EDGES[1:], n))
        print(f"[dog] placed={self.static.located} ring={len(ring)} pts; points by height vs pose: {bands}")

    def _global_scan(self, ring: np.ndarray, pos: np.ndarray) -> np.ndarray:
        """What the whole-room search matches: the accumulated map near the dog, thinned."""
        loc = self.localizer
        acc = loc.accumulated_scan
        if acc is None:
            return loc.correct_points(ring)
        near = acc[np.hypot(acc[:, 0] - pos[0], acc[:, 1] - pos[1]) <= GLOBAL_SCAN_RANGE]
        if len(near) == 0:
            return loc.correct_points(ring)
        _, keep = np.unique(np.floor(near / GLOBAL_SCAN_VOXEL).astype(int), axis=0, return_index=True)
        near = near[keep]
        if len(near) > GLOBAL_SCAN_MAX:
            near = near[:: -(-len(near) // GLOBAL_SCAN_MAX)]
        return near

    def _map_ring(self, ring: np.ndarray, transform: Transform3) -> None:
        now = monotonic()
        if not self.static.located or now - self._mapped_at < OCCUPANCY_INTERVAL:
            return
        self._mapped_at = now
        loc = self.localizer
        p = transform.position
        x, z = loc.correct_points(np.array([[p.x, p.z]]))[0]
        self.occupancy.update(make_transform2(float(x), float(z), 0.0), loc.correct_points(ring))

    def _corrected(self, transform: Transform3) -> Transform3:
        pose = transform2_from_transform3(transform)
        corrected = self.localizer.correct_pose(pose)
        p, r = transform.position, transform.rotation
        return Transform3(
            position=Vector3(x=corrected.position.x, y=p.y, z=corrected.position.y),
            rotation=Vector3(x=r.x, y=r.y + (corrected.rotation - pose.rotation), z=r.z),
        )

    def accumulated_points(self):
        return self.localizer.accumulated_scan

    def clear_scan(self) -> None:
        self.localizer.clear_scan()
        self.static.reset()

    def get_device_info(self) -> DeviceInfo:
        info = super().get_device_info()
        info.transform3.CopyFrom(self.transform3)
        info.lidar3.CopyFrom(self.lidar3)
        return info

    def to_dict(self) -> dict[str, Any]:
        ret = super().to_dict()
        ret["transform3"] = _message_dict(self.transform3)
        ret["lidar3"] = _message_dict(self.lidar3)
        return ret