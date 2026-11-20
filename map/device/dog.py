"""The robot dog: 3D poses and point clouds, in server coordinates.

Its point cloud is flattened into a ring of points and matched with ICP
against the dog's own accumulated map; the ICP correction is applied to the
ring (its live scan) and to the pose it sends.
"""

from __future__ import annotations

from typing import Any, ClassVar

from google.protobuf.json_format import ParseDict

from common.transform2 import transform2_from_transform3
from map.algorithms.lidar import simplify_points
from map.algorithms.localization import ScanLocalizer
from map.algorithms.pointcloud import raw_cloud_to_ring
from map.device.device import Device, _message_dict
from realityapi_pb2 import DeviceInfo, DeviceType, Lidar3, Response, Transform3, Vector3


class Dog(Device):
    DEVICE_TYPE: ClassVar[int] = DeviceType.DOG

    def __init__(self, id: str, timestamp: float = 0.0) -> None:
        super().__init__(id, timestamp)
        self.transform3 = Transform3()
        self.lidar3 = Lidar3()
        # ICP against the dog's own accumulated map.
        self.localizer = ScanLocalizer()

    @classmethod
    def from_dict(cls, data: dict[str, Any], **kwargs: Any) -> Dog:
        dog = super().from_dict(data, **kwargs)
        dog.transform3 = ParseDict(data.get("transform3", {}), Transform3())
        dog.lidar3 = ParseDict(data.get("lidar3", {}), Lidar3())
        return dog

    def handle_device_info(self, device_info: DeviceInfo) -> Response:
        super().handle_device_info(device_info)
        if device_info.HasField("raw_point_cloud"):
            ring, height = raw_cloud_to_ring(device_info.raw_point_cloud)
            # Nothing in range keeps the last scan.
            if len(ring):
                self.localizer.update(ring)
                corrected = simplify_points(self.localizer.correct_points(ring))
                self.lidar3 = Lidar3(points=[Vector3(x=float(x), y=height, z=float(z)) for x, z in corrected])
        elif device_info.HasField("lidar3"):
            self.lidar3.CopyFrom(device_info.lidar3)

        # Corrected after the scan, so the pose gets this scan's ICP correction.
        if device_info.HasField("transform3"):
            self.transform3 = self._corrected(device_info.transform3)
        elif device_info.HasField("raw_point_cloud"):
            self.transform3 = self._corrected(device_info.raw_point_cloud.transform)
        return Response(ok={})

    def _corrected(self, transform: Transform3) -> Transform3:
        """A server-coordinate Transform3 with the ICP correction applied on the ground plane."""
        pose = transform2_from_transform3(transform)
        corrected = self.localizer.correct_pose(pose)
        p, r = transform.position, transform.rotation
        # Heading is a rotation about the server's UP axis, which is rotation.y;
        # Transform2's Vector2.y carries the server's Z.
        return Transform3(
            position=Vector3(x=corrected.position.x, y=p.y, z=corrected.position.y),
            rotation=Vector3(x=r.x, y=r.y + (corrected.rotation - pose.rotation), z=r.z),
        )

    def accumulated_points(self):
        """The ICP map: server-frame (N, 2) X/Z points, or None before any scan."""
        return self.localizer.accumulated_scan

    def clear_scan(self) -> None:
        """Forget the accumulated map; the pose is no longer corrected."""
        self.localizer.clear_scan()

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
