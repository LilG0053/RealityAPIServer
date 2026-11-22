"""The robot dog: its on-device conversion (clients/robot-dog/dog_protocol.py) and the server's Dog."""

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "clients" / "robot-dog"))


import dog_protocol
from dog_protocol import DOG_START, cloud_request, pose_request, server_points, server_transform
from map.map import Map
from realityapi_pb2 import DeviceType, ServerRequest, Transform3, Vector3

# A raw ROS pose: x north, y west, z up, yaw about z.
POSE = Transform3(position=Vector3(x=0.5, y=0.2, z=0.3), rotation=Vector3(z=0.3))


def point_cloud(points: np.ndarray) -> SimpleNamespace:
    """A sensor_msgs/msg/PointCloud2 of float32 x/y/z points."""
    field = lambda name, offset: SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
    return SimpleNamespace(
        fields=[field("x", 0), field("y", 4), field("z", 8)],
        width=len(points), height=1, point_step=12, row_step=12 * len(points),
        data=np.asarray(points, dtype="<f4").tobytes(), is_bigendian=False,
    )


def ring_around(pose: Transform3, radius=3.0, count=360) -> np.ndarray:
    """ROS-frame points on a circle around the dog, at its height."""
    angles = np.linspace(-math.pi, math.pi, count, endpoint=False)
    p = pose.position
    return np.column_stack((p.x + radius * np.cos(angles), p.y + radius * np.sin(angles), np.full(count, p.z + 0.05)))


class DogProtocolTest(unittest.TestCase):
    def test_start_pose_places_the_odometry_origin(self):
        start = server_transform(Transform3())
        self.assertAlmostEqual(start.position.x, DOG_START.position.x, places=5)
        self.assertAlmostEqual(start.position.z, DOG_START.position.y, places=5)
        # ROS yaw 0 (north) is heading pi in server axes, then turned by the start rotation.
        expected = math.pi + DOG_START.rotation
        self.assertAlmostEqual(math.remainder(start.rotation.y - expected, math.tau), 0.0, places=5)

    def test_points_and_poses_convert_alike(self):
        placed = server_transform(POSE).position
        point = server_points(np.array([[POSE.position.x, POSE.position.y, POSE.position.z]]))[0]
        np.testing.assert_allclose(point, [placed.x, placed.y, placed.z], atol=1e-5)

    def test_requests_carry_server_coordinates(self):
        request = pose_request(POSE)
        self.assertEqual(request.post_device_info.device_type, DeviceType.DOG)
        self.assertEqual(request.post_device_info.transform3, server_transform(POSE))

        cloud = cloud_request(point_cloud(ring_around(POSE)), POSE)
        info = cloud.post_device_info
        self.assertEqual(info.transform3, info.raw_point_cloud.transform)
        xyz = np.frombuffer(info.raw_point_cloud.xyz, dtype="<f4").reshape(-1, 3)
        np.testing.assert_allclose(xyz, server_points(ring_around(POSE)), atol=1e-4)


class DogServerTest(unittest.TestCase):
    def setUp(self):
        self.map = Map()

    def post(self, request: ServerRequest):
        return self.map.handle_device_info(request.post_device_info)

    def test_cloud_becomes_a_ring_at_the_dogs_height(self):
        reply = self.post(cloud_request(point_cloud(ring_around(POSE)), POSE))
        dog = self.map._devices[dog_protocol.DEVICE_ID]
        placed = server_transform(POSE)
        self.assertEqual(dog.transform3, placed)  # the first scan has no ICP correction
        # The reply is the server's view of the dog: its corrected pose and ring.
        self.assertEqual(reply.device_info, dog.get_device_info())

        points = np.array([[p.x, p.y, p.z] for p in dog.lidar3.points])
        self.assertGreater(len(points), 10)
        np.testing.assert_allclose(points[:, 1], placed.position.y, atol=1e-5)
        distances = np.hypot(points[:, 0] - placed.position.x, points[:, 2] - placed.position.z)
        np.testing.assert_allclose(distances, 3.0, atol=0.05)

    def test_icp_keeps_a_repeated_scan_in_place_and_maps_it(self):
        request = cloud_request(point_cloud(ring_around(POSE)), POSE)
        self.post(request)
        self.post(request)
        dog = self.map._devices[dog_protocol.DEVICE_ID]
        np.testing.assert_allclose(dog.localizer.correction, np.eye(3), atol=1e-3)

        lidar = self.map.get_lidar_data().lidar_data
        self.assertGreater(len(lidar.points), 0)
        self.map.clear_accumulated_scan()
        self.assertIsNone(dog.accumulated_points())
        self.assertEqual(len(self.map.get_lidar_data().lidar_data.points), 0)

    def test_pose_only_update(self):
        self.post(pose_request(POSE))
        info = self.map.get_device_info(dog_protocol.DEVICE_ID).device_info
        self.assertEqual(info.transform3, server_transform(POSE))
        self.assertEqual(len(info.lidar3.points), 0)


if __name__ == "__main__":
    unittest.main()
