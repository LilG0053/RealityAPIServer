"""Offline integration test for the raw LiDAR protobuf-to-server pipeline."""

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from google.protobuf.empty_pb2 import Empty

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import server
from map.lidar import LIDAR_EPSILON, simplified_contour
from map.map import Map
from realityapi_pb2 import DeviceInfo, DeviceType, Lidar, RawLidarScan, ServerRequest, Transform, Vector3


class RawLidarPipelineTest(unittest.TestCase):
    """Proves a synthetic scan reaches the normal server map and JSON export."""

    def setUp(self):
        self.original_map = server.server_map
        self.original_directory = Path.cwd()
        self.temporary_directory = tempfile.TemporaryDirectory()
        os.chdir(self.temporary_directory.name)
        Path("dashboard").mkdir()
        server.server_map = Map()

    def tearDown(self):
        server.server_map = self.original_map
        os.chdir(self.original_directory)
        self.temporary_directory.cleanup()

    @staticmethod
    def dispatch(request: ServerRequest):
        """Serialize first, just as a WebSocket client does, then dispatch it."""
        received_request = ServerRequest()
        received_request.ParseFromString(request.SerializeToString())
        return server.handle_request(received_request)

    def test_raw_scan_creates_dashboard_lidar_for_registered_device(self):
        device_id = "fake-turtlebot"

        registration = DeviceInfo(
            id=device_id,
            device_type=DeviceType.TURTLE,
            transform=Transform(),
        )
        registration_response = self.dispatch(
            ServerRequest(post_device_info=registration)
        )
        self.assertTrue(registration_response.HasField("ok"))

        scan = RawLidarScan(
            id=device_id,
            transform=Transform(
                position=Vector3(x=1.0, y=2.0, z=0.0),
                rotation=Vector3(z=0.0),
            ),
            ranges=[1.0] * 8,
            range_min=0.1,
            range_max=5.0,
            angle_min=0.0,
            angle_increment=math.tau / 8,
        )
        scan_response = self.dispatch(
            ServerRequest(post_raw_lidar_scan=scan)
        )
        self.assertTrue(scan_response.HasField("ok"))

        all_devices_response = self.dispatch(
            ServerRequest(fetch_all_device_info=Empty())
        )
        device = all_devices_response.all_device_info.devices[0]
        self.assertEqual(device.id, device_id)
        self.assertGreater(len(device.lidar.points), 0)

        exported_data = json.loads(Path("dashboard/updated_pos.json").read_text())
        self.assertEqual(exported_data[0]["id"], device_id)
        self.assertGreater(len(exported_data[0]["lidar"]["points"]), 0)

    def test_save_keyframe_stores_unadjusted_contour(self):
        device_id = "fake-turtlebot"
        square = [
            Vector3(x=0.0, y=1.0, z=0.0),
            Vector3(x=1.0, y=1.0, z=0.0),
            Vector3(x=1.0, y=1.0, z=1.0),
            Vector3(x=0.0, y=1.0, z=1.0),
        ]
        registration = DeviceInfo(
            id=device_id,
            device_type=DeviceType.TURTLE,
            transform=Transform(),
        )
        self.dispatch(ServerRequest(post_device_info=registration))

        # A prior scan creates the localizer. The correction is then shifted so
        # the scan stored on the device is no longer the incoming one.
        server.server_map.handle_lidar_scan(device_id, Transform(), Lidar(points=square))
        server.server_map._localizers[device_id].correction = np.array([
            [1.0, 0.0, 5.0],
            [0.0, 1.0, 5.0],
            [0.0, 0.0, 1.0],
        ])
        server.server_map.handle_lidar_scan(device_id, Transform(), Lidar(points=square))

        response = self.dispatch(ServerRequest(save_lidar_keyframe=Empty()))
        self.assertTrue(response.HasField("ok"))

        saved_path = next(Path("dashboard/keyframes").glob("*.json"))
        saved = json.loads(saved_path.read_text())
        expected = simplified_contour(
            np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32),
            LIDAR_EPSILON,
        ).reshape(-1, 2)
        actual = np.array([[point["x"], point["z"]] for point in saved["contour"]])
        self.assertEqual(saved["id"], device_id)
        np.testing.assert_allclose(actual, expected, atol=1e-5)

        server.server_map._latest_unadjusted_id = None
        server.server_map._unadjusted_scans.clear()
        missing = self.dispatch(ServerRequest(save_lidar_keyframe=Empty()))
        self.assertTrue(missing.HasField("err_cannot_execute"))

    def test_overlapping_keyframe_reports_loop_enclosed(self):
        device_id = "fake-turtlebot"
        square = [
            Vector3(x=0.0, y=1.0, z=0.0),
            Vector3(x=2.0, y=1.0, z=0.0),
            Vector3(x=2.0, y=1.0, z=2.0),
            Vector3(x=0.0, y=1.0, z=2.0),
        ]
        self.dispatch(ServerRequest(post_device_info=DeviceInfo(
            id=device_id,
            device_type=DeviceType.TURTLE,
            transform=Transform(),
        )))
        server.server_map.handle_lidar_scan(device_id, Transform(), Lidar(points=square))
        first = self.dispatch(ServerRequest(save_lidar_keyframe=Empty()))
        self.assertTrue(first.HasField("ok"))

        contour = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]], dtype=np.float32)
        ious = server.server_map.iou_with_saved_keyframes(contour)
        self.assertEqual(len(ious), 1)
        self.assertGreater(ious[0], 0.95)

        server.server_map.handle_lidar_scan(device_id, Transform(), Lidar(points=square))
        second = self.dispatch(ServerRequest(save_lidar_keyframe=Empty()))
        self.assertTrue(second.HasField("loop_enclosed"))

        shifted = [Vector3(x=point.x + 20.0, y=point.y, z=point.z) for point in square]
        server.server_map.handle_lidar_scan(device_id, Transform(), Lidar(points=shifted))
        separate = self.dispatch(ServerRequest(save_lidar_keyframe=Empty()))
        self.assertTrue(separate.HasField("ok"))

        cleared = self.dispatch(ServerRequest(clear_accumulated_scan=Empty()))
        self.assertTrue(cleared.HasField("ok"))
        self.assertEqual(list(Path("dashboard/keyframes").glob("*.json")), [])
        self.assertIsNone(server.server_map._localizers[device_id].accumulated_scan)


if __name__ == "__main__":
    unittest.main()
