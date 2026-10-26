"""Offline integration test for the raw LiDAR protobuf-to-server pipeline."""

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

from google.protobuf.empty_pb2 import Empty

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import server
from map.map import Map
from realityapi_pb2 import DeviceInfo, DeviceType, RawLidarScan, ServerRequest, Transform, Vector3


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


if __name__ == "__main__":
    unittest.main()
