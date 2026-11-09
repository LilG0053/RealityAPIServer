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

    def test_rmse_histogram_saves_png_and_clear_forgets_samples(self):
        missing = server.save_rmse_histogram()
        self.assertFalse(missing["ok"])
        self.assertIn("No scan RMSEs", missing["error"])

        server.server_map.scan_rmses.extend([0.02, 0.05, 0.05, 0.11])
        saved = server.save_rmse_histogram()
        self.assertTrue(saved["ok"], saved.get("error"))
        self.assertEqual(saved["count"], 4)
        image = Path(saved["path"])
        self.assertTrue(image.read_bytes().startswith(b"\x89PNG"))
        self.assertEqual(json.loads(image.with_suffix(".json").read_text()), [0.02, 0.05, 0.05, 0.11])

        self.assertTrue(server.server_map.clear_accumulated_scan().HasField("ok"))
        self.assertEqual(server.server_map.scan_rmses, [])

    def test_scan_rmse_list_drops_samples_older_than_the_limit(self):
        from map.map import SCAN_RMSE_LIMIT
        for value in range(SCAN_RMSE_LIMIT + 3):
            server.server_map.record_scan_rmse(float(value))
        self.assertEqual(len(server.server_map.scan_rmses), SCAN_RMSE_LIMIT)
        self.assertEqual(server.server_map.scan_rmses[0], 3.0)
        self.assertEqual(server.server_map.scan_rmses[-1], float(SCAN_RMSE_LIMIT + 2))

    def test_turtlebot_rmse_above_the_limit_is_not_merged(self):
        from map.localization import ScanLocalizer
        from realityapi_pb2 import Lidar

        def ring() -> Lidar:
            points = [Vector3(x=math.cos(i * math.tau / 12), z=math.sin(i * math.tau / 12)) for i in range(12)]
            return Lidar(points=points)

        localizer = ScanLocalizer()
        localizer.update(ring())
        kept = localizer.accumulated_scan.copy()
        correction = localizer.correction.copy()
        # Any finite RMSE, including a perfect 0, is above this stand-in limit.
        self.assertFalse(localizer.update(ring(), rmse_limit=-1.0))
        self.assertEqual(localizer.last_rmse, 0.0)
        self.assertTrue(np.array_equal(localizer.accumulated_scan, kept))
        self.assertTrue(np.allclose(localizer.correction, correction))

        from map.device import Dog, TurtleBot, device_from_info

        self.assertEqual(TurtleBot.RMSE_LIMIT, 0.2)
        self.assertEqual(Dog.RMSE_LIMIT, math.inf)
        server.server_map.add_device(device_from_info(DeviceInfo(
            id="turtle", device_type=DeviceType.TURTLE, transform=Transform(),
        )))
        server.server_map.add_device(device_from_info(DeviceInfo(
            id="dog", device_type=DeviceType.DOG, transform=Transform(),
        )))
        self.assertEqual(server.server_map.rmse_limit_for("turtle"), 0.2)
        self.assertEqual(server.server_map.rmse_limit_for("dog"), math.inf)


if __name__ == "__main__":
    unittest.main()
