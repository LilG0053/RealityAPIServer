"""Raw scans to points, and scan simplification (map/lidar.py)."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from map.device.turtlebot import LIDAR_OFFSET
from map.lidar import scan_to_device_points, simplify_points
from realityapi_pb2 import RawLidar2Scan, Transform2, Vector2

CENTERED = Transform2(position=Vector2())


def scan(ranges, angle_min=0.0, increment=math.pi / 2):
    return RawLidar2Scan(ranges=ranges, range_min=0.1, range_max=10.0, angle_min=angle_min, angle_increment=increment)


class ScanToDevicePointsTest(unittest.TestCase):
    def test_beams_counterclockwise_from_forward(self):
        # 0 rad is forward (+X); pi/2 is the device's left (+Z).
        points = scan_to_device_points(scan([1.0, 2.0]), CENTERED)
        np.testing.assert_allclose(points, [[1.0, 0.0], [0.0, 2.0]], atol=1e-6)

    def test_out_of_range_beams_are_dropped(self):
        points = scan_to_device_points(scan([0.05, 1.0, float("inf"), 20.0]), CENTERED)
        np.testing.assert_allclose(points, [[0.0, 1.0]], atol=1e-6)

    def test_offset_moves_and_turns_the_sensor(self):
        mount = Transform2(position=Vector2(x=0.5, y=0.0), rotation=math.pi / 2)
        # The sensor's 0 rad axis points left, so a 0 rad beam lands to the left of the sensor.
        np.testing.assert_allclose(scan_to_device_points(scan([1.0]), mount), [[0.5, 1.0]], atol=1e-6)

    def test_turtlebot_mount_sees_straight_ahead(self):
        # Its sensor is turned +90 degrees, so a beam at -90 degrees points forward,
        # from 4 cm behind the robot's center.
        points = scan_to_device_points(scan([1.0], angle_min=-math.pi / 2), LIDAR_OFFSET)
        np.testing.assert_allclose(points, [[0.96, 0.0]], atol=1e-6)


class SimplifyTest(unittest.TestCase):
    def test_straight_run_collapses_to_its_ends(self):
        line = np.column_stack((np.linspace(0, 1, 50), np.zeros(50)))
        simplified = simplify_points(line, closed=False)
        np.testing.assert_allclose(simplified, [[0, 0], [1, 0]], atol=1e-6)

    def test_empty_scan(self):
        self.assertEqual(simplify_points(np.empty((0, 2))).shape, (0, 2))


if __name__ == "__main__":
    unittest.main()
