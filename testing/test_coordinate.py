"""Coordinate-system conversion the robots run on-device (common/coordinate.py)."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.coordinate import (
    SERVER_COORDS,
    TURTLEBOT_COORDS,
    convert_points,
    convert_transform,
    get_transform_matrix,
)
from realityapi_pb2 import Transform3, Vector3


class CoordinateTest(unittest.TestCase):
    def test_ros_axes_to_server_axes(self):
        # ROS x north, y west, z up -> server x south, y up, z east.
        np.testing.assert_array_equal(
            get_transform_matrix(TURTLEBOT_COORDS, SERVER_COORDS),
            [[-1, 0, 0], [0, 0, 1], [0, -1, 0]],
        )

    def test_transform_position_and_heading(self):
        ros = Transform3(position=Vector3(x=1, y=2, z=0.5), rotation=Vector3(z=0.5))
        server = convert_transform(ros, TURTLEBOT_COORDS, SERVER_COORDS)
        self.assertEqual((server.position.x, server.position.y, server.position.z), (-1.0, 0.5, -2.0))
        # ROS yaw is counterclockwise from north; the server's heading is from south.
        self.assertAlmostEqual(server.rotation.y, 0.5 + math.pi, places=5)

    def test_points_match_transform_positions(self):
        points = np.array([[1.0, 2.0, 0.5], [-3.0, 0.0, 1.0]])
        converted = convert_points(points, TURTLEBOT_COORDS, SERVER_COORDS)
        for point, expected in zip(points, converted):
            p = convert_transform(Transform3(position=Vector3(x=point[0], y=point[1], z=point[2])),
                                  TURTLEBOT_COORDS, SERVER_COORDS).position
            np.testing.assert_allclose([p.x, p.y, p.z], expected)

    def test_same_system_is_unchanged(self):
        t = Transform3(position=Vector3(x=1, y=2, z=3), rotation=Vector3(y=0.2))
        self.assertEqual(convert_transform(t, SERVER_COORDS, SERVER_COORDS), t)


if __name__ == "__main__":
    unittest.main()
