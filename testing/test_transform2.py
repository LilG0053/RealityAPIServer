"""Pose arithmetic on Transform2 (common/transform2.py)."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.transform2 import (
    apply_transform2,
    as_tuple,
    compose_transform2,
    invert_transform2,
    make_transform2,
    pose_change,
    transform2_from_transform3,
)
from realityapi_pb2 import Transform3, Vector3


def assert_pose(test, pose, x, z, rotation, places=5):
    test.assertAlmostEqual(pose.position.x, x, places=places)
    test.assertAlmostEqual(pose.position.y, z, places=places)
    test.assertAlmostEqual(math.remainder(pose.rotation - rotation, math.tau), 0.0, places=places)


class Transform2Test(unittest.TestCase):
    def test_make_and_as_tuple_use_vector2_y_for_z(self):
        pose = make_transform2(1.0, 2.0, 0.5)
        self.assertEqual(pose.position.y, 2.0)
        self.assertEqual(as_tuple(pose), (1.0, 2.0, 0.5))

    def test_compose_places_a_relative_pose(self):
        # One meter ahead of a pose at (1, 2) facing +Z is (1, 3), still facing +Z.
        a = make_transform2(1.0, 2.0, math.pi / 2)
        assert_pose(self, compose_transform2(a, make_transform2(1.0, 0.0, 0.0)), 1.0, 3.0, math.pi / 2)

    def test_invert_undoes_a_pose(self):
        pose = make_transform2(1.5, -2.0, 0.7)
        assert_pose(self, compose_transform2(pose, invert_transform2(pose)), 0.0, 0.0, 0.0)
        assert_pose(self, compose_transform2(invert_transform2(pose), pose), 0.0, 0.0, 0.0)

    def test_apply_matches_compose_for_points(self):
        pose = make_transform2(1.0, 2.0, 0.3)
        points = np.array([[1.0, 0.0], [0.5, -2.0]])
        placed = apply_transform2(pose, points)
        for point, expected in zip(points, placed):
            single = compose_transform2(pose, make_transform2(point[0], point[1], 0.0))
            self.assertAlmostEqual(single.position.x, expected[0], places=5)
            self.assertAlmostEqual(single.position.y, expected[1], places=5)

    def test_pose_change_wraps_heading(self):
        distance, turn = pose_change(make_transform2(0, 0, math.pi - 0.1), make_transform2(3, 4, -math.pi + 0.1))
        self.assertAlmostEqual(distance, 5.0)
        self.assertAlmostEqual(turn, 0.2, places=5)

    def test_from_transform3_takes_the_ground_plane(self):
        pose = transform2_from_transform3(Transform3(position=Vector3(x=1, y=9, z=2), rotation=Vector3(y=0.4)))
        assert_pose(self, pose, 1.0, 2.0, 0.4)


if __name__ == "__main__":
    unittest.main()
