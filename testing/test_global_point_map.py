"""Offline checks for scan correction and the global point-map methods."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from map.localization import ScanLocalizer
from map.map import Map
from realityapi_pb2 import Transform, Vector3


def points_from_xz(points):
    xz = np.asarray(points, dtype=float).reshape(-1, 2)
    return np.column_stack((xz[:, 0], np.zeros(len(xz)), xz[:, 1]))


class GlobalPointMapTest(unittest.TestCase):
    def test_known_translation_is_corrected_and_exported(self):
        # Irregular spacing reduces ambiguity in nearest-neighbor matches.
        reference = np.array([
            [0.00, 0.00], [0.23, 0.00], [0.49, 0.00], [0.78, 0.00],
            [0.00, 0.21], [0.00, 0.45], [0.00, 0.74], [0.23, 0.74],
            [0.49, 0.74], [0.78, 0.74], [0.78, 0.53], [0.78, 0.28],
            [0.32, 0.24], [0.55, 0.31], [0.41, 0.53],
        ])
        odometry_error = np.array([0.02, -0.015])
        localizer = ScanLocalizer()
        point_map = Map()

        status, corrected_scan = localizer.update_correction(points_from_xz(reference))
        self.assertEqual(status, "first_scan")
        point_map.update_point_map(corrected_scan)

        shifted_scan = points_from_xz(reference + odometry_error)
        status, corrected_scan = localizer.update_correction(shifted_scan)
        self.assertEqual(status, "updated")
        actual = corrected_scan[:, [0, 2]]
        np.testing.assert_allclose(actual, reference, atol=1e-5)

        # An odometry pose with the same error should be corrected consistently.
        pose = Transform(position=Vector3(x=float(odometry_error[0]), z=float(odometry_error[1])))
        corrected_pose = localizer.corrected_transform(pose, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(corrected_pose.position.x, 0.0, places=5)
        self.assertAlmostEqual(corrected_pose.position.z, 0.0, places=5)

        point_map.update_point_map(corrected_scan)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "global_map.json"
            point_map.export_global_points(str(path))
            exported = json.loads(path.read_text())
        self.assertGreater(len(exported), 0)
        self.assertTrue(all(set(point) == {"x", "y", "z"} for point in exported))
        self.assertTrue(all(len(cell) <= point_map.max_points_per_cell
                            for cell in point_map.point_map.values()))

    def test_cell_limit_and_negative_coordinates(self):
        point_map = Map()
        point_map.max_points_per_cell = 2
        scan = points_from_xz([[-0.02, -0.02], [-0.03, -0.03], [-0.04, -0.04]])
        point_map.update_point_map(scan)
        self.assertEqual(set(point_map.point_map), {(-1, -1)})
        self.assertEqual(len(point_map.point_map[(-1, -1)]), 2)

    def test_no_overlap_does_not_change_correction_or_reference(self):
        localizer = ScanLocalizer()
        first = points_from_xz([[i * 0.1, (i % 3) * 0.2] for i in range(15)])
        far = points_from_xz([[10 + i * 0.1, 10 + (i % 3) * 0.2] for i in range(15)])
        status, _ = localizer.update_correction(first)
        self.assertEqual(status, "first_scan")
        previous = localizer.prev_scan.copy()
        status, _ = localizer.update_correction(far)
        self.assertEqual(status, "no_update")
        np.testing.assert_array_equal(localizer.correction, np.eye(3))
        np.testing.assert_array_equal(localizer.prev_scan, previous)


if __name__ == "__main__":
    unittest.main()
