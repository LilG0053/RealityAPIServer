"""Accumulated maps: plain (AccumulatedMap) and ICP-aligned (ScanLocalizer)."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.transform2 import make_transform2
from map.localization import AccumulatedMap, ScanLocalizer
from sim import body_scan

# A robot at (4, 6) facing +X; its scan placed in the room.
POSE = (4.0, 6.0, 0.0)
SCAN = body_scan(POSE) + np.array(POSE[:2])


class AccumulatedMapTest(unittest.TestCase):
    def test_first_scan_starts_the_map(self):
        accumulated = AccumulatedMap()
        self.assertFalse(accumulated.update(SCAN))
        self.assertEqual(len(accumulated.points), len(SCAN))
        self.assertIsNone(accumulated.last_rmse)

    def test_matching_scan_merges_and_records_rmse(self):
        accumulated = AccumulatedMap()
        accumulated.update(SCAN)
        self.assertTrue(accumulated.update(SCAN + 0.01))
        self.assertLess(accumulated.last_rmse, 0.05)

    def test_disagreeing_scan_is_not_merged(self):
        accumulated = AccumulatedMap()
        accumulated.update(SCAN)
        before = accumulated.points.copy()
        self.assertFalse(accumulated.update(SCAN + [0.15, 0.0], rmse_limit=0.05))
        self.assertGreater(accumulated.last_rmse, 0.05)
        np.testing.assert_array_equal(accumulated.points, before)

    def test_clear(self):
        accumulated = AccumulatedMap()
        accumulated.update(SCAN)
        accumulated.clear()
        self.assertIsNone(accumulated.points)


class ScanLocalizerTest(unittest.TestCase):
    def test_icp_corrects_a_shifted_scan(self):
        localizer = ScanLocalizer()
        localizer.update(SCAN)
        # The same scan, reported 10 cm off and slightly turned.
        angle = math.radians(2.0)
        c, s = math.cos(angle), math.sin(angle)
        shifted = (SCAN - POSE[:2]) @ np.array([[c, -s], [s, c]]).T + POSE[:2] + [0.1, -0.05]
        self.assertTrue(localizer.update(shifted))
        np.testing.assert_allclose(localizer.correct_points(shifted), SCAN, atol=0.02)

        corrected = localizer.correct_pose(make_transform2(POSE[0] + 0.1, POSE[1] - 0.05, angle))
        self.assertAlmostEqual(corrected.position.x, POSE[0], delta=0.02)
        self.assertAlmostEqual(corrected.position.y, POSE[1], delta=0.02)
        self.assertAlmostEqual(corrected.rotation, 0.0, delta=math.radians(0.5))

    def test_clear_resets_the_correction(self):
        localizer = ScanLocalizer()
        localizer.update(SCAN)
        localizer.update(SCAN + [0.1, 0.0])
        localizer.clear_scan()
        np.testing.assert_array_equal(localizer.correction, np.eye(3))
        self.assertIsNone(localizer.accumulated_scan)


if __name__ == "__main__":
    unittest.main()
