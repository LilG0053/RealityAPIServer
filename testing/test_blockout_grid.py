"""The closeness-to-wall grid matches the measured blockout.

Run with --image PATH to also save the likelihood grid as a PNG.
"""

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from map.blockout import (
    WallGrid,
    distance_to_segments,
    load_blockout_polygons,
    polygon_segments,
)
from map.room import server_room_corners

BLOCKOUT = str(ROOT / "dashboard" / "blockout.json")


def points_along_walls(polygons, step=0.02):
    """Points spaced along every blockout edge, like a perfect scan of the walls."""
    points = []
    for x0, z0, x1, z1 in polygon_segments(polygons):
        count = max(2, int(np.hypot(x1 - x0, z1 - z0) / step))
        t = np.linspace(0.0, 1.0, count)[:, None]
        points.append(np.hstack((x0 + t * (x1 - x0), z0 + t * (z1 - z0))))
    return np.vstack(points)


class WallGridTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.polygons = load_blockout_polygons(BLOCKOUT)
        cls.grid = WallGrid.from_blockout(BLOCKOUT)

    def test_room_outline_matches_room_module(self):
        np.testing.assert_allclose(self.polygons[0], np.array(server_room_corners()))

    def test_segment_distance(self):
        segments = np.array([[0.0, 0.0, 1.0, 0.0]])
        points = np.array([[0.5, 0.3], [-0.3, 0.4], [1.0, 0.0]])
        np.testing.assert_allclose(distance_to_segments(points, segments), [0.3, 0.5, 0.0])

    def test_grid_covers_room_with_margin(self):
        corners = np.array(server_room_corners())
        index = self.grid.world_to_index(corners)
        self.assertTrue(self.grid.inside(index).all())

    def test_walls_score_high_and_shifted_walls_score_low(self):
        walls = points_along_walls(self.polygons)
        on_walls = self.grid.score(walls)
        self.assertGreater(on_walls, 0.9)
        self.assertLess(self.grid.score(walls + [0.5, 0.0]), 0.5 * on_walls)

    def test_pillar_is_a_wall(self):
        pillar = self.polygons[1]
        center = pillar.mean(axis=0)
        face = np.array([[pillar[:, 0].min(), center[1]]])
        self.assertGreater(self.grid.likelihood_at(face)[0], 0.9)
        # The middle of the room, away from everything, is not.
        self.assertLess(self.grid.likelihood_at(np.array([[1.4, 7.0]]))[0], 0.01)

    def test_off_grid_points_score_zero(self):
        far = np.array([[100.0, 100.0]])
        self.assertEqual(self.grid.likelihood_at(far)[0], 0.0)
        self.assertEqual(self.grid.distance_at(far)[0], np.inf)


def save_image(path):
    import cv2

    grid = WallGrid.from_blockout(BLOCKOUT)
    # Transpose so z runs down the image, flip so z = 0 is at the bottom.
    image = (255 * (1.0 - grid.likelihood.T[::-1])).astype(np.uint8)
    cv2.imwrite(path, image)
    print(f"saved {grid.likelihood.shape} grid to {path}")


if __name__ == "__main__":
    if "--image" in sys.argv:
        position = sys.argv.index("--image")
        save_image(sys.argv[position + 1])
        del sys.argv[position:position + 2]
    unittest.main()
