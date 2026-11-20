"""The closeness-to-wall grid matches the room, and blends in occupancy on refresh().

Run with --image PATH to also save the walls-only likelihood grid as a PNG.
"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from map.occupancy import OccupancyGrid
from map.room import BLOCKOUT_PATH, Room, polygon_array
from map.wallgrid import WallGrid, distance_to_segments, polygon_segments
from realityapi_pb2 import Transform2, Vector2

ROOM = Room.from_json(BLOCKOUT_PATH)
POLYGONS = [polygon_array(polygon) for polygon in ROOM.polygons()]


def points_along_walls(step=0.02):
    """Points spaced along every room edge, like a perfect scan of the walls."""
    points = []
    for x0, z0, x1, z1 in polygon_segments(POLYGONS):
        count = max(2, int(np.hypot(x1 - x0, z1 - z0) / step))
        t = np.linspace(0.0, 1.0, count)[:, None]
        points.append(np.hstack((x0 + t * (x1 - x0), z0 + t * (z1 - z0))))
    return np.vstack(points)


class WallGridTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = WallGrid(ROOM)

    def test_segment_distance(self):
        segments = np.array([[0.0, 0.0, 1.0, 0.0]])
        points = np.array([[0.5, 0.3], [-0.3, 0.4], [1.0, 0.0]])
        np.testing.assert_allclose(distance_to_segments(points, segments), [0.3, 0.5, 0.0])

    def test_grid_covers_room_with_margin(self):
        index = self.grid.world_to_index(polygon_array(ROOM.corners))
        self.assertTrue(self.grid.inside(index).all())

    def test_walls_score_high_and_shifted_walls_score_low(self):
        walls = points_along_walls()
        on_walls = self.grid.score(walls)
        self.assertGreater(on_walls, 0.9)
        self.assertLess(self.grid.score(walls + [0.5, 0.0]), 0.5 * on_walls)

    def test_pillar_is_a_wall(self):
        pillar = POLYGONS[1]
        center = pillar.mean(axis=0)
        face = np.array([[pillar[:, 0].min(), center[1]]])
        self.assertGreater(self.grid.likelihood_at(face)[0], 0.9)
        # The middle of the room, away from everything, is not.
        self.assertLess(self.grid.likelihood_at(np.array([[1.4, 7.0]]))[0], 0.01)

    def test_off_grid_points_score_zero(self):
        far = np.array([[100.0, 100.0]])
        self.assertEqual(self.grid.likelihood_at(far)[0], 0.0)
        self.assertEqual(self.grid.distance_at(far)[0], np.inf)

    def test_without_occupancy_refresh_does_nothing(self):
        self.assertFalse(self.grid.refresh())
        self.assertIs(self.grid.likelihood, self.grid.wall_likelihood)


class WallGridRefreshTest(unittest.TestCase):
    def setUp(self):
        self.original_weight = WallGrid.OCCUPANCY_WEIGHT
        WallGrid.OCCUPANCY_WEIGHT = 1.0
        self.occupancy = OccupancyGrid(ROOM)
        self.grid = WallGrid(ROOM, self.occupancy)

    def tearDown(self):
        WallGrid.OCCUPANCY_WEIGHT = self.original_weight

    def test_likelihood_changes_only_on_refresh(self):
        self.assertIs(self.grid.likelihood, self.grid.wall_likelihood)
        self.assertTrue(self.grid.refresh())
        self.assertEqual(self.grid.version, 1)
        # Unknown occupancy (0.5) with weight 1: (wall + 0.5) / 2.
        np.testing.assert_allclose(self.grid.likelihood, (self.grid.wall_likelihood + 0.5) / 2)
        self.assertFalse(self.grid.refresh())
        self.assertEqual(self.grid.version, 1)

    def test_occupancy_changes_wait_but_weight_changes_do_not(self):
        self.grid.refresh()
        pose = Transform2(position=Vector2(x=2.0, y=5.0))
        self.occupancy.update(pose, np.array([[2.05, 6.55]]))
        # The occupancy grid moved, but within REFRESH_INTERVAL of the last blend.
        self.assertFalse(self.grid.refresh())
        before = self.grid.likelihood
        WallGrid.OCCUPANCY_WEIGHT = 0.5
        self.assertTrue(self.grid.refresh())
        self.assertIsNot(self.grid.likelihood, before)
        # The filled cell now scores higher than an unknown one the same distance from walls.
        filled = self.grid.likelihood_at(np.array([[2.05, 6.55]]))[0]
        unknown = self.grid.likelihood_at(np.array([[2.05 + 0.3, 6.55]]))[0]
        self.assertGreater(filled, unknown)

    def test_occupancy_change_is_picked_up_after_the_interval(self):
        self.grid.refresh()
        self.occupancy.update(Transform2(position=Vector2(x=2.0, y=5.0)), np.array([[2.05, 6.55]]))
        self.grid._blended_at -= WallGrid.REFRESH_INTERVAL
        self.assertTrue(self.grid.refresh())


def save_image(path):
    import cv2

    grid = WallGrid(ROOM)
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
