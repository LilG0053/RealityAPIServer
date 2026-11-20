"""The room loaded from a blockout (map/room.py)."""

import json
import sys
import tempfile
import unittest

import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from map.room import BLOCKOUT_PATH, Room, cells_covering_room, display_offset, point_in_polygon
from realityapi_pb2 import Polygon2, Vector2, Vector3


def square(x0, z0, x1, z1):
    return Polygon2(vertices=[Vector2(x=x0, y=z0), Vector2(x=x0, y=z1), Vector2(x=x1, y=z1), Vector2(x=x1, y=z0)])


class RoomTest(unittest.TestCase):
    def test_loads_the_lab_blockout(self):
        room = Room.from_json(BLOCKOUT_PATH)
        data = json.loads(Path(BLOCKOUT_PATH).read_text())
        # Vector2 stores float32.
        np.testing.assert_allclose([(v.x, v.y) for v in room.outline.vertices], data["room"], atol=1e-5)
        complete = [o for o in data["obstacles"] if len(o.get("polygon") or []) >= 3]
        self.assertEqual(len(room.obstacles), len(complete))

    def test_skips_obstacles_that_are_not_measured_yet(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "blockout.json"
            path.write_text(json.dumps({
                "room": [[0, 0], [0, 4], [3, 4], [3, 0]],
                "obstacles": [
                    {"name": "box", "polygon": [[1, 1], [1, 2], [2, 2], [2, 1]]},
                    {"name": "todo", "polygon": [[]]},
                ],
            }))
            room = Room.from_json(path, offset=Vector3())
        self.assertEqual([o.name for o in room.obstacles], ["box"])

    def test_rejects_a_bad_outline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "blockout.json"
            path.write_text(json.dumps({"room": [[0, 0], [1, 1]]}))
            with self.assertRaises(ValueError):
                Room.from_json(path)

    def test_server_frame_and_bounds_use_the_offset(self):
        room = Room(square(0, 0, 4, 6), (), offset=Vector3(x=1.0, z=2.0))
        self.assertEqual([(v.x, v.y) for v in room.corners.vertices], [(-1, -2), (-1, 4), (3, 4), (3, -2)])
        bounds = room.bounds()
        self.assertEqual((bounds.origin.x, bounds.origin.y), (1.0, 2.0))
        self.assertEqual(bounds.room, room.outline)
        self.assertTrue(room.contains(0.0, 0.0))
        self.assertFalse(room.contains(3.5, 0.0))

    def test_each_room_gets_its_own_offset(self):
        a, b = Room(square(0, 0, 1, 1), ()), Room(square(0, 0, 1, 1), ())
        self.assertEqual(a.offset, display_offset())
        self.assertIsNot(a.offset, b.offset)

    def test_cells_covering_room(self):
        cells = cells_covering_room(square(0.0, 0.0, 1.0, 0.5), 0.25)
        self.assertEqual(cells, {(i, j) for i in range(4) for j in range(2)})
        self.assertTrue(point_in_polygon(0.5, 0.25, square(0.0, 0.0, 1.0, 0.5)))


if __name__ == "__main__":
    unittest.main()
