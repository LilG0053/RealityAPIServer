"""Verify occupancy evidence and the dashboard export format."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from map.map import Map
from realityapi_pb2 import Transform, Vector3


class OccupancyExportTest(unittest.TestCase):
    def test_ray_marks_free_hit_and_keeps_unobserved_unknown(self):
        world = Map()
        world.cell_size = 1.0
        origin = Transform(position=Vector3(x=0.2, z=0.2))
        world.update_occupancy_grid(origin, np.array([[3.2, 0.0, 0.2]]))

        self.assertEqual(world.occupancy_grid[(0, 0)], -1)
        self.assertEqual(world.occupancy_grid[(1, 0)], -1)
        self.assertEqual(world.occupancy_grid[(3, 0)], 3)
        self.assertNotIn((4, 0), world.occupancy_grid)

        # A zero value is observed but balanced; absent cells are unknown.
        world.occupancy_grid[(0, 1)] = 0
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'occupancy_grid.json'
            world.export_occupancy_grid(str(path))
            data = json.loads(path.read_text())
        self.assertEqual(data['cell_size'], 1.0)
        cells = {(cell['x'], cell['z']): cell['status'] for cell in data['cells']}
        self.assertEqual(cells[(0, 1)], 0)
        self.assertEqual(cells[(3, 0)], 1)
        self.assertNotIn((4, 0), cells)


if __name__ == '__main__':
    unittest.main()
