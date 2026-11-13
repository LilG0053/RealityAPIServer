"""The occupancy grid is the whole room, fixed in the server frame."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from map.occupancy import (
    GridCell,
    OCCUPIED_SCAN_ODDS,
    OccupancyGrid,
    log_odds,
    p_from_log_odds,
    range_weight,
)
from map.room import DISPLAY_OFFSET, point_in_polygon, server_room_corners
from map.tunable import list_tunables, set_tunable
from realityapi_pb2 import Transform, Vector3


class OccupancyExportTest(unittest.TestCase):
    def test_grid_covers_the_room_and_exports_only_pending_cells(self):
        grid = OccupancyGrid()
        corners = server_room_corners()
        self.assertGreater(len(grid.room_cells), 10000)

        for cell_x, cell_z in grid.room_cells:
            size = grid.cell_size
            samples = (
                ((cell_x + 0.5) * size, (cell_z + 0.5) * size),
                (cell_x * size, cell_z * size),
                ((cell_x + 1) * size, cell_z * size),
                (cell_x * size, (cell_z + 1) * size),
                ((cell_x + 1) * size, (cell_z + 1) * size),
            )
            self.assertTrue(any(point_in_polygon(x, z, corners) for x, z in samples))

        # A hit inside the left half of the room, and one far outside it.
        origin = Transform(position=Vector3(x=2.0, z=5.0))
        self.assertEqual(grid.cells[(0, 0)].p_occupied, 0.5)
        grid.update(origin, np.array([[2.0, 0.0, 6.5], [100.0, 0.0, 100.0]]))
        self.assertLess(grid.cells[(20, 50)].p_occupied, 0.5)
        self.assertGreater(grid.cells[(20, 65)].p_occupied, 0.5)
        self.assertNotIn((1000, 1000), grid.cells)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "occupancy_grid.json"
            grid.export(str(path))
            data = json.loads(path.read_text())

            self.assertEqual(data["frame"], "server")
            self.assertEqual(data["cell_size"], 0.1)
            self.assertEqual(data["origin_x"], DISPLAY_OFFSET[0])
            self.assertEqual(data["origin_z"], DISPLAY_OFFSET[2])
            self.assertFalse(data["reset"])
            cells = {(cell["x"], cell["z"]): cell for cell in data["cells"]}
            self.assertTrue(set(cells).issubset(grid.room_cells))
            self.assertLess(len(cells), 200)
            self.assertGreater(cells[(20, 65)]["p"], 0.5)
            self.assertLess(cells[(20, 50)]["p"], 0.5)
            self.assertNotIn("status", cells[(20, 65)])
            self.assertNotIn((0, 0), cells)
            self.assertNotIn((1000, 1000), cells)

            # Pending set was cleared; a second export with no new updates is empty.
            self.assertEqual(grid._pending_export_cells, set())
            grid.export(str(path))
            empty = json.loads(path.read_text())
            self.assertEqual(empty["cells"], [])
            self.assertFalse(empty["reset"])

            # A later scan only exports cells touched since the previous export.
            grid.update(origin, np.array([[2.0, 0.0, 6.5]]))
            grid.export(str(path))
            second = json.loads(path.read_text())
            second_cells = {(cell["x"], cell["z"]) for cell in second["cells"]}
            self.assertIn((20, 65), second_cells)
            self.assertTrue(second_cells.issubset(grid.room_cells))

            clear_path = Path(directory) / "cleared_occupancy_grid.json"
            grid.path = str(clear_path)
            grid.clear()
            cleared = json.loads(clear_path.read_text())
        self.assertEqual(cleared["cells"], [])
        self.assertTrue(cleared["reset"])
        self.assertEqual(grid._pending_export_cells, set())

    def test_range_weight_and_tunable_gamma_follow_update_equation(self):
        names = {parameter["name"] for parameter in list_tunables()}
        self.assertIn("OccupancyGrid.GAMMA", names)
        self.assertNotIn("OccupancyGrid.DECAY", names)

        # w(r) = 1 / (1 + r / rmax): 1.0 at r=0 and 0.5 at r=rmax.
        self.assertAlmostEqual(range_weight(0.0, 10.0), 1.0)
        self.assertAlmostEqual(range_weight(10.0, 10.0), 0.5)

        cell = GridCell(x=0, z=0, p_occupied=0.5)
        cell.update(OCCUPIED_SCAN_ODDS, distance=5.0, max_range=10.0, gamma=0.5)
        l_first = cell.l_occupied
        expected_first = (1.0 / 1.5) * OCCUPIED_SCAN_ODDS
        self.assertAlmostEqual(l_first, expected_first, places=6)

        # Second update checks l_new = l_initial + w(r)(l_scan - l_initial) + gamma(l_prev - l_initial).
        cell.update(OCCUPIED_SCAN_ODDS, distance=10.0, max_range=10.0, gamma=0.5)
        expected_second = 0.5 * OCCUPIED_SCAN_ODDS + 0.5 * expected_first
        self.assertEqual(cell.p_occupied, p_from_log_odds(expected_second))
        self.assertAlmostEqual(cell.l_occupied, expected_second, places=6)

        original = OccupancyGrid.GAMMA
        try:
            set_tunable("OccupancyGrid.GAMMA", 0.0)
            self.assertAlmostEqual(OccupancyGrid.GAMMA, 0.0)

            grid = OccupancyGrid()
            origin = Transform(position=Vector3(x=2.0, z=5.0))
            # Near hit gets stronger confidence than a far hit.
            grid.update(origin, np.array([[2.0, 0.0, 6.0], [2.0, 0.0, 11.0]]), r_max=10.0)
            self.assertGreater(grid.cells[(20, 60)].p_occupied, grid.cells[(20, 110)].p_occupied)

            # Free cells nearer on the same Bresenham ray get a stronger free update.
            grid = OccupancyGrid()
            grid.update(origin, np.array([[2.0, 0.0, 8.0]]), r_max=10.0)
            self.assertLess(grid.cells[(20, 55)].p_occupied, grid.cells[(20, 70)].p_occupied)

            # With gamma=0, repeating the same scan forgets the previous measurement instead of compounding.
            grid = OccupancyGrid()
            grid.update(origin, np.array([[2.0, 0.0, 6.0]]), r_max=10.0)
            once = grid.cells[(20, 60)].p_occupied
            grid.update(origin, np.array([[2.0, 0.0, 6.0]]), r_max=10.0)
            self.assertEqual(grid.cells[(20, 60)].p_occupied, once)
        finally:
            OccupancyGrid.GAMMA = original


if __name__ == "__main__":
    unittest.main()
