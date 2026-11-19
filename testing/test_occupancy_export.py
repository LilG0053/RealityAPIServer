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
    center_weight,
    p_from_log_odds,
    point_to_segment_distance,
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

        # A hit at the center of cell (20, 65), and one far outside the room.
        origin = Transform(position=Vector3(x=2.0, z=5.0))
        self.assertEqual(grid.cells[(0, 0)].p_occupied, 0.5)
        grid.update(origin, np.array([[2.05, 0.0, 6.55], [100.0, 0.0, 100.0]]))
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

            # Pending set was cleared; a second export still rewrites touched cells
            # so a slow dashboard poll does not miss earlier colors.
            first_cells = set(cells)
            self.assertEqual(grid._pending_export_cells, set())
            grid.export(str(path))
            rewritten = json.loads(path.read_text())
            self.assertEqual(
                {(cell["x"], cell["z"]) for cell in rewritten["cells"]},
                first_cells,
            )
            self.assertFalse(rewritten["reset"])

            # A later scan keeps earlier cells in the file and adds any new ones.
            grid.update(origin, np.array([[2.05, 0.0, 6.55]]))
            grid.export(str(path))
            second = json.loads(path.read_text())
            second_cells = {(cell["x"], cell["z"]) for cell in second["cells"]}
            self.assertTrue(first_cells.issubset(second_cells))
            self.assertIn((20, 65), second_cells)
            self.assertTrue(second_cells.issubset(grid.room_cells))

            clear_path = Path(directory) / "cleared_occupancy_grid.json"
            grid.path = str(clear_path)
            grid.clear()
            cleared = json.loads(clear_path.read_text())
        self.assertEqual(cleared["cells"], [])
        self.assertTrue(cleared["reset"])
        self.assertEqual(grid._pending_export_cells, set())
        self.assertEqual(grid._touched_cells, set())

    def test_center_weight_and_tunable_gamma_follow_update_equation(self):
        names = {parameter["name"] for parameter in list_tunables()}
        self.assertIn("OccupancyGrid.GAMMA", names)
        self.assertIn("OccupancyGrid.CENTER_WEIGHT_B", names)
        self.assertAlmostEqual(OccupancyGrid.CENTER_WEIGHT_B, 0.5)
        self.assertNotIn("OccupancyGrid.DECAY", names)

        # Full strength at the center; zero at a cell corner (half-diagonal).
        # With b=1 the fade is linear in the unit-scaled offset.
        half_diagonal = 0.1 * (0.5 ** 0.5)
        self.assertAlmostEqual(center_weight(0.0, 0.1, b=1.0), 1.0)
        self.assertAlmostEqual(center_weight(half_diagonal, 0.1, b=1.0), 0.0)
        self.assertAlmostEqual(center_weight(half_diagonal / 2.0, 0.1, b=1.0), 0.5)

        # Hit on the segment projects to zero distance.
        self.assertAlmostEqual(
            point_to_segment_distance(1.0, 0.0, 0.0, 0.0, 2.0, 0.0),
            0.0,
        )
        self.assertAlmostEqual(
            point_to_segment_distance(1.0, 0.5, 0.0, 0.0, 2.0, 0.0),
            0.5,
        )

        cell = GridCell(x=0, z=0, p_occupied=0.5)
        cell.update(OCCUPIED_SCAN_ODDS, weight=1.0, gamma=0.5)
        self.assertAlmostEqual(cell.l_occupied, OCCUPIED_SCAN_ODDS, places=6)

        # Half weight blends halfway from prior (0) toward the scan odds.
        cell = GridCell(x=0, z=0, p_occupied=0.5)
        cell.update(OCCUPIED_SCAN_ODDS, weight=0.5, gamma=0.5)
        expected_first = 0.5 * OCCUPIED_SCAN_ODDS
        self.assertAlmostEqual(cell.l_occupied, expected_first, places=6)

        cell.update(OCCUPIED_SCAN_ODDS, weight=0.0, gamma=0.5)
        expected_second = 0.5 * expected_first
        self.assertEqual(cell.p_occupied, p_from_log_odds(expected_second))
        self.assertAlmostEqual(cell.l_occupied, expected_second, places=6)

        # Many tiny steps still refresh p once cumulative L past the last
        # p-refresh anchor crosses LOG_ODDS_P_REFRESH_THRESHOLD.
        cell = GridCell(x=0, z=0, p_occupied=0.5)
        for _ in range(40):
            cell.update(OCCUPIED_SCAN_ODDS, weight=0.05, gamma=1.0)
        self.assertGreater(cell.p_occupied, 0.5)
        self.assertEqual(cell.p_occupied, p_from_log_odds(cell.l_occupied))
        self.assertAlmostEqual(cell.l_at_last_p_refresh, cell.l_occupied, places=6)

        original = OccupancyGrid.GAMMA
        try:
            set_tunable("OccupancyGrid.GAMMA", 0.0)
            self.assertAlmostEqual(OccupancyGrid.GAMMA, 0.0)

            grid = OccupancyGrid()
            origin = Transform(position=Vector3(x=2.0, z=5.0))
            # Hit at the cell center is stronger than a hit near the cell edge.
            grid.update(origin, np.array([[2.05, 0.0, 6.05]]))
            on_center = grid.cells[(20, 60)].p_occupied
            grid = OccupancyGrid()
            grid.update(origin, np.array([[2.049, 0.0, 6.001]]))
            near_edge = grid.cells[(20, 60)].p_occupied
            self.assertGreaterEqual(on_center, near_edge)

            # With gamma=0, repeating the same centered hit is idempotent.
            grid = OccupancyGrid()
            grid.update(origin, np.array([[2.05, 0.0, 6.05]]))
            once = grid.cells[(20, 60)].p_occupied
            grid.update(origin, np.array([[2.05, 0.0, 6.05]]))
            self.assertEqual(grid.cells[(20, 60)].p_occupied, once)
        finally:
            OccupancyGrid.GAMMA = original


if __name__ == "__main__":
    unittest.main()
