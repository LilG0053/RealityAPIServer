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
    OCCUPIED_SCAN_ODDS,
    GridCell,
    OccupancyGrid,
    center_weight,
    p_from_log_odds,
    point_to_segment_distance,
)
from map.room import BLOCKOUT_PATH, Room, point_in_polygon
from map.tunable import list_tunables, set_tunable
from realityapi_pb2 import Transform2, Vector2

ROOM = Room.from_json(BLOCKOUT_PATH)
# Robot at (2, 5) in the server frame; Transform2's Vector2.y is Z.
ORIGIN = Transform2(position=Vector2(x=2.0, y=5.0))


class OccupancyExportTest(unittest.TestCase):
    def test_grid_covers_the_room_and_exports_only_pending_cells(self):
        grid = OccupancyGrid(ROOM)
        corners = ROOM.corners
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
        self.assertEqual(grid.cells[(0, 0)].p_occupied, 0.5)
        grid.update(ORIGIN, np.array([[2.05, 6.55], [100.0, 100.0]]))
        self.assertLess(grid.cells[(20, 50)].p_occupied, 0.5)
        self.assertGreater(grid.cells[(20, 65)].p_occupied, 0.5)
        self.assertNotIn((1000, 1000), grid.cells)

        # A dashboard with nothing applied yet gets every observed cell.
        first = grid.snapshot(0)
        self.assertTrue(first.reset)
        self.assertAlmostEqual(first.cell_size, 0.1, places=6)
        cells = {(cell.x, cell.z): cell for cell in first.cells}
        self.assertTrue(set(cells).issubset(grid.room_cells))
        self.assertLess(len(cells), 200)
        self.assertGreater(cells[(20, 65)].p, 0.5)
        self.assertLess(cells[(20, 50)].p, 0.5)
        self.assertNotIn((0, 0), cells)
        self.assertNotIn((1000, 1000), cells)

        # Nothing changed since that revision, so the next fetch is empty.
        unchanged = grid.snapshot(first.revision)
        self.assertFalse(unchanged.reset)
        self.assertEqual(list(unchanged.cells), [])
        self.assertEqual(unchanged.revision, first.revision)

        # A later scan only sends cells changed since the revision applied.
        grid.update(ORIGIN, np.array([[2.05, 6.55]]))
        second = grid.snapshot(first.revision)
        self.assertFalse(second.reset)
        self.assertGreater(second.revision, first.revision)
        second_cells = {(cell.x, cell.z) for cell in second.cells}
        self.assertIn((20, 65), second_cells)
        self.assertTrue(second_cells.issubset(grid.room_cells))

        # After clear, a dashboard holding an older revision is told to wipe.
        grid.clear()
        cleared = grid.snapshot(second.revision)
        self.assertTrue(cleared.reset)
        self.assertEqual(list(cleared.cells), [])
        self.assertFalse(grid.snapshot(cleared.revision).reset)

        # A revision from another server run (newer than this one) also resets.
        self.assertTrue(grid.snapshot(cleared.revision + 10**12).reset)

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

        original = OccupancyGrid.GAMMA
        try:
            set_tunable("OccupancyGrid.GAMMA", 0.0)
            self.assertAlmostEqual(OccupancyGrid.GAMMA, 0.0)

            grid = OccupancyGrid(ROOM)
            # Hit at the cell center is stronger than a hit near the cell edge.
            grid.update(ORIGIN, np.array([[2.05, 6.05]]))
            on_center = grid.cells[(20, 60)].p_occupied
            grid = OccupancyGrid(ROOM)
            grid.update(ORIGIN, np.array([[2.049, 6.001]]))
            near_edge = grid.cells[(20, 60)].p_occupied
            self.assertGreaterEqual(on_center, near_edge)

            # With gamma=0, repeating the same centered hit is idempotent.
            grid = OccupancyGrid(ROOM)
            grid.update(ORIGIN, np.array([[2.05, 6.05]]))
            once = grid.cells[(20, 60)].p_occupied
            grid.update(ORIGIN, np.array([[2.05, 6.05]]))
            self.assertEqual(grid.cells[(20, 60)].p_occupied, once)
        finally:
            OccupancyGrid.GAMMA = original


class OccupancyStorageTest(unittest.TestCase):
    def test_without_a_path_nothing_touches_disk(self):
        grid = OccupancyGrid(ROOM)
        grid.update(ORIGIN, np.array([[2.05, 6.55]]))
        self.assertFalse(grid.save())

    def test_save_and_load_round_trip_as_x_z_p_arrays(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grid.json"
            grid = OccupancyGrid(ROOM, path)
            self.assertFalse(grid.save())  # nothing changed yet
            grid.update(ORIGIN, np.array([[2.05, 6.55], [3.05, 6.55]]))
            self.assertTrue(grid.save())
            self.assertFalse(grid.save())  # already saved

            payload = json.loads(path.read_text())
            self.assertEqual(payload["cell_size"], 0.1)
            self.assertTrue(all(isinstance(cell, list) and len(cell) == 3 for cell in payload["cells"]))
            saved = {(x, z): p for x, z, p in payload["cells"]}

            loaded = OccupancyGrid(ROOM, path)
        restored = {(c.x, c.z): c.p for c in loaded.snapshot(0).cells}
        self.assertEqual(restored.keys(), saved.keys())
        for index, p in saved.items():
            self.assertAlmostEqual(restored[index], p, places=2)
        # Loading is not a change, so there is nothing new to save.
        self.assertFalse(loaded.save())

    def test_old_object_format_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grid.json"
            path.write_text(json.dumps({"cell_size": 0.1, "cells": [{"x": 20, "z": 65, "p": 0.9}]}))
            grid = OccupancyGrid(ROOM, path)
        self.assertEqual(list(grid.snapshot(0).cells), [])

    def test_probabilities_at(self):
        grid = OccupancyGrid(ROOM)
        np.testing.assert_allclose(grid.probabilities_at(np.array([[2.05, 6.55]])), [0.5])
        grid.update(ORIGIN, np.array([[2.05, 6.55]]))
        hit, free, far = grid.probabilities_at(np.array([[2.05, 6.55], [2.05, 5.95], [50.0, 50.0]]))
        self.assertGreater(hit, 0.5)
        self.assertLess(free, 0.5)
        self.assertEqual(far, 0.5)


if __name__ == "__main__":
    unittest.main()
