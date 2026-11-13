"""Room occupancy in the server frame.

The cell lattice covers the whole room and stays fixed. Every cell starts at
probability 0.5. Cells a ray crosses are free evidence; the cell a ray ends in
is occupied evidence.

Exports are deltas: only cells touched since the last write. clear() exports
an empty list with reset=true so the dashboard can wipe its colors.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os

import numpy as np

from map.room import DISPLAY_OFFSET, cells_covering_room, server_room_corners
from map.tunable import TunableParameter
from realityapi_pb2 import Transform, Vector3

OCCUPANCY_PATH = "dashboard/occupancy_grid.json"

INITIAL_PROBABILITY = 0.5
FREE_SCAN_PROBABILITY = 0.2
OCCUPIED_SCAN_PROBABILITY = 0.8
DEFAULT_MAX_RANGE = 12.0
LOG_ODDS_BOUNDS = (-3.5, 3.5)
# Stored p is rounded to this many decimals. Skip exp when |ΔL| cannot change it:
# max |dp/dL| = 0.25 at p=0.5, so |ΔL| < 0.005 / 0.25 = 0.02.
PROBABILITY_DECIMALS = 2
LOG_ODDS_UPDATE_THRESHOLD = 0.02

CellIndex = tuple[int, int]


def clamp_log_odds(value: float) -> float:
    low, high = LOG_ODDS_BOUNDS
    return max(low, min(high, value))


def log_odds(probability: float) -> float:
    return clamp_log_odds(math.log(probability / (1.0 - probability)))


def probability_from_log_odds(log_odds_value: float) -> float:
    """Convert log-odds to probability, rounded to PROBABILITY_DECIMALS."""
    return round(1.0 / (1.0 + math.exp(-log_odds_value)), PROBABILITY_DECIMALS)


def p_from_log_odds(value: float) -> float:
    return probability_from_log_odds(clamp_log_odds(value))


def range_weight(distance: float, max_range: float) -> float:
    """How strongly this measurement counts: 1 at the robot, 0.5 at max_range."""
    return 1.0 - (distance / (max_range * 2.0))


FREE_SCAN_ODDS = log_odds(FREE_SCAN_PROBABILITY)
OCCUPIED_SCAN_ODDS = log_odds(OCCUPIED_SCAN_PROBABILITY)


def keep_nearest(ranges: dict[CellIndex, float], cell: CellIndex, distance: float) -> None:
    """Keep the shortest distance seen for this cell in the current scan."""
    previous = ranges.get(cell)
    if previous is None or distance < previous:
        ranges[cell] = distance


@dataclass
class GridCell:
    x: int
    z: int
    p_occupied: float = INITIAL_PROBABILITY

    def __post_init__(self) -> None:
        self.p_initial = self.p_occupied
        self.l_initial = log_odds(self.p_initial)
        self.l_occupied = self.l_initial

    def update(self, l_scan: float, distance: float, max_range: float, gamma: float) -> bool:
        """Update log-odds; recompute p only when |ΔL| can change two-decimal probability.

        Returns True when p_occupied changed (caller may queue an export).
        """
        weight = range_weight(distance, max_range)
        updated = clamp_log_odds(
            self.l_initial
            + weight * (l_scan - self.l_initial)
            + gamma * (self.l_occupied - self.l_initial)
        )
        delta_l = updated - self.l_occupied
        self.l_occupied = updated
        if abs(delta_l) < LOG_ODDS_UPDATE_THRESHOLD:
            return False

        new_probability = probability_from_log_odds(self.l_occupied)
        if new_probability == self.p_occupied:
            return False
        self.p_occupied = new_probability
        return True

    def as_export(self) -> dict:
        return {"x": self.x, "z": self.z, "p": self.p_occupied}


class _CellMap(dict[CellIndex, GridCell]):
    """Sparse observed cells; untouched room cells stay at the 0.5 prior."""

    def __init__(self, room_cells: frozenset[CellIndex]) -> None:
        super().__init__()
        self._room_cells = room_cells

    def __missing__(self, key: CellIndex) -> GridCell:
        if key not in self._room_cells:
            raise KeyError(key)
        cell = GridCell(key[0], key[1])
        self[key] = cell
        return cell


class OccupancyGrid:
    GAMMA = TunableParameter(
        0.95, minimum=0.0, maximum=1.0, step=0.01, label="Occupancy gamma",
    )

    def __init__(self, path: str = OCCUPANCY_PATH, r_max: float = DEFAULT_MAX_RANGE) -> None:
        self.path = path
        self.r_max = r_max
        self.cell_size = 0.10
        self.room_corners = server_room_corners()
        self.room_cells = cells_covering_room(self.room_corners, self.cell_size)
        self.cells: dict[CellIndex, GridCell] = _CellMap(self.room_cells)
        self._pending_export_cells: set[CellIndex] = set()

    def world_to_cell(self, x: float, z: float) -> CellIndex:
        return (math.floor(x / self.cell_size), math.floor(z / self.cell_size))

    @staticmethod
    def cast_ray(
        start: CellIndex,
        end: CellIndex,
        hit_range: float,
    ) -> dict[CellIndex, float]:
        """Bresenham walk from start to end with a range on each cell.

        Spreads the true Euclidean hit distance evenly across grid steps:

            distance_i = hit_range * i / n_steps

        Start is 0; end is hit_range. Returns {cell: distance} for the whole ray.
        """
        x, z = start
        end_x, end_z = end
        delta_x = abs(end_x - x)
        delta_z = abs(end_z - z)
        step_count = max(delta_x, delta_z)
        distance_per_step = hit_range / step_count if step_count else 0.0

        step_x = 1 if x < end_x else -1
        step_z = 1 if z < end_z else -1
        error = delta_x - delta_z

        distances: dict[CellIndex, float] = {}
        step = 0
        while True:
            distances[(x, z)] = step * distance_per_step if step_count else hit_range
            if x == end_x and z == end_z:
                break

            doubled_error = 2 * error
            if doubled_error > -delta_z:
                error -= delta_z
                x += step_x
            if doubled_error < delta_x:
                error += delta_x
                z += step_z
            step += 1

        return distances

    def _apply_evidence(
        self,
        distances: dict[CellIndex, float],
        l_scan: float,
        max_range: float,
        gamma: float,
    ) -> None:
        """Given a map of cells to their respective distances from the robot, update the cells' states.
        
        The log odds are precomputed based on whether these cells are occupied or free.
        """
        for cell, distance in distances.items():
            if cell not in self.room_cells:
                continue
            if self.cells[cell].update(l_scan, distance, max_range, gamma):
                self._pending_export_cells.add(cell)

    def update(
        self,
        robot_pos: Transform,
        corrected_points: np.ndarray,
        r_max: float | None = None,
    ) -> None:
        """Incorporate server-frame LiDAR hits into the room grid.

        Each hit: one Euclidean range, then cast_ray. Across rays the nearest
        distance wins. Ray endpoints are occupied; earlier cells are free.
        Occupied overrides free. Out-of-room cells are skipped.
        """
        max_range = (
            r_max if r_max is not None and math.isfinite(r_max) and r_max > 0.0 else self.r_max
        )
        gamma = self.GAMMA
        robot_x = robot_pos.position.x
        robot_z = robot_pos.position.z
        robot_cell = self.world_to_cell(robot_x, robot_z)

        free_distances: dict[CellIndex, float] = {}
        occupied_distances: dict[CellIndex, float] = {}

        for hit_x, _hit_y, hit_z in corrected_points:
            hit_x = float(hit_x)
            hit_z = float(hit_z)
            hit_cell = self.world_to_cell(hit_x, hit_z)
            hit_range = math.hypot(hit_x - robot_x, hit_z - robot_z)

            for cell, distance in self.cast_ray(robot_cell, hit_cell, hit_range).items():
                target = occupied_distances if cell == hit_cell else free_distances
                keep_nearest(target, cell, distance)

        for cell in occupied_distances:
            free_distances.pop(cell, None)

        self._apply_evidence(free_distances, FREE_SCAN_ODDS, max_range, gamma)
        self._apply_evidence(occupied_distances, OCCUPIED_SCAN_ODDS, max_range, gamma)

    def update_dashboard(
        self,
        robot_pos: Transform,
        corrected_points: np.ndarray,
        r_max: float | None = None,
    ) -> None:
        """Shift dashboard-frame pose/cloud into the server frame, then update."""
        origin_x, origin_z = DISPLAY_OFFSET[0], DISPLAY_OFFSET[2]
        server_pose = Transform(
            position=Vector3(
                x=robot_pos.position.x - origin_x,
                y=robot_pos.position.y,
                z=robot_pos.position.z - origin_z,
            ),
            rotation=robot_pos.rotation,
        )
        points = np.array(corrected_points, dtype=float, copy=True)
        points[:, 0] -= origin_x
        points[:, 2] -= origin_z
        self.update(server_pose, points, r_max=r_max)

    def export(self, filepath: str | None = None, *, reset: bool = False) -> None:
        """Write pending cell updates (or an empty reset) and clear the pending set."""
        cells = [] if reset else [self.cells[cell].as_export() for cell in self._pending_export_cells]
        payload = {
            "frame": "server",
            "cell_size": self.cell_size,
            "origin_x": DISPLAY_OFFSET[0],
            "origin_z": DISPLAY_OFFSET[2],
            "reset": reset,
            "cells": cells,
        }
        destination = filepath if filepath is not None else self.path
        temporary_path = destination + ".tmp"
        try:
            with open(temporary_path, "w") as file:
                file.write(json.dumps(payload))
            os.replace(temporary_path, destination)
            self._pending_export_cells.clear()
        except OSError as error:
            print(f"[!] could not write {destination}: {error}")

    def export_pending_updates(self) -> None:
        if self._pending_export_cells:
            self.export()

    def clear(self) -> None:
        """Drop observed cells and tell the dashboard to wipe its colors."""
        self.cells.clear()
        self._pending_export_cells.clear()
        self.export(reset=True)
