"""Room occupancy in the server frame.

The cell lattice covers the whole room and stays fixed. Every cell starts at
probability 0.5. Cells a ray crosses are free evidence; the cell a ray ends in
is occupied evidence.

Scan strength is scaled by how close the ray feature is to the cell center:
occupied uses the hit-to-center distance, free uses the line-to-center distance.
A feature through the center keeps full scan log-odds; toward the cell edge the
update fades to zero.

Exports list every cell touched since the last clear, with current p. That way
a missed poll still catches up on the next file. clear() writes an empty list
with reset=true so the dashboard can wipe its colors.
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
LOG_ODDS_BOUNDS = (-3.5, 3.5)
# Stored p is rounded to this many decimals.
PROBABILITY_DECIMALS = 2
# Skip the sigmoid (exp) that refreshes p_occupied until cumulative log-odds has
# moved enough that a two-decimal probability *could* change.
# At p=0.5, |dp/dL| peaks at 0.25, so |ΔL| < 0.005 / 0.25 = 0.02 cannot change
# a value rounded to 0.01. This threshold is compared to L since the last p
# refresh — not to the size of the latest single update.
LOG_ODDS_P_REFRESH_THRESHOLD = 0.02

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


def center_weight(offset: float, cell_size: float, b: float | None = None) -> float:
    """How strongly a ray feature counts, from cell-center distance.

    1 when the hit or line passes through the cell center; 0 at a cell corner
    (half-diagonal away). ``b`` controls the fade curve; b=1 is linear in the
    normalized offset (default tunable is 0.5).
    """
    if cell_size <= 0.0:
        return 0.0
    if b is None:
        b = OccupancyGrid.CENTER_WEIGHT_B
    # Unit-cell form passes through (0, 1) and (sqrt(0.5), 0); scale offset by
    # cell_size so the zero stays at this cell's half-diagonal.
    t = math.sqrt(2.0) * offset / cell_size
    if t <= 0.0:
        return 1.0
    if t >= 1.0:
        return 0.0
    return (1.0 - t**b) ** (1.0 / b)


def point_to_segment_distance(
    point_x: float,
    point_z: float,
    start_x: float,
    start_z: float,
    end_x: float,
    end_z: float,
) -> float:
    """Shortest distance from a point to a line segment in the XZ plane."""
    delta_x = end_x - start_x
    delta_z = end_z - start_z
    length_sq = delta_x * delta_x + delta_z * delta_z
    if length_sq == 0.0:
        return math.hypot(point_x - start_x, point_z - start_z)
    t = ((point_x - start_x) * delta_x + (point_z - start_z) * delta_z) / length_sq
    t = max(0.0, min(1.0, t))
    closest_x = start_x + t * delta_x
    closest_z = start_z + t * delta_z
    return math.hypot(point_x - closest_x, point_z - closest_z)


FREE_SCAN_ODDS = log_odds(FREE_SCAN_PROBABILITY)
OCCUPIED_SCAN_ODDS = log_odds(OCCUPIED_SCAN_PROBABILITY)


def keep_nearest(offsets: dict[CellIndex, float], cell: CellIndex, offset: float) -> None:
    """Keep the smallest center-offset seen for this cell in the current scan."""
    previous = offsets.get(cell)
    if previous is None or offset < previous:
        offsets[cell] = offset


@dataclass
class GridCell:
    x: int
    z: int
    p_occupied: float = INITIAL_PROBABILITY

    def __post_init__(self) -> None:
        self.p_initial = self.p_occupied
        # Prior log-odds for this cell (from p_initial). Fixed for the cell's life.
        self.l_initial = log_odds(self.p_initial)
        # Current belief in log-odds. Updated on every scan that hits this cell.
        self.l_occupied = self.l_initial
        # Log-odds value the last time we ran the expensive L→p conversion and
        # possibly changed p_occupied. Used only by the skip-exp optimization
        # below — not part of the Bayesian update itself.
        self.l_at_last_p_refresh = self.l_occupied

    def update(self, l_scan: float, weight: float, gamma: float) -> bool:
        """Update log-odds; refresh rounded p only when cumulative L moved enough.

        weight scales how much of l_scan is applied (1 at cell center, 0 at the edge).

        Always writes the new total into l_occupied. The displayed probability
        p_occupied is rounded to PROBABILITY_DECIMALS, so converting L→p (a
        sigmoid / exp) is skipped until |l_occupied - l_at_last_p_refresh| reaches
        LOG_ODDS_P_REFRESH_THRESHOLD. Comparing against l_at_last_p_refresh — not
        against the previous step's ΔL — means many tiny updates still refresh p
        once their combined drift is large enough.

        Returns True when p_occupied changed (caller may queue an export).
        """
        self.l_occupied = clamp_log_odds(
            self.l_initial
            + weight * (l_scan - self.l_initial)
            + gamma * (self.l_occupied - self.l_initial)
        )

        # Optimization: skip exp until cumulative L since the last p refresh
        # could change a two-decimal probability.
        cumulative_delta_l = self.l_occupied - self.l_at_last_p_refresh
        if abs(cumulative_delta_l) < LOG_ODDS_P_REFRESH_THRESHOLD:
            return False

        new_probability = probability_from_log_odds(self.l_occupied)
        # Advance the refresh anchor even when rounded p is unchanged, so the
        # next checks measure drift from this L rather than re-running exp every
        # scan after the threshold was crossed once.
        self.l_at_last_p_refresh = self.l_occupied
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
    CENTER_WEIGHT_B = TunableParameter(
        0.5, minimum=0.01, maximum=4.0, step=0.01, label="Occupancy center-weight b",
    )

    def __init__(self, path: str = OCCUPANCY_PATH) -> None:
        self.path = path
        self.cell_size = 0.10
        self.room_corners = server_room_corners()
        self.room_cells = cells_covering_room(self.room_corners, self.cell_size)
        self.cells: dict[CellIndex, GridCell] = _CellMap(self.room_cells)
        # Cells whose p changed since the last clear; rewritten on every export.
        self._touched_cells: set[CellIndex] = set()
        # Subset of _touched_cells that changed since the last successful write.
        self._pending_export_cells: set[CellIndex] = set()

    def world_to_cell(self, x: float, z: float) -> CellIndex:
        return (math.floor(x / self.cell_size), math.floor(z / self.cell_size))

    def cell_center(self, cell: CellIndex) -> tuple[float, float]:
        return ((cell[0] + 0.5) * self.cell_size, (cell[1] + 0.5) * self.cell_size)

    @staticmethod
    def cast_ray(start: CellIndex, end: CellIndex) -> list[CellIndex]:
        """Bresenham cells from start to end, including both ends."""
        x, z = start
        end_x, end_z = end
        delta_x = abs(end_x - x)
        delta_z = abs(end_z - z)
        step_x = 1 if x < end_x else -1
        step_z = 1 if z < end_z else -1
        error = delta_x - delta_z

        cells: list[CellIndex] = []
        while True:
            cells.append((x, z))
            if x == end_x and z == end_z:
                break

            doubled_error = 2 * error
            if doubled_error > -delta_z:
                error -= delta_z
                x += step_x
            if doubled_error < delta_x:
                error += delta_x
                z += step_z

        return cells

    def _apply_evidence(
        self,
        offsets: dict[CellIndex, float],
        l_scan: float,
        gamma: float,
    ) -> None:
        """Update cells using center-offset weights for this scan's evidence."""
        for cell, offset in offsets.items():
            if cell not in self.room_cells:
                continue
            weight = center_weight(offset, self.cell_size)
            if weight <= 0.0:
                continue
            if self.cells[cell].update(l_scan, weight, gamma):
                self._touched_cells.add(cell)
                self._pending_export_cells.add(cell)

    def update(
        self,
        robot_pos: Transform,
        corrected_points: np.ndarray,
    ) -> None:
        """Incorporate server-frame LiDAR hits into the room grid.

        Each hit walks Bresenham to its cell. Free cells store the distance from
        the continuous ray to the cell center; the hit cell stores the distance
        from the hit to the cell center. Across rays the nearest offset wins.
        Occupied overrides free. Out-of-room cells are skipped.
        """
        gamma = self.GAMMA
        robot_x = robot_pos.position.x
        robot_z = robot_pos.position.z
        robot_cell = self.world_to_cell(robot_x, robot_z)

        free_offsets: dict[CellIndex, float] = {}
        occupied_offsets: dict[CellIndex, float] = {}

        for hit_x, _hit_y, hit_z in corrected_points:
            hit_x = float(hit_x)
            hit_z = float(hit_z)
            hit_cell = self.world_to_cell(hit_x, hit_z)

            for cell in self.cast_ray(robot_cell, hit_cell):
                center_x, center_z = self.cell_center(cell)
                if cell == hit_cell:
                    offset = math.hypot(hit_x - center_x, hit_z - center_z)
                    keep_nearest(occupied_offsets, cell, offset)
                else:
                    offset = point_to_segment_distance(
                        center_x, center_z, robot_x, robot_z, hit_x, hit_z,
                    )
                    keep_nearest(free_offsets, cell, offset)

        for cell in occupied_offsets:
            free_offsets.pop(cell, None)

        self._apply_evidence(free_offsets, FREE_SCAN_ODDS, gamma)
        self._apply_evidence(occupied_offsets, OCCUPIED_SCAN_ODDS, gamma)

    def update_dashboard(
        self,
        robot_pos: Transform,
        corrected_points: np.ndarray,
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
        self.update(server_pose, points)

    def export(self, filepath: str | None = None, *, reset: bool = False) -> None:
        """Write touched cells (or an empty reset) and clear the pending-write set.

        Each non-reset write includes every cell touched since clear(), so a
        dashboard poll that skipped an intermediate file still gets current colors.
        """
        cells = [] if reset else [self.cells[cell].as_export() for cell in self._touched_cells]
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
        self._touched_cells.clear()
        self._pending_export_cells.clear()
        self.export(reset=True)
