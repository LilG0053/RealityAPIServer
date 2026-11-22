"""Room occupancy in the server frame.

The cell lattice covers the whole room and stays fixed. Every cell starts at
probability 0.5. Cells a ray crosses are free evidence; the cell a ray ends in
is occupied evidence.

Scan strength is scaled by how close the ray feature is to the cell center:
occupied uses the hit-to-center distance, free uses the line-to-center distance.
A feature through the center keeps full scan log-odds; toward the cell edge the
update fades to zero.

Dashboards fetch the grid over the websocket. Every cell change gets a new
revision number; a dashboard sends back the revision it has applied and gets
only the cells changed since. After clear(), or when the dashboard's revision
is from an earlier server run, the reply has reset=true and every observed cell.

A grid made with a file path loads from that file when it is created and
save() writes back to it (the server saves at most once a second while it
changes). A grid made without a path never touches the disk.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from map.room import Room, cells_covering_room
from map.tunable import TunableParameter
from realityapi_pb2 import OccupancyCell, Transform2
from realityapi_pb2 import OccupancyGrid as OccupancyGridMessage

INITIAL_PROBABILITY = 0.5
FREE_SCAN_PROBABILITY = 0.2
OCCUPIED_SCAN_PROBABILITY = 0.8
LOG_ODDS_BOUNDS = (-3.5, 3.5)
# Stored p is rounded to this many decimals.
PROBABILITY_DECIMALS = 2
# Skip the sigmoid (exp) that refreshes p_occupied until log-odds has moved this
# far since the last refresh. At p=0.5, |dp/dL| peaks at 0.25, so |ΔL| < 0.02
# cannot change a value rounded to 0.01. Measured from the last refresh, not per
# update, so many tiny updates still refresh p once they add up.
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
        self.l_initial = log_odds(self.p_initial)
        self.l_occupied = self.l_initial
        # l_occupied when p_occupied was last recomputed (skip-exp optimization only).
        self.l_at_last_p_refresh = self.l_occupied

    def update(self, l_scan: float, weight: float, gamma: float) -> bool:
        """Update log-odds; refresh rounded p only once L has drifted enough since the last refresh.

        weight scales how much of l_scan is applied (1 at cell center, 0 at the edge).
        Returns True when p_occupied changed (the grid then gives it a new revision).
        """
        self.l_occupied = clamp_log_odds(
            self.l_initial
            + weight * (l_scan - self.l_initial)
            + gamma * (self.l_occupied - self.l_initial)
        )
        if abs(self.l_occupied - self.l_at_last_p_refresh) < LOG_ODDS_P_REFRESH_THRESHOLD:
            return False

        new_probability = probability_from_log_odds(self.l_occupied)
        # Move the anchor even if rounded p is unchanged, so drift is measured from here.
        self.l_at_last_p_refresh = self.l_occupied
        if new_probability == self.p_occupied:
            return False
        self.p_occupied = new_probability
        return True

    def as_message(self) -> OccupancyCell:
        return OccupancyCell(x=self.x, z=self.z, p=self.p_occupied)


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

    def __init__(self, room: Room, path: str | Path | None = None) -> None:
        """room: the room the grid covers.
        path: file to load the grid from now and save it to later; None for no disk.
        """
        self.room = room
        self.path = path
        self.cell_size = 0.10
        self.room_cells = cells_covering_room(room.corners, self.cell_size)
        self.cells: dict[CellIndex, GridCell] = _CellMap(self.room_cells)
        # Revisions start at the wall clock so every revision from this run is
        # newer than any a dashboard kept from an earlier run.
        self.revision = time.time_ns()
        self._reset_revision = self.revision
        self._cell_revisions: dict[CellIndex, int] = {}
        # Revision last written by save(); nothing to write until it moves.
        self._saved_revision = self.revision
        if path is not None:
            print(f"occupancy grid: {self._load()} cells loaded from {path}")

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
                self.revision += 1
                self._cell_revisions[cell] = self.revision

    def update(
        self,
        robot_pose: Transform2,
        corrected_points: np.ndarray,
    ) -> None:
        """Incorporate server-frame LiDAR hits, an (N, 2) X/Z array, into the room grid.

        Each hit walks Bresenham to its cell. Free cells store the distance from
        the continuous ray to the cell center; the hit cell stores the distance
        from the hit to the cell center. Across rays the nearest offset wins.
        Occupied overrides free. Out-of-room cells are skipped.
        """
        gamma = self.GAMMA
        # Transform2: Vector2.y carries the server's Z.
        robot_x, robot_z = robot_pose.position.x, robot_pose.position.y
        robot_cell = self.world_to_cell(robot_x, robot_z)

        free_offsets: dict[CellIndex, float] = {}
        occupied_offsets: dict[CellIndex, float] = {}

        for hit_x, hit_z in np.asarray(corrected_points, dtype=float).reshape(-1, 2):
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

    def snapshot(self, since: int = 0) -> OccupancyGridMessage:
        """Cells changed after revision `since`, or every observed cell with reset=True.

        A dashboard that applied revision R sends R back. It needs a reset when
        R is from before the last clear(), or from a different server run.
        """
        reset = since < self._reset_revision or since > self.revision
        # Every cell that changed since clear() has a revision; the rest are
        # still at the prior. (Reading self.cells can create prior cells.)
        cells = [
            self.cells[index].as_message()
            for index, revision in self._cell_revisions.items()
            if reset or revision > since
        ]
        return OccupancyGridMessage(
            cell_size=self.cell_size,
            revision=self.revision,
            reset=reset,
            cells=cells,
        )

    def clear(self) -> None:
        """Drop observed cells; dashboards get reset=True on their next fetch."""
        self.cells.clear()
        self._cell_revisions.clear()
        self.revision += 1
        self._reset_revision = self.revision

    def save(self) -> bool:
        """Write every changed cell to the grid's file if the grid moved since the last save."""
        if self.path is None or self.revision == self._saved_revision:
            return False
        payload = {
            "frame": "server",
            "cell_size": self.cell_size,
            "origin_x": self.room.offset.x,
            "origin_z": self.room.offset.z,
            # [x, z, p] per changed cell: server-frame cell index and probability filled.
            "cells": [
                [cell.x, cell.z, cell.p_occupied]
                for cell in (self.cells[index] for index in self._cell_revisions)
            ],
        }
        path = str(self.path)
        temporary_path = path + ".tmp"
        try:
            with open(temporary_path, "w") as file:
                file.write(json.dumps(payload))
            # Swap the file in whole, so a crash never leaves half a grid.
            os.replace(temporary_path, path)
        except OSError as error:
            print(f"[!] could not save occupancy grid to {path}: {error}")
            return False
        self._saved_revision = self.revision
        return True

    def _load(self) -> int:
        """Restore cells saved by save(). Returns how many cells were loaded."""
        path = self.path
        try:
            with open(path) as file:
                payload = json.load(file)
        except FileNotFoundError:
            return 0
        except (OSError, json.JSONDecodeError) as error:
            print(f"[!] could not load occupancy grid from {path}: {error}")
            return 0
        if not math.isclose(payload.get("cell_size", 0.0), self.cell_size):
            print(f"[!] {path} uses cell size {payload.get('cell_size')}, not {self.cell_size}; not loaded")
            return 0

        # Parse every cell before touching the grid, so a bad file loads nothing.
        try:
            entries = [(int(x), int(z), float(p)) for x, z, p in payload.get("cells", [])]
        except (TypeError, ValueError) as error:
            # A cell that is not [x, z, p], such as the old {"x", "z", "p"} objects.
            print(f"[!] {path} has a cell that is not [x, z, p] ({error}); not loaded")
            return 0

        loaded = 0
        for x, z, p in entries:
            index = (x, z)
            if index not in self.room_cells:
                continue
            cell = self.cells[index]
            # Only the rounded probability is saved, so the log-odds restart from it.
            cell.l_occupied = log_odds(p)
            cell.p_occupied = probability_from_log_odds(cell.l_occupied)
            self.revision += 1
            self._cell_revisions[index] = self.revision
            loaded += 1
        # Just loaded, so the file already matches.
        self._saved_revision = self.revision
        return loaded

    def probabilities_at(self, points: np.ndarray) -> np.ndarray:
        """Probability each server-frame (N, 2) X/Z point is filled; INITIAL_PROBABILITY if unobserved."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        values = np.full(len(points), INITIAL_PROBABILITY)
        if not self._cell_revisions:
            return values
        keys = np.array(list(self._cell_revisions))
        low = keys.min(axis=0)
        dense = np.full(keys.max(axis=0) - low + 1, INITIAL_PROBABILITY)
        dense[keys[:, 0] - low[0], keys[:, 1] - low[1]] = [
            self.cells[index].p_occupied for index in self._cell_revisions
        ]
        index = np.floor(points / self.cell_size).astype(int) - low
        inside = (index >= 0).all(axis=1) & (index < dense.shape).all(axis=1)
        values[inside] = dense[index[inside, 0], index[inside, 1]]
        return values
