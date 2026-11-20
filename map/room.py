"""The lab room, fixed in the server frame.

A Room is loaded from a blockout JSON (the lab's is assets/blockout.json, which
the server's Map loads): its outline plus measured obstacles such as the
pillars. The blockout is stored in the coordinates the dashboard draws; the
server frame is that with the room's offset removed, so a server-frame point
(x, z) is drawn at (x + offset.x, z + offset.z).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np

from realityapi_pb2 import Polygon2, RoomBounds, Vector2, Vector3


def display_offset() -> Vector3:
    """Where the server frame's (0, 0, 0) is in the lab blockout's (drawn) coordinates."""
    return Vector3(x=0.1, y=0.1, z=2.0)

BLOCKOUT_PATH = Path(__file__).resolve().parents[1] / "assets" / "blockout.json"


@dataclass
class Obstacle:
    name: str
    polygon: Polygon2


@dataclass
class Room:
    """A room outline and its obstacles, in drawn coordinates (meters).

    Polygons are Polygon2 messages: Vector2.x is X and Vector2.y is Z. offset is
    where the server frame's (0, 0, 0) is in drawn coordinates.
    """
    outline: Polygon2
    obstacles: tuple[Obstacle, ...]
    offset: Vector3 = field(default_factory=display_offset)

    @classmethod
    def from_json(cls, path: str | Path, offset: Vector3 | None = None) -> Room:
        """Read and check a blockout JSON: a "room" outline and "obstacles".

        offset defaults to the lab's display_offset().
        """
        with open(path) as file:
            data = json.load(file)

        outline = _polygon(data.get("room"), "room")
        obstacles = []
        for index, entry in enumerate(data.get("obstacles", [])):
            name = entry.get("name") or f"obstacle {index}"
            try:
                obstacles.append(Obstacle(name, _polygon(entry.get("polygon"), f"obstacle {name!r}")))
            except ValueError as error:
                # Obstacles are often added before they are measured; skip those.
                print(f"[!] skipping {error} in {path}")
        return cls(outline, tuple(obstacles), offset if offset is not None else display_offset())

    def _to_server(self, polygon: Polygon2) -> Polygon2:
        return Polygon2(vertices=[
            Vector2(x=vertex.x - self.offset.x, y=vertex.y - self.offset.z)
            for vertex in polygon.vertices
        ])

    @property
    def corners(self) -> Polygon2:
        """The outline in the server frame."""
        return self._to_server(self.outline)

    def polygons(self) -> list[Polygon2]:
        """Outline first, then each obstacle, in the server frame."""
        return [self._to_server(polygon) for polygon in [self.outline] + [o.polygon for o in self.obstacles]]

    def contains(self, x: float, z: float) -> bool:
        """Whether a server-frame point is inside the outline."""
        return point_in_polygon(x, z, self.corners)

    def bounds(self) -> RoomBounds:
        """The room as a RoomBounds message: drawn polygons, and where server (0, 0) is drawn."""
        return RoomBounds(
            origin=Vector2(x=self.offset.x, y=self.offset.z),
            room=self.outline,
            obstacles=[obstacle.polygon for obstacle in self.obstacles],
        )


def _polygon(points, label: str) -> Polygon2:
    """Validate a JSON polygon: a list of at least three [x, z] pairs."""
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError(f"{label} needs at least three [x, z] points")
    vertices = []
    for point in points:
        if (
            not isinstance(point, list)
            or len(point) != 2
            or not all(isinstance(value, (int, float)) for value in point)
        ):
            raise ValueError(f"{label} has a point that is not [x, z]: {point!r}")
        vertices.append(Vector2(x=float(point[0]), y=float(point[1])))
    return Polygon2(vertices=vertices)


def polygon_array(polygon: Polygon2) -> np.ndarray:
    """The vertices as a K x 2 array of (x, z)."""
    return np.array([[vertex.x, vertex.y] for vertex in polygon.vertices], dtype=float).reshape(-1, 2)


def point_in_polygon(x: float, z: float, polygon: Polygon2) -> bool:
    """Even-odd test. Vertices are (x, z) in order around the polygon."""
    inside = False
    previous = polygon.vertices[-1]
    for vertex in polygon.vertices:
        if (vertex.y > z) != (previous.y > z):
            crossing_x = (previous.x - vertex.x) * (z - vertex.y) / (previous.y - vertex.y) + vertex.x
            if x < crossing_x:
                inside = not inside
        previous = vertex
    return inside


def cells_covering_room(polygon: Polygon2, cell_size: float) -> frozenset[tuple[int, int]]:
    """Every cell whose square meets the polygon. Indices are server-frame cells."""
    xs = [vertex.x for vertex in polygon.vertices]
    zs = [vertex.y for vertex in polygon.vertices]
    i_min = math.floor(min(xs) / cell_size)
    i_max = math.floor(max(xs) / cell_size)
    j_min = math.floor(min(zs) / cell_size)
    j_max = math.floor(max(zs) / cell_size)
    cells: set[tuple[int, int]] = set()
    for i in range(i_min, i_max + 1):
        for j in range(j_min, j_max + 1):
            samples = (
                ((i + 0.5) * cell_size, (j + 0.5) * cell_size),
                (i * cell_size, j * cell_size),
                ((i + 1) * cell_size, j * cell_size),
                (i * cell_size, (j + 1) * cell_size),
                ((i + 1) * cell_size, (j + 1) * cell_size),
            )
            if any(point_in_polygon(x, z, polygon) for x, z in samples):
                cells.add((i, j))
    return frozenset(cells)
