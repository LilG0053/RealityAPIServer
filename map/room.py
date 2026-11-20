"""The lab room, fixed in the server frame.

The room is loaded from assets/blockout.json: its outline plus measured
obstacles such as the pillars. The blockout is stored in the coordinates the
dashboard draws. Incoming poses are shifted by DISPLAY_OFFSET before they are
drawn, so a server-frame point (x, z) appears at
(x + DISPLAY_OFFSET[0], z + DISPLAY_OFFSET[2]). The occupancy grid uses the
server-frame polygon and never moves with the robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cache, lru_cache
import json
import math
from pathlib import Path

from realityapi_pb2 import Polygon2, RoomBounds, Vector2

# x, y, z. Shared with server.py so the grid and the live poses stay aligned.
DISPLAY_OFFSET = (0.1, 0.1, 2.0)

BLOCKOUT_PATH = Path(__file__).resolve().parents[1] / "assets" / "blockout.json"


@dataclass(frozen=True)
class Obstacle:
    name: str
    polygon: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class Blockout:
    """The room in the coordinates the dashboard draws, meters (x, z).

    (0, 0) is the corner furthest from the door.
    """
    room: tuple[tuple[float, float], ...]
    obstacles: tuple[Obstacle, ...]


def _polygon(points, label: str) -> tuple[tuple[float, float], ...]:
    """Validate a JSON polygon: a list of at least three [x, z] pairs."""
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError(f"{label} needs at least three [x, z] points")
    polygon = []
    for point in points:
        if (
            not isinstance(point, list)
            or len(point) != 2
            or not all(isinstance(value, (int, float)) for value in point)
        ):
            raise ValueError(f"{label} has a point that is not [x, z]: {point!r}")
        polygon.append((float(point[0]), float(point[1])))
    return tuple(polygon)


@cache
def load_blockout(path: str | Path = BLOCKOUT_PATH) -> Blockout:
    """Read the room outline and obstacles from the blockout JSON."""
    with open(path) as file:
        data = json.load(file)

    room = _polygon(data.get("room"), "room")
    obstacles = []
    for index, entry in enumerate(data.get("obstacles", [])):
        name = entry.get("name") or f"obstacle {index}"
        try:
            obstacles.append(Obstacle(name, _polygon(entry.get("polygon"), f"obstacle {name!r}")))
        except ValueError as error:
            # Obstacles are often added before they are measured; skip those.
            print(f"[!] skipping {error} in {path}")
    return Blockout(room=room, obstacles=tuple(obstacles))


# Outline of the room in the coordinates the dashboard draws, meters (x, z).
ROOM_CORNERS_DRAWN = load_blockout().room


def room_bounds(blockout: Blockout | None = None) -> RoomBounds:
    """The blockout as a RoomBounds message.

    Polygons stay in the dashboard's drawn coordinates (Vector2.x is X and
    Vector2.y is Z). origin is where the server frame's (0, 0) is drawn.
    """
    blockout = blockout or load_blockout()

    def polygon(points: tuple[tuple[float, float], ...]) -> Polygon2:
        return Polygon2(vertices=[Vector2(x=x, y=z) for x, z in points])

    return RoomBounds(
        origin=Vector2(x=DISPLAY_OFFSET[0], y=DISPLAY_OFFSET[2]),
        room=polygon(blockout.room),
        obstacles=[polygon(obstacle.polygon) for obstacle in blockout.obstacles],
    )


def server_room_corners(offset: tuple[float, float, float] = DISPLAY_OFFSET) -> tuple[tuple[float, float], ...]:
    """Room outline with the dashboard shift removed."""
    origin_x, origin_z = offset[0], offset[2]
    return tuple((x - origin_x, z - origin_z) for x, z in ROOM_CORNERS_DRAWN)


def point_in_polygon(x: float, z: float, corners: tuple[tuple[float, float], ...]) -> bool:
    """Even-odd test. Corners are (x, z) in order around the room."""
    inside = False
    previous_x, previous_z = corners[-1]
    for corner_x, corner_z in corners:
        if (corner_z > z) != (previous_z > z):
            crossing_x = (previous_x - corner_x) * (z - corner_z) / (previous_z - corner_z) + corner_x
            if x < crossing_x:
                inside = not inside
        previous_x, previous_z = corner_x, corner_z
    return inside


@lru_cache(maxsize=8)
def cells_covering_room(
    corners: tuple[tuple[float, float], ...],
    cell_size: float,
) -> frozenset[tuple[int, int]]:
    """Every cell whose square meets the room. Indices are server-frame cells."""
    xs = [x for x, _z in corners]
    zs = [z for _x, z in corners]
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
            if any(point_in_polygon(x, z, corners) for x, z in samples):
                cells.add((i, j))
    return frozenset(cells)