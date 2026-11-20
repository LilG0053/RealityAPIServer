"""The lab room blockout, loaded from assets/blockout.json.

The blockout is stored in the coordinates the dashboard draws, meters (x, z),
with (0, 0) at the corner furthest from the door. Incoming poses are shifted by
DISPLAY_OFFSET before they are drawn, so a server-frame point (x, z) appears at
(x + DISPLAY_OFFSET[0], z + DISPLAY_OFFSET[2]). map/blockout.py removes that
shift to get the room into the server frame.

This is the small part of the room module that map/blockout.py needs. If you
later merge the branch with the full map/room.py (occupancy cells, RoomBounds),
that file replaces this one: it has everything here under the same names.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cache
import json
from pathlib import Path

# x, y, z. Same as OFFSET in server.py, so the room and the live poses line up.
DISPLAY_OFFSET = (0.1, 0.1, 2.0)

BLOCKOUT_PATH = Path(__file__).resolve().parents[1] / "assets" / "blockout.json"


@dataclass(frozen=True)
class Obstacle:
    name: str
    polygon: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class Blockout:
    """The room outline and measured obstacles, meters (x, z)."""
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