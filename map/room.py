"""The lab room, fixed in the server frame.

The dashboard draws the room at ROOM_CORNERS_DRAWN. Incoming poses are shifted by
DISPLAY_OFFSET before they are drawn, so a server-frame point (x, z) appears at
(x + DISPLAY_OFFSET[0], z + DISPLAY_OFFSET[2]). The occupancy grid uses the
server-frame polygon and never moves with the robot.
"""

from functools import lru_cache
import math

# x, y, z. Shared with server.py so the grid and the live poses stay aligned.
DISPLAY_OFFSET = (0.1, 0.1, 2.0)

# Outline of the room in the coordinates the dashboard draws, meters (x, z).
# (0, 0) is the corner furthest from the door.
ROOM_CORNERS_DRAWN = (
    (0.0, 0.0),
    (0.0, 15.45),
    (5.16, 15.45),
    (5.16, 12.26),
    (9.07, 12.26),
    (9.07, 0.0),
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
