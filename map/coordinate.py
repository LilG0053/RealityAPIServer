from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math
import numpy as np

from realityapi_pb2 import Lidar, Transform, Vector3


class Direction(Enum):
    """A signed axis. Opposites share an absolute value and cancel each other."""
    EAST = auto()
    WEST = -EAST

    NORTH = auto()
    SOUTH = -NORTH

    UP = auto()
    DOWN = -UP

    def __neg__(self) -> Direction:
        return Direction(-self.value)

    def __abs__(self) -> Direction:
        return Direction(abs(self.value))
    
    def clockwise90(self) -> Direction:
        if abs(self) == Direction.UP:
            raise ValueError("Clockwise/counterclockwise rotation is not defined for the up-down axis.")
        
        directions = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]
        return directions[(directions.index(self) + 1) % 4]
    
    def counterclockwise90(self) -> Direction:
        return -self.clockwise90()

@dataclass(frozen=True)
class CoordinateSystem:
    """Defines a coordinate system with the positive directions of its three axes."""
    x: Direction
    y: Direction
    z: Direction
    zero_heading: Direction | None = None

    def __post_init__(self):
        direction_set = {abs(d) for d in (self.x, self.y, self.z)}
        if len(direction_set) != 3:
            raise ValueError("All axes must be linearly independent (e.g. north, east, up).")
        if self.zero_heading is None:
            object.__setattr__(self, "zero_heading", self.x)
        if abs(self.zero_heading) == Direction.UP:
            raise ValueError("zero_heading must be a horizontal direction (north, south, east, west).")


def get_transform_matrix(source: CoordinateSystem, target: CoordinateSystem) -> np.ndarray:
    """Return M such that v_target = M @ v_source.

    Each column i in M is the i-th source axis expressed in the target frame.
    """
    source_axes = (source.x, source.y, source.z)
    target_axes = (target.x, target.y, target.z)

    # Each target axis owns its row. The opposite direction uses the same row
    # with a negative sign, so every source axis is one lookup.
    placement: dict[Direction, tuple[int, int]] = {}
    for row, direction in enumerate(target_axes):
        placement[direction] = (row, 1)
        placement[-direction] = (row, -1)

    matrix = np.zeros((3, 3), dtype=int)
    for column, direction in enumerate(source_axes):
        row, sign = placement[direction]
        matrix[row, column] = sign
    return matrix


def _apply_matrix(vec: Vector3, matrix: np.ndarray) -> Vector3:
    x, y, z = matrix @ np.array([vec.x, vec.y, vec.z], dtype=float)
    return Vector3(x=float(x), y=float(y), z=float(z))


def convert_vector(
    vector: list[float],
    source: CoordinateSystem,
    target: CoordinateSystem,
) -> list[float]:
    """Convert a [x, y, z] list from one coordinate system to another."""
    matrix = get_transform_matrix(source, target)
    return (matrix @ np.array(vector)).tolist()


def convert_vec3(
    vec: Vector3,
    source: CoordinateSystem,
    target: CoordinateSystem,
) -> Vector3:
    """Convert a protobuf Vector3 from source to target coordinate system."""
    if source == target:
        return Vector3(x=vec.x, y=vec.y, z=vec.z)
    matrix = get_transform_matrix(source, target)
    return _apply_matrix(vec, matrix)


def _heading_offset(source_heading: Direction, target_heading: Direction) -> float:
    """Return the counterclockwise angle (in radians, in (-pi, pi]) from target_heading to source_heading."""
    ccw_order = [
        target_heading,
        target_heading.counterclockwise90(),
        -target_heading,
        target_heading.clockwise90(),
    ]
    angles = [0.0, math.pi / 2, math.pi, -math.pi / 2]
    return angles[ccw_order.index(source_heading)]


def convert_transform(
    transform: Transform,
    source: CoordinateSystem,
    target: CoordinateSystem,
) -> Transform:
    """Convert a protobuf Transform (position and rotation) from source to target."""
    if source == target:
        return Transform(position=transform.position, rotation=transform.rotation)
    matrix = get_transform_matrix(source, target)
    position = _apply_matrix(transform.position, matrix)
    rotation = _apply_matrix(transform.rotation, matrix)

    # Shift yaw about the target's vertical (UP/DOWN) axis to account for any
    # difference between source.zero_heading and target.zero_heading.
    yaw_offset = _heading_offset(source.zero_heading, target.zero_heading)
    for axis_name, direction in (("x", target.x), ("y", target.y), ("z", target.z)):
        if abs(direction) == Direction.UP:
            sign = 1.0 if direction == Direction.UP else -1.0
            setattr(rotation, axis_name, getattr(rotation, axis_name) + sign * yaw_offset)
            break

    return Transform(position=position, rotation=rotation)


def convert_lidar(
    lidar: Lidar,
    source: CoordinateSystem,
    target: CoordinateSystem,
) -> Lidar:
    """Convert all points in a protobuf Lidar message from source to target."""
    if source == target or not lidar.points:
        return Lidar(points=lidar.points)
    matrix = get_transform_matrix(source, target)
    return Lidar(points=[_apply_matrix(pt, matrix) for pt in lidar.points])
