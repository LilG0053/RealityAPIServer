"""Where AR headsets sit in the server's coordinate system.

A headset that has done ALIGN reports positions (and depth points) in its
ALIGN frame: meters, origin on the floor cross, +Y up, -Z pointing at the
wall cross, +X to the right. Which way that is in the room depends only on
where the two tape crosses are, so their positions live in tape.json:

    { "floor_cross": [x, z], "wall_cross": [x, z] }

x and z are server coordinates in meters (the same numbers the dashboard
uses). Edit the file whenever you move the tape; the server picks up the
change within a second, no restart needed.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np

from map.coordinate import CoordinateSystem, Direction
from realityapi_pb2 import Lidar, Transform, Vector3

# Compass directions as (east, north) unit vectors, for working out "right".
_COMPASS = {
    Direction.EAST: (1.0, 0.0), Direction.WEST: (-1.0, 0.0),
    Direction.NORTH: (0.0, 1.0), Direction.SOUTH: (0.0, -1.0),
}


class HeadsetFrame:
    def __init__(self, tape_path: str, server_coords: CoordinateSystem):
        self.tape_path = tape_path
        axes = (server_coords.x, server_coords.y, server_coords.z)
        self.up = next(i for i, d in enumerate(axes) if abs(d) == Direction.UP)
        self.up_sign = 1.0 if axes[self.up] == Direction.UP else -1.0
        self.horizontal = [i for i in range(3) if i != self.up]          # the two floor axes, in order
        self.compass = [_COMPASS[axes[i]] for i in self.horizontal]      # each floor axis as (east, north)
        self.version = 0                  # goes up every time a new placement loads
        self.placement = None             # (floor_cross, wall_cross) currently in use
        self._mtime = None
        self._warned_missing = False
        self.reload_if_changed()

    # ---- tape.json -------------------------------------------------------------

    def reload_if_changed(self) -> None:
        """Re-read tape.json if it was edited since the last read (cheap: one stat)."""
        try:
            mtime = os.stat(self.tape_path).st_mtime
        except FileNotFoundError:
            if not self._warned_missing:
                print(f"[!] {self.tape_path} not found: headsets can't be placed until it exists")
                self._warned_missing = True
            return
        if mtime == self._mtime:
            return
        self._mtime = mtime
        try:
            with open(self.tape_path, encoding="utf-8") as f:
                data = json.load(f)
            floor = [float(v) for v in data["floor_cross"]]
            wall = [float(v) for v in data["wall_cross"]]
            if len(floor) != 2 or len(wall) != 2:
                raise ValueError("floor_cross and wall_cross must each be [x, z]")
            if math.dist(floor, wall) < 0.3:
                raise ValueError("the crosses must be at least 0.3 m apart")
        except (OSError, ValueError, KeyError, TypeError) as e:
            print(f"[!] {self.tape_path} not used ({e}); keeping the previous placement")
            return
        self._set_placement(floor, wall)

    def _set_placement(self, floor, wall) -> None:
        h0, h1 = self.horizontal
        origin = np.zeros(3)
        origin[h0], origin[h1] = floor
        forward = np.zeros(3)
        forward[h0], forward[h1] = wall[0] - floor[0], wall[1] - floor[1]
        forward /= np.linalg.norm(forward)

        # "Right" is "forward" turned 90 degrees clockwise seen from above. Do the
        # turn in compass terms so it's correct whatever the server axes are.
        east = forward[h0] * self.compass[0][0] + forward[h1] * self.compass[1][0]
        north = forward[h0] * self.compass[0][1] + forward[h1] * self.compass[1][1]
        cw_east, cw_north = north, -east
        right = np.zeros(3)
        right[h0] = cw_east * self.compass[0][0] + cw_north * self.compass[0][1]
        right[h1] = cw_east * self.compass[1][0] + cw_north * self.compass[1][1]

        up = np.zeros(3)
        up[self.up] = self.up_sign

        # ALIGN (x, y, z) -> server: origin + x*right + y*up - z*forward
        self.basis = np.stack([right, up, -forward], axis=1)   # columns = where ALIGN x, y, z point
        self.origin = origin
        self.placement = (floor, wall)
        self.version += 1
        facing = _compass_name(east, north)
        print(f"tape.json loaded (#{self.version}): floor cross {floor}, wall cross {wall}, "
              f"wall cross is {facing} of the floor cross")

    # ---- conversions --------------------------------------------------------------

    @property
    def ready(self) -> bool:
        return self.placement is not None

    def points_to_server(self, points: np.ndarray) -> np.ndarray:
        """(N, 3) ALIGN-frame points -> (N, 3) server-frame points."""
        return points @ self.basis.T + self.origin

    def vec3_to_server(self, v: Vector3) -> Vector3:
        x, y, z = self.points_to_server(np.array([[v.x, v.y, v.z]]))[0]
        return Vector3(x=float(x), y=float(y), z=float(z))

    def transform_to_server(self, t: Transform) -> Transform:
        """Position, plus heading: rotation.y in = heading in the ALIGN frame (radians from
        +X toward +Z); rotation's vertical component out = the same in server axes."""
        look = np.array([math.cos(t.rotation.y), 0.0, math.sin(t.rotation.y)]) @ self.basis.T
        h0, h1 = self.horizontal
        rotation = [0.0, 0.0, 0.0]
        rotation[self.up] = math.atan2(look[h1], look[h0])
        return Transform(position=self.vec3_to_server(t.position),
                         rotation=Vector3(x=rotation[0], y=rotation[1], z=rotation[2]))

    def lidar_to_server(self, lidar: Lidar) -> Lidar:
        if not lidar.points:
            return Lidar()
        pts = np.array([(p.x, p.y, p.z) for p in lidar.points], dtype=float)
        return Lidar(points=[Vector3(x=float(x), y=float(y), z=float(z))
                             for x, y, z in self.points_to_server(pts)])


def _compass_name(east: float, north: float) -> str:
    names = ["east", "north-east", "north", "north-west", "west", "south-west", "south", "south-east"]
    return names[round(math.degrees(math.atan2(north, east)) / 45) % 8]
