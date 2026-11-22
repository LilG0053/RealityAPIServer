"""TurtleBots: the simplest 2D device."""

from __future__ import annotations

import math
from typing import ClassVar

import numpy as np

from map.algorithms.lidar import scan_to_device_points
from map.device.device2d import Device2D
from map.wallgrid import WallGrid
from realityapi_pb2 import DeviceType, RawLidar2Scan, Transform2, Vector2

# The RPLIDAR's pose relative to base_link, in the robot's frame (x forward,
# Vector2.y being Z, to its left). Found with
#   ros2 run tf2_ros tf2_echo base_link rplidar_link
# It is mounted turned +90 degrees (rpy="0 0 pi/2" in turtlebot4.urdf.xacro),
# so its 0-radian axis points out the robot's left. (It also sits 0.193 m up,
# which a 2D scan does not use.)
LIDAR_OFFSET = Transform2(position=Vector2(x=-0.040, y=0.000), rotation=math.pi / 2)


class TurtleBot(Device2D):
    DEVICE_TYPE: ClassVar[int] = DeviceType.TURTLE

    def __init__(self, id: str, wall_grid: WallGrid, timestamp: float = 0.0) -> None:
        """wall_grid: the grid every TurtleBot shares (see Map)."""
        super().__init__(id, wall_grid, timestamp)

    def scan_to_body_points(self, raw_scan: RawLidar2Scan) -> np.ndarray:
        return scan_to_device_points(raw_scan, LIDAR_OFFSET)
