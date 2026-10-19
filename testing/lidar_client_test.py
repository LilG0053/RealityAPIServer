#!/usr/bin/env python

"""Replay the existing robot poses with a simulated LiDAR scan at each pose."""

import math
import random
from time import sleep

from websockets.sync.client import connect

from client_test import PACKET_RATE, URI, build_device_info, exchange
from realityapi_pb2 import Lidar, ServerRequest, Vector3


POINT_COUNT = 360


def build_lidar_scan(transform, scan_number: int) -> Lidar:
    """Create points around the robot in the same coordinate frame as its pose."""
    points = []
    for index in range(POINT_COUNT):
        angle = 2 * math.pi * index / POINT_COUNT + transform.rotation.y
        # Vary the distance to make a recognizable, changing test scan.
        distance = 1.5 + 0.3 * math.sin(3 * angle + scan_number * 0.1)
        points.append(Vector3(
            x=transform.position.x + distance * math.cos(angle),
            y=transform.position.y,
            z=transform.position.z + distance * math.sin(angle),
        ))
    return Lidar(points=points)


def main():
    devices = build_device_info()
    with connect(URI) as websocket:
        for scan_number, device in enumerate(devices):
            device.lidar.CopyFrom(build_lidar_scan(device.transform, scan_number))
            reply = exchange(websocket, ServerRequest(post_device_info=device))
            print(f"<<< {reply.WhichOneof('data')} | {device.id}: {len(device.lidar.points)} LiDAR points")
            sleep(PACKET_RATE + random.uniform(-0.025, 0.025))


if __name__ == "__main__":
    main()
