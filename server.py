#!/usr/bin/env python

import asyncio

from websockets.asyncio.server import serve
from realityapi_pb2 import Packet, Vector3, DeviceType, Heartbeat
from enum import Enum
from time import time
from map.map import Map
from map.device import DeviceType as MapDeviceType

# Offset added to every incoming position vector. Adjust to your task's spec.
OFFSET = (1.0, 2.0, 3.0)

# A device is dropped from the map once it has been quiet this long (seconds).
TIMEOUT = 2.0

# How often the monitor task sweeps the map for timed-out devices (seconds).
MONITOR_INTERVAL = 0.04

map = Map()

def translate(vec: Vector3, offset=OFFSET) -> Vector3:
    """Return a new Vector3 shifted by the offset."""
    return Vector3(
        x=vec.x + offset[0],
        y=vec.y + offset[1],
        z=vec.z + offset[2],
    )


def to_map_device_type(devicetype: int) -> MapDeviceType:
    """Convert the proto enum to the map's enum by name, not by value.

    The proto enum starts at 0 and map.device.DeviceType uses auto() from 1,
    so matching on the number would shift every device one type over.
    """
    return MapDeviceType[DeviceType.Name(devicetype)]


def handle_packet(pkt: Packet) -> Packet:
    """Decode which oneof field is set, act on it, return a reply Packet."""
    updated = translate(pkt.position)
    print(f"<<< position ({pkt.position.x}, {pkt.position.y}, {pkt.position.z})")
    print(f">>> position ({updated.x}, {updated.y}, {updated.z})")

    print(f"Device type: {DeviceType.Name(pkt.devicetype)}")
    print(f"ID: {pkt.id}")

    return Packet(position=updated, devicetype=pkt.devicetype, id=pkt.id)


def handle_heartbeat(pkt: Packet) -> Packet:
    """Record the keepalive and acknowledge it with the server's own clock."""
    now = time()

    if map.exists(pkt.id):
        map.update_last_time(pkt.id, now)
    else:
        # The device is alive but has not reported a position yet.
        map.add_device(
            pkt.id,
            device_type=to_map_device_type(pkt.devicetype),
            pos=Vector3(x=0, y=0, z=0),
            last_heartbeat=now,
        )

    print(f"<<< heartbeat from {pkt.id} ({DeviceType.Name(pkt.devicetype)})")

    return Packet(
        heartbeat=Heartbeat(current_time=now),
        devicetype=pkt.devicetype,
        id=pkt.id,
    )


async def handler(websocket):
    async for message in websocket:
        if isinstance(message, str):
            print("[!] ignoring text frame (expected binary protobuf)")
            continue

        pkt = Packet()
        pkt.ParseFromString(message)                      # decode

        # A heartbeat carries no position, so it must not move the device.
        if pkt.HasField("heartbeat"):
            handled_packet = handle_heartbeat(pkt)
        else:
            handled_packet = handle_packet(pkt)
            update_map(handled_packet)

        await websocket.send(handled_packet.SerializeToString())   # encode + send binary

def update_map(pkt: Packet):
    map.set_device_position(pkt.id, pkt.position, to_map_device_type(pkt.devicetype))
    

async def monitor(map: Map):
    """Sweep the map so devices that stop sending heartbeats disappear."""
    while True:
        map.check_all(TIMEOUT)
        await asyncio.sleep(MONITOR_INTERVAL)


async def main():
    server = await serve(handler, "0.0.0.0", 65432)
    asyncio.create_task(monitor(map))
    print("realityapi server on ws://0.0.0.0:65432")
    await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())
