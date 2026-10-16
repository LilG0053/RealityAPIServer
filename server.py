#!/usr/bin/env python

import asyncio
from websockets.asyncio.server import serve
from google.protobuf.message import DecodeError
from realityapi_pb2 import (
    ClientBoundPacket,
    DogInfo,
    RequestType,
    ServerBoundPacket,
    TurtleInfo,
    Vector3,
)
from time import time
from map.map import Map

# Offset added to every incoming position vector. Adjust to your task's spec.
OFFSET = (0.1, 0.1, 2.0)

# A device is dropped from the map once it has been quiet this long (seconds).
TIMEOUT = 2.0

# How often the monitor task sweeps the map for timed-out devices (seconds).
MONITOR_INTERVAL = 0.04

map = Map()
map.load_from_json("dashboard/updated_pos.json")

def translate_vec3(vec: Vector3, offset=OFFSET) -> Vector3:
    """Return a new Vector3 shifted by the offset."""
    return Vector3(
        x=vec.x + offset[0],
        y=vec.y + offset[1],
        z=vec.z + offset[2],
    )

# need better way to do this maybe
def translate_turtlebot_packet(pkt: ServerBoundPacket) -> ServerBoundPacket:
    return ServerBoundPacket(
        id=pkt.id,
        timestamp=pkt.timestamp,
        req_type=pkt.req_type,
        turtle=TurtleInfo(
            position=translate_vec3(pkt.turtle.position),
            rotation=pkt.turtle.rotation
        )
    )

def translate_dog_packet(pkt: ServerBoundPacket) -> ServerBoundPacket:
    return ServerBoundPacket(
        id=pkt.id,
        timestamp=pkt.timestamp,
        req_type=pkt.req_type,
        dog=DogInfo(
            position=translate_vec3(pkt.dog.position),
            rotation=pkt.dog.rotation,
        ),
    )

def handle_packet(pkt: ServerBoundPacket) -> ClientBoundPacket:
    if pkt.req_type == RequestType.TURTLE_INFO and pkt.HasField("turtle"):
        pkt = translate_turtlebot_packet(pkt)
    elif pkt.req_type == RequestType.DOG_INFO and pkt.HasField("dog"):
        pkt = translate_dog_packet(pkt)

    map.handle_info_packet(pkt)
    print(f"<<< packet from {pkt.id} ({map.get_device_type(pkt.id)})")

    return ClientBoundPacket(
        id=pkt.id,
        timestamp=time(),
        status = 200,
    )

async def handler(websocket):
    async for message in websocket:
        if isinstance(message, str):
            print("[!] ignoring text frame (expected binary protobuf)")
            continue

        pkt = ServerBoundPacket()
        try:
            pkt.ParseFromString(message)
        except DecodeError:
            print("[!] ignoring invalid protobuf frame")
            continue

        ret: ClientBoundPacket | None = None
        match pkt.req_type:
            case RequestType.HEARTBEAT:
                ret = handle_packet(pkt)
            case RequestType.TURTLE_INFO if pkt.HasField("turtle"):
                ret = handle_packet(pkt)
            case RequestType.DOG_INFO if pkt.HasField("dog"):
                ret = handle_packet(pkt)
            case RequestType.TURTLE_INFO | RequestType.DOG_INFO:
                print(f"[!] ignoring {RequestType.Name(pkt.req_type)} without matching data")

        if ret is None:
            continue

        await websocket.send(ret.SerializeToString())   # encode + send binary

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
