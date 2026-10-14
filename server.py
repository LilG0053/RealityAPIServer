#!/usr/bin/env python

import asyncio
from websockets.asyncio.server import serve
from realityapi_pb2 import RequestType, ServerBoundPacket, ClientBoundPacket, TurtleInfo, Vector3
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

def handle_packet(pkt: ServerBoundPacket) -> ClientBoundPacket:
    if pkt.req_type == RequestType.TURTLE_INFO and pkt.HasField("turtle"):
        pkt = translate_turtlebot_packet(pkt)

    map.handle_info_packet(pkt)
    print(f"<<< heartbeat from {pkt.id} ({map.get_device_type(pkt.id)})")

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
        pkt.ParseFromString(message)                      # decode

        ret: ClientBoundPacket | None = None
        match pkt.req_type:
            case RequestType.HEARTBEAT | RequestType.TURTLE_INFO:
                ret = handle_packet(pkt)

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
