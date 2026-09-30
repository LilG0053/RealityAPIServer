#!/usr/bin/env python

import asyncio

from websockets.asyncio.server import serve
from realityapi_pb2 import Packet, Vector3, Text, Heartbeat
from map.map import Map
from map.device import Device,DeviceType
from time import time

# Offset added to every incoming position vector. Adjust to your task's spec.
OFFSET = (1.0, 2.0, 3.0)
map = Map()
TIMEOUT = 0.12

def translate(vec: Vector3, offset=OFFSET) -> Vector3:
    """Return a new Vector3 shifted by the offset."""
    return Vector3(
        x=vec.x + offset[0],
        y=vec.y + offset[1],
        z=vec.z + offset[2],
    )


def handle_packet(pkt: Packet, map:Map) -> Packet:
    """Decode which oneof field is set, act on it, return a reply Packet."""
    kind = pkt.WhichOneof("body")          # 'hello', 'text', 'position', or None
    global TIMEOUT

    if kind == "position":
        updated = translate(pkt.position)
        print(f"<<< position ({pkt.position.x}, {pkt.position.y}, {pkt.position.z})")
        print(f">>> position ({updated.x}, {updated.y}, {updated.z})")
        return Packet(position=updated)

    elif kind == "hello":
        print(f"<<< hello from {pkt.hello.name}")
        return Packet(text=Text(text=f"Hello {pkt.hello.name}!"))

    elif kind == "text":
        print(pkt)
        return Packet(text=Text(text="ack"))

    elif kind == "heartbeat":
        name = pkt.heartbeat.name
        new_heartbeat = time()
        

        if map.exists(name):
            map.update_last_time(id=name, new_time=new_heartbeat)
            return pkt
        else:
            device_type = DeviceType(int(pkt.heartbeat.device_type))
            map.add_device(last_heartbeat=new_heartbeat,pos=Vector3(x=0,y=0,z=0), id=name, device_type=device_type) #placeholder position vector
            return pkt
    print("[!] empty packet (no oneof field set)")
    return Packet(text=Text(text="error: empty packet"))


async def handler(websocket):
    async for message in websocket:
        if isinstance(message, str):
            print("[!] ignoring text frame (expected binary protobuf)")
            continue

        pkt = Packet()
        pkt.ParseFromString(message)                     # decode
        reply = handle_packet(pkt, map)

        
        await websocket.send(reply.SerializeToString())   # encode + send binary

async def monitor(map: Map):
    global TIMEOUT
    while True:
        map.check_all(timeout=TIMEOUT)
        await asyncio.sleep(0.04)

async def main():
    server = await serve(handler, "0.0.0.0", 8765)
    asyncio.create_task(monitor(map))
    print("realityapi server on ws://0.0.0.0:8765")
    await server.serve_forever()




if __name__ == "__main__":
    asyncio.run(main())