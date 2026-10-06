#!/usr/bin/env python

import asyncio

from websockets.asyncio.server import serve
from realityapi_pb2 import Packet, Vector3, DeviceType
from enum import Enum
from map.map import Map
from map.device import DeviceType as MapDeviceType

# Offset added to every incoming position vector. Adjust to your task's spec.
OFFSET = (1.0, 2.0, 3.0)

map = Map()
map.load_from_json("dashboard/updated_pos.json")

def translate(vec: Vector3, offset=OFFSET) -> Vector3:
    """Return a new Vector3 shifted by the offset."""
    return Vector3(
        x=vec.x + offset[0],
        y=vec.y + offset[1],
        z=vec.z + offset[2],
    )


def handle_packet(pkt: Packet) -> Packet:
    """Decode which oneof field is set, act on it, return a reply Packet."""
    updated = translate(pkt.position)
    print(f"<<< position ({pkt.position.x}, {pkt.position.y}, {pkt.position.z})")
    print(f">>> position ({updated.x}, {updated.y}, {updated.z})")

    print(f"Device type: {DeviceType.Name(pkt.devicetype)}")
    print(f"ID: {pkt.id}")

    return Packet(position=updated, devicetype=pkt.devicetype, id=pkt.id)


async def handler(websocket):
    async for message in websocket:
        if isinstance(message, str):
            print("[!] ignoring text frame (expected binary protobuf)")
            continue

        pkt = Packet()
        pkt.ParseFromString(message)                      # decode

        handled_packet = handle_packet(pkt)
        update_map(handled_packet)
        await websocket.send(handled_packet.SerializeToString())   # encode + send binary

def update_map(pkt: Packet):
    # Convert protobuf DeviceType integer to MapDeviceType enum
    device_type_map = {
        0: MapDeviceType.VR,
        1: MapDeviceType.AR,
        2: MapDeviceType.DOG,
        3: MapDeviceType.ARM,
        4: MapDeviceType.TURTLE,
    }
    map_device_type = device_type_map.get(pkt.devicetype, MapDeviceType.TURTLE)
    map.set_device_position(pkt.id, pkt.position, map_device_type)
    

async def main():
    server = await serve(handler, "0.0.0.0", 65432)
    print("realityapi server on ws://0.0.0.0:65432")
    await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())