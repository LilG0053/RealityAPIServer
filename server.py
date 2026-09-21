#!/usr/bin/env python

import asyncio

from websockets.asyncio.server import serve
from realityapi_pb2 import Packet, Vector3, Text

# Offset added to every incoming position vector. Adjust to your task's spec.
OFFSET = (1.0, 2.0, 3.0)


def translate(vec: Vector3, offset=OFFSET) -> Vector3:
    """Return a new Vector3 shifted by the offset."""
    return Vector3(
        x=vec.x + offset[0],
        y=vec.y + offset[1],
        z=vec.z + offset[2],
    )


def handle_packet(pkt: Packet) -> Packet:
    """Decode which oneof field is set, act on it, return a reply Packet."""
    kind = pkt.WhichOneof("body")          # 'hello', 'text', 'position', or None

    if kind == "position":
        updated = translate(pkt.position)
        print(f"<<< position ({pkt.position.x}, {pkt.position.y}, {pkt.position.z})")
        print(f">>> position ({updated.x}, {updated.y}, {updated.z})")
        return Packet(position=updated)

    if kind == "hello":
        print(f"<<< hello from {pkt.hello.name}")
        return Packet(text=Text(text=f"Hello {pkt.hello.name}!"))

    if kind == "text":
        print(f"<<< text: {pkt.text.text}")
        return Packet(text=Text(text="ack"))

    print("[!] empty packet (no oneof field set)")
    return Packet(text=Text(text="error: empty packet"))


async def handler(websocket):
    async for message in websocket:
        if isinstance(message, str):
            print("[!] ignoring text frame (expected binary protobuf)")
            continue

        pkt = Packet()
        pkt.ParseFromString(message)                      # decode

        reply = handle_packet(pkt)
        await websocket.send(reply.SerializeToString())   # encode + send binary


async def main():
    server = await serve(handler, "0.0.0.0", 8765)
    print("realityapi server on ws://0.0.0.0:8765")
    await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())