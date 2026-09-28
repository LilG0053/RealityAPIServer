#!/usr/bin/env python

from websockets.sync.client import connect
from realityapi_pb2 import Packet, Vector3, Hello, Text

URI = "ws://10.89.53.91:65432"


def ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Please enter a number.")


def describe(pkt: Packet) -> str:
    p = pkt.position
    device_type = pkt.devicetype.name
    id = pkt.id

    return f"position ({p.x}, {p.y}, {p.z}, devicetype {device_type}, id {id})"


def exchange(websocket, pkt: Packet) -> Packet:
    websocket.send(pkt.SerializeToString())
    reply = Packet()
    reply.ParseFromString(websocket.recv())
    return reply


def hello():
    name = input("What's your name? ")
    text = input("What's your message? ")
    x = ask_float("X coordinate? ")
    y = ask_float("Y coordinate? ")
    z = ask_float("Z coordinate? ")

    packets = [
        Packet(hello=Hello(name=name)),
        Packet(text=Text(text=text)),
        Packet(position=Vector3(x=x, y=y, z=z)),
    ]

    with connect(URI) as websocket:
        for pkt in packets:
            print(f">>> {describe(pkt)}")
            reply = exchange(websocket, pkt)
            print(f"<<< {describe(reply)}")


if __name__ == "__main__":
    hello()