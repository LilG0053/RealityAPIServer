#!/usr/bin/env python

from websockets.sync.client import connect
from realityapi_pb2 import Packet
def hello():
    uri = "ws://localhost:8765"
    with connect(uri) as websocket:
        name = input("What's your name? ")
        text = input("What's your message? ")
        x = float(input("X coordinate?"))
        y = float(input("Y coordinate?"))
        z = float(input("Z coordinate?"))

        pkt = Packet()
        pkt.body.position = Vecotr()
        websocket.send(name)
        print(f">>> {name}")

        greeting = websocket.recv()
        print(f"<<< {greeting}")

if __name__ == "__main__":
    hello()