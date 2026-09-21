#!/usr/bin/env python

from websockets.sync.client import connect
from realityapi_pb2 import Packet, Vector3
def hello():
    uri = "ws://10.89.53.91:65432"
    with connect(uri) as websocket:
        name = input("What's your name? ")
        text = input("What's your message? ")
        x = float(input("X coordinate?"))
        y = float(input("Y coordinate?"))
        z = float(input("Z coordinate?"))

        pkt = Packet(position=Vector3(x=x, y=y, z=z))
        websocket.send(pkt.SerializeToString())
        print(f">>> {pkt.position}")

if __name__ == "__main__":
    hello()