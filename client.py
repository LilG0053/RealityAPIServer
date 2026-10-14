#!/usr/bin/env python

from time import sleep, time
from turtle import Turtle

from websockets.sync.client import connect
from realityapi_pb2 import RequestType, ServerBoundPacket, ClientBoundPacket, TurtleInfo, Vector3

URI = "ws://localhost:65432"

# Seconds between keepalives. Must be well under the server's TIMEOUT.
HEARTBEAT_INTERVAL = 0.05


def ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Please enter a number.")

def exchange(websocket, pkt: ServerBoundPacket) -> ClientBoundPacket:
    websocket.send(pkt.SerializeToString())
    reply = ClientBoundPacket()
    reply.ParseFromString(websocket.recv())
    return reply


def build_turtle_info_packet() -> ServerBoundPacket:
    print("Building turtlebot packet")
    device_id = input("What's your id? ")
    x = ask_float("X coordinate? ")
    y = ask_float("Y coordinate? ")
    z = ask_float("Z coordinate? ")

    return ServerBoundPacket(
        id=device_id,
        timestamp=time(),
        req_type=RequestType.TURTLE_INFO,
        turtle=TurtleInfo(
            position=Vector3(x=x, y=y, z=z),
            rotation=Vector3(x=0, y=0, z=0)
        )
    )

def send_heartbeats(websocket, pkt: ServerBoundPacket):
    """Keep telling the server this device is alive until interrupted."""
    while True:
        beat = ServerBoundPacket(
            timestamp=time(),
            id=pkt.id,
        )
        print(f">>> {beat}")
        reply = exchange(websocket, beat)
        print(f"<<< {reply}")

        sleep(HEARTBEAT_INTERVAL)


def main():
    pkt = build_turtle_info_packet()
    with connect(URI) as websocket:
        print(f">>> {pkt}")
        reply = exchange(websocket, pkt)
        print(f"<<< {reply}")
        send_heartbeats(websocket, pkt)


if __name__ == "__main__":
    main()
