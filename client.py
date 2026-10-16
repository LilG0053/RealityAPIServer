#!/usr/bin/env python

from time import sleep, time
from uuid import uuid4

from websockets.sync.client import connect
from realityapi_pb2 import DeviceInfo, DeviceType, Response, ServerRequest, Transform, Vector3

URI = "ws://localhost:65432"

# Seconds between keepalives. Must be well under the server's TIMEOUT.
HEARTBEAT_INTERVAL = 0.05


def ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Please enter a number.")

def exchange(websocket, req: ServerRequest) -> Response:
    """Send one request and return the server's matching response."""
    req.req_uuid = str(uuid4())
    websocket.send(req.SerializeToString())

    reply = Response()
    reply.ParseFromString(websocket.recv())
    if reply.req_uuid != req.req_uuid:
        print(f"[!] response {reply.req_uuid} does not match request {req.req_uuid}")
    return reply


def build_device_info() -> DeviceInfo:
    print("Building turtlebot device info")
    device_id = input("What's your id? ")
    x = ask_float("X coordinate? ")
    y = ask_float("Y coordinate? ")
    z = ask_float("Z coordinate? ")

    return DeviceInfo(
        id=device_id,
        device_type=DeviceType.TURTLE,
        timestamp=time(),
        transform=Transform(
            position=Vector3(x=x, y=y, z=z),
            rotation=Vector3(x=0, y=0, z=0)
        )
    )

def send_heartbeats(websocket, device_id: str):
    """Keep telling the server this device is alive until interrupted."""
    while True:
        beat = ServerRequest(post_heartbeat=device_id)
        print(f">>> heartbeat {device_id}")
        reply = exchange(websocket, beat)
        print(f"<<< {reply.WhichOneof('data')}")

        sleep(HEARTBEAT_INTERVAL)


def main():
    info = build_device_info()
    with connect(URI) as websocket:
        print(f">>> {info}")
        reply = exchange(websocket, ServerRequest(post_device_info=info))
        print(f"<<< {reply.WhichOneof('data')}")
        send_heartbeats(websocket, info.id)


if __name__ == "__main__":
    main()
