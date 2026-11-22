#!/usr/bin/env python

import sys
from pathlib import Path
from time import sleep
from uuid import uuid4

from websockets.sync.client import connect

# realityapi_pb2 lives in the repo root
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import json
import random

from realityapi_pb2 import (
    DeviceInfo,
    DeviceType,
    Response,
    ServerRequest,
    Transform2,
    Transform3,
    Vector2,
    Vector3,
)

URI = "ws://localhost:65432"
PACKET_RATE = 0.1
DATA_FILE = Path(__file__).parent / "test_data.json"


def exchange(websocket, req: ServerRequest) -> Response:
    """Send one request and return the server's matching response."""
    req.req_uuid = str(uuid4())
    websocket.send(req.SerializeToString())

    reply = Response()
    reply.ParseFromString(websocket.recv())
    if reply.req_uuid != req.req_uuid:
        print(f"[!] response {reply.req_uuid} does not match request {req.req_uuid}")
    return reply


def build_device_info() -> list[DeviceInfo]:
    with open(DATA_FILE, 'r', encoding='utf-8') as file:
        data = json.load(file)

    # Recorded poses are in server coordinates. TurtleBots use the 2D messages
    # (Vector2.y is Z, rotation is the heading about the up axis, rotY); other
    # devices use the 3D ones. The server ignores a TurtleBot's poses until a
    # raw scan has localized it.
    device_info = []
    for entry in data:
        info = DeviceInfo(
            id=entry['device_id'],
            device_type=DeviceType.Value(entry['device_type']),
            timestamp=entry['time'],
        )
        if info.device_type == DeviceType.TURTLE:
            info.transform2.CopyFrom(Transform2(
                position=Vector2(x=entry['posX'], y=entry['posZ']),
                rotation=entry['rotY'],
            ))
        else:
            info.transform3.CopyFrom(Transform3(
                position=Vector3(x=entry['posX'], y=entry['posY'], z=entry['posZ']),
                rotation=Vector3(x=entry['rotX'], y=entry['rotY'], z=entry['rotZ']),
            ))
        device_info.append(info)

    return device_info


def main():
    info = build_device_info()
    with connect(URI) as websocket:
        for data in info:
            reply = exchange(websocket, ServerRequest(post_device_info=data))
            print(f"<<< {reply.WhichOneof('data')}")
            # random incoming packet rate for realism
            sleep(PACKET_RATE + random.uniform(-0.025, 0.025))


if __name__ == "__main__":
    main()
