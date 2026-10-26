#!/usr/bin/env python

import asyncio
from websockets.asyncio.server import serve
from google.protobuf.empty_pb2 import Empty
from google.protobuf.message import DecodeError
from realityapi_pb2 import Command, DeviceInfo, Lidar, Response, ServerRequest, Transform, Vector3
from map.coordinate import CoordinateSystem, Direction
from map.map import Map

SERVER_COORDS = CoordinateSystem(
    x=Direction.SOUTH,
    y=Direction.UP,
    z=Direction.EAST,
)

# Offset added to every incoming position vector. Adjust to your task's spec.
OFFSET = (0.1, 0.1, 2.0)

# A device is dropped from the map once it has been quiet this long (seconds).
TIMEOUT = 2.0

# How often the monitor task sweeps the map for timed-out devices (seconds).
MONITOR_INTERVAL = 0.04

server_map = Map()

def translate_vec3(vec: Vector3, offset=OFFSET) -> Vector3:
    """Return a new Vector3 shifted by the offset."""
    return Vector3(
        x=vec.x + offset[0],
        y=vec.y + offset[1],
        z=vec.z + offset[2],
    )

def translate_transform(transform: Transform, offset=OFFSET) -> Transform:
    """Return a new Transform with its position shifted by the offset."""
    return Transform(
        position=translate_vec3(transform.position, offset),
        rotation=transform.rotation,
    )

def translate_lidar(lidar: Lidar, offset=OFFSET) -> Lidar:
    return Lidar(points=(translate_vec3(point, offset) for point in lidar.points))

# Handlers for each ServerRequest.data case.

def post_device_info(info: DeviceInfo) -> Response:
    if info.HasField("transform"):
        info.transform.CopyFrom(translate_transform(info.transform))

    if info.HasField("lidar"):
        info.lidar.CopyFrom(translate_lidar(info.lidar))

    print(f"<<< device info from {info.id}")
    return server_map.handle_device_info(info)



def post_heartbeat(device_id: str) -> Response:
    print(f"<<< heartbeat from {device_id}")
    return server_map.handle_heartbeat(device_id)

def fetch_all_device_info(_: Empty) -> Response:
    return server_map.get_all_device_info()

def fetch_single_device_info(device_id: str) -> Response:
    return server_map.get_device_info(device_id)

def fetch_lidar_data(_: Empty) -> Response:
    return server_map.get_lidar_data()

def send_command(command: Command) -> Response:
    if not server_map.exists(command.target_id):
        return Response(err_id_not_found={})

    # TODO: forward to the target device as a ClientRequest once the server
    # tracks each device's connection.
    return Response(err_not_supported={})

# Keyed by the ServerRequest.data oneof field names.
HANDLERS = {
    "post_device_info": post_device_info,
    "post_heartbeat": post_heartbeat,
    "fetch_all_device_info": fetch_all_device_info,
    "fetch_single_device_info": fetch_single_device_info,
    "send_command": send_command,
    "fetch_lidar_data": fetch_lidar_data
}

def handle_request(req: ServerRequest) -> Response:
    kind = req.WhichOneof("data")
    if kind is None:
        ret = Response(err_not_supported={})
    else:
        ret = HANDLERS[kind](getattr(req, kind))

    ret.req_uuid = req.req_uuid
    return ret

async def handler(websocket):
    async for message in websocket:
        if isinstance(message, str):
            print("[!] ignoring text frame (expected binary protobuf)")
            continue

        req = ServerRequest()
        try:
            req.ParseFromString(message)                  # decode
        except DecodeError:
            print("[!] ignoring frame that is not a ServerRequest")
            continue

        ret = handle_request(req)
        await websocket.send(ret.SerializeToString())   # encode + send binary

async def monitor(map: Map):
    """Sweep the map so devices that stop sending heartbeats disappear."""
    while True:
        map.check_all(TIMEOUT)
        await asyncio.sleep(MONITOR_INTERVAL)


async def main():
    server_map.load_from_json("dashboard/updated_pos.json")
    server = await serve(handler, "0.0.0.0", 65432)
    asyncio.create_task(monitor(server_map))
    print("realityapi server on ws://0.0.0.0:65432")
    await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())
