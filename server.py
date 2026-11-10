#!/usr/bin/env python

"""Reality API websocket server."""

import asyncio
import json
import math
import subprocess
import sys
import traceback
from datetime import datetime
from pathlib import Path

from google.protobuf.empty_pb2 import Empty
from google.protobuf.message import DecodeError
from websockets.asyncio.server import serve

from map.coordinate import (
    DOG_COORDS,
    SERVER_COORDS,
    TURTLEBOT_COORDS,
    convert_lidar,
    convert_transform,
)
from map.lidar import scan_to_world_points
from map.map import DASHBOARD_PATH, Map
from map.tunable import list_tunables, set_tunable
from map.pointcloud import raw_cloud_to_scan
from realityapi_pb2 import (
    Command,
    DeviceInfo,
    DeviceType,
    Lidar,
    RawLidarScan,
    RawPointCloud,
    Response,
    ServerRequest,
    Transform,
    Vector3,
)

HOST = "0.0.0.0"
PORT = 65432

# Offset added to every incoming position vector. Adjust to your task's spec.
OFFSET = (0.1, 0.1, 2.0)

# A device is dropped from the map once it has been quiet this long (seconds).
TIMEOUT = 2.0

# How often the monitor task sweeps the map for timed-out devices (seconds).
MONITOR_INTERVAL = 0.04

# Where the dog starts in the room, in dashboard axes (meters, X and Z), and which
# way it faces there (radians: 0 = toward +X, pi/2 = toward +Z). The dog's own
# odometry always starts at (0, 0) facing "forward", so these two numbers decide
# where that start is drawn. Change them to match where you set the dog down.
DOG_START_POSITION = (1.0, 2.0)
DOG_START_YAW = math.pi

# Devices that send plain pose updates in their own axes. Value("DOG") instead of
# DeviceType.DOG so this also works on older protobuf runtimes.
SENDER_COORDS = {DeviceType.Value("DOG"): DOG_COORDS}

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
    """Return a new Lidar with every point shifted by the offset."""
    return Lidar(points=[translate_vec3(point, offset) for point in lidar.points])


def place_dog_vec3(vec: Vector3) -> Vector3:
    """Rotate a dog-start-relative point by the start heading, then move it to the start."""
    c, s = math.cos(DOG_START_YAW), math.sin(DOG_START_YAW)
    return Vector3(
        x=c * vec.x - s * vec.z + DOG_START_POSITION[0],
        y=vec.y + OFFSET[1],
        z=s * vec.x + c * vec.z + DOG_START_POSITION[1],
    )


def place_dog_transform(transform: Transform) -> Transform:
    """Put a dog pose (already in server axes) at the dog's start in the room."""
    r = transform.rotation
    return Transform(
        position=place_dog_vec3(transform.position),
        rotation=Vector3(x=r.x, y=r.y + DOG_START_YAW, z=r.z),
    )


def place_dog_lidar(lidar: Lidar) -> Lidar:
    return Lidar(points=[place_dog_vec3(point) for point in lidar.points])


# Handlers for each ServerRequest.data case.

def post_device_info(info: DeviceInfo) -> Response:
    # Devices with their own axes (the dog) are converted to server axes first.
    sender = SENDER_COORDS.get(info.device_type)
    if info.HasField("transform"):
        if sender is not None:
            converted = convert_transform(info.transform, sender, SERVER_COORDS)
            info.transform.CopyFrom(place_dog_transform(converted))
        else:
            # The transform and the lidar are shifted together, otherwise scan
            # points drift away from the robot that produced them.
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


def clear_accumulated_scan(_: Empty) -> Response:
    return server_map.clear_accumulated_scan()


def handle_raw_lidar_scan(raw_scan: RawLidarScan) -> Response:
    if not server_map.exists(raw_scan.id):
        return Response(err_id_not_found={})

    # Goes through Map.handle_lidar_scan so the scan is run through the
    # localizer. post_device_info would skip it.
    lidar = scan_to_world_points(raw_scan)
    transform = convert_transform(raw_scan.transform, TURTLEBOT_COORDS, SERVER_COORDS)
    return server_map.handle_lidar_scan(
        raw_scan.id,
        translate_transform(transform),
        translate_lidar(convert_lidar(lidar, TURTLEBOT_COORDS, SERVER_COORDS)),
    )

#This method is specifically for the dog, whose raw LIDAR data is in three dimensions.
def handle_raw_point_cloud(raw: RawPointCloud) -> Response:
    # Like a raw scan, a cloud can update an existing device but not create one.
    if not server_map.exists(raw.id):
        return Response(err_id_not_found={})

    # Flatten the 3D cloud to a ring, then convert it and the pose to server
    # axes. Map.handle_lidar_scan runs the localizer, so this skips
    # post_device_info (which would convert the pose a second time).
    scan = raw_cloud_to_scan(raw)
    lidar = convert_lidar(scan, DOG_COORDS, SERVER_COORDS)
    transform = place_dog_transform(convert_transform(raw.transform, DOG_COORDS, SERVER_COORDS))
    if not lidar.points:
        # Nothing in range: update the pose but keep the last scan on the map.
        return server_map.handle_device_info(DeviceInfo(id=raw.id, transform=transform))
    return server_map.handle_lidar_scan(
        raw.id,
        transform,
        place_dog_lidar(lidar),
    )


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
    "fetch_lidar_data": fetch_lidar_data,
    "post_raw_lidar_scan": handle_raw_lidar_scan,
    "post_raw_point_cloud": handle_raw_point_cloud,
    "clear_accumulated_scan": clear_accumulated_scan,
}


def handle_request(req: ServerRequest) -> Response:
    kind = req.WhichOneof("data")
    handle = HANDLERS.get(kind)
    if handle is None:
        # kind is None when the sender used a field this server's generated
        # realityapi_pb2.py does not know (e.g. it was not regenerated).
        print(f"[!] unsupported request: {kind}")
        ret = Response(err_not_supported={})
    else:
        try:
            ret = handle(getattr(req, kind))
        except Exception:
            # One bad packet must not tear down the client's connection.
            print(f"[!] {kind} failed:")
            traceback.print_exc()
            ret = Response(err_failed_execute={})

    ret.req_uuid = req.req_uuid
    return ret


def save_rmse_histogram() -> dict:
    """Write the collected scan RMSEs and run the histogram script."""
    if not server_map.scan_rmses:
        return {"ok": False, "error": "No scan RMSEs yet. Match a few scans first."}
    directory = Path("saved_scans") / "rmse-histograms"
    directory.mkdir(parents=True, exist_ok=True)
    stem = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    values_path = directory / f"{stem}.json"
    image_path = directory / f"{stem}.png"
    values_path.write_text(json.dumps(server_map.scan_rmses))
    script = Path(__file__).resolve().parent / "testing" / "rmse_histogram.py"
    completed = subprocess.run(
        [sys.executable, str(script), str(values_path), "-o", str(image_path)],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip() or "Histogram script failed"
        return {"ok": False, "error": detail}
    return {"ok": True, "path": str(image_path), "count": len(server_map.scan_rmses)}


def handle_text_control(message: str) -> dict:
    """Dashboard controls that are not part of the protobuf robot protocol."""
    request_id = None
    try:
        request = json.loads(message)
        if not isinstance(request, dict):
            raise ValueError("Expected an object")
        request_id = request.get("id")
        action = request.get("action")
        if action == "save_rmse_histogram":
            return {"id": request_id, **save_rmse_histogram()}
        if action == "fetch_tunables":
            return {"id": request_id, "ok": True, "parameters": list_tunables()}
        if action == "set_tunable":
            updated = set_tunable(request.get("name"), request.get("value"))
            return {"id": request_id, "ok": True, "parameter": updated}
        return {"id": request_id, "ok": False, "error": "Unknown dashboard action"}
    except (KeyError, TypeError, ValueError, OSError) as exc:
        return {"id": request_id, "ok": False, "error": str(exc)}


async def handler(websocket):
    async for message in websocket:
        if isinstance(message, str):
            await websocket.send(json.dumps(handle_text_control(message)))
            continue

        req = ServerRequest()
        try:
            req.ParseFromString(message)  # decode
        except DecodeError:
            print("[!] ignoring frame that is not a ServerRequest")
            continue

        ret = handle_request(req)
        await websocket.send(ret.SerializeToString())  # encode + send binary


async def monitor(device_map: Map):
    """Sweep the map so devices that stop sending heartbeats disappear."""
    while True:
        try:
            device_map.check_all(TIMEOUT)
        except Exception:
            traceback.print_exc()
        await asyncio.sleep(MONITOR_INTERVAL)


async def main():
    server_map.load_from_json(DASHBOARD_PATH)
    monitor_task = asyncio.create_task(monitor(server_map))
    try:
        async with serve(handler, HOST, PORT, compression=None) as server:
            print(f"realityapi server on ws://{HOST}:{PORT}")
            await server.serve_forever()
    finally:
        monitor_task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
