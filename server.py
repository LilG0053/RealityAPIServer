#!/usr/bin/env python

"""Reality API websocket server."""

import asyncio
import json
import subprocess
import sys
import traceback
from datetime import datetime
from pathlib import Path

from google.protobuf.empty_pb2 import Empty
from google.protobuf.message import DecodeError
from websockets.asyncio.server import serve

from map.map import Map
from map.tunable import list_tunables, set_tunable
from realityapi_pb2 import (
    Command,
    DeviceInfo,
    OccupancyGridReq,
    Response,
    ServerRequest,
)

HOST = "0.0.0.0"
PORT = 65432

# A device is dropped from the map once it has been quiet this long (seconds).
TIMEOUT = 2.0

# How often the monitor task sweeps the map for timed-out devices (seconds).
MONITOR_INTERVAL = 0.04

# How often the occupancy grid is saved while it keeps changing (seconds).
OCCUPANCY_SAVE_INTERVAL = 1.0

server_map = Map()


# Handlers for each ServerRequest.data case.

def post_device_info(info: DeviceInfo) -> Response:
    # Clients send everything in server coordinates; the map localizes and
    # merges any lidar in the post.
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


def fetch_room_bounds(_: Empty) -> Response:
    return server_map.get_room_bounds()


def fetch_occupancy_grid(request: OccupancyGridReq) -> Response:
    return server_map.get_occupancy_grid(request.device_type, request.since_revision)


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
    "clear_accumulated_scan": clear_accumulated_scan,
    "fetch_room_bounds": fetch_room_bounds,
    "fetch_occupancy_grid": fetch_occupancy_grid,
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


async def save_occupancy(device_map: Map):
    """Save the occupancy grids at most once per interval, only when they changed."""
    while True:
        try:
            for grid in device_map.occupancy_grids():
                grid.save()
        except Exception:
            traceback.print_exc()
        await asyncio.sleep(OCCUPANCY_SAVE_INTERVAL)


async def main():
    monitor_task = asyncio.create_task(monitor(server_map))
    save_task = asyncio.create_task(save_occupancy(server_map))
    try:
        async with serve(handler, HOST, PORT, compression=None) as server:
            print(f"realityapi server on ws://{HOST}:{PORT}")
            await server.serve_forever()
    finally:
        monitor_task.cancel()
        save_task.cancel()
        # Keep whatever changed since the last periodic save.
        for grid in server_map.occupancy_grids():
            grid.save()


if __name__ == "__main__":
    asyncio.run(main())
