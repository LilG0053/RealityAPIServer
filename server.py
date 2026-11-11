#!/usr/bin/env python

import asyncio
import math
import os
import time
import json
import shutil
from datetime import datetime
from pathlib import Path
from websockets.asyncio.server import serve
from google.protobuf.empty_pb2 import Empty
from google.protobuf.message import DecodeError
from realityapi_pb2 import Command, DeviceInfo, Lidar, Response, ServerRequest, Transform, Vector3, RawLidarScan, DeviceType
from map.coordinate import CoordinateSystem, Direction, convert_lidar, convert_transform, convert_point_array
from map.map import Map
from map.lidar import LIDAR_EPSILON, simplify_lidar_2d, points_to_lidar
from map.localization import ScanLocalizer
from map.scan_pipeline import register_scan
from map.icp_diagnostics import ICPDiagnostics
import numpy as np

SERVER_COORDS = CoordinateSystem(
    x=Direction.SOUTH,
    y=Direction.UP,
    z=Direction.EAST,
)

TURTLEBOT_COORDS = CoordinateSystem(
    x=Direction.NORTH,
    y=Direction.WEST,
    z=Direction.UP,
)

# Offset added to every incoming position vector. Adjust to your task's spec.
OFFSET = (0.0, 0.0, 0.0)

# A device is dropped from the map once it has been quiet this long (seconds).
TIMEOUT = 2.0

# How often the monitor task sweeps the map for timed-out devices (seconds).
MONITOR_INTERVAL = 0.10

server_map = Map()

# unique scan localizer for each device_id
scan_localizers: dict[str, ScanLocalizer] = {}
icp_diagnostics: ICPDiagnostics | None = None

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

def handle_raw_odom(info: DeviceInfo) -> Transform:
    # DeviceInfo odometry is already in server coordinates; registration uses ROS XY.
    if info.id not in scan_localizers:
        scan_localizers[info.id] = ScanLocalizer()

    localizer = scan_localizers[info.id]
    return localizer.corrected_transform(info.transform, (0, 0, 0))

# Handlers for each ServerRequest.data case.
def post_device_info(info: DeviceInfo, *, pose_is_corrected: bool = False) -> Response:
    if info.HasField("transform"):
        corrected = info.transform if pose_is_corrected else handle_raw_odom(info)
        info.transform.CopyFrom(translate_transform(corrected))

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

def handle_raw_lidar_scan(raw_scan: RawLidarScan) -> Response:
    global icp_diagnostics
    registered = server_map.exists(raw_scan.id)
    received_utc = None
    if icp_diagnostics is not None:
        try:
            # records the scan
            received_utc = icp_diagnostics.record_scan(raw_scan, registered=registered)
        except OSError as exc:
            print(f'ICP recording stopped: {exc}', flush=True)
            icp_diagnostics.close()
            icp_diagnostics = None
    # A scan can update an existing device but cannot create one by itself.
    if not registered:
        return Response(err_id_not_found={})

    # ScanLocalizer consumes odometry-placed server-coordinate points.
    if raw_scan.id not in scan_localizers:
        scan_localizers[raw_scan.id] = ScanLocalizer()

    localizer = scan_localizers[raw_scan.id]
    started = time.perf_counter()
    result, predicted_pose, corrected_robot_transform, reference_count = register_scan(
        localizer, raw_scan, TURTLEBOT_COORDS, SERVER_COORDS)
    elapsed_ms = (time.perf_counter() - started) * 1000
    if icp_diagnostics is not None:
        try:
            icp_diagnostics.write(raw_scan, result, predicted_pose, elapsed_ms,
                                  reference_count,
                                  received_utc=received_utc)
        except OSError as exc:
            print(f'ICP recording stopped: {exc}', flush=True)
            icp_diagnostics.close()
            icp_diagnostics = None
    corrected_points = result.points

    # Bad matches can be displayed at the predicted pose but must not enter either map.
    if result.accepted:
        server_map.update_point_map(corrected_points)
        server_map.update_occupancy_grid(corrected_robot_transform, corrected_points)
        server_map.export_global_points("dashboard/global_map.json")
        server_map.export_occupancy_grid("dashboard/occupancy_grid.json")

    # turns the corrected lidar points back to protobuf LIDAR message
    corrected_lidar = points_to_lidar(corrected_points)
    
    device_lidar_info = DeviceInfo(
        id = raw_scan.id,
        transform=corrected_robot_transform,
        lidar=simplify_lidar_2d(corrected_lidar, epsilon=LIDAR_EPSILON), # Moved simplify here because we dont want to simplify our ICP calculations
    )

    return post_device_info(device_lidar_info, pose_is_corrected=True)

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
}

def handle_request(req: ServerRequest) -> Response:
    kind = req.WhichOneof("data")
    if kind is None:
        ret = Response(err_not_supported={})
    else:
        ret = HANDLERS[kind](getattr(req, kind))

    ret.req_uuid = req.req_uuid
    return ret


# Dashboard map controls use text frames; robot traffic remains protobuf.
def handle_map_control(action: str) -> dict:
    dashboard = Path("dashboard")
    if action == "icp_status":
        return {"ok": True, "devices": [
            {"id": device_id, "last_stop_reason": localizer.last_stop_reason,
             "counts": dict(localizer.convergence_counts)}
            for device_id, localizer in scan_localizers.items()
            if isinstance(localizer, ScanLocalizer)
        ]}
    if action == "save_maps":
        archive_root = Path("saved_scans")
        archive_root.mkdir(exist_ok=True)
        stem = datetime.now().strftime("%Y-%m-%d_%H-%M")
        archive = archive_root / stem
        suffix = 2
        while archive.exists():
            archive = archive_root / f"{stem}_{suffix}"
            suffix += 1
        archive.mkdir()
        try:
            # updated_pos contains the latest per-device LiDAR scan and pose.
            defaults = {"updated_pos.json": [], "global_map.json": [],
                        "occupancy_grid.json": {"cell_size": server_map.cell_size, "cells": []}}
            for name, empty in defaults.items():
                source = dashboard / name
                if source.exists():
                    shutil.copyfile(source, archive / name)
                else:
                    (archive / name).write_text(json.dumps(empty))
        except OSError:
            shutil.rmtree(archive, ignore_errors=True)
            raise
        return {"ok": True, "path": str(archive)}
    if action == "clear_maps":
        server_map.point_map.clear()
        server_map.occupancy_grid.clear()
        scan_localizers.clear()
        server_map.clear_lidar_scans()
        server_map.export_global_points(str(dashboard / "global_map.json"))
        server_map.export_occupancy_grid(str(dashboard / "occupancy_grid.json"))
        server_map.export_to_json(str(dashboard / "updated_pos.json"))
        return {"ok": True}
    return {"ok": False, "error": "Unknown map action"}

async def handler(websocket):
    async for message in websocket:
        if isinstance(message, str):
            request_id = None
            try:
                request = json.loads(message)
                if not isinstance(request, dict):
                    raise ValueError("Expected an object")
                request_id = request.get("id")
                result = handle_map_control(request.get("action"))
                await websocket.send(json.dumps({"id": request_id, **result}))
            except (ValueError, OSError) as exc:
                await websocket.send(json.dumps({"id": request_id, "ok": False, "error": str(exc)}))
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
        for device_id in list(scan_localizers):
            if not map.exists(device_id):
                del scan_localizers[device_id]
        await asyncio.sleep(MONITOR_INTERVAL)


async def main():
    global icp_diagnostics
    log_dir = os.environ.get('ICP_LOG_DIR')
    if log_dir:
        icp_diagnostics = ICPDiagnostics(log_dir, name=os.environ.get('ICP_RUN_NAME'))
    server_map.load_from_json("dashboard/updated_pos.json")
    server = await serve(handler, "0.0.0.0", 65432)
    asyncio.create_task(monitor(server_map))
    print("realityapi server on ws://0.0.0.0:65432")
    try:
        await server.serve_forever()
    finally:
        if icp_diagnostics is not None:
            icp_diagnostics.close()


if __name__ == "__main__":
    asyncio.run(main())
