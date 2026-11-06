#!/usr/bin/env python

import asyncio
import math
from websockets.asyncio.server import serve
from google.protobuf.empty_pb2 import Empty
from google.protobuf.message import DecodeError
from realityapi_pb2 import Command, DeviceInfo, Lidar, Response, ServerRequest, Transform, Vector3, RawLidarScan, DeviceType
from map.coordinate import CoordinateSystem, Direction, convert_lidar, convert_transform, convert_point_array
from map.map import Map
from map.lidar import LIDAR_EPSILON, simplify_lidar_2d, points_to_lidar
from map.kiss_icp_2d import OdometryICP2D
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
scan_localizers: dict[str, OdometryICP2D] = {}

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
        scan_localizers[info.id] = OdometryICP2D()

    localizer = scan_localizers[info.id]
    ros_odom = convert_transform(info.transform, SERVER_COORDS, TURTLEBOT_COORDS)
    corrected_ros = localizer.corrected_transform(ros_odom)
    return convert_transform(corrected_ros, TURTLEBOT_COORDS, SERVER_COORDS)

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
    # A scan can update an existing device but cannot create one by itself.
    if not server_map.exists(raw_scan.id):
        return Response(err_id_not_found={})

    # Registration receives robot-local points and the odometry pose separately.
    if raw_scan.id not in scan_localizers:
        scan_localizers[raw_scan.id] = OdometryICP2D()

    localizer = scan_localizers[raw_scan.id]
    result = localizer.register_scan(raw_scan)
    corrected_ros = localizer.corrected_transform(raw_scan.transform)
    corrected_robot_transform = convert_transform(corrected_ros, TURTLEBOT_COORDS, SERVER_COORDS)
    ros_points = np.column_stack((
        result.points,
        np.full(len(result.points), raw_scan.transform.position.z),
    ))
    corrected_points = convert_point_array(ros_points, TURTLEBOT_COORDS, SERVER_COORDS)

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
        for device_id in list(scan_localizers):
            if not map.exists(device_id):
                del scan_localizers[device_id]
        await asyncio.sleep(MONITOR_INTERVAL)


async def main():
    server_map.load_from_json("dashboard/updated_pos.json")
    server = await serve(handler, "0.0.0.0", 65432)
    asyncio.create_task(monitor(server_map))
    print("realityapi server on ws://0.0.0.0:65432")
    await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())
