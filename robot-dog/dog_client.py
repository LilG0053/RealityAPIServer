#!/usr/bin/env python3
 
"""Native ROS 2 client for publishing the dog's pose and lidar to Reality API."""
 
from __future__ import annotations
 
import asyncio
import logging
import sys
from pathlib import Path
from time import monotonic
from typing import Any, Optional
 
import rclpy
from geometry_msgs.msg import PoseStamped
from google.protobuf.message import DecodeError
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException
 
CLIENT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(CLIENT_DIR))
sys.path.insert(0, str(CLIENT_DIR.parent))
 
from realityapi_pb2 import Response
from dog_protocol import (
    DEVICE_ID,
    LIDAR_TOPIC,
    ROS_TOPIC,
    cloud_request,
    heartbeat_request,
    pose_request,
)
 
 
SERVER_URI = "ws://192.168.123.222:65432"
HEARTBEAT_INTERVAL = 1.0
RECONNECT_DELAY = 1.5
LOG_EVERY = 10
LIDAR_LOG_EVERY = 20
 
log = logging.getLogger("realityapi-client")
 
 
class Latest:
 
    def __init__(self) -> None:
        self._value: Optional[Any] = None
        self._event = asyncio.Event()
 
    def put(self, value: Any) -> None:
        self._value = value
        self._event.set()
 
    async def get(self) -> Any:
        await self._event.wait()
        self._event.clear()
        value, self._value = self._value, None
        return value
 
 
class PoseSubscriber(Node):
    def __init__(self, poses: Latest, clouds: Latest) -> None:
        super().__init__("realityapi_pose_subscriber")
        self._poses = poses
        self._clouds = clouds
        # Latest valid dog position (x, y, z) in the ROS frame. Every cloud is
        # sliced and re-centred on the pose it arrived with.
        self._origin: Optional[tuple[float, float, float]] = None
        self._pose_frame: Optional[str] = None
        self._frames_checked = False
 
        self.create_subscription(
            PoseStamped, ROS_TOPIC, self._pose_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, LIDAR_TOPIC, self._lidar_callback, qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to %s" % ROS_TOPIC)
        self.get_logger().info("Subscribed to %s" % LIDAR_TOPIC)
 
    def _pose_callback(self, message: PoseStamped) -> None:
        try:
            request = pose_request(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning("Ignoring invalid ROS pose: %s" % error)
            return
        position = message.pose.position
        self._origin = (position.x, position.y, position.z)
        self._pose_frame = message.header.frame_id
        self._poses.put(request)
 
    def _lidar_callback(self, message: PointCloud2) -> None:
        if self._origin is None:
            return  # no pose yet, nothing to center the scan on
        if not self._frames_checked:
            self._frames_checked = True
            cloud_frame = message.header.frame_id
            if cloud_frame == self._pose_frame:
                self.get_logger().info("Pose and cloud share frame %r" % cloud_frame)
            else:
                self.get_logger().warning(
                    "Cloud frame %r differs from pose frame %r; lidar slicing assumes "
                    "they match, so scans may be misplaced" % (cloud_frame, self._pose_frame)
                )
        self._clouds.put((message, self._origin))
 
 
async def spin_ros(node: PoseSubscriber) -> None:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.0)
            await asyncio.sleep(0.002)
    finally:
        executor.remove_node(node)
 
 
async def send_heartbeats(websocket: Any) -> None:
    while True:
        await websocket.send(heartbeat_request().SerializeToString())
        await asyncio.sleep(HEARTBEAT_INTERVAL)
 
 
async def send_poses(websocket: Any, poses: Latest) -> None:
    sent_positions = 0
    while True:
        request = await poses.get()
        await websocket.send(request.SerializeToString())
        sent_positions += 1
        if sent_positions == 1 or sent_positions % LOG_EVERY == 0:
            position = request.post_device_info.transform.position
            log.info(
                "Sent dog position #%d for %s: x=%.3f y=%.3f z=%.3f",
                sent_positions,
                DEVICE_ID,
                position.x,
                position.y,
                position.z,
            )
 
 
async def send_lidar(websocket: Any, clouds: Latest) -> None:
    loop = asyncio.get_running_loop()
    sent_clouds = 0
    window_start = monotonic()
    while True:
        message, origin = await clouds.get()
        try:
            # Decode and pack in a worker thread so the event loop (ROS spin,
            # heartbeats, pose) keeps running. Clouds that arrive meanwhile
            # replace each other; the newest is sent next. The server does the
            # cropping and thinning.
            request = await loop.run_in_executor(None, cloud_request, message, origin)
        except (AttributeError, TypeError, ValueError) as error:
            log.warning("Ignoring invalid ROS point cloud: %s", error)
            continue
 
        await websocket.send(request.SerializeToString())
        sent_clouds += 1
        if sent_clouds % LIDAR_LOG_EVERY == 0:
            now = monotonic()
            log.info(
                "Sent point cloud #%d: %d points, %.1f Hz",
                sent_clouds,
                len(request.post_raw_point_cloud.xyz) // 12,
                LIDAR_LOG_EVERY / (now - window_start),
            )
            window_start = now
 
 
async def receive_replies(websocket: Any) -> None:
    async for raw in websocket:
        if isinstance(raw, str):
            log.warning("Ignoring text frame from server")
            continue
 
        reply = Response()
        try:
            reply.ParseFromString(raw)
        except DecodeError:
            log.warning("Ignoring invalid protobuf from server")
            continue
 
        kind = reply.WhichOneof("data")
        if kind is not None and kind.startswith("err_"):
            log.warning("Server rejected request %s: %s", reply.req_uuid, kind)
        else:
            log.debug("Server reply: req_uuid=%s data=%s", reply.req_uuid, kind)
 
 
async def client_once(poses: Latest, clouds: Latest) -> None:

    async with connect(SERVER_URI, compression=None) as server:
        log.info("Connected to Reality API server at %s", SERVER_URI)
        tasks = [
            asyncio.create_task(send_heartbeats(server)),
            asyncio.create_task(send_poses(server, poses)),
            asyncio.create_task(send_lidar(server, clouds)),
            asyncio.create_task(receive_replies(server)),
        ]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
            for task in done:
                task.result()
        finally:
            # Always tear the whole set down. A task left running would keep
            # pulling from the mailboxes after we reconnect.
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
 
 
async def run_client(poses: Latest, clouds: Latest) -> None:
    while rclpy.ok():
        try:
            await client_once(poses, clouds)
        except (ConnectionError, WebSocketException, OSError, asyncio.TimeoutError) as error:
            log.warning("Connection lost: %s; retrying", error)
        await asyncio.sleep(RECONNECT_DELAY)
 
 
async def run() -> None:
    poses = Latest()
    clouds = Latest()
    node = PoseSubscriber(poses, clouds)
    try:
        await asyncio.gather(spin_ros(node), run_client(poses, clouds))
    finally:
        node.destroy_node()
 
 
def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init()
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        log.info("Stopping native ROS pose subscriber")
    finally:
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()