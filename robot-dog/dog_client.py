#!/usr/bin/env python3

"""Native ROS 2 client for publishing the dog's pose to Reality API."""

from __future__ import annotations

import asyncio
import logging
import sys
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from google.protobuf.message import DecodeError
from rclpy.node import Node
from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

CLIENT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(CLIENT_DIR))
sys.path.insert(0, str(CLIENT_DIR.parent))

from realityapi_pb2 import Response, ServerRequest
from dog_protocol import DEVICE_ID, ROS_TOPIC, heartbeat_request, pose_request


SERVER_URI = "ws://10.89.39.9:65432"
HEARTBEAT_INTERVAL = 1.0
RECONNECT_DELAY = 1.5
LOG_EVERY = 10

log = logging.getLogger("realityapi-client")


class PoseSubscriber(Node):
    def __init__(self, requests: asyncio.Queue[ServerRequest]) -> None:
        super().__init__("realityapi_pose_subscriber")
        self._requests = requests
        self.create_subscription(PoseStamped, ROS_TOPIC, self._pose_callback, 10)
        self.get_logger().info("Subscribed to %s" % ROS_TOPIC)

    def _pose_callback(self, message: PoseStamped) -> None:
        try:
            request = pose_request(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning("Ignoring invalid ROS pose: %s" % error)
            return

        if self._requests.full():
            self._requests.get_nowait()
        self._requests.put_nowait(request)


async def spin_ros(node: PoseSubscriber) -> None:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.01)


async def send_heartbeats(websocket: Any) -> None:
    while True:
        await websocket.send(heartbeat_request().SerializeToString())
        await asyncio.sleep(HEARTBEAT_INTERVAL)


async def send_poses(
    websocket: Any,
    requests: asyncio.Queue[ServerRequest],
) -> None:
    sent_positions = 0
    while True:
        request = await requests.get()
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

        log.debug(
            "Server reply: req_uuid=%s data=%s",
            reply.req_uuid,
            reply.WhichOneof("data"),
        )


async def client_once(requests: asyncio.Queue[ServerRequest]) -> None:
    async with connect(SERVER_URI) as server:
        log.info("Connected to Reality API server at %s", SERVER_URI)
        tasks = [
            asyncio.create_task(send_heartbeats(server)),
            asyncio.create_task(send_poses(server, requests)),
            asyncio.create_task(receive_replies(server)),
        ]
        done, pending = await asyncio.wait(
            tasks,
            return_when=asyncio.FIRST_EXCEPTION,
        )
        for task in done:
            task.result()
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)


async def run_client(requests: asyncio.Queue[ServerRequest]) -> None:
    while rclpy.ok():
        try:
            await client_once(requests)
        except (ConnectionError, WebSocketException, OSError) as error:
            log.warning("Connection lost: %s; retrying", error)
        await asyncio.sleep(RECONNECT_DELAY)


async def run(node: PoseSubscriber, requests: asyncio.Queue[ServerRequest]) -> None:
    await asyncio.gather(spin_ros(node), run_client(requests))


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init()
    requests: asyncio.Queue[ServerRequest] = asyncio.Queue(maxsize=1)
    node = PoseSubscriber(requests)
    try:
        asyncio.run(run(node, requests))
    except KeyboardInterrupt:
        log.info("Stopping native ROS pose subscriber")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
