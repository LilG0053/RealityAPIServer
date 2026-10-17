#!/usr/bin/env python3

import asyncio
import json
import logging
from typing import Any, Callable

from google.protobuf.message import DecodeError
from realityapi_pb2 import Response
from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from dog_client import DEVICE_ID, ROS_TOPIC, heartbeat_request, position_request
from ros_bridge import build_subscription, decode_publish_message


ROS_URI = "ws://127.0.0.1:9090"
SERVER_URI = "ws://127.0.0.1:65432"

# The server removes devices after two seconds without a packet.
HEARTBEAT_INTERVAL = 1.0
ROS_SILENCE_WARNING = 5.0
RECONNECT_DELAY = 1.5
LOG_EVERY = 10

log = logging.getLogger("realityapi-client")


async def send_heartbeats(websocket: Any) -> None:
    while True:
        await websocket.send(heartbeat_request().SerializeToString())
        await asyncio.sleep(HEARTBEAT_INTERVAL)


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


async def bridge_once(
    build_position_request: Callable[[dict[str, Any]], Any] = position_request,
) -> None:
    async with connect(ROS_URI) as ros:
        log.info("Connected to rosbridge at %s", ROS_URI)
        async with connect(SERVER_URI) as server:
            log.info("Connected to Reality API server at %s", SERVER_URI)
            await ros.send(json.dumps(build_subscription(ROS_TOPIC)))
            log.info("Subscribed to %s", ROS_TOPIC)

            tasks = [
                asyncio.create_task(send_heartbeats(server)),
                asyncio.create_task(receive_replies(server)),
            ]
            sent_positions = 0
            try:
                while True:
                    for task in tasks:
                        if task.done():
                            task.result()
                            raise ConnectionError("server connection task ended")

                    try:
                        raw = await asyncio.wait_for(ros.recv(), ROS_SILENCE_WARNING)
                    except asyncio.TimeoutError:
                        log.warning(
                            "No data from rosbridge for %.0fs on %s",
                            ROS_SILENCE_WARNING,
                            ROS_TOPIC,
                        )
                        continue

                    if isinstance(raw, bytes):
                        log.warning("Ignoring binary frame from rosbridge")
                        continue

                    message = decode_publish_message(raw)
                    if message is None:
                        log.warning("Ignoring invalid or non-publish ROS message: %.200s", raw)
                        continue

                    try:
                        request = build_position_request(message)
                    except ValueError as error:
                        log.warning("Invalid ROS position: %s", error)
                        continue

                    await server.send(request.SerializeToString())
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
            finally:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)


async def run_forever() -> None:
    while True:
        try:
            await bridge_once()
        except (ConnectionError, WebSocketException, OSError) as error:
            log.warning("Connection lost: %s; retrying", error)
        await asyncio.sleep(RECONNECT_DELAY)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    log.info("Starting ROS bridge for %s as %s", ROS_TOPIC, DEVICE_ID)
    asyncio.run(run_forever())


if __name__ == "__main__":
    main()
