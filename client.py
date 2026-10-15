#!/usr/bin/env python3

import asyncio
import json
import logging
import math
from time import time
from typing import Any

from google.protobuf.message import DecodeError
from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from realityapi_pb2 import (
    ClientBoundPacket,
    DogInfo,
    RequestType,
    ServerBoundPacket,
    Vector3,
)


ROS_URI = "ws://127.0.0.1:9090"
SERVER_URI = "ws://127.0.0.1:65432"
ROS_TOPIC = "/utlidar/robot_pose"
DEVICE_ID = "robot"

# The server removes devices after two seconds without a packet.
HEARTBEAT_INTERVAL = 1.0
ROS_SILENCE_WARNING = 5.0
RECONNECT_DELAY = 1.5
LOG_EVERY = 10

log = logging.getLogger("realityapi-client")


def build_subscription() -> dict[str, Any]:
    """Build the rosbridge subscription for the robot pose topic."""
    return {
        "op": "subscribe",
        "id": "realityapi-position",
        "topic": ROS_TOPIC,
        "queue_length": 1,
        "throttle_rate": 0,
    }


def coordinate(position: dict[str, Any], name: str) -> float:
    value = position.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"invalid {name} coordinate: {value!r}")
    return float(value)


def find_position(payload: Any) -> dict[str, Any] | None:
    """Find position in Pose, PoseStamped, or Odometry-shaped ROS payloads."""
    current = payload
    while isinstance(current, dict) and "position" not in current:
        current = current.get("pose")
    return current.get("position") if isinstance(current, dict) else None


def position_packet(message: dict[str, Any]) -> ServerBoundPacket:
    payload = message.get("msg")
    if not isinstance(payload, dict):
        raise ValueError("payload is not an object")

    position = find_position(payload)
    if not isinstance(position, dict):
        raise ValueError("no position object in message")

    return ServerBoundPacket(
        id=DEVICE_ID,
        timestamp=time(),
        req_type=RequestType.DOG_INFO,
        dog=DogInfo(
            position=Vector3(
                x=coordinate(position, "x"),
                y=coordinate(position, "z"),
                z=coordinate(position, "y"),
            ),
            rotation=Vector3(),
        ),
    )


def heartbeat_packet() -> ServerBoundPacket:
    return ServerBoundPacket(
        id=DEVICE_ID,
        timestamp=time(),
        req_type=RequestType.HEARTBEAT,
    )


async def send_heartbeats(websocket: Any) -> None:
    while True:
        await websocket.send(heartbeat_packet().SerializeToString())
        await asyncio.sleep(HEARTBEAT_INTERVAL)


async def receive_replies(websocket: Any) -> None:
    async for raw in websocket:
        if isinstance(raw, str):
            log.warning("Ignoring text frame from server")
            continue

        reply = ClientBoundPacket()
        try:
            reply.ParseFromString(raw)
        except DecodeError:
            log.warning("Ignoring invalid protobuf from server")
            continue

        log.debug("Server reply: id=%s status=%s", reply.id, reply.status)


async def bridge_once() -> None:
    async with connect(ROS_URI) as ros:
        log.info("Connected to rosbridge at %s", ROS_URI)
        async with connect(SERVER_URI) as server:
            log.info("Connected to Reality API server at %s", SERVER_URI)
            await ros.send(json.dumps(build_subscription()))
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
                        raw = await asyncio.wait_for(
                            ros.recv(), ROS_SILENCE_WARNING
                        )
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

                    try:
                        message = json.loads(raw)
                    except json.JSONDecodeError:
                        log.warning("Invalid JSON from rosbridge: %.200s", raw)
                        continue

                    if not isinstance(message, dict):
                        continue
                    if message.get("op") != "publish":
                        log.warning("rosbridge message: %.300s", raw)
                        continue

                    try:
                        packet = position_packet(message)
                    except ValueError as error:
                        log.warning("Invalid ROS position: %s", error)
                        continue

                    await server.send(packet.SerializeToString())
                    sent_positions += 1
                    if sent_positions == 1 or sent_positions % LOG_EVERY == 0:
                        position = packet.dog.position
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
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    log.info("Starting ROS bridge for %s as %s", ROS_TOPIC, DEVICE_ID)
    asyncio.run(run_forever())


if __name__ == "__main__":
    main()
