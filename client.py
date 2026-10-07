#!/usr/bin/env python3

import asyncio
import json
import logging
import os
from contextlib import suppress
from time import time
from typing import Any, Dict

import websockets
from google.protobuf.message import DecodeError
from websockets.exceptions import ConnectionClosed, WebSocketException

from realityapi_pb2 import DeviceType, Heartbeat, Packet, Vector3


ROSBRIDGE_URI = os.getenv("ROSBRIDGE_URI", "ws://127.0.0.1:9090")
SERVER_URI = os.getenv("SERVER_URI", "ws://10.89.53.91:65432")
ROS_TOPIC = os.getenv("ROS_TOPIC", "/utlidar/robot_pose")
ROS_MESSAGE_TYPE = os.getenv("ROS_MESSAGE_TYPE", "geometry_msgs/PoseStamped")
DEVICE_ID = os.getenv("DEVICE_ID", "robot")
DEVICE_TYPE = os.getenv("DEVICE_TYPE", "TURTLE").upper()
HEARTBEAT_INTERVAL = float(os.getenv("HEARTBEAT_INTERVAL", "1.0"))
RECONNECT_DELAY_SECONDS = 1.5

logger = logging.getLogger("client")


def build_subscription() -> Dict[str, Any]:
    subscription: Dict[str, Any] = {
        "op": "subscribe",
        "topic": ROS_TOPIC,
        "queue_length": 1,
        "throttle_rate": 0,
    }
    if ROS_MESSAGE_TYPE:
        subscription["type"] = ROS_MESSAGE_TYPE
    return subscription


def _coordinate(payload: Dict[str, Any], name: str) -> float:
    value = payload.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"pose position has an invalid {name} coordinate")
    return float(value)


def _device_type_value() -> int:
    try:
        return DeviceType.Value(DEVICE_TYPE)
    except ValueError as error:
        raise ValueError(f"unknown DEVICE_TYPE: {DEVICE_TYPE}") from error


def build_position_packet(message: Dict[str, Any]) -> Packet:
    topic = message.get("topic")
    payload = message.get("msg")
    if not isinstance(topic, str) or not isinstance(payload, dict):
        raise ValueError("rosbridge publish message has an invalid topic or payload")

    position = payload.get("position")
    if not isinstance(position, dict):
        raise ValueError("rosbridge pose message has no position object")

    return Packet(
        position=Vector3(
            x=_coordinate(position, "x"),
            y=_coordinate(position, "y"),
            z=_coordinate(position, "z"),
        ),
        devicetype=_device_type_value(),
        id=DEVICE_ID,
    )


def build_heartbeat_packet() -> Packet:
    return Packet(
        heartbeat=Heartbeat(current_time=time()),
        devicetype=_device_type_value(),
        id=DEVICE_ID,
    )


async def send_heartbeats(server_socket: Any) -> None:
    while True:
        await server_socket.send(build_heartbeat_packet().SerializeToString())
        await asyncio.sleep(HEARTBEAT_INTERVAL)


async def receive_server_replies(server_socket: Any) -> None:
    async for raw_message in server_socket:
        if isinstance(raw_message, str):
            logger.warning("Ignoring text frame received from GTXR server")
            continue

        reply = Packet()
        try:
            reply.ParseFromString(raw_message)
        except DecodeError:
            logger.warning("Ignoring invalid protobuf received from GTXR server")
            continue

        logger.debug("Received server reply: %s", reply)


async def bridge_once() -> None:
    async with websockets.connect(ROSBRIDGE_URI) as ros_socket:
        logger.info("Connected to rosbridge at %s", ROSBRIDGE_URI)
        async with websockets.connect(SERVER_URI) as server_socket:
            logger.info("Connected to GTXR server at %s", SERVER_URI)
            await ros_socket.send(json.dumps(build_subscription()))
            logger.info("Subscribed to ROS topic %s", ROS_TOPIC)

            heartbeat_task = asyncio.create_task(send_heartbeats(server_socket))
            reply_task = asyncio.create_task(receive_server_replies(server_socket))
            try:
                async for raw_message in ros_socket:
                    try:
                        message = json.loads(raw_message)
                    except json.JSONDecodeError:
                        logger.warning("Ignoring invalid JSON received from rosbridge")
                        continue

                    if message.get("op") != "publish":
                        continue

                    try:
                        packet = build_position_packet(message)
                    except ValueError as error:
                        logger.warning("Ignoring malformed rosbridge message: %s", error)
                        continue

                    await server_socket.send(packet.SerializeToString())
                    logger.debug("Forwarded %s pose for device %s", ROS_TOPIC, DEVICE_ID)
            finally:
                for task in (heartbeat_task, reply_task):
                    task.cancel()
                with suppress(asyncio.CancelledError):
                    await asyncio.gather(heartbeat_task, reply_task)


async def run_forever() -> None:
    while True:
        try:
            await bridge_once()
        except (WebSocketException,ConnectionClosed, OSError, asyncio.TimeoutError) as error:
            logger.warning("Connection lost: %s; retrying", error)
            await asyncio.sleep(RECONNECT_DELAY_SECONDS)


def main() -> None:
    logging.basicConfig(
        level=os.getenv("LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    logger.info("Starting ROS bridge client for %s", ROS_TOPIC)
    asyncio.run(run_forever())


if __name__ == "__main__":
    main()
