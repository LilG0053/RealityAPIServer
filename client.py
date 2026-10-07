#!/usr/bin/env python

from websockets.sync.client import connect
from realityapi_pb2 import Packet, DeviceType

URI = "ws://10.89.53.91:65432"


def ask_float(prompt: str) -> float:
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
        except (ConnectionClosed, OSError, asyncio.TimeoutError) as error:
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