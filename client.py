#!/usr/bin/env python

from time import sleep, time

from websockets.sync.client import connect
from realityapi_pb2 import Packet, DeviceType, Heartbeat

URI = "ws://10.89.53.91:65432"

# Seconds between keepalives. Must be well under the server's TIMEOUT.
HEARTBEAT_INTERVAL = 0.05


def ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Please enter a number.")


def describe(pkt: Packet) -> str:
    if pkt.HasField("heartbeat"):
        return f"heartbeat at {pkt.heartbeat.current_time:.3f}, id {pkt.id}"

    p = pkt.position
    return (f"position ({p.x}, {p.y}, {p.z}), "
            f"devicetype {DeviceType.Name(pkt.devicetype)}, id {pkt.id}")


def exchange(websocket, pkt: Packet) -> Packet:
    websocket.send(pkt.SerializeToString())
    reply = Packet()
    reply.ParseFromString(websocket.recv())
    return reply


def build_packet() -> Packet:
    device_id = input("What's your id? ")
    device_type = input("What's your device? ")
    x = ask_float("X coordinate? ")
    y = ask_float("Y coordinate? ")
    z = ask_float("Z coordinate? ")

    pkt = Packet()
    pkt.id = device_id                 # use int(device_id) if id is an int field
    pkt.devicetype = DeviceType.Value(device_type)
    pkt.position.x = x                 # sets the 'position' branch of the oneof
    pkt.position.y = y
    pkt.position.z = z
    return pkt


def send_heartbeats(websocket, pkt: Packet):
    """Keep telling the server this device is alive until interrupted."""
    while True:
        beat = Packet(
            heartbeat=Heartbeat(current_time=time()),
            devicetype=pkt.devicetype,
            id=pkt.id,
        )
        print(f">>> {describe(beat)}")
        reply = exchange(websocket, beat)
        print(f"<<< {describe(reply)}")

        sleep(HEARTBEAT_INTERVAL)


def main():
    pkt = build_packet()
    with connect(URI) as websocket:
        print(f">>> {describe(pkt)}")
        reply = exchange(websocket, pkt)
        print(f"<<< {describe(reply)}")

        send_heartbeats(websocket, pkt)


if __name__ == "__main__":
    main()
