#!/usr/bin/env python

from websockets.sync.client import connect
from realityapi_pb2 import Packet, Vector3, Hello, Text, Heartbeat
from time import sleep, perf_counter, time

URI = "ws://localhost:8765"


def ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Please enter a number.")


def describe(pkt: Packet) -> str:
    kind = pkt.WhichOneof("body")
    if kind == "position":
        p = pkt.position
        return f"position ({p.x}, {p.y}, {p.z})"
    if kind == "text":
        return f"text: {pkt.text.text}"
    if kind == "hello":
        return f"hello from {pkt.hello.name}"

    if kind == "heartbeat":
        return f"{pkt.heartbeat.current_time}, {pkt.heartbeat.name}"
    return "empty packet"


def exchange(websocket, pkt: Packet) -> Packet:
    websocket.send(pkt.SerializeToString())
    reply = Packet()
    reply.ParseFromString(websocket.recv())
    return reply


def hello():
    name = input("What's your name? ")
    text = input("What's your message? ")
    x = ask_float("X coordinate? ")
    y = ask_float("Y coordinate? ")
    z = ask_float("Z coordinate? ")
    CURRENT_TIME = time()
    

    packets = [
        Packet(hello=Hello(name=name)),
        Packet(text=Text(text=text)),
        Packet(position=Vector3(x=x, y=y, z=z)),
        Packet(heartbeat=Heartbeat(current_time=CURRENT_TIME))
    ]

    with connect(URI) as websocket:
        for pkt in packets:
            print(f">>> {describe(pkt)}")
            reply = exchange(websocket, pkt)
            print(f"<<< {describe(reply)}")
        send_heartbeat(websocket, name)

def send_heartbeat(websocket, device_name):
    while True:
        CURRENT_TIME = time()
        pkt = Packet(heartbeat=Heartbeat(current_time=CURRENT_TIME, name=device_name))
        print(f">>> Current heartbeat: {describe(pkt)}")
        reply = exchange(websocket, pkt)
        print(f"<<< {describe(reply)}")
    
        sleep(0.05)

if __name__ == "__main__":
    hello()
