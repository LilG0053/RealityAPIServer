#!/usr/bin/env python

from websockets.sync.client import connect
from realityapi_pb2 import Packet, Vector3, Hello, Text, Heartbeat
from time import sleep,time
from map.device import DeviceType

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
        return f"{DeviceType(int(pkt.heartbeat.device_type))}"
    return "empty packet"


def exchange(websocket, pkt: Packet) -> Packet:
    websocket.send(pkt.SerializeToString())
    reply = Packet()
    reply.ParseFromString(websocket.recv())
    return reply

def send_heartbeat(websocket, device_name, text):
    while True:
        CURRENT_TIME = 0
        pkt = Packet(heartbeat=Heartbeat(current_time=CURRENT_TIME, name=device_name, device_type=text))
        print(f">>> Current heartbeat: {describe(pkt)}")
        reply = exchange(websocket, pkt)
        print(f"<<< {describe(reply)}")
    
        sleep(0.05)

def hello():
    name = input("What's your name? ")
    text = input("What's your device_type? ")
    x = ask_float("X coordinate? ")
    y = ask_float("Y coordinate? ")
    z = ask_float("Z coordinate? ")
    CURRENT_TIME = time()

    packets = [
        Packet(hello=Hello(name=name)),
        Packet(text=Text(text=text)),
        Packet(position=Vector3(x=x, y=y, z=z)),
        Packet(heartbeat=Heartbeat(current_time=CURRENT_TIME,name=name, device_type=text))
    ]

    with connect(URI) as websocket:
        for pkt in packets:
            reply = exchange(websocket, pkt)
            print(f"<<< {describe(reply)}")
        
        send_heartbeat(websocket, name, text)



if __name__ == "__main__":
    hello()
