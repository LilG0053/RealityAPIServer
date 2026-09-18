"""
Generic TCP socket client — reuse this on each device (TurtleBot, UR7e
driver PC, VR headset, etc.) by giving it a unique name at startup.

Run:  python client.py <name> <server_ip>
e.g.  python client.py turtlebot 192.168.1.50
"""

import socket
import json
import sys
import threading

PORT = 65432


def listen(sock):
    """Background thread: print whatever the server sends."""
    buffer = ""
    while True:
        data = sock.recv(4096)
        if not data:
            print("Server closed the connection.")
            break
        buffer += data.decode()
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            if line.strip():
                print("Received:", json.loads(line))


def send(sock, message: dict):
    sock.sendall((json.dumps(message) + "\n").encode())


def main():
    if len(sys.argv) < 3:
        print("Usage: python client.py <name> <server_ip>")
        sys.exit(1)

    name, server_ip = sys.argv[1], sys.argv[2]

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {server_ip}:{PORT} ...")
    try:
        sock.connect((server_ip, PORT))
    except OSError as e:
        print(f"Could not connect: {e}")
        sys.exit(1)
    print("Connected. Sending identity...")
    send(sock, {"name": name})  # identify to the server
    print(f"Ready as '{name}'. Type a message and press Enter (or 'quit').")

    threading.Thread(target=listen, args=(sock,), daemon=True).start()

    # Replace this loop with real data: odometry reads, joint states,
    # VR pose updates, etc. This is just a manual test harness for now.
    try:
        while True:
            text = input("> ")
            if text == "quit":
                break
            send(sock, {"text": text})
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
