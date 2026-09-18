"""
TCP socket server for the laptop side of the robot + VR control system.
Accepts connections from multiple named clients (e.g. "turtlebot", "ur7e",
"vr_headset") and exchanges newline-delimited JSON messages with each.
"""

import socket
import threading
import json

HOST = "0.0.0.0"      # listen on all network interfaces
PORT = 65432

clients = {}           # name -> socket connection
clients_lock = threading.Lock()


def send_to(name, message: dict):
    """Send a JSON message to one named client."""
    with clients_lock:
        conn = clients.get(name)
    if conn is None:
        print(f"[!] No client named '{name}' connected")
        return
    conn.sendall((json.dumps(message) + "\n").encode())


def handle_client(conn, addr):
    name = None
    buffer = ""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buffer += data.decode()
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
                msg = json.loads(line)

                if name is None:
                    # first message from a client is its identity
                    name = msg.get("name", str(addr))
                    with clients_lock:
                        clients[name] = conn
                    print(f"[+] {name} connected from {addr}")
                    continue

                print(f"[{name}] {msg}")
                # TODO: route incoming positional data / instructions here.
                # e.g. if name == "turtlebot": update_turtlebot_state(msg)
    except (ConnectionResetError, json.JSONDecodeError) as e:
        print(f"[!] {name or addr}: {e}")
    finally:
        if name:
            with clients_lock:
                clients.pop(name, None)
            print(f"[-] {name} disconnected")
        conn.close()


def console_input():
    """Type '<name>: <message>' in the server terminal to send a message
    to a connected client — lets you test the server -> client direction
    without any real robot in the loop."""
    while True:
        line = input()
        if ":" not in line:
            print("Format: <client_name>: <message>")
            continue
        name, _, text = line.partition(":")
        send_to(name.strip(), {"text": text.strip()})


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen()
    print(f"Server listening on {HOST}:{PORT}")
    print("Type '<client_name>: <message>' here to send to a client.")

    threading.Thread(target=console_input, daemon=True).start()

    try:
        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        server.close()


if __name__ == "__main__":
    main()
