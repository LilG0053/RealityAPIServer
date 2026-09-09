import asyncio
import websockets

async def echo(websocket):
    async for message in websocket:
        print(f"Received this message from client: {message}")
        await websocket.send(f"Server received your message.")
async def main():
    async with websockets.serve(echo, "localhost", 8765):
        print("Server started on local port 8765. Listening for incoming connections...")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())