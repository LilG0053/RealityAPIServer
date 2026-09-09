import asyncio
import websockets

async def communicate():
    uri = "ws://localhost:8765"
    
    
    async with websockets.connect(uri) as websocket:
        message_to_send = "Hello, WebSocket!"
        print(f"Sending: {message_to_send}")
        
      
        await websocket.send(message_to_send)
        
       
        response = await websocket.recv()
        print(f"Server replied: {response}")

if __name__ == "__main__":
    asyncio.run(communicate())