import asyncio
import websockets
import json

async def send_data():
    uri = "ws://localhost:3001"
    async with websockets.connect(uri) as websocket:
        data = {"key": "value", "number": 123}
        message = json.dumps(data)
        await websocket.send(message)
        print(f"Sent: {message}")

asyncio.run(send_data())