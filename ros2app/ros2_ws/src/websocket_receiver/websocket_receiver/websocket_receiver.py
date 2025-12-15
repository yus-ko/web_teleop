import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import websockets

class WebSocketReceiver(Node):
    def __init__(self):
        super().__init__('websocket_receiver')
        self.publisher = self.create_publisher(String, 'websocket_data', 10)

    async def websocket_handler(self, websocket):
        self.get_logger().info('Client connected')
        try:
            async for message in websocket:
                try:
                    if isinstance(message, bytes):
                        message = message.decode('utf-8', errors='ignore')
                    self.publisher.publish(String(data=message))
                    self.get_logger().info(f'Received: {message}')
                except Exception as e:
                    self.get_logger().error(f'Error: {e}')
        finally:
            self.get_logger().info('Client disconnected')

    async def run_server(self):
        self.get_logger().info('Starting WebSocket server on ws://0.0.0.0:3001')
        async with websockets.serve(self.websocket_handler, "0.0.0.0", 3001):
            await asyncio.Future()

    async def spin_node(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()

async def main():
    rclpy.init()
    node = WebSocketReceiver()
    try:
        await asyncio.gather(
            node.spin_node(),
            node.run_server()
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

def sync_main():
    asyncio.run(main())

if __name__ == '__main__':
    sync_main()
