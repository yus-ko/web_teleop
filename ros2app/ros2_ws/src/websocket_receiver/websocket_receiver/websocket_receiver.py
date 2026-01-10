import asyncio
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
import websockets

class WebSocketReceiver(Node):
    def __init__(self):
        super().__init__('websocket_receiver')
        self.publisher = self.create_publisher(Joy, 'joy', 10)

    async def websocket_handler(self, websocket):
        self.get_logger().info('Client connected')
        try:
            async for message in websocket:
                try:
                    if isinstance(message, bytes):
                        message = message.decode('utf-8', errors='ignore')
                    
                    # Parse JSON data
                    data = json.loads(message)
                    
                    # Only process gamepad_state messages
                    if data.get('type') == 'gamepad_state':
                        joy_msg = Joy()
                        joy_msg.header.stamp = self.get_clock().now().to_msg()
                        joy_msg.header.frame_id = 'gamepad'
                        
                        # Extract axes values
                        axes_data = data.get('axes', [])
                        joy_msg.axes = [float(axis['value']) for axis in axes_data]
                        
                        # Extract button values (convert pressed boolean to int)
                        buttons_data = data.get('buttons', [])
                        joy_msg.buttons = [int(btn['pressed']) for btn in buttons_data]
                        
                        self.publisher.publish(joy_msg)
                        self.get_logger().info(
                            f'Published Joy: axes={len(joy_msg.axes)}, buttons={len(joy_msg.buttons)}'
                        )
                    
                except json.JSONDecodeError as e:
                    self.get_logger().error(f'JSON decode error: {e}')
                except Exception as e:
                    self.get_logger().error(f'Error: {e}')
        finally:
            self.get_logger().info('Client disconnected')

    async def run_server(self):
        self.get_logger().info('Starting WebSocket server on ws://0.0.0.0:8080')
        async with websockets.serve(self.websocket_handler, "0.0.0.0", 8080):
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
