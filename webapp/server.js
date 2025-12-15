const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 3001 });

// ROS2 joy コンテナの WebSocket サーバーに接続
const ros2joyWs = new WebSocket('ws://ros2joy:3001');

ros2joyWs.on('open', () => {
    console.log('Connected to ROS2 joy WebSocket server');
});

ros2joyWs.on('close', () => {
    console.log('Disconnected from ROS2 joy WebSocket server');
});

ros2joyWs.on('error', (error) => {
    console.error('ROS2 joy WebSocket error:', error);
});

wss.on('connection', (ws) => {
    console.log('Client connected');

    ws.on('message', (message) => {
        console.log('Received:', message.toString());
        // ROS2 joy にメッセージを転送
        if (ros2joyWs.readyState === WebSocket.OPEN) {
            ros2joyWs.send(message);
        } else {
            console.error('ROS2 joy WebSocket is not open');
        }
    });

    ws.on('close', () => {
        console.log('Client disconnected');
    });
});

console.log('WebSocket server running on ws://localhost:3001');