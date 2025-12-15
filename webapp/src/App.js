import React, { useState, useEffect, useRef } from 'react';
import './App.css';

function App() {
  const [gamepad, setGamepad] = useState(null);
  const previousButtons = useRef([]);
  const ws = useRef(null);

  const handleButtonPress = (buttonIndex) => {
    console.log(`Button ${buttonIndex} pressed!`);
    // ここにコールバックの処理を追加
  };

  useEffect(() => {
    // WebSocket接続
    ws.current = new WebSocket('ws://localhost:3001');
    ws.current.onopen = () => {
      console.log('Connected to WebSocket server');
    };
    ws.current.onclose = () => {
      console.log('Disconnected from WebSocket server');
    };
    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  useEffect(() => {
    const updateGamepad = () => {
      const gamepads = navigator.getGamepads();
      if (gamepads[0]) {
        const currentGamepad = gamepads[0];
        setGamepad(currentGamepad);

        // ボタンの状態をチェック
        currentGamepad.buttons.forEach((button, index) => {
          const wasPressed = previousButtons.current[index]?.pressed || false;
          if (button.pressed && !wasPressed) {
            handleButtonPress(index);
          }
        });

        // 前のボタン状態を更新
        previousButtons.current = currentGamepad.buttons.map(button => ({ pressed: button.pressed }));

        // WebSocketでゲームパッドの状態を送信
        if (ws.current && ws.current.readyState === WebSocket.OPEN) {
          const gamepadData = {
            id: currentGamepad.id,
            buttons: currentGamepad.buttons.map(button => ({ pressed: button.pressed, value: button.value })),
            axes: currentGamepad.axes
          };
          ws.current.send(JSON.stringify(gamepadData));
        }
      }
    };

    const interval = setInterval(updateGamepad, 100); // 100msごとに更新

    return () => clearInterval(interval);
  }, []);

  return (
    <div className="App">
      <header className="App-header">
        <h1>Gamepad Input</h1>
        {gamepad ? (
          <div>
            <h2>Gamepad Connected: {gamepad.id}</h2>
            <h3>Buttons:</h3>
            <ul>
              {gamepad.buttons.map((button, index) => (
                <li key={index}>Button {index}: {button.pressed ? 'Pressed' : 'Not Pressed'} (Value: {button.value.toFixed(2)})</li>
              ))}
            </ul>
            <h3>Axes:</h3>
            <ul>
              {gamepad.axes.map((axis, index) => (
                <li key={index}>Axis {index}: {axis.toFixed(3)}</li>
              ))}
            </ul>
          </div>
        ) : (
          <p>No gamepad connected. Please connect a gamepad and press a button.</p>
        )}
      </header>
    </div>
  );
}

export default App;
