# Gamepad Extensions

このディレクトリは、元の[gamepad-viewer](../gamepad-viewer)プロジェクトに変更を加えずに、WebSocket機能を追加する拡張レイヤーです。

## 構成

- `index.html` - gamepad-viewerの元のファイルを参照する拡張版HTML
- `js/websocket.js` - WebSocket接続を管理するクラス
- `js/gamepad-adapter.js` - 元のGamepadクラスを拡張してWebSocket機能を追加するアダプタ
- `docker-compose.yml` - gamepad-viewerとこの拡張をマウントするDocker設定

## 使い方

### Docker経由で起動

```bash
docker-compose up -d
```

ブラウザで http://localhost:8082 を開きます。

### WebSocketサーバーの指定

URLパラメータで接続先を指定できます：

- `ws` または `wsUrl` - WebSocketサーバーのURL（デフォルト: `ws://localhost:8080`）

例：
```
http://localhost:8082?ws=ws://192.168.1.100:8080
```

## 仕組み

1. 元の`gamepad-viewer`のHTML/CSS/JSはそのまま読み込まれます
2. `gamepad-adapter.js`が元の`Gamepad`クラスをラップし、WebSocket機能を追加します
3. ゲームパッドの状態変化を自動的にWebSocketサーバーに送信します

元のgamepad-viewerには一切変更を加えていないため、upstream の更新にも追従しやすくなっています。

## 送信されるメッセージフォーマット

```json
{
  "type": "gamepad_state",
  "index": 0,
  "id": "Xbox 360 Controller (XInput STANDARD GAMEPAD)",
  "mapping": "standard",
  "timestamp": 1234567890,
  "template": "xbox-one",
  "color": "black",
  "triggersMeter": false,
  "zoom": 1,
  "buttons": [
    { "index": 0, "pressed": false, "value": 0 }
  ],
  "axes": [
    { "index": 0, "value": 0.0 }
  ]
}
```
