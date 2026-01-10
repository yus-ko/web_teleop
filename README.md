# web_teleop

ブラウザのゲームパッド入力を WebSocket で送信し、ROS 2 側で `sensor_msgs/Joy` に変換して `teleop_twist_joy` でロボット操作につなげるための一式です。

- Web UI: `nginx` が [gamepad-extensions](gamepad-extensions/) を配信（内部で [gamepad-viewer](gamepad-viewer/) を参照）
- `str2joy`: WebSocket 受信 → `Joy` トピック `/joy` に publish
- `joy2cmd`: `teleop_twist_joy` を起動（設定: [ros2app/xbox.config.yaml](ros2app/xbox.config.yaml)）

## 構成

- [docker-compose.yml](docker-compose.yml): 全体起動（web / str2joy / joy2cmd）
- [gamepad-viewer](gamepad-viewer/): upstream の gamepad-viewer（Git submodule）
- [gamepad-extensions](gamepad-extensions/): upstream を改変せず WebSocket 送信を追加する拡張レイヤ
- [ros2app](ros2app/): ROS 2 Humble コンテナとワークスペース

## 前提

- Docker
- Docker Compose v2（`docker compose`）

## セットアップ

このリポジトリは `gamepad-viewer` を submodule として含みます。

```bash
# これからcloneする場合
git clone --recurse-submodules <this-repo-url>

# すでにclone済みの場合
git submodule update --init --recursive
```

## 起動

```bash
docker compose up -d --build
```

## 使い方

1. ブラウザで `http://localhost:8081` を開きます
2. ゲームパッドを接続すると、状態が WebSocket で送信されます（デフォルト: `ws://localhost:8080`）
3. ROS 2 側で `/joy` が publish され、`teleop_twist_joy` が必要なトピックへ変換します

### WebSocket 接続先を変える（別ホストに接続する場合）

`gamepad-extensions` は URL パラメータで WebSocket サーバーを指定できます。

- `ws` または `wsUrl`（デフォルト: `ws://localhost:8080`）

例:

```
http://localhost:8081?ws=ws://192.168.1.100:8080
```

## ポート

- `8081/tcp`: Web UI（nginx）
- `8080/tcp`: WebSocket サーバー（str2joy）

## 確認コマンド（任意）

```bash
# コンテナの状態
docker compose ps

# ログ
docker compose logs -f web
docker compose logs -f str2joy
docker compose logs -f joy2cmd
docker exec -it joy2cmd /ros_entrypoint.sh ros2 topic echo /cmd_vel
```

## 注意

- upstream の [gamepad-viewer](gamepad-viewer/) は submodule です。機能追加は [gamepad-extensions](gamepad-extensions/) 側で行う前提です。
- [ros2app/ros2_ws/test_client.py](ros2app/ros2_ws/test_client.py) は簡易クライアント例ですが、現在の WebSocket ポート（`8080`）と一致していない場合があります。利用する場合は適宜修正してください。

## Acknowledgements

- https://github.com/e7d/gamepad-viewer
