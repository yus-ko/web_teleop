// Lightweight WebSocket helper for gamepad streaming
if (typeof window.GamepadWebSocket === 'undefined') {
    window.GamepadWebSocket = class GamepadWebSocket {
        constructor({ url, reconnectDelay = 2000, heartbeatInterval = 30000, debug = false } = {}) {
            this.url = url;
            this.reconnectDelay = reconnectDelay;
            this.heartbeatInterval = heartbeatInterval;
            this.debug = debug;
            this.shouldReconnect = !!url;
            this.ws = null;
            this.heartbeatTimer = null;
            this.handshakePayload = null;
        }

        connect(handshakePayload) {
            this.handshakePayload = handshakePayload;
            this.open();
        }

        setDebug(debug) {
            this.debug = debug;
        }

        close() {
            this.shouldReconnect = false;
            this.stopHeartbeat();
            if (this.ws) {
                this.ws.close();
            }
        }

        open() {
            if (!this.url) return;

            if (this.ws && this.ws.readyState === WebSocket.OPEN) {
                this.ws.close();
            }

            try {
                this.ws = new WebSocket(this.url);
            } catch (error) {
                console.error("WebSocket connection failed", error);
                return;
            }

            this.ws.onopen = () => {
                this.startHeartbeat();
                if (this.handshakePayload) {
                    this.send(this.handshakePayload);
                }
            };

            this.ws.onmessage = (event) => {
                if (this.debug) {
                    console.log("WebSocket message received", event.data);
                }
            };

            this.ws.onerror = (error) => {
                console.error("WebSocket error", error);
            };

            this.ws.onclose = () => {
                this.stopHeartbeat();
                if (this.shouldReconnect) {
                    window.setTimeout(() => this.open(), this.reconnectDelay);
                }
            };
        }

        startHeartbeat() {
            this.stopHeartbeat();
            this.heartbeatTimer = window.setInterval(() => {
                this.send({ type: "ping", ts: Date.now() });
            }, this.heartbeatInterval);
        }

        stopHeartbeat() {
            if (this.heartbeatTimer) {
                window.clearInterval(this.heartbeatTimer);
                this.heartbeatTimer = null;
            }
        }

        send(message) {
            if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
            try {
                this.ws.send(JSON.stringify(message));
            } catch (error) {
                console.error("WebSocket send failed", error);
            }
        }

        sendGamepadState(gamepad, meta = {}) {
            if (!gamepad) return;
            const { template, color, triggersMeter, zoom } = meta;
            this.send({
                type: "gamepad_state",
                index: gamepad.index,
                id: gamepad.id,
                mapping: gamepad.mapping,
                timestamp: gamepad.timestamp,
                template,
                color,
                triggersMeter,
                zoom,
                buttons: gamepad.buttons.map((btn, i) => ({
                    index: i,
                    pressed: btn.pressed,
                    value: btn.value,
                })),
                axes: gamepad.axes.map((value, i) => ({
                    index: i,
                    value,
                })),
            });
        }
    };
}
