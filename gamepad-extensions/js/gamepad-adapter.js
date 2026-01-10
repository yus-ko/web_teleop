// gamepad-extensions adapter
// gamepad-viewer本体は変更せず、window.gamepad(インスタンス)にWebSocket送信を追加します

(function () {
    if (window.__gamepadAdapterLoaded) return;
    window.__gamepadAdapterLoaded = true;

    function getUrlParamFallback(name) {
        try {
            const params = new URLSearchParams(window.location.search);
            return params.get(name);
        } catch (_) {
            return null;
        }
    }

    function initForInstance(gamepadInstance) {
        if (!gamepadInstance || gamepadInstance.__wsAdapterApplied) return;
        gamepadInstance.__wsAdapterApplied = true;

        try {
            console.info("[gamepad-adapter] applied", {
                hasGamepadWebSocket: !!window.GamepadWebSocket,
            });
        } catch (_) {
            // ignore
        }

        const ensureWsClient = () => {
            if (gamepadInstance.wsClient !== undefined) return;

            gamepadInstance.wsClient = null;
            const getParam = typeof gamepadInstance.getUrlParam === "function"
                ? (k) => gamepadInstance.getUrlParam(k)
                : (k) => getUrlParamFallback(k);

            gamepadInstance.wsUrl =
                getParam("ws") ||
                getParam("wsUrl") ||
                `ws://${window.location.hostname}:8080`;

            if (!gamepadInstance.wsUrl || !window.GamepadWebSocket) return;

            try {
                console.info("[gamepad-adapter] ws target", gamepadInstance.wsUrl);
            } catch (_) {
                // ignore
            }

            gamepadInstance.wsClient = new window.GamepadWebSocket({
                url: gamepadInstance.wsUrl,
                reconnectDelay: 2000,
                heartbeatInterval: 30000,
                debug: !!gamepadInstance.debug,
            });
            gamepadInstance.wsClient.connect({
                type: "handshake",
                userAgent: navigator.userAgent,
                haveGamepadEvents: !!gamepadInstance.haveEvents,
            });
        };

        const originalPollStatus = gamepadInstance.pollStatus;
        if (typeof originalPollStatus === "function") {
            gamepadInstance.pollStatus = function (force = false) {
                // 元の処理を先に実行（タイムスタンプ更新・描画更新など）
                const result = originalPollStatus.call(this, force);

                try {
                    ensureWsClient();
                    if (!this.wsClient) return result;
                    if (typeof this.getActive !== "function") return result;

                    const active = this.getActive();
                    if (!active) return result;

                    // Prefer timestamp-based suppression when available, but fall back
                    // to a signature of buttons/axes so we still send when timestamp is static.
                    const ts = this.lastTimestamp;
                    const sig = computeStateSignature(active);
                    if (
                        (ts && ts === this.__wsLastSentTimestamp) &&
                        sig === this.__wsLastSentSignature
                    ) {
                        return result;
                    }
                    if (ts) this.__wsLastSentTimestamp = ts;
                    this.__wsLastSentSignature = sig;

                    this.wsClient.sendGamepadState(active, {
                        template: this.type,
                        color: this.colorName,
                        triggersMeter: this.triggersMeter,
                        zoom: this.zoomLevel,
                    });

                    if (!this.__wsSentOnce) {
                        this.__wsSentOnce = true;
                        try {
                            console.info("[gamepad-adapter] sent gamepad_state", {
                                index: active.index,
                                id: active.id,
                                buttons: Array.isArray(active.buttons)
                                    ? active.buttons.length
                                    : undefined,
                                axes: Array.isArray(active.axes)
                                    ? active.axes.length
                                    : undefined,
                            });
                        } catch (_) {
                            // ignore
                        }
                    }
                } catch (e) {
                    // 送信失敗は描画を止めない
                    if (this.debug) {
                        console.debug("WebSocket send suppressed error", e);
                    }
                }

                return result;
            };
        }
    }

    function computeStateSignature(active) {
        try {
            const buttons = Array.isArray(active.buttons) ? active.buttons : [];
            const axes = Array.isArray(active.axes) ? active.axes : [];
            const b = buttons
                .map((btn) => `${btn.pressed ? 1 : 0}:${Math.round((btn.value || 0) * 1000)}`)
                .join("|");
            const a = axes.map((v) => Math.round((v || 0) * 1000)).join(",");
            return `${b}#${a}`;
        } catch (_) {
            return "";
        }
    }

    // gamepad.jsはwindow.gamepad(インスタンス)を作るので、それを待ってから適用
    const tryInit = () => {
        if (window.gamepad) {
            initForInstance(window.gamepad);
            return true;
        }
        return false;
    };

    if (!tryInit()) {
        const timer = window.setInterval(() => {
            if (tryInit()) {
                window.clearInterval(timer);
            }
        }, 50);
        window.setTimeout(() => {
            try {
                window.clearInterval(timer);
            } catch (_) {
                // ignore
            }
        }, 5000);
    }
})();
