/**
 * SAINT.OS WebSocket Client
 *
 * Handles WebSocket connection to the SAINT.OS server.
 */

class SaintWebSocket {
    constructor() {
        this.ws = null;
        this.connected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 10;
        this.reconnectDelay = 1000;
        this.messageHandlers = new Map();
        this.pendingRequests = new Map();
        this.requestId = 0;
    }

    /**
     * Connect to the WebSocket server.
     */
    connect() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host;
        const url = `${protocol}//${host}/api/ws`;

        console.log(`Connecting to ${url}...`);

        try {
            this.ws = new WebSocket(url);

            this.ws.onopen = () => this._onOpen();
            this.ws.onclose = (event) => this._onClose(event);
            this.ws.onerror = (error) => this._onError(error);
            this.ws.onmessage = (event) => this._onMessage(event);
        } catch (error) {
            console.error('Failed to create WebSocket:', error);
            this._scheduleReconnect();
        }
    }

    /**
     * Send a message to the server.
     * Can be called as send(type, action, params) or send({type, action, params})
     */
    send(typeOrMessage, action, params = {}) {
        if (!this.connected) {
            console.warn('Not connected, cannot send message');
            return Promise.reject(new Error('Not connected'));
        }

        // Support both calling conventions
        let type, messageParams;
        if (typeof typeOrMessage === 'object') {
            // Called as send({type, action, params})
            type = typeOrMessage.type;
            action = typeOrMessage.action;
            messageParams = typeOrMessage.params || {};
        } else {
            // Called as send(type, action, params)
            type = typeOrMessage;
            messageParams = params;
        }

        const id = `req_${++this.requestId}`;
        const message = {
            id,
            type,
            action,
            params: messageParams
        };

        return new Promise((resolve, reject) => {
            this.pendingRequests.set(id, { resolve, reject });

            // Timeout after 30 seconds
            setTimeout(() => {
                if (this.pendingRequests.has(id)) {
                    this.pendingRequests.delete(id);
                    reject(new Error('Request timeout'));
                }
            }, 30000);

            this.ws.send(JSON.stringify(message));
        });
    }

    /**
     * Send a command to the server.
     */
    command(target, action, params = {}) {
        return this.send('command', action, { target, ...params });
    }

    /**
     * Send a management command.
     */
    management(action, params = {}) {
        return this.send('management', action, params);
    }

    /**
     * Send a router command.
     */
    router(action, params = {}) {
        return this.send('router', action, params);
    }

    /**
     * Subscribe to topics.
     */
    subscribe(topics, rateHz = 10) {
        return this.send('subscribe', 'subscribe', { topics, rate_hz: rateHz });
    }

    /**
     * Unsubscribe from topics.
     */
    unsubscribe(topics) {
        return this.send('subscribe', 'unsubscribe', { topics });
    }

    /**
     * Register a message handler.
     */
    on(type, handler) {
        if (!this.messageHandlers.has(type)) {
            this.messageHandlers.set(type, []);
        }
        this.messageHandlers.get(type).push(handler);
    }

    /**
     * Remove a message handler.
     */
    off(type, handler) {
        if (this.messageHandlers.has(type)) {
            const handlers = this.messageHandlers.get(type);
            const index = handlers.indexOf(handler);
            if (index !== -1) {
                handlers.splice(index, 1);
            }
        }
    }

    // Private methods

    _onOpen() {
        console.log('WebSocket connected');
        this.connected = true;
        this.reconnectAttempts = 0;
        this._emit('connected');
    }

    _onClose(event) {
        console.log(`WebSocket closed: ${event.code} ${event.reason}`);
        this.connected = false;
        this._emit('disconnected', { code: event.code, reason: event.reason });
        this._scheduleReconnect();
    }

    _onError(error) {
        console.error('WebSocket error:', error);
        this._emit('error', error);
    }

    _onMessage(event) {
        try {
            const message = JSON.parse(event.data);

            // Handle response to pending request
            if (message.id && this.pendingRequests.has(message.id)) {
                const { resolve, reject } = this.pendingRequests.get(message.id);
                this.pendingRequests.delete(message.id);

                if (message.status === 'ok') {
                    resolve(message.data);
                } else {
                    reject(new Error(message.message || 'Request failed'));
                }
                return;
            }

            // Handle other message types
            this._emit(message.type, message);

        } catch (error) {
            console.error('Failed to parse message:', error);
        }
    }

    _emit(type, data = null) {
        if (this.messageHandlers.has(type)) {
            for (const handler of this.messageHandlers.get(type)) {
                try {
                    handler(data);
                } catch (error) {
                    console.error(`Handler error for ${type}:`, error);
                }
            }
        }
    }

    _scheduleReconnect() {
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.error('Max reconnect attempts reached');
            this._emit('reconnect_failed');
            return;
        }

        const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts);
        this.reconnectAttempts++;

        console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`);

        setTimeout(() => {
            this.connect();
        }, delay);
    }
}

// Global instance
window.saintWS = new SaintWebSocket();
