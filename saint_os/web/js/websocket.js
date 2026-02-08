/**
 * SAINT.OS WebSocket Client
 *
 * Handles WebSocket connection to the SAINT.OS server with authentication support.
 */

class SaintWebSocket {
    constructor() {
        this.ws = null;
        this.connected = false;
        this.authenticated = false;
        this.authRequired = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 10;
        this.reconnectDelay = 1000;
        this.messageHandlers = new Map();
        this.pendingRequests = new Map();
        this.requestId = 0;
        this.serverName = null;
        this.clientId = null;

        // Track if we've had a successful session (for auto-reload on reconnect)
        this.hadSession = false;
        this.readyEmitted = false;  // Track if 'ready' was emitted for current connection
        this.autoReloadOnReconnect = true;

        // Load settings from localStorage
        this.settings = this.loadSettings();
    }

    /**
     * Load connection settings from localStorage.
     */
    loadSettings() {
        const defaults = {
            host: window.location.host,
            password: '',
            autoConnect: true
        };

        try {
            const saved = localStorage.getItem('saint_ws_settings');
            if (saved) {
                return { ...defaults, ...JSON.parse(saved) };
            }
        } catch (e) {
            console.warn('Failed to load settings:', e);
        }

        return defaults;
    }

    /**
     * Save connection settings to localStorage.
     */
    saveSettings(settings) {
        this.settings = { ...this.settings, ...settings };
        try {
            localStorage.setItem('saint_ws_settings', JSON.stringify(this.settings));
        } catch (e) {
            console.warn('Failed to save settings:', e);
        }
    }

    /**
     * Get current settings.
     */
    getSettings() {
        return { ...this.settings };
    }

    /**
     * Connect to the WebSocket server.
     * @param {string} [host] - Optional host override (e.g., "localhost:8080")
     */
    connect(host = null) {
        const targetHost = host || this.settings.host || window.location.host;
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const url = `${protocol}//${targetHost}/api/ws`;

        console.log(`Connecting to ${url}...`);

        // Reset state
        this.authenticated = false;
        this.authRequired = false;

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
     * Disconnect from the server.
     */
    disconnect() {
        this.reconnectAttempts = this.maxReconnectAttempts; // Prevent auto-reconnect
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
        this.connected = false;
        this.authenticated = false;
    }

    /**
     * Authenticate with the server.
     * @param {string} password - The password to authenticate with
     */
    authenticate(password = null) {
        const pwd = password !== null ? password : this.settings.password;

        if (!this.connected) {
            return Promise.reject(new Error('Not connected'));
        }

        return new Promise((resolve, reject) => {
            // Set up one-time handler for auth result
            const authHandler = (message) => {
                this.off('auth_result', authHandler);

                if (message.status === 'ok') {
                    this.authenticated = true;
                    // Save password on successful auth
                    if (password !== null) {
                        this.saveSettings({ password });
                    }
                    resolve(message);
                } else {
                    reject(new Error(message.message || 'Authentication failed'));
                }
            };

            this.on('auth_result', authHandler);

            // Send auth message (no id needed for auth)
            this.ws.send(JSON.stringify({
                type: 'auth',
                action: 'login',
                password: pwd
            }));

            // Timeout after 10 seconds
            setTimeout(() => {
                this.off('auth_result', authHandler);
                reject(new Error('Authentication timeout'));
            }, 10000);
        });
    }

    /**
     * Check if currently authenticated (or auth not required).
     */
    isAuthenticated() {
        return this.authenticated || !this.authRequired;
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

        if (!this.isAuthenticated()) {
            console.warn('Not authenticated, cannot send message');
            return Promise.reject(new Error('Authentication required'));
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
        this.readyEmitted = false;  // Reset for new connection
        // Don't emit 'connected' yet - wait for server's connected message
    }

    _onClose(event) {
        console.log(`WebSocket closed: ${event.code} ${event.reason}`);
        this.connected = false;
        this.authenticated = false;
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

            // Handle server's initial connected message
            if (message.type === 'connected') {
                this.serverName = message.server_name;
                this.clientId = message.client_id;
                this.authRequired = message.auth_required || false;

                console.log(`Connected to ${this.serverName} as ${this.clientId}, auth_required: ${this.authRequired}`);

                // If auth not required, we're authenticated
                if (!this.authRequired) {
                    this.authenticated = true;
                    this._emit('connected', message);
                    this._emit('ready');
                } else {
                    // Try auto-auth with saved password
                    this._emit('connected', message);
                    this._emit('auth_required', message);

                    if (this.settings.password) {
                        this.authenticate().then(() => {
                            console.log('Auto-authenticated with saved password');
                            this._emit('ready');
                        }).catch((err) => {
                            console.warn('Auto-auth failed:', err.message);
                            this._emit('auth_failed', { message: err.message });
                        });
                    }
                }
                return;
            }

            // Handle auth result
            if (message.type === 'auth_result') {
                this._emit('auth_result', message);
                if (message.status === 'ok') {
                    this._emit('ready');
                }
                return;
            }

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
        // Handle auto-reload on reconnect (only process 'ready' once per connection)
        if (type === 'ready') {
            if (this.readyEmitted) {
                // Already emitted 'ready' for this connection, ignore duplicate
                return;
            }
            this.readyEmitted = true;

            if (this.hadSession && this.autoReloadOnReconnect) {
                console.log('Reconnected after session - reloading page for fresh code...');
                // Save current state for restore after reload
                this._saveStateForReload();
                // Small delay to ensure connection is stable
                setTimeout(() => {
                    window.location.reload();
                }, 100);
                return;
            }
            // Mark that we've had a successful session
            this.hadSession = true;
        }

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

    /**
     * Save current state before reload so we can restore it.
     */
    _saveStateForReload() {
        const state = {
            autoReconnect: true,
            page: window.location.hash || '#dashboard',
            timestamp: Date.now()
        };
        try {
            sessionStorage.setItem('saint_reload_state', JSON.stringify(state));
        } catch (e) {
            console.warn('Failed to save reload state:', e);
        }
    }

    /**
     * Check if we should auto-reconnect after a reload.
     * Returns the saved state if valid, null otherwise.
     */
    getReloadState() {
        try {
            const saved = sessionStorage.getItem('saint_reload_state');
            if (saved) {
                sessionStorage.removeItem('saint_reload_state');
                const state = JSON.parse(saved);
                // Only valid if saved within the last 30 seconds
                if (Date.now() - state.timestamp < 30000) {
                    return state;
                }
            }
        } catch (e) {
            console.warn('Failed to load reload state:', e);
        }
        return null;
    }
}

// Global instance
window.saintWS = new SaintWebSocket();
