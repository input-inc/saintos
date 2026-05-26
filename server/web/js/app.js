/**
 * SAINT.OS Administration Interface
 *
 * Main application logic for the web admin interface.
 */

class SaintApp {
    constructor() {
        this.currentPage = 'dashboard';
        this.systemStatus = null;
        this.nodes = { adopted: [], unadopted: [] };
        this.connectedClients = [];
        this.activityLog = [];
        this.currentNodeId = null;  // Currently viewed node
        this.currentNodeInfo = null;

        // Logs page state.
        //   logsSelectedSource: 'server' (default) or `node:<id>`
        //   logsPerNode:        per-node ring buffers populated when
        //                       the operator selects a node on the
        //                       Logs page. We subscribe to
        //                       node_log/<id> on selection.
        //   logsExpandedKey:    which row is showing its raw-JSON
        //                       detail (only one open at a time).
        //   logsNodeSubscription: the topic we're currently
        //                       subscribed to (so we can unsubscribe
        //                       when switching sources).
        this.logsSelectedSource = 'server';
        this.logsPerNode = new Map();
        this.logsExpandedKey = null;
        this.logsNodeSubscription = null;

        // Refresh interval for node detail page
        this.nodeDetailRefreshInterval = null;
        this.nodeDetailRefreshRate = 2000;  // Refresh every 2 seconds

        // Load display preferences
        this.temperatureUnit = localStorage.getItem('saint_temp_unit') || 'celsius';

        // Settings dirty tracking
        this.settingsOriginalValues = {};
        this.settingsDirty = false;
    }

    /**
     * Format temperature based on user preference.
     * @param {number} celsius - Temperature in Celsius
     * @returns {string} Formatted temperature string
     */
    formatTemperature(celsius) {
        if (celsius === null || celsius === undefined || isNaN(celsius)) {
            return '--';
        }

        if (this.temperatureUnit === 'fahrenheit') {
            const fahrenheit = (celsius * 9 / 5) + 32;
            return `${fahrenheit.toFixed(1)}°F`;
        }
        return `${celsius.toFixed(1)}°C`;
    }

    /**
     * Set temperature unit preference.
     * @param {string} unit - 'celsius' or 'fahrenheit'
     */
    setTemperatureUnit(unit) {
        this.temperatureUnit = unit;
        localStorage.setItem('saint_temp_unit', unit);

        // Update any displayed temperatures
        if (this.currentNodeInfo) {
            this.updateNodeOverview();
        }
        // Force a re-render so the change is visible immediately on
        // the Live Readings tab and dashboard widget cards. Both
        // re-render on a debounce/timer anyway, so this is just a UX
        // smoothing — no need to chase every numeric span manually.
        if (window.nodeLiveManager) {
            try { window.nodeLiveManager.renderGrid(); } catch (_) {}
        }
        if (window.widgetsDashboard) {
            try {
                window.widgetsDashboard.render();
                window.widgetsDashboard.renderValues();
            } catch (_) {}
        }
    }

    /**
     * Active temperature unit symbol — '°C' or '°F' — based on the
     * user's preference. Used by other JS modules that render
     * temperature values themselves rather than going through
     * formatTemperature().
     */
    temperatureUnitSymbol() {
        return this.temperatureUnit === 'fahrenheit' ? '°F' : '°C';
    }

    /**
     * Convert a Celsius temperature to the user's preferred unit and
     * return JUST THE NUMBER (no unit symbol). Use when the unit
     * symbol is rendered in a separate element from the value (e.g.
     * the FAS100 widget card lays them out as two spans).
     *
     * Returns '--' for null/undefined/NaN to match formatTemperature().
     */
    formatTemperatureValue(celsius) {
        if (celsius === null || celsius === undefined || isNaN(celsius)) {
            return '--';
        }
        const t = this.temperatureUnit === 'fahrenheit'
            ? (celsius * 9 / 5) + 32
            : celsius;
        return t.toFixed(1);
    }

    /**
     * Detect whether a peripheral channel descriptor represents a
     * temperature reading. Used by render code in widgets.js /
     * nodelive.js to decide between the generic numeric formatter
     * and the temperature-aware one.
     *
     * Pattern-matches the channel ids actually emitted across our
     * peripheral types (PeripheralChannel definitions in
     * peripheral_model.py): temp, temp1, temp2, temp_1, temp_2,
     * cpu_temp. Falls back to checking the display string for a
     * literal °C/°F or "temp" word so a future peripheral that
     * names its temperature channel something unexpected still
     * gets the right treatment.
     */
    isTemperatureChannel(ch) {
        if (!ch) return false;
        const id = (typeof ch === 'string' ? ch : ch.id) || '';
        if (id === 'temp' || /^temp[_0-9]/.test(id) || /(^|_)temp$/.test(id)
            || id.includes('cpu_temp')) {
            return true;
        }
        const disp = (ch && ch.display) || '';
        return /°[CF]|temperat/i.test(disp);
    }

    /**
     * Initialize the application.
     */
    init() {
        this.setupLoginForm();
        this.setupNavigation();
        this.setupWebSocket();
        this.setupEventListeners();

        // Load saved settings into login form
        const settings = window.saintWS.getSettings();
        document.getElementById('login-host').value = settings.host || '';
        document.getElementById('login-password').value = settings.password || '';

        // Check if we're reloading after a server restart - auto-reconnect
        const reloadState = window.saintWS.getReloadState();
        if (reloadState && reloadState.autoReconnect && settings.host) {
            console.log('Auto-reconnecting after reload, restoring page:', reloadState.page);
            this.pendingPage = reloadState.page;  // Save page to restore after connect

            // Show reconnecting status on login screen
            document.getElementById('login-status').innerHTML =
                '<span class="inline-block w-4 h-4 border-2 border-cyan-500 border-t-transparent rounded-full animate-spin mr-2"></span>' +
                'Reconnecting after server restart...';
            document.getElementById('login-status').classList.remove('hidden');
            document.getElementById('login-btn').disabled = true;

            window.saintWS.connect(settings.host);
        }
    }

    /**
     * Setup login form handlers.
     */
    setupLoginForm() {
        const form = document.getElementById('login-form');
        form.addEventListener('submit', (e) => {
            e.preventDefault();
            this.handleLoginSubmit();
        });

        // Settings button in header
        document.getElementById('btn-connection-settings')?.addEventListener('click', () => {
            this.showLoginScreen();
        });
    }

    /**
     * Handle login form submission.
     */
    async handleLoginSubmit() {
        const hostInput = document.getElementById('login-host');
        const passwordInput = document.getElementById('login-password');
        const errorEl = document.getElementById('login-error');
        const statusEl = document.getElementById('login-status');
        const loginBtn = document.getElementById('login-btn');

        const host = hostInput.value.trim() || window.location.host;
        const password = passwordInput.value;

        // Show loading state
        errorEl.classList.add('hidden');
        statusEl.classList.remove('hidden');
        loginBtn.disabled = true;

        // Save settings
        window.saintWS.saveSettings({ host, password });

        // Disconnect if already connected
        window.saintWS.disconnect();
        window.saintWS.reconnectAttempts = 0;

        // Connect to server
        window.saintWS.connect(host);
    }

    /**
     * Show login screen, hide main app.
     */
    showLoginScreen() {
        document.getElementById('login-screen').classList.remove('hidden');
        document.getElementById('app').classList.add('hidden');

        // Reset login form state
        document.getElementById('login-error').classList.add('hidden');
        document.getElementById('login-status').classList.add('hidden');
        document.getElementById('login-btn').disabled = false;

        // Load current settings
        const settings = window.saintWS.getSettings();
        document.getElementById('login-host').value = settings.host || '';
        document.getElementById('login-password').value = settings.password || '';
    }

    /**
     * Show main app, hide login screen.
     */
    showMainApp() {
        document.getElementById('login-screen').classList.add('hidden');
        document.getElementById('app').classList.remove('hidden');

        // Use pending page from reload state, or fall back to URL hash
        let hash = window.location.hash.slice(1) || 'dashboard';
        if (this.pendingPage) {
            hash = this.pendingPage.replace('#', '');
            window.location.hash = this.pendingPage;  // Update URL to match
            this.pendingPage = null;
        }

        // Check for special URL hashes
        if (hash.startsWith('node/')) {
            const nodeId = hash.slice(5);
            this.currentNodeId = nodeId;
            this.showPage('node-detail');
        } else if (hash === 'livelink') {
            this.showPage('livelink-detail');
        } else {
            this.showPage(hash);
        }
    }

    /**
     * Setup navigation click handlers.
     */
    setupNavigation() {
        document.querySelectorAll('.nav-link').forEach(link => {
            link.addEventListener('click', (e) => {
                e.preventDefault();
                const page = link.dataset.page;
                this.showPage(page);
                window.location.hash = page;
            });
        });
    }

    /**
     * Setup WebSocket connection and handlers.
     */
    setupWebSocket() {
        const ws = window.saintWS;

        ws.on('connected', (message) => {
            this.updateConnectionStatus('connected', message.auth_required);
            // Update login status
            document.getElementById('login-status').innerHTML =
                '<span class="inline-block w-4 h-4 border-2 border-cyan-500 border-t-transparent rounded-full animate-spin mr-2"></span>' +
                (message.auth_required ? 'Authenticating...' : 'Connected!');
        });

        ws.on('ready', () => {
            // Authenticated (or auth not required) - ready to use
            this.updateConnectionStatus('ready');
            this.showMainApp();
            this.requestInitialData();
            if (window.updateManager) {
                window.updateManager.init();
            }
        });

        ws.on('auth_required', () => {
            // If no saved password, show error in login form
            if (!ws.settings.password) {
                this.showLoginError('Password required');
            }
        });

        ws.on('auth_failed', (data) => {
            this.updateConnectionStatus('auth_failed');
            this.showLoginError(data?.message || 'Authentication failed');
        });

        ws.on('disconnected', () => {
            this.updateConnectionStatus('disconnected');
        });

        ws.on('error', () => {
            this.showLoginError('Connection failed. Check server address.');
        });

        ws.on('reconnect_failed', () => {
            this.showLoginError('Could not connect to server');
        });

        ws.on('state', (message) => {
            this.handleStateUpdate(message);
        });

        ws.on('activity', (message) => {
            this.addActivityLogEntry(message);
        });

        // Don't auto-connect - wait for login form submission
        // ws.connect();
    }

    /**
     * Show error in login form.
     */
    showLoginError(message) {
        const errorEl = document.getElementById('login-error');
        const statusEl = document.getElementById('login-status');
        const loginBtn = document.getElementById('login-btn');

        errorEl.textContent = message;
        errorEl.classList.remove('hidden');
        statusEl.classList.add('hidden');
        loginBtn.disabled = false;
    }


    /**
     * Setup button click handlers.
     */
    setupEventListeners() {
        // Dashboard buttons
        document.getElementById('btn-scan-nodes')?.addEventListener('click', () => {
            this.scanForNodes();
        });

        document.getElementById('btn-estop')?.addEventListener('click', () => {
            this.emergencyStop();
        });

        // Refresh WebSocket clients
        document.getElementById('btn-refresh-clients')?.addEventListener('click', () => {
            this.loadWebSocketClients();
        });

        // Logs page filters and global-level selector. The two
        // dropdowns intentionally do different things:
        //   logs-level-select → global server log level (apply_log_level
        //     via set_settings; persists across restart)
        //   logs-row-filter   → client-side row filter on currently
        //     buffered events (no server impact)
        document.getElementById('logs-row-filter')?.addEventListener('change', () => {
            this.renderLogsPage();
        });
        document.getElementById('logs-search')?.addEventListener('input', () => {
            this.renderLogsPage();
        });
        document.getElementById('logs-level-select')?.addEventListener('change', (e) => {
            this.applyServerLogLevel(e.target.value);
        });
        document.getElementById('logs-clear-btn')?.addEventListener('click', () => {
            this.clearLogsForCurrentSource();
        });

        // Node detail tab switching
        document.querySelectorAll('.node-tab').forEach(tab => {
            tab.addEventListener('click', (e) => {
                this.switchNodeTab(e.target.dataset.tab);
            });
        });

        // Settings sub-tab switching
        document.querySelectorAll('.settings-tab').forEach(tab => {
            tab.addEventListener('click', (e) => {
                const tabName = e.currentTarget.dataset.settingsTab;
                this.switchSettingsTab(tabName);
            });
        });

        // Node control buttons
        document.getElementById('btn-node-restart')?.addEventListener('click', () => {
            this.restartNode();
        });
        document.getElementById('btn-node-identify')?.addEventListener('click', () => {
            this.identifyNode();
        });
        document.getElementById('btn-node-estop')?.addEventListener('click', () => {
            this.estopNode();
        });
        document.getElementById('btn-node-factory-reset')?.addEventListener('click', () => {
            this.factoryResetNode();
        });
        document.getElementById('btn-node-unadopt')?.addEventListener('click', () => {
            this.unadoptNode();
        });
        document.getElementById('btn-force-firmware-update')?.addEventListener('click', () => {
            this.showFirmwareModal();
        });
        document.getElementById('btn-fw-simulation')?.addEventListener('click', () => {
            this.forceFirmwareUpdate('simulation');
        });
        document.getElementById('btn-fw-hardware')?.addEventListener('click', () => {
            this.forceFirmwareUpdate('hardware');
        });

        // Firmware update button
        document.getElementById('btn-firmware-update')?.addEventListener('click', () => {
            this.updateFirmware();
        });

        // Firmware update badge (in overview tab) - click to go to Control tab
        document.getElementById('node-firmware-update-badge')?.addEventListener('click', () => {
            this.switchNodeTab('control');
        });

        // Update All button on Nodes page
        document.getElementById('btn-update-all-nodes')?.addEventListener('click', () => {
            this.updateAllNodes();
        });
    }

    /**
     * Show a page by ID.
     */
    showPage(pageId) {
        console.log(`showPage called: ${pageId}`);

        // Cleanup when leaving certain pages
        if (this.currentPage === 'livelink-detail' && pageId !== 'livelink-detail') {
            // Unsubscribe from blend shapes when leaving LiveLink detail
            if (window.liveLinkManager) {
                window.liveLinkManager.unsubscribe();
            }
        }

        // Stop node detail refresh when leaving that page
        if (this.currentPage === 'node-detail' && pageId !== 'node-detail') {
            this.stopNodeDetailRefresh();
        }

        // Stop widget dashboard subscriptions when leaving the dashboard
        if (this.currentPage === 'dashboard' && pageId !== 'dashboard') {
            if (window.widgetsDashboard) window.widgetsDashboard.deactivate();
        }

        // Close the terminal session when leaving the Terminal page so
        // we don't keep an idle shell running on the server.
        if (this.currentPage === 'terminal' && pageId !== 'terminal') {
            if (window.terminalManager) {
                window.terminalManager.closeSession();
            }
        }

        // Drop the node_log subscription when leaving the Logs page —
        // otherwise we'd keep streaming entries into the per-node
        // ring buffer for a source the operator isn't watching.
        if (this.currentPage === 'logs' && pageId !== 'logs'
            && this.logsNodeSubscription) {
            const sub = this.logsNodeSubscription;
            this.logsNodeSubscription = null;
            if (sub.startsWith('node:')) {
                const nodeId = sub.slice('node:'.length);
                try {
                    window.saintWS.unsubscribe([`node_log/${nodeId}`]);
                } catch (_) { /* ignore */ }
            }
        }

        // Update navigation
        document.querySelectorAll('.nav-link').forEach(link => {
            if (link.dataset.page === pageId) {
                link.classList.add('active');
            } else {
                link.classList.remove('active');
            }
        });

        // Show page
        document.querySelectorAll('.page').forEach(page => {
            if (page.id === `page-${pageId}`) {
                page.classList.add('active');
            } else {
                page.classList.remove('active');
            }
        });

        // Pages that want the full viewport under the nav (routes and
        // logs, currently — both are master-detail shells) break out of
        // <main>'s max-w / padding via body-level classes the CSS keys
        // off of. Kept as separate classes so the styling rules can
        // diverge if one ever needs nav-bar-relative height tweaks.
        document.body.classList.toggle('routing-fullwidth', pageId === 'routes');
        document.body.classList.toggle('logs-fullwidth', pageId === 'logs');

        this.currentPage = pageId;

        // Load page-specific data
        this.loadPageData(pageId);
    }

    /**
     * Load data for a specific page.
     */
    async loadPageData(pageId) {
        console.log(`loadPageData called for: ${pageId}`);
        const ws = window.saintWS;
        if (!ws || !ws.connected) {
            console.warn(`loadPageData: WebSocket not connected, skipping ${pageId}`);
            return;
        }

        try {
            console.log(`loadPageData: switching on pageId="${pageId}"`);
            switch (pageId) {
                case 'dashboard':
                    await this.loadDashboardData();
                    break;
                case 'nodes':
                    await this.loadNodesData();
                    break;
                case 'node-detail':
                    await this.loadNodeDetailData();
                    break;
                case 'routes':
                    await this.loadRoutesData();
                    break;
                case 'inputs':
                    await this.loadInputsData();
                    break;
                case 'livelink-detail':
                    await this.loadLiveLinkDetailData();
                    break;
                case 'logs':
                    await this.loadLogsData();
                    break;
                case 'control':
                    console.log('loadPageData: matched control case');
                    await this.loadControlPageData();
                    break;
                case 'settings':
                    await this.loadSettingsPageData();
                    break;
                case 'terminal':
                    if (window.terminalManager) {
                        await window.terminalManager.openSession();
                    }
                    break;
                default:
                    console.log(`loadPageData: no match for "${pageId}"`);
            }
        } catch (error) {
            console.error(`Failed to load ${pageId} data:`, error);
        }
    }

    /**
     * Load control page data.
     */
    async loadControlPageData() {
        console.log('loadControlPageData called, window.controlPage defined:', typeof window.controlPage !== 'undefined');
        if (typeof window.controlPage !== 'undefined') {
            console.log('Calling controlPage.load()...');
            await window.controlPage.load();
            console.log('controlPage.load() completed');
        } else {
            console.error('controlPage is undefined! Check if controlpage.js loaded correctly.');
        }
    }

    /**
     * Load settings page data.
     */
    async loadSettingsPageData() {
        const ws = window.saintWS;

        // Load firmware build info
        try {
            const result = await ws.management('get_firmware_builds', {});

            // Update simulation build info (RP2040)
            this.updateFirmwareBuildDisplay('sim', result.simulation);

            // Update hardware build info (RP2040)
            this.updateFirmwareBuildDisplay('hw', result.hardware);

            // Update Pi 5 firmware info
            this.updatePi5FirmwareDisplay(result.rpi5);

            // Update Steam Deck controller AppImage info
            this.updateControllerFirmwareDisplay(result.controller);
        } catch (error) {
            console.error('Failed to load firmware builds:', error);
        }

        // Load server settings
        try {
            const result = await ws.management('get_settings', {});
            this.populateSettingsForm(result);
        } catch (error) {
            console.error('Failed to load settings:', error);
        }

        // Load WiFi AP credentials + current channel into the Wireless card.
        try {
            await this.loadWifiConfig();
        } catch (error) {
            console.error('Failed to load WiFi config:', error);
        }

        // Wire the Wireless card buttons (idempotent — guarded by data-bound).
        this.setupWirelessCardHandlers();

        // Load local display preferences
        this.loadDisplayPreferences();

        // Set up save button handler
        this.setupSettingsSaveHandler();
    }

    /**
     * Load and set display preferences from localStorage.
     */
    loadDisplayPreferences() {
        // Temperature unit
        const celsiusRadio = document.getElementById('temp-unit-celsius');
        const fahrenheitRadio = document.getElementById('temp-unit-fahrenheit');

        if (celsiusRadio && fahrenheitRadio) {
            if (this.temperatureUnit === 'fahrenheit') {
                fahrenheitRadio.checked = true;
            } else {
                celsiusRadio.checked = true;
            }

            // Add event listeners (only once)
            if (!celsiusRadio.hasAttribute('data-listener-set')) {
                celsiusRadio.setAttribute('data-listener-set', 'true');
                celsiusRadio.addEventListener('change', () => {
                    if (celsiusRadio.checked) {
                        this.setTemperatureUnit('celsius');
                    }
                });
            }

            if (!fahrenheitRadio.hasAttribute('data-listener-set')) {
                fahrenheitRadio.setAttribute('data-listener-set', 'true');
                fahrenheitRadio.addEventListener('change', () => {
                    if (fahrenheitRadio.checked) {
                        this.setTemperatureUnit('fahrenheit');
                    }
                });
            }
        }
    }

    /**
     * Populate settings form with current values.
     */
    populateSettingsForm(settings) {
        // Server settings
        const serverName = document.getElementById('settings-server-name');
        if (serverName && settings.server) {
            serverName.value = settings.server.name || '';
        }

        // WebSocket settings
        const wsPassword = document.getElementById('settings-ws-password');
        const wsAuthTimeout = document.getElementById('settings-ws-auth-timeout');
        if (settings.websocket) {
            if (wsPassword) wsPassword.value = settings.websocket.password || '';
            if (wsAuthTimeout) wsAuthTimeout.value = settings.websocket.auth_timeout || 10;
        }

        // Network settings
        const webPort = document.getElementById('settings-web-port');
        const wsPort = document.getElementById('settings-ws-port');
        if (settings.network) {
            if (webPort) webPort.value = settings.network.web_port || 80;
            if (wsPort) wsPort.value = settings.network.websocket_port || '';
        }

        // ROS Bridge settings
        const rosThrottle = document.getElementById('settings-ros-throttle');
        if (rosThrottle && settings.ros_bridge) {
            rosThrottle.value = settings.ros_bridge.throttle_ms || 50;
        }

        // LiveLink settings
        const llEnabled = document.getElementById('settings-livelink-enabled');
        const llPort = document.getElementById('settings-livelink-port');
        if (settings.livelink) {
            if (llEnabled) llEnabled.checked = settings.livelink.enabled !== false;
            if (llPort) llPort.value = settings.livelink.port || 11111;
        }

        // Store original values for dirty tracking
        this.settingsOriginalValues = this.getSettingsFormValues();
        this.settingsDirty = false;
        this.updateSettingsSaveButton();

        // Set up dirty tracking listeners
        this.setupSettingsDirtyTracking();
    }

    /**
     * Get current values from settings form.
     */
    getSettingsFormValues() {
        // Get temperature unit from radio buttons
        const tempUnitCelsius = document.getElementById('temp-unit-celsius');
        const tempUnit = tempUnitCelsius?.checked ? 'celsius' : 'fahrenheit';

        return {
            serverName: document.getElementById('settings-server-name')?.value || '',
            wsPassword: document.getElementById('settings-ws-password')?.value || '',
            wsAuthTimeout: document.getElementById('settings-ws-auth-timeout')?.value || '10',
            webPort: document.getElementById('settings-web-port')?.value || '80',
            wsPort: document.getElementById('settings-ws-port')?.value || '',
            rosThrottle: document.getElementById('settings-ros-throttle')?.value || '50',
            llEnabled: document.getElementById('settings-livelink-enabled')?.checked ?? true,
            llPort: document.getElementById('settings-livelink-port')?.value || '11111',
            tempUnit: tempUnit,
        };
    }

    /**
     * Set up dirty tracking for settings form inputs.
     */
    setupSettingsDirtyTracking() {
        const inputs = [
            'settings-server-name',
            'settings-ws-password',
            'settings-ws-auth-timeout',
            'settings-web-port',
            'settings-ws-port',
            'settings-ros-throttle',
            'settings-livelink-port',
        ];

        inputs.forEach(id => {
            const el = document.getElementById(id);
            if (el && !el.hasAttribute('data-dirty-listener')) {
                el.setAttribute('data-dirty-listener', 'true');
                el.addEventListener('input', () => this.checkSettingsDirty());
            }
        });

        // Checkbox needs change event
        const llEnabled = document.getElementById('settings-livelink-enabled');
        if (llEnabled && !llEnabled.hasAttribute('data-dirty-listener')) {
            llEnabled.setAttribute('data-dirty-listener', 'true');
            llEnabled.addEventListener('change', () => this.checkSettingsDirty());
        }

        // Radio buttons need change event
        const tempRadios = ['temp-unit-celsius', 'temp-unit-fahrenheit'];
        tempRadios.forEach(id => {
            const el = document.getElementById(id);
            if (el && !el.hasAttribute('data-dirty-listener')) {
                el.setAttribute('data-dirty-listener', 'true');
                el.addEventListener('change', () => this.checkSettingsDirty());
            }
        });
    }

    /**
     * Check if settings have changed from original values.
     */
    checkSettingsDirty() {
        const current = this.getSettingsFormValues();
        const original = this.settingsOriginalValues;

        this.settingsDirty = (
            current.serverName !== original.serverName ||
            current.wsPassword !== original.wsPassword ||
            current.wsAuthTimeout !== original.wsAuthTimeout ||
            current.webPort !== original.webPort ||
            current.wsPort !== original.wsPort ||
            current.rosThrottle !== original.rosThrottle ||
            current.llEnabled !== original.llEnabled ||
            current.llPort !== original.llPort ||
            current.tempUnit !== original.tempUnit
        );

        this.updateSettingsSaveButton();
    }

    /**
     * Update save button state based on dirty status.
     */
    updateSettingsSaveButton() {
        const saveBtn = document.getElementById('settings-save-btn');
        if (!saveBtn) return;

        if (this.settingsDirty) {
            saveBtn.disabled = false;
            saveBtn.classList.remove('opacity-50', 'cursor-not-allowed');
            saveBtn.innerHTML = '<span class="material-icons text-sm">save</span> Save Changes';
        } else {
            saveBtn.disabled = true;
            saveBtn.classList.add('opacity-50', 'cursor-not-allowed');
            saveBtn.innerHTML = '<span class="material-icons text-sm">check</span> Saved';
        }
    }

    /**
     * Set up the settings save button handler.
     */
    setupSettingsSaveHandler() {
        const saveBtn = document.getElementById('settings-save-btn');
        if (!saveBtn || saveBtn.hasAttribute('data-handler-set')) return;

        saveBtn.setAttribute('data-handler-set', 'true');
        saveBtn.addEventListener('click', () => this.saveSettings());
    }

    /**
     * Collect settings from form and save to server.
     */
    async saveSettings() {
        const saveBtn = document.getElementById('settings-save-btn');
        const statusEl = document.getElementById('settings-save-status');

        // Disable button and show saving status
        saveBtn.disabled = true;
        saveBtn.innerHTML = '<span class="material-icons text-sm animate-spin">refresh</span> Saving...';
        statusEl.textContent = '';
        statusEl.className = 'text-sm text-slate-500';

        // Collect values from form
        const settings = {
            server: {
                name: document.getElementById('settings-server-name')?.value || 'SAINT-01',
            },
            websocket: {
                password: document.getElementById('settings-ws-password')?.value || null,
                auth_timeout: parseFloat(document.getElementById('settings-ws-auth-timeout')?.value) || 10,
            },
            network: {
                web_port: parseInt(document.getElementById('settings-web-port')?.value) || 80,
                websocket_port: parseInt(document.getElementById('settings-ws-port')?.value) || null,
            },
            ros_bridge: {
                throttle_ms: parseInt(document.getElementById('settings-ros-throttle')?.value) || 50,
            },
            livelink: {
                enabled: document.getElementById('settings-livelink-enabled')?.checked ?? true,
                port: parseInt(document.getElementById('settings-livelink-port')?.value) || 11111,
            },
        };

        // Handle empty password
        if (!settings.websocket.password) {
            settings.websocket.password = null;
        }

        // Handle empty websocket port
        if (!settings.network.websocket_port || isNaN(settings.network.websocket_port)) {
            settings.network.websocket_port = null;
        }

        // Save client-side preferences to localStorage
        const tempUnitCelsius = document.getElementById('temp-unit-celsius');
        const newTempUnit = tempUnitCelsius?.checked ? 'celsius' : 'fahrenheit';
        this.setTemperatureUnit(newTempUnit);

        try {
            const ws = window.saintWS;
            const result = await ws.management('set_settings', { settings });

            // Show success
            saveBtn.innerHTML = '<span class="material-icons text-sm">check</span> Saved';
            statusEl.textContent = result.message || 'Settings saved successfully';
            statusEl.className = 'text-sm text-emerald-400';

            // Mark as clean - update original values to current
            this.settingsOriginalValues = this.getSettingsFormValues();
            this.settingsDirty = false;

            // Reset button after delay
            setTimeout(() => {
                this.updateSettingsSaveButton();
                statusEl.textContent = '';
            }, 2000);
        } catch (error) {
            console.error('Failed to save settings:', error);

            // Show error
            saveBtn.innerHTML = '<span class="material-icons text-sm">error</span> Failed';
            statusEl.textContent = error.message || 'Failed to save settings';
            statusEl.className = 'text-sm text-red-400';

            // Reset button after delay (keep dirty state)
            setTimeout(() => {
                this.updateSettingsSaveButton();
            }, 3000);
        }
    }

    // ── Wireless settings (SSID / password / channel) ────────────────
    //
    // Backed by the wifi_admin module on the server, which talks to
    // NetworkManager. Every save here triggers an AP restart and a
    // ~5-10 s websocket outage — the switching-overlay covers the
    // gap and the existing websocket auto-reconnect picks it back up.

    async loadWifiConfig() {
        const ws = window.saintWS;
        const cfg = await ws.management('wifi_get_config', {});
        if (!cfg || !cfg.ok) return;
        const ssidInput = document.getElementById('wifi-ssid');
        const pwInput   = document.getElementById('wifi-password');
        const chSpan    = document.getElementById('wifi-current-channel');
        if (ssidInput && cfg.ssid) ssidInput.value = cfg.ssid;
        if (pwInput && cfg.password) pwInput.value = cfg.password;
        if (chSpan) {
            const bandLabel = cfg.band === 'a' ? '5 GHz' : (cfg.band === 'bg' ? '2.4 GHz' : cfg.band || '?');
            chSpan.textContent = cfg.channel
                ? `Currently ${bandLabel}, channel ${cfg.channel}`
                : `Currently ${bandLabel}, channel auto`;
        }
        // Mirror the same SSID/band/channel onto the dashboard's
        // WiFi Status card if it exists. Cheap; keeps both surfaces
        // coherent after an AP restart pulls fresh values.
        this.updateWifiStatusConfig(cfg);
    }

    setupWirelessCardHandlers() {
        const saveBtn = document.getElementById('wifi-save-creds-btn');
        const toggleBtn = document.getElementById('wifi-password-toggle');
        const findBtn = document.getElementById('wifi-find-channel-btn');
        const applyBtn = document.getElementById('wifi-channel-apply-btn');
        // Second "Find better channel" entry-point on the dashboard's
        // WiFi Status card — same handler as the settings page button.
        const dashFindBtn = document.getElementById('wifi-status-find-channel-btn');

        if (saveBtn && !saveBtn.dataset.bound) {
            saveBtn.dataset.bound = '1';
            saveBtn.addEventListener('click', () => this.saveWifiCredentials());
        }
        if (toggleBtn && !toggleBtn.dataset.bound) {
            toggleBtn.dataset.bound = '1';
            toggleBtn.addEventListener('click', () => {
                const inp = document.getElementById('wifi-password');
                if (!inp) return;
                inp.type = inp.type === 'password' ? 'text' : 'password';
            });
        }
        if (findBtn && !findBtn.dataset.bound) {
            findBtn.dataset.bound = '1';
            findBtn.addEventListener('click', () => this.openWifiChannelModal());
        }
        if (dashFindBtn && !dashFindBtn.dataset.bound) {
            dashFindBtn.dataset.bound = '1';
            dashFindBtn.addEventListener('click', () => this.openWifiChannelModal());
        }
        if (applyBtn && !applyBtn.dataset.bound) {
            applyBtn.dataset.bound = '1';
            applyBtn.addEventListener('click', () => this.applyWifiChannel());
        }
    }

    async saveWifiCredentials() {
        const ssid = document.getElementById('wifi-ssid')?.value || '';
        const password = document.getElementById('wifi-password')?.value || '';
        const status = document.getElementById('wifi-creds-status');

        if (!confirm(
            `Save new credentials and restart the AP?\n\n` +
            `New SSID: ${ssid}\n\n` +
            `Every connected client will drop and reconnect (~5–10 s). ` +
            `If you change the SSID, your laptop/phone will need to ` +
            `reconnect to the new network name manually.`
        )) {
            return;
        }

        try {
            const ws = window.saintWS;
            const r = await ws.management('wifi_set_credentials', { ssid, password });
            if (r && r.switching) {
                this._showWifiSwitchingOverlay(
                    `New SSID: ${ssid}. If the dashboard doesn't reconnect ` +
                    `within 30 s, refresh after rejoining the WiFi network manually.`
                );
            } else if (status) {
                status.textContent = 'Saved.';
                status.className = 'text-sm text-emerald-400';
            }
        } catch (err) {
            console.error('wifi_set_credentials failed:', err);
            if (status) {
                status.textContent = `Save failed: ${err.message || err}`;
                status.className = 'text-sm text-red-300';
            } else {
                alert(`Save failed: ${err.message || err}`);
            }
        }
    }

    async openWifiChannelModal() {
        const modal = document.getElementById('wifi-channel-modal');
        const loading = document.getElementById('wifi-channel-loading');
        const results = document.getElementById('wifi-channel-results');
        const errEl = document.getElementById('wifi-channel-error');
        if (!modal) return;

        // Reset modal state on every open. The async survey can run
        // for a few seconds; the spinner is the user feedback.
        modal.classList.remove('hidden');
        loading.classList.remove('hidden');
        results.classList.add('hidden');
        errEl.classList.add('hidden');
        this._wifiSelectedChannel = null;
        document.getElementById('wifi-channel-apply-btn').disabled = true;

        try {
            const ws = window.saintWS;
            const r = await ws.management('wifi_survey', {});
            const data = r || {};
            if (!data.ok) {
                errEl.textContent = data.error || 'Survey failed (no detail).';
                errEl.classList.remove('hidden');
                loading.classList.add('hidden');
                return;
            }
            this._renderWifiChannelRows(data.channels || [], data.current_channel);
            loading.classList.add('hidden');
            results.classList.remove('hidden');
        } catch (err) {
            console.error('wifi_survey failed:', err);
            errEl.textContent = `Survey failed: ${err.message || err}`;
            errEl.classList.remove('hidden');
            loading.classList.add('hidden');
        }
    }

    _renderWifiChannelRows(channels, currentChannel) {
        const tbody = document.getElementById('wifi-channel-rows');
        if (!tbody) return;
        if (!channels.length) {
            tbody.innerHTML =
                `<tr><td colspan="6" class="p-4 text-center text-slate-500">No channels available</td></tr>`;
            this._renderWifiChannelSummary(null);
            return;
        }

        // Pick a "Best" recommendation. The backend already sorts by
        // (current → ap_count asc → signal asc), so we just have to
        // skip the current row and prefer a non-DFS candidate when one
        // exists at the same busyness tier — DFS channels work but
        // come with the CAC silence delay, so they're a less-friendly
        // first suggestion. If only DFS alternatives are available,
        // recommend the best DFS one anyway.
        const alternatives = channels.filter(c => !c.is_current);
        let best = null;
        if (alternatives.length > 0) {
            const minApCount = alternatives[0].ap_count;
            const tied = alternatives.filter(c => c.ap_count === minApCount);
            best = tied.find(c => !c.is_dfs) || tied[0];
        }

        this._renderWifiChannelSummary(best);

        tbody.innerHTML = channels.map((c, i) => {
            const bandLabel = c.band === '5' ? '5 GHz' : '2.4 GHz';
            const sig = (c.strongest_signal_dbm != null)
                ? `${Math.round(c.strongest_signal_dbm)} dBm`
                : '—';
            const badges = [];
            if (c.is_current) badges.push(
                '<span class="px-1.5 py-0.5 text-xs rounded bg-cyan-500/20 text-cyan-300" ' +
                'title="The AP is currently on this channel.">Current</span>');
            if (best && c === best) badges.push(
                '<span class="px-1.5 py-0.5 text-xs rounded bg-emerald-500/20 text-emerald-300" ' +
                'title="Fewest nearby APs among non-current channels. Non-DFS preferred when available.">Best</span>');
            if (c.is_dfs) badges.push(
                '<span class="px-1.5 py-0.5 text-xs rounded bg-amber-500/20 text-amber-300 cursor-help" ' +
                'title="DFS — Dynamic Frequency Selection. 5 GHz channels overlapping ' +
                'weather/military radar bands. On activation: 60 s silent listen before transmit ' +
                '(CAC). After activation: if the radio detects a radar pulse it must vacate the ' +
                'channel within seconds. Usually fine for ground robots; avoid near airports / ' +
                'weather radar installations. Quieter than non-DFS 5 GHz channels because most ' +
                'consumer gear avoids them.">DFS</span>');

            // AP-count color steps tuned to consumer-WiFi reality:
            // 0 = clear, 1-2 = light traffic, 3-5 = busy, 6+ = avoid.
            // Anything past ~5 co-channel APs in scan distance means
            // sustained airtime contention even when none are active.
            const apClass =
                c.ap_count === 0      ? 'text-emerald-400 font-semibold' :
                c.ap_count <= 2       ? 'text-slate-300' :
                c.ap_count <= 5       ? 'text-amber-300' :
                                        'text-red-300 font-semibold';

            const rowClass =
                (best && c === best)  ? 'bg-emerald-500/10 hover:bg-emerald-500/20' :
                c.is_current          ? 'bg-cyan-500/5 hover:bg-cyan-500/10' :
                                        'hover:bg-slate-700/30';

            return `
                <tr class="border-t border-slate-700 cursor-pointer ${rowClass}"
                    data-channel-idx="${i}"
                    data-band="${c.band === '5' ? 'a' : 'bg'}"
                    data-channel-num="${c.channel}"
                    data-is-current="${c.is_current ? '1' : '0'}">
                    <td class="p-2 text-slate-300">${bandLabel}</td>
                    <td class="p-2 font-mono">${c.channel}</td>
                    <td class="p-2 text-slate-500 text-xs">${c.freq_mhz} MHz</td>
                    <td class="p-2 ${apClass}">${c.ap_count}</td>
                    <td class="p-2 text-slate-400">${sig}</td>
                    <td class="p-2 flex gap-1 flex-wrap">${badges.join('')}</td>
                </tr>
            `;
        }).join('');

        tbody.querySelectorAll('tr[data-channel-idx]').forEach(row => {
            row.addEventListener('click', () => {
                // Clear all selection rings, restore baseline row tinting.
                tbody.querySelectorAll('tr').forEach(r => r.classList.remove('ring-2', 'ring-cyan-400'));
                row.classList.add('ring-2', 'ring-cyan-400');
                this._wifiSelectedChannel = {
                    band: row.dataset.band,
                    channel: parseInt(row.dataset.channelNum, 10),
                };
                // Disable Apply when the operator picks the row they're
                // already on — restarting the AP to switch to the same
                // channel is just an outage with no benefit.
                const applyBtn = document.getElementById('wifi-channel-apply-btn');
                if (applyBtn) {
                    applyBtn.disabled = row.dataset.isCurrent === '1';
                }
            });
        });
    }

    /** Top-of-modal summary line — names the recommended channel + why,
     *  so the operator doesn't have to read the table to find it.
     *  Expands into a two-line caveat when the recommendation is a
     *  DFS channel, because "Best pick: ch 100" without the radar-
     *  silence-on-activation context is misleading. */
    _renderWifiChannelSummary(best) {
        const el = document.getElementById('wifi-channel-summary');
        if (!el) return;
        if (!best) {
            el.classList.add('hidden');
            el.textContent = '';
            return;
        }
        const bandLabel = best.band === '5' ? '5 GHz' : '2.4 GHz';
        const apsText = best.ap_count === 0
            ? 'no nearby APs'
            : `${best.ap_count} nearby AP${best.ap_count === 1 ? '' : 's'}`;
        const dfsCaveat = best.is_dfs
            ? `<div class="text-xs text-amber-200/90 mt-1">
                   <span class="material-icons icon-sm align-middle">info</span>
                   This is a DFS channel. Activation requires a 60-second silent
                   listen for radar before the AP comes back up, and the radio
                   may move off this channel later if it detects a radar pulse.
                   Generally fine for indoor / yard operation; avoid if you're
                   near an airport or weather radar installation.
               </div>`
            : '';
        el.innerHTML =
            `<div>
                 <span class="material-icons icon-sm align-middle text-emerald-300">recommend</span>
                 Best pick: <strong>${bandLabel} channel ${best.channel}</strong>
                 (${apsText}).
             </div>` +
            dfsCaveat;
        el.classList.remove('hidden');
    }

    closeWifiChannelModal() {
        document.getElementById('wifi-channel-modal')?.classList.add('hidden');
    }

    async applyWifiChannel() {
        const sel = this._wifiSelectedChannel;
        if (!sel) return;
        const bandLabel = sel.band === 'a' ? '5 GHz' : '2.4 GHz';
        if (!confirm(
            `Switch the AP to ${bandLabel} channel ${sel.channel}?\n\n` +
            `Every connected client (including this dashboard) will ` +
            `drop and reconnect over ~5–10 s.`
        )) {
            return;
        }
        try {
            const ws = window.saintWS;
            await ws.management('wifi_set_channel', sel);
            this.closeWifiChannelModal();
            this._showWifiSwitchingOverlay(
                `Switching to ${bandLabel} channel ${sel.channel}.`
            );
        } catch (err) {
            console.error('wifi_set_channel failed:', err);
            alert(`Channel switch failed: ${err.message || err}`);
        }
    }

    _showWifiSwitchingOverlay(detail) {
        const overlay = document.getElementById('wifi-switching-overlay');
        const detailEl = document.getElementById('wifi-switching-detail');
        if (detailEl && detail) detailEl.textContent = detail;
        if (overlay) overlay.classList.remove('hidden');
        // Hide the overlay once the websocket reconnects — the
        // existing 'connected' event is the right signal. We bind
        // a one-shot listener since the overlay only matters once.
        const ws = window.saintWS;
        if (ws && typeof ws.on === 'function') {
            const onReconnected = () => {
                overlay?.classList.add('hidden');
                // refresh the Wireless card with new values
                this.loadWifiConfig().catch(() => {});
            };
            ws.once ? ws.once('connected', onReconnected) : ws.on('connected', onReconnected);
        }
        // Safety net — never leave the overlay up forever if the
        // 'connected' event doesn't fire for some reason.
        setTimeout(() => overlay?.classList.add('hidden'), 60_000);
    }

    /**
     * Request initial data after connection.
     */
    async requestInitialData() {
        const ws = window.saintWS;

        try {
            // Subscribe to state updates. 'estop' carries the latching
            // E-Stop state — broadcast by the server whenever the
            // latch flips so every connected dashboard tracks it.
            // 'pin_state/host_controller' streams the server's own
            // system_monitor channels (cpu/mem/temp/wifi_*), used by
            // the dashboard's WiFi Status card.
            await ws.subscribe(
                ['system', 'nodes', 'estop', 'pin_state/host_controller'], 1);

            // Pull the current latch state so the button doesn't
            // start in a stale "released" look when the page loads
            // mid-engagement.
            try {
                const r = await ws.management('get_estop_state', {});
                if (r && typeof r.active === 'boolean') {
                    this.setEstopState(r.active);
                }
            } catch (e) {
                // Non-fatal — broadcast will catch us up.
            }

            // Load current page data
            await this.loadPageData(this.currentPage);
        } catch (error) {
            console.error('Failed to request initial data:', error);
        }
    }

    /**
     * Update connection status indicator.
     */
    updateConnectionStatus(status, authRequired = false) {
        const statusDot = document.querySelector('.status-dot');
        const statusText = document.querySelector('.status-text');

        // Remove all status classes
        statusDot.classList.remove('bg-amber-500', 'bg-red-500', 'bg-emerald-500', 'bg-cyan-500', 'animate-pulse-dot');

        switch (status) {
            case 'ready':
                // Fully connected and authenticated
                statusDot.classList.add('bg-emerald-500');
                statusText.textContent = 'Connected';
                break;
            case 'connected':
                // Connected but may need auth
                if (authRequired) {
                    statusDot.classList.add('bg-cyan-500', 'animate-pulse-dot');
                    statusText.textContent = 'Authenticating...';
                } else {
                    statusDot.classList.add('bg-emerald-500');
                    statusText.textContent = 'Connected';
                }
                break;
            case 'auth_failed':
                statusDot.classList.add('bg-amber-500');
                statusText.textContent = 'Auth Required';
                break;
            case 'disconnected':
            default:
                statusDot.classList.add('bg-red-500', 'animate-pulse-dot');
                statusText.textContent = 'Disconnected';
                break;
        }
    }

    /**
     * Handle state updates from server.
     */
    handleStateUpdate(message) {
        if (message.node === 'system') {
            this.systemStatus = message.data;
            this.updateSystemStatus();
        } else if (message.node === 'nodes') {
            // Update cached node data
            const data = message.data;
            if (data.adopted) {
                this.nodes.adopted = data.adopted;
            }
            if (data.unadopted) {
                this.nodes.unadopted = data.unadopted;
            }
            // Update UI based on current page
            this.updateNodesUI();
            // Re-render the Logs source list so newly adopted nodes
            // appear (and removed ones disappear) on the left rail.
            if (this.currentPage === 'logs') {
                this.renderLogsSourceList();
            }
        } else if (message.node === 'estop') {
            // Server-side latch flipped (by us or by another dashboard).
            // Mirror the visual state.
            this.setEstopState(!!message.data?.active);
        } else if (typeof message.node === 'string'
                   && message.node.startsWith('node_log/')) {
            // Live node-log streams. NodeLogsManager already handles
            // the node-detail Logs tab; this catches the same frames
            // for the global Logs page so the row table updates in
            // real time when the operator is watching a node source.
            const nodeId = message.node.slice('node_log/'.length);
            this.onNodeLogEntry(nodeId, message.data);
        } else if (message.node === 'pin_state/host_controller') {
            // The host_controller's system_monitor channels live on
            // this topic. We pluck the wifi_* fields out and feed
            // the WiFi Status card. cpu/mem/temp/uptime/throttle
            // continue to flow through the 'system' message above —
            // host_controller carries the same data on a different
            // schema, used elsewhere for routing-graph values.
            this.updateWifiStatusCard(message.data);
        }
        // Other pin_state/<id> broadcasts are handled by widgetsDashboard
        // — it subscribes itself when the dashboard activates.
    }

    /**
     * Update nodes-related UI elements based on current page.
     */
    updateNodesUI() {
        // If on nodes page, refresh the list. The old dashboard
        // "Nodes Summary" card is gone — the Nodes tab is the
        // canonical view, and the dashboard now shows WiFi instead.
        if (this.currentPage === 'nodes') {
            this.renderNodesList();
        }

        // If on control page and controlPage is loaded, update node status
        if (this.currentPage === 'control' && typeof window.controlPage !== 'undefined') {
            window.controlPage.updateNodeStatus(this.nodes.adopted);
        }
    }

    /**
     * Update system status display.
     */
    updateSystemStatus() {
        const status = this.systemStatus;
        if (!status) return;

        // Update status badge
        const statusBadge = document.getElementById('server-status');
        if (status.server_online) {
            statusBadge.textContent = 'Online';
            statusBadge.className = 'px-2 py-1 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400';
        } else {
            statusBadge.textContent = 'Offline';
            statusBadge.className = 'px-2 py-1 text-xs font-medium rounded-full bg-red-500/20 text-red-400';
        }

        // Update values
        document.getElementById('server-uptime').textContent =
            this.formatUptime(status.uptime_seconds);
        document.getElementById('server-name').textContent =
            status.server_name || '--';

        // Update CPU with progress bar
        const cpuValue = status.cpu_usage?.toFixed(1) || '0';
        document.getElementById('server-cpu').textContent = `${cpuValue}%`;
        document.getElementById('cpu-bar').style.width = `${cpuValue}%`;

        // Update Memory with progress bar
        const memValue = status.memory_usage?.toFixed(1) || '0';
        document.getElementById('server-memory').textContent = `${memValue}%`;
        document.getElementById('memory-bar').style.width = `${memValue}%`;

        // CPU temperature — color-coded against the Pi's throttle thresholds
        // (warns at 65°C, critical at 80°C where soft-throttle kicks in).
        const tempEl = document.getElementById('server-cpu-temp');
        if (tempEl) {
            const tempC = status.cpu_temp_c;
            if (typeof tempC === 'number') {
                tempEl.textContent = this.formatTemperature(tempC);
                let cls = 'stat-value';
                if (tempC >= 80) cls += ' text-red-400';
                else if (tempC >= 65) cls += ' text-amber-400';
                else cls += ' text-emerald-400';
                tempEl.className = cls;
            } else {
                tempEl.textContent = '--';
                tempEl.className = 'stat-value text-slate-500';
            }
        }

        // Throttle status — comes from `vcgencmd get_throttled` on Pi
        const throttleEl = document.getElementById('server-throttle');
        if (throttleEl) {
            const t = status.throttle;
            if (!t) {
                throttleEl.textContent = 'n/a';
                throttleEl.className = 'stat-value text-sm text-slate-500';
                throttleEl.title = 'vcgencmd not available on this host';
            } else if (t.status === 'ok') {
                throttleEl.textContent = 'OK';
                throttleEl.className = 'stat-value text-sm text-emerald-400';
                throttleEl.title = `${t.summary} (${t.raw})`;
            } else if (t.status === 'warning') {
                throttleEl.textContent = 'Past events';
                throttleEl.className = 'stat-value text-sm text-amber-400';
                throttleEl.title = `${t.summary} (${t.raw}): ${(t.descriptions || []).join(', ')}`;
            } else {
                throttleEl.textContent = 'Active';
                throttleEl.className = 'stat-value text-sm text-red-400';
                throttleEl.title = `${t.summary} (${t.raw}): ${(t.descriptions || []).join(', ')}`;
            }
        }

        // Footer: full version + git hash so the operator always knows
        // which build is running.
        const versionEl = document.getElementById('version');
        if (versionEl && status.server_version) {
            versionEl.textContent = status.server_version;
        }
        const hashEl = document.getElementById('version-hash');
        if (hashEl) {
            const sha = status.server_git_sha;
            hashEl.textContent = sha ? `(${sha.slice(0, 7)})` : '';
            hashEl.title = status.server_built_at
                ? `Built ${status.server_built_at}` : '';
        }
    }

    /**
     * Update display for a single firmware build.
     */
    updateFirmwareBuildDisplay(prefix, info) {
        const versionEl = document.getElementById(`settings-fw-${prefix}-version`);
        const hashEl = document.getElementById(`settings-fw-${prefix}-hash`);
        const buildEl = document.getElementById(`settings-fw-${prefix}-build`);
        const statusEl = document.getElementById(`settings-fw-${prefix}-status`);

        if (!versionEl) return;

        if (info?.available) {
            versionEl.textContent = info.version_full || info.version || '--';
            hashEl.textContent = info.git_hash || '--';
            buildEl.textContent = info.build_date || '--';

            statusEl.textContent = 'Available';
            statusEl.className = 'px-2 py-1 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400';
        } else {
            versionEl.textContent = '--';
            hashEl.textContent = '--';
            buildEl.textContent = '--';

            statusEl.textContent = 'Not Built';
            statusEl.className = 'px-2 py-1 text-xs font-medium rounded-full bg-slate-700 text-slate-400';
        }
    }

    /**
     * Update display for Pi 5 firmware.
     */
    updatePi5FirmwareDisplay(info) {
        const versionEl = document.getElementById('settings-fw-rpi5-version');
        const packageEl = document.getElementById('settings-fw-rpi5-package');
        const buildEl = document.getElementById('settings-fw-rpi5-build');
        const statusEl = document.getElementById('settings-fw-rpi5-status');

        if (!versionEl) return;

        if (info?.available) {
            versionEl.textContent = info.version || '--';
            packageEl.textContent = info.filename || '--';
            buildEl.textContent = info.build_date || '--';

            statusEl.textContent = 'Available';
            statusEl.className = 'px-2 py-1 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400';
        } else {
            versionEl.textContent = '--';
            packageEl.textContent = '--';
            buildEl.textContent = '--';

            statusEl.textContent = 'Not Available';
            statusEl.className = 'px-2 py-1 text-xs font-medium rounded-full bg-slate-700 text-slate-400';
        }
    }

    /**
     * Update display for the Steam Deck controller AppImage staged on the
     * server. Same shape as Pi 5 firmware — the in-app OTA flow in the
     * controller's Settings tab fetches the file from this same staging.
     */
    updateControllerFirmwareDisplay(info) {
        const versionEl = document.getElementById('settings-fw-controller-version');
        const packageEl = document.getElementById('settings-fw-controller-package');
        const buildEl = document.getElementById('settings-fw-controller-build');
        const statusEl = document.getElementById('settings-fw-controller-status');

        if (!versionEl) return;

        if (info?.available) {
            versionEl.textContent = info.version || '--';
            packageEl.textContent = info.filename || '--';
            buildEl.textContent = info.build_date || '--';

            statusEl.textContent = 'Available';
            statusEl.className = 'px-2 py-1 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400';
        } else {
            versionEl.textContent = '--';
            packageEl.textContent = '--';
            buildEl.textContent = '--';

            statusEl.textContent = 'Not Available';
            statusEl.className = 'px-2 py-1 text-xs font-medium rounded-full bg-slate-700 text-slate-400';
        }
    }

    /**
     * Format uptime seconds to human readable string.
     */
    formatUptime(seconds) {
        if (!seconds) return '--';

        const days = Math.floor(seconds / 86400);
        const hours = Math.floor((seconds % 86400) / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);

        if (days > 0) {
            return `${days}d ${hours}h ${minutes}m`;
        } else if (hours > 0) {
            return `${hours}h ${minutes}m`;
        } else {
            return `${minutes}m`;
        }
    }

    /**
     * Add entry to the in-memory activity stream. The dashboard
     * Activity Log card was removed in favor of the master-detail
     * Logs page (Server source), so we update the buffer and — only
     * when the Logs page is open AND looking at the Server source —
     * incrementally prepend the new row so the user's scroll/expand
     * state survives the update.
     */
    addActivityLogEntry(message) {
        const level = message.level || 'info';
        const timestamp = message.timestamp || (Date.now() / 1000);
        const entry = {
            time: timestamp,
            text: message.text || message.message,
            level: level,
        };

        this.activityLog.unshift(entry);
        if (this.activityLog.length > 500) {
            this.activityLog = this.activityLog.slice(0, 500);
        }

        if (this.currentPage === 'logs'
            && this.logsSelectedSource === 'server') {
            this._prependLogRow(entry, this.activityLog.length - 1);
        }
    }

    /**
     * Load dashboard data.
     */
    async loadDashboardData() {
        const ws = window.saintWS;

        try {
            const [adopted, unadopted] = await Promise.all([
                ws.management('list_adopted'),
                ws.management('list_unadopted')
            ]);

            this.nodes.adopted = adopted.nodes || [];
            this.nodes.unadopted = unadopted.nodes || [];

            if (window.widgetsDashboard) {
                await window.widgetsDashboard.activate(this.nodes.adopted);
            }
        } catch (error) {
            console.error('Failed to load dashboard data:', error);
        }

        // Seed the WiFi card with config-time info (SSID, band,
        // channel). Live metrics fill in from pin_state/host_controller
        // as it streams. Best-effort — missing iw / non-Pi host just
        // leaves the static fields as "--".
        try {
            const cfg = await ws.management('wifi_get_config', {});
            this.updateWifiStatusConfig(cfg);
        } catch (e) {
            // Non-fatal.
        }

        // Wire the dashboard's "Find better channel" button. Same
        // handler as the settings page copy; setupWirelessCardHandlers
        // is idempotent (data-bound guard) so this is safe to call
        // from both code paths.
        this.setupWirelessCardHandlers();
    }

    /**
     * Populate the static-ish portion of the WiFi card (SSID + band/channel)
     * from a wifi_get_config response. Called on dashboard load and after
     * a successful AP restart (when the post-switch reload hook fires).
     */
    updateWifiStatusConfig(cfg) {
        if (!cfg || !cfg.ok) return;
        const ssidEl = document.getElementById('wifi-status-ssid');
        const bcEl   = document.getElementById('wifi-status-bandch');
        if (ssidEl) ssidEl.textContent = cfg.ssid || '--';
        if (bcEl) {
            const bandLabel = cfg.band === 'a' ? '5 GHz'
                : (cfg.band === 'bg' ? '2.4 GHz' : (cfg.band || '?'));
            bcEl.textContent = cfg.channel
                ? `${bandLabel} · ch ${cfg.channel}`
                : `${bandLabel} · auto`;
        }
    }

    /**
     * Live update from pin_state/host_controller. We sift the channel
     * list for the four system_monitor.wifi_* readings and refresh
     * just those fields. Missing channels (e.g. on a host with no
     * WiFi interface) leave the previous value in place so the card
     * doesn't flicker between "--" and a real number every second.
     */
    updateWifiStatusCard(data) {
        if (!data || !Array.isArray(data.channels)) return;
        let signal, retry, noise, bitrate;
        for (const ch of data.channels) {
            if (ch.peripheral_id !== 'system_monitor') continue;
            if (ch.channel_id === 'wifi_signal') signal = ch.value;
            else if (ch.channel_id === 'wifi_retry_pct') retry = ch.value;
            else if (ch.channel_id === 'wifi_noise') noise = ch.value;
            else if (ch.channel_id === 'wifi_bitrate') bitrate = ch.value;
        }
        const set = (id, val, fmt) => {
            const el = document.getElementById(id);
            if (el && typeof val === 'number') el.textContent = fmt(val);
        };
        set('wifi-status-signal',  signal,  v => `${Math.round(v)} dBm`);
        set('wifi-status-retry',   retry,   v => `${v.toFixed(1)}%`);
        set('wifi-status-noise',   noise,   v => `${Math.round(v)} dBm`);
        set('wifi-status-bitrate', bitrate, v => `${v.toFixed(1)} Mbps`);

        const ageEl = document.getElementById('wifi-status-age');
        if (ageEl) ageEl.textContent = new Date().toLocaleTimeString();
    }

    /**
     * Load nodes page data.
     */
    async loadNodesData() {
        const ws = window.saintWS;

        try {
            const [adopted, unadopted] = await Promise.all([
                ws.management('list_adopted'),
                ws.management('list_unadopted')
            ]);

            this.nodes.adopted = adopted.nodes || [];
            this.nodes.unadopted = unadopted.nodes || [];

            this.renderNodesList();
        } catch (error) {
            console.error('Failed to load nodes data:', error);
        }
    }

    /**
     * Render nodes lists.
     */
    renderNodesList() {
        // Adopted nodes
        const adoptedContainer = document.getElementById('adopted-nodes');
        if (this.nodes.adopted.length === 0) {
            adoptedContainer.innerHTML = '<p class="text-slate-400 col-span-full">No adopted nodes</p>';
        } else {
            adoptedContainer.innerHTML = this.nodes.adopted.map(node => {
                const fwUpdate = node.firmware_update_available === true && node.server_firmware_version;
                const fwBadge = fwUpdate
                    ? `<span class="ml-2 px-1.5 py-0.5 text-[10px] font-medium rounded bg-cyan-500/20 text-cyan-300 border border-cyan-500/30" title="Update available: ${node.server_firmware_version}">↑ ${node.server_firmware_version}</span>`
                    : '';
                return `
                <div class="node-card">
                    <div class="flex items-start justify-between mb-3">
                        <div class="flex items-center gap-2">
                            <span class="w-2.5 h-2.5 rounded-full ${node.online ? 'bg-emerald-500' : 'bg-slate-500'}"></span>
                            <span class="font-medium text-white">${node.display_name || node.node_id}</span>
                        </div>
                        <span class="px-2 py-0.5 text-xs rounded-full bg-cyan-500/20 text-cyan-400">${node.role}</span>
                    </div>
                    <div class="text-sm text-slate-400 mb-3 space-y-1">
                        <p>${node.hardware_model || 'Unknown hardware'}</p>
                        <p class="text-xs">FW: ${node.firmware_version || '--'}${fwBadge}</p>
                        <p class="text-xs">BL: ${node.bootloader_version || 'unknown'}</p>
                    </div>
                    <div class="flex items-center gap-2 pt-3 border-t border-slate-700">
                        <button class="btn-sm bg-slate-700 hover:bg-slate-600 text-slate-200" onclick="app.viewNode('${node.node_id}')">
                            View
                        </button>
                        <button class="btn-sm bg-slate-700 hover:bg-red-600 text-slate-200" onclick="app.resetNode('${node.node_id}')">
                            Reset
                        </button>
                        <button class="btn-sm bg-slate-700 hover:bg-red-600 text-slate-400 hover:text-white" onclick="app.removeNode('${node.node_id}')" title="Remove node">
                            <span class="material-icons icon-sm">delete</span>
                        </button>
                    </div>
                </div>
            `;
            }).join('');
        }

        this.updateAllNodesButtonVisibility();

        // Unadopted nodes
        const unadoptedContainer = document.getElementById('unadopted-nodes');
        if (this.nodes.unadopted.length === 0) {
            unadoptedContainer.innerHTML = '<p class="text-slate-400 col-span-full">No unadopted nodes found</p>';
        } else {
            unadoptedContainer.innerHTML = this.nodes.unadopted.map(node => {
                const isOnline = node.online !== false;
                const statusDot = isOnline
                    ? 'bg-amber-500'
                    : 'bg-slate-500';
                const statusBadge = isOnline
                    ? '<span class="px-2 py-0.5 text-xs rounded-full bg-amber-500/20 text-amber-400">New</span>'
                    : '<span class="px-2 py-0.5 text-xs rounded-full bg-slate-500/20 text-slate-400">Offline</span>';
                const borderColor = isOnline ? 'border-amber-500/30' : 'border-slate-600/30 opacity-60';
                const adoptButton = isOnline
                    ? `<button class="btn-sm bg-cyan-600 hover:bg-cyan-500 text-white flex-1" onclick="app.adoptNode('${node.node_id}')">Adopt Node</button>`
                    : `<button class="btn-sm bg-slate-700 text-slate-400 flex-1 cursor-not-allowed" disabled>Offline</button>`;

                return `
                <div class="node-card ${borderColor}">
                    <div class="flex items-start justify-between mb-3">
                        <div class="flex items-center gap-2">
                            <span class="w-2.5 h-2.5 rounded-full ${statusDot}"></span>
                            <span class="font-medium text-white">${node.node_id}</span>
                        </div>
                        ${statusBadge}
                    </div>
                    <div class="text-sm text-slate-400 mb-3 space-y-1">
                        <p>${node.hardware_model || 'Unknown hardware'}</p>
                        <p class="font-mono text-xs">${node.ip_address || '--'}</p>
                        <p class="text-xs">FW: ${node.firmware_version || '--'}</p>
                        <p class="text-xs">BL: ${node.bootloader_version || 'unknown'}</p>
                    </div>
                    <div class="flex items-center gap-2 pt-3 border-t border-slate-700">
                        ${adoptButton}
                        <button class="btn-sm bg-slate-700 hover:bg-red-600 text-slate-400 hover:text-white" onclick="app.removeNode('${node.node_id}')" title="Remove node">
                            <span class="material-icons icon-sm">delete</span>
                        </button>
                    </div>
                </div>
            `}).join('');
        }
    }

    /**
     * Load node detail page data.
     */
    async loadNodeDetailData() {
        if (!this.currentNodeId) return;

        const ws = window.saintWS;

        try {
            // Get node info
            const adoptedResult = await ws.management('list_adopted');
            this.currentNodeInfo = (adoptedResult.nodes || []).find(n => n.node_id === this.currentNodeId);

            if (!this.currentNodeInfo) {
                console.error('Node not found:', this.currentNodeId);
                return;
            }

            // Update header
            document.getElementById('node-detail-title').textContent =
                this.currentNodeInfo.display_name || this.currentNodeId;
            document.getElementById('node-detail-subtitle').textContent =
                `${this.currentNodeInfo.role} - ${this.currentNodeInfo.node_id}`;

            // Update online status
            this.updateNodeOnlineStatus(this.currentNodeInfo.online);

            // Update overview tab
            this.updateNodeOverview();

            // Load peripherals
            if (typeof peripheralManager !== 'undefined') {
                await peripheralManager.loadNodeData(this.currentNodeId, this.currentNodeInfo);
            }

            // Point the Logs + Live + State tabs at this node so they're
            // ready when the operator clicks over. Each setNode() call also
            // (re)subscribes to its server topic.
            if (typeof window.nodeLogsManager !== 'undefined') {
                window.nodeLogsManager.setNode(this.currentNodeId);
            }
            if (typeof window.nodeLiveManager !== 'undefined') {
                window.nodeLiveManager.setNode(this.currentNodeId);
            }
            if (typeof window.stateControlManager !== 'undefined') {
                window.stateControlManager.setNode(this.currentNodeId, this.currentNodeInfo);
            }

            // If the State tab happens to be the currently-active tab,
            // refresh its sidebar (status, count) — setNode() above
            // handles the controls grid.
            const stateTab = document.getElementById('node-tab-state');
            if (stateTab && !stateTab.classList.contains('hidden')) {
                const statusEl = document.getElementById('state-node-status');
                if (statusEl) statusEl.textContent = this.currentNodeInfo.online ? 'Online' : 'Offline';
            }

            // Start periodic refresh for live data
            this.startNodeDetailRefresh();
        } catch (error) {
            console.error('Failed to load node detail:', error);
        }
    }

    /**
     * Start periodic refresh of node detail data.
     */
    startNodeDetailRefresh() {
        // Clear any existing interval
        this.stopNodeDetailRefresh();

        this.nodeDetailRefreshInterval = setInterval(async () => {
            if (!this.currentNodeId || this.currentPage !== 'node-detail') {
                this.stopNodeDetailRefresh();
                return;
            }

            try {
                const ws = window.saintWS;
                if (!ws || !ws.connected) return;

                const adoptedResult = await ws.management('list_adopted');
                const updatedNode = (adoptedResult.nodes || []).find(n => n.node_id === this.currentNodeId);

                if (updatedNode) {
                    this.currentNodeInfo = updatedNode;
                    this.updateNodeOnlineStatus(updatedNode.online);
                    this.updateNodeOverview();
                }
            } catch (error) {
                console.error('Failed to refresh node detail:', error);
            }
        }, this.nodeDetailRefreshRate);
    }

    /**
     * Stop periodic refresh of node detail data.
     */
    stopNodeDetailRefresh() {
        if (this.nodeDetailRefreshInterval) {
            clearInterval(this.nodeDetailRefreshInterval);
            this.nodeDetailRefreshInterval = null;
        }
    }

    /**
     * Update node online status display.
     */
    updateNodeOnlineStatus(online) {
        const statusEl = document.getElementById('node-online-status');
        if (!statusEl) return;

        const dot = statusEl.querySelector('span:first-child');
        const text = statusEl.querySelector('span:last-child');

        if (online) {
            dot.className = 'w-2 h-2 rounded-full bg-emerald-500';
            text.textContent = 'Online';
            text.className = 'text-sm text-emerald-400';
        } else {
            dot.className = 'w-2 h-2 rounded-full bg-slate-500';
            text.textContent = 'Offline';
            text.className = 'text-sm text-slate-400';
        }
    }

    /**
     * Update node overview tab.
     */
    updateNodeOverview() {
        const info = this.currentNodeInfo;
        if (!info) return;

        document.getElementById('node-info-id').textContent = info.node_id;
        document.getElementById('node-info-role').textContent = info.role || '--';
        document.getElementById('node-info-hardware').textContent = info.hardware_model || 'Unknown';
        document.getElementById('node-info-firmware').textContent = info.firmware_version || '--';
        document.getElementById('node-info-firmware-build').textContent = info.firmware_build ? `Built: ${info.firmware_build}` : '';
        document.getElementById('node-info-bootloader').textContent = info.bootloader_version || 'unknown';
        document.getElementById('node-info-ip').textContent = info.ip_address || '--';
        document.getElementById('node-info-uptime').textContent =
            info.uptime_seconds ? this.formatUptime(info.uptime_seconds) : '--';

        document.getElementById('node-stat-temp').textContent =
            this.formatTemperature(info.cpu_temp);
        document.getElementById('node-stat-state').textContent = info.state || 'Unknown';
        document.getElementById('node-stat-lastseen').textContent =
            info.last_seen ? new Date(info.last_seen * 1000).toLocaleString() : '--';

        // Update firmware update badge and button
        this.updateFirmwareUpdateUI(info);
    }

    /**
     * Update firmware update UI elements based on availability.
     */
    updateFirmwareUpdateUI(info) {
        const badge = document.getElementById('node-firmware-update-badge');
        const button = document.getElementById('btn-firmware-update');
        const versionSpan = document.getElementById('firmware-update-version');

        const updateAvailable = info.firmware_update_available === true;
        const serverVersion = info.server_firmware_version;

        // Update badge visibility
        if (badge) {
            if (updateAvailable && serverVersion) {
                badge.classList.remove('hidden');
                badge.textContent = `Update to ${serverVersion}`;
            } else {
                badge.classList.add('hidden');
            }
        }

        // Update button visibility
        if (button) {
            if (updateAvailable && serverVersion) {
                button.classList.remove('hidden');
                if (versionSpan) {
                    versionSpan.textContent = `(${info.firmware_version} -> ${serverVersion})`;
                }
            } else {
                button.classList.add('hidden');
            }
        }
    }

    /**
     * Switch node detail tab.
     */
    switchNodeTab(tabId) {
        // Update tab buttons
        document.querySelectorAll('.node-tab').forEach(tab => {
            if (tab.dataset.tab === tabId) {
                tab.classList.add('active');
            } else {
                tab.classList.remove('active');
            }
        });

        // Update tab content
        document.querySelectorAll('.node-tab-content').forEach(content => {
            if (content.id === `node-tab-${tabId}`) {
                content.classList.remove('hidden');
                content.classList.add('active');
            } else {
                content.classList.add('hidden');
                content.classList.remove('active');
            }
        });

        // Handle tab-specific logic
        if (tabId === 'state' && this.currentNodeId) {
            this.loadStateTabControls();
        }
        if (tabId === 'logs' && this.currentNodeId
                && typeof window.nodeLogsManager !== 'undefined') {
            window.nodeLogsManager.setNode(this.currentNodeId);
        }
        if (tabId === 'live' && this.currentNodeId
                && typeof window.nodeLiveManager !== 'undefined') {
            window.nodeLiveManager.setNode(this.currentNodeId);
        }
    }

    /**
     * Switch settings sub-tab.
     */
    switchSettingsTab(tabId) {
        // Update tab buttons
        document.querySelectorAll('.settings-tab').forEach(tab => {
            if (tab.dataset.settingsTab === tabId) {
                tab.classList.add('active');
            } else {
                tab.classList.remove('active');
            }
        });

        // Update tab panels
        document.querySelectorAll('.settings-panel').forEach(panel => {
            if (panel.id === `settings-panel-${tabId}`) {
                panel.classList.add('active');
            } else {
                panel.classList.remove('active');
            }
        });

        // Boards tab fetches its data on activation (lazy — saves a
        // round-trip until the operator actually opens it).
        if (tabId === 'boards' && window.boardsManager) {
            window.boardsManager.activate();
        }
    }

    /**
     * Load peripheral controls for the State tab.
     *
     * Delegates rendering to StateControlManager, which enumerates the
     * node's peripherals from the catalog and renders one widget per
     * writable channel. Controls are present even before hardware is
     * wired up, so the operator can exercise the full command path
     * (UI → server → firmware) before connecting the physical device.
     */
    async loadStateTabControls() {
        if (!this.currentNodeId || !this.currentNodeInfo) return;

        document.getElementById('state-node-status').textContent =
            this.currentNodeInfo.online ? 'Online' : 'Offline';

        if (window.stateControlManager) {
            await window.stateControlManager.setNode(this.currentNodeId, this.currentNodeInfo);
            const count = stateControlManager.peripherals.length;
            document.getElementById('state-pin-count').textContent = String(count);
        }

        this._wireStateQuickActions();
    }

    _wireStateQuickActions() {
        const btn = document.getElementById('btn-all-outputs-neutral');
        if (!btn || !window.stateControlManager) return;
        btn.onclick = async () => {
            const mgr = window.stateControlManager;
            for (const p of mgr.peripherals) {
                const type = mgr._typeFor(p.type);
                if (!type) continue;
                for (const ch of (type.channels || [])) {
                    if (ch.dir !== 'out') continue;
                    const spec = mgr._channelSpec(p.type, ch);
                    if (spec.unsupported) continue;
                    const neutral = spec.neutral !== undefined ? spec.neutral : 0;
                    await mgr._sendChannelValue(p.id, ch.id, neutral);
                }
            }
            this.addActivityLogEntry({ text: 'Reset all outputs to neutral', level: 'info' });
        };
    }

    /**
     * Load routes page data.
     */
    async loadRoutesData() {
        if (window.routingPage) {
            await window.routingPage.activate();
        }
    }

    /**
     * Load inputs page data.
     */
    async loadInputsData() {
        // Subscribe to LiveLink status updates
        const ws = window.saintWS;
        if (ws && ws.connected) {
            await ws.subscribe(['livelink']);
            // Load WebSocket clients
            await this.loadWebSocketClients();
        }
    }

    /**
     * Load and display connected WebSocket clients.
     */
    async loadWebSocketClients() {
        const ws = window.saintWS;
        if (!ws || !ws.connected) return;

        const container = document.getElementById('ws-clients');
        if (!container) return;

        try {
            const result = await ws.management('list_clients');
            const clients = result?.clients || [];
            this.connectedClients = clients;
            this.renderWebSocketClients();
        } catch (err) {
            console.error('Failed to load WebSocket clients:', err);
            container.innerHTML = `<p class="text-red-400 text-sm">Failed to load clients</p>`;
        }
    }

    /**
     * Render the WebSocket clients list.
     */
    renderWebSocketClients() {
        const container = document.getElementById('ws-clients');
        if (!container) return;

        const clients = this.connectedClients;
        if (!clients || clients.length === 0) {
            container.innerHTML = `<p class="text-slate-400 text-sm">No clients connected</p>`;
            return;
        }

        const now = Date.now() / 1000;
        let html = '';
        for (const client of clients) {
            const connectedDuration = now - client.connected_at;
            const durationStr = this.formatDuration(connectedDuration);

            // Determine client type from user agent
            let clientType = 'Unknown';
            let clientIcon = 'devices';
            if (client.user_agent.includes('Tauri') || client.user_agent.includes('saint-controller')) {
                clientType = 'Controller';
                clientIcon = 'sports_esports';
            } else if (client.user_agent.includes('Mozilla') || client.user_agent.includes('Chrome') || client.user_agent.includes('Safari')) {
                clientType = 'Browser';
                clientIcon = 'language';
            }

            const selfBadge = client.is_self
                ? '<span class="px-1.5 py-0.5 text-xs bg-cyan-500/20 text-cyan-400 rounded">you</span>'
                : '';

            const disconnectBtn = client.is_self
                ? ''
                : `<button onclick="app.disconnectClient('${client.id}')"
                          class="p-1 rounded hover:bg-red-500/20 transition-colors" title="Disconnect">
                      <span class="material-icons text-sm text-red-400">close</span>
                   </button>`;

            html += `
                <div class="flex items-center justify-between p-2 bg-slate-800/50 rounded-lg">
                    <div class="flex items-center gap-3 min-w-0">
                        <span class="material-icons text-slate-400">${clientIcon}</span>
                        <div class="min-w-0">
                            <div class="flex items-center gap-2">
                                <span class="text-sm text-white font-mono">${client.ip_address}</span>
                                ${selfBadge}
                            </div>
                            <div class="text-xs text-slate-500 truncate">
                                ${clientType} · ${durationStr}
                            </div>
                        </div>
                    </div>
                    ${disconnectBtn}
                </div>
            `;
        }

        container.innerHTML = html;
    }

    /**
     * Disconnect a WebSocket client.
     */
    async disconnectClient(clientId) {
        const ws = window.saintWS;
        if (!ws || !ws.connected) return;

        if (!confirm('Disconnect this client?')) return;

        try {
            await ws.management('disconnect_client', { client_id: clientId });
            // Refresh the list
            await this.loadWebSocketClients();
        } catch (err) {
            console.error('Failed to disconnect client:', err);
            alert('Failed to disconnect client: ' + err.message);
        }
    }

    /**
     * Format duration in seconds to human readable string.
     */
    formatDuration(seconds) {
        if (seconds < 60) return `${Math.floor(seconds)}s`;
        if (seconds < 3600) return `${Math.floor(seconds / 60)}m`;
        if (seconds < 86400) return `${Math.floor(seconds / 3600)}h ${Math.floor((seconds % 3600) / 60)}m`;
        return `${Math.floor(seconds / 86400)}d`;
    }

    /**
     * Show LiveLink detail page.
     */
    showLiveLinkDetail() {
        this.showPage('livelink-detail');
        window.location.hash = 'livelink';
    }

    /**
     * Load LiveLink detail page data.
     */
    async loadLiveLinkDetailData() {
        // Subscribe to blend shapes for live visualization
        if (window.liveLinkManager) {
            await window.liveLinkManager.subscribe();
        }
    }

    /**
     * Load logs page data. Hydrates the server-source ring from the
     * recent server-side activity buffer (so the page isn't empty on
     * first open), renders the source list, and selects whatever
     * source was last open (default: 'server').
     */
    async loadLogsData() {
        const ws = window.saintWS;
        if (!ws.connected) return;

        // Pull recent server-side activity events into the local
        // activityLog ring. Existing connections keep getting new
        // events via the 'activity' WebSocket message — this just
        // backfills history on first open.
        try {
            const result = await ws.management('get_logs', { limit: 200 });
            const logs = result?.logs || [];
            const existingTimes = new Set(this.activityLog.map(e => e.time));
            for (const log of logs) {
                if (!existingTimes.has(log.time)) this.activityLog.push(log);
            }
            this.activityLog.sort((a, b) => b.time - a.time);
            if (this.activityLog.length > 500) {
                this.activityLog = this.activityLog.slice(0, 500);
            }
        } catch (error) {
            console.error('Failed to load server logs:', error);
        }

        // Sync the level dropdown with the current server-side setting
        // so the UI reflects reality (e.g. a config file edit before
        // first open).
        try {
            const settings = await ws.management('get_settings', {});
            const level = settings?.logging?.level || 'WARNING';
            const sel = document.getElementById('logs-level-select');
            if (sel) sel.value = level;
        } catch (_) { /* non-fatal */ }

        this.renderLogsSourceList();
        await this.selectLogsSource(this.logsSelectedSource || 'server');
    }

    /**
     * Render the left rail with "Server" + each adopted node.
     */
    renderLogsSourceList() {
        const list = document.getElementById('logs-source-list');
        if (!list) return;

        const items = [];
        items.push({
            key: 'server',
            label: 'Server',
            icon: 'dns',
            meta: '',
        });
        const adopted = (this.nodes && this.nodes.adopted) || [];
        for (const node of adopted) {
            const id = node.node_id;
            if (!id) continue;
            items.push({
                key: `node:${id}`,
                label: node.display_name || id,
                icon: 'memory',
                meta: id.length > 14 ? id.slice(0, 14) + '…' : id,
            });
        }

        list.innerHTML = items.map(item => {
            const active = (item.key === this.logsSelectedSource) ? ' active' : '';
            return `<div class="logs-source-item${active}" data-source="${escapeHtml(item.key)}">` +
                   `<span class="material-icons">${item.icon}</span>` +
                   `<span class="truncate">${escapeHtml(item.label)}</span>` +
                   (item.meta ? `<span class="logs-source-meta">${escapeHtml(item.meta)}</span>` : '') +
                   `</div>`;
        }).join('');

        list.querySelectorAll('.logs-source-item').forEach(el => {
            el.addEventListener('click', () => {
                const key = el.dataset.source;
                if (key) this.selectLogsSource(key);
            });
        });
    }

    /**
     * Switch the right pane to a specific source. For node sources we
     * subscribe to node_log/<id> + fetch history; for 'server' the
     * rows come straight from `this.activityLog`.
     */
    async selectLogsSource(key) {
        if (!key) return;
        const previous = this.logsSelectedSource;
        this.logsSelectedSource = key;
        this.logsExpandedKey = null;

        // Update the left-rail highlight without a full re-render.
        document.querySelectorAll('.logs-source-item').forEach(el => {
            el.classList.toggle('active', el.dataset.source === key);
        });

        // Header text reflects the selection.
        const titleEl = document.getElementById('logs-source-title');
        const subEl = document.getElementById('logs-source-subtitle');

        // Tear down any previous node subscription before subscribing
        // to a new one — we only stream from one node at a time on
        // this page to keep noise contained.
        if (this.logsNodeSubscription && this.logsNodeSubscription !== key) {
            const oldId = this.logsNodeSubscription.slice('node:'.length);
            try {
                await window.saintWS.unsubscribe([`node_log/${oldId}`]);
            } catch (_) { /* ignore */ }
            this.logsNodeSubscription = null;
        }

        if (key === 'server') {
            if (titleEl) titleEl.textContent = 'Server';
            if (subEl) subEl.textContent = 'Activity events from the server process.';
            this.renderLogsPage();
            return;
        }

        if (!key.startsWith('node:')) return;
        const nodeId = key.slice('node:'.length);
        if (titleEl) titleEl.textContent = nodeId;
        if (subEl) subEl.textContent = 'Per-node events streamed from this node.';

        try {
            await window.saintWS.subscribe([`node_log/${nodeId}`]);
            this.logsNodeSubscription = key;
            const result = await window.saintWS.management('get_node_logs', { node_id: nodeId });
            const entries = (result && result.entries) || [];
            // Store newest-first to match server-source ring layout.
            this.logsPerNode.set(nodeId, [...entries].reverse());
            this.renderLogsPage();
        } catch (err) {
            console.error('Failed to load node logs:', err);
            const rows = document.getElementById('logs-rows');
            if (rows) rows.innerHTML = `<p class="logs-empty">Failed to load: ${escapeHtml(err.message || err)}</p>`;
        }
    }

    /**
     * Receiver for live node_log/<id> events. Called from the
     * WebSocket 'state' dispatch path in handleStateUpdate.
     *
     * Critical: this MUST NOT trigger a full renderLogsPage on every
     * frame — that would rebuild the entire DOM, destroying the
     * user's scroll position and any expanded-row state. Incremental
     * prepend instead, with scroll compensation so a busy stream
     * doesn't push the row the user is reading down the page.
     */
    onNodeLogEntry(nodeId, entry) {
        if (!nodeId || !entry) return;
        const ring = this.logsPerNode.get(nodeId) || [];
        ring.unshift(entry);
        if (ring.length > 500) ring.length = 500;
        this.logsPerNode.set(nodeId, ring);
        if (this.currentPage === 'logs'
            && this.logsSelectedSource === `node:${nodeId}`) {
            this._prependLogRow(entry, ring.length - 1);
        }
    }

    /**
     * Filter state derived from the toolbar — shared by full-render
     * and incremental-prepend so a streaming entry is held to the
     * same predicate as one drawn from history.
     */
    _logFiltersAndRank() {
        const rowFilter = document.getElementById('logs-row-filter')?.value || 'all';
        const search = (document.getElementById('logs-search')?.value || '').toLowerCase();
        const LEVEL_RANK = { debug: 10, info: 20, warn: 30, warning: 30, error: 40, fatal: 50 };
        let threshold = 0;
        if (rowFilter === 'debug') threshold = LEVEL_RANK.debug;
        else if (rowFilter === 'info') threshold = LEVEL_RANK.info;
        else if (rowFilter === 'warn') threshold = LEVEL_RANK.warn;
        else if (rowFilter === 'error') threshold = LEVEL_RANK.error;
        return { rowFilter, search, threshold, LEVEL_RANK };
    }

    _logEntryPassesFilter(entry, ctx) {
        const rank = ctx.LEVEL_RANK[(entry.level || 'info').toLowerCase()] || 20;
        if (ctx.threshold > 0) {
            if (ctx.rowFilter === 'error') {
                if (rank < ctx.LEVEL_RANK.error) return false;
            } else if (rank < ctx.threshold) {
                return false;
            }
        }
        if (ctx.search) {
            const hay = (entry.text || '').toLowerCase()
                + ' ' + (entry.peripheral || '').toLowerCase();
            if (!hay.includes(ctx.search)) return false;
        }
        return true;
    }

    _fallbackLogSource() {
        return this.logsSelectedSource === 'server'
            ? 'saint_server'
            : this.logsSelectedSource.slice('node:'.length);
    }

    /**
     * Build the HTML for one row. `idx` is a stable index used to
     * derive a click key — kept distinct from array position so
     * subsequent prepends don't collide with an already-rendered key.
     */
    _logRowHtml(entry, idx, fallbackSource) {
        const t = entry.time ? new Date(entry.time * 1000) : new Date();
        const ts = t.toLocaleTimeString([], { hour12: false })
                   + '.' + String(t.getMilliseconds()).padStart(3, '0');
        const level = (entry.level || 'info').toLowerCase();
        const source = entry.peripheral || fallbackSource;
        const rowKey = `${entry.time || 'x'}:${idx}`;
        const expanded = (this.logsExpandedKey === rowKey) ? ' expanded' : '';
        const detail = (this.logsExpandedKey === rowKey)
            ? `<div class="logs-detail">${escapeHtml(JSON.stringify(entry, null, 2))}</div>`
            : '';
        return `<div class="logs-row ${level}${expanded}" data-row-key="${escapeHtml(rowKey)}">` +
               `<span class="logs-time">${ts}</span>` +
               `<span class="logs-source">${escapeHtml(source)}</span>` +
               `<span class="logs-summary">${escapeHtml(entry.text || '')}</span>` +
               detail +
               `</div>`;
    }

    /**
     * Single delegated click listener installed once on the container.
     * Avoids the previous pattern of re-binding per-row on every
     * render (which only worked because we were re-rendering — and
     * exactly that re-rendering was destroying scroll state). With
     * delegation, prepended rows pick up the handler for free.
     */
    _ensureLogsContainerListener(container) {
        if (container.dataset.logsListenerBound) return;
        container.dataset.logsListenerBound = 'true';
        container.addEventListener('click', (ev) => {
            const row = ev.target.closest('.logs-row');
            if (!row || !container.contains(row)) return;
            // Don't toggle when the user is selecting text inside the
            // expanded detail panel.
            if (ev.target.closest('.logs-detail')) return;
            const key = row.dataset.rowKey;
            if (!key) return;
            const wasExpanded = (this.logsExpandedKey === key);
            // Collapse any currently-open row in the DOM directly so
            // we don't have to nuke the whole list.
            if (this.logsExpandedKey && this.logsExpandedKey !== key) {
                const prev = container.querySelector(
                    `.logs-row.expanded[data-row-key="${CSS.escape(this.logsExpandedKey)}"]`);
                if (prev) {
                    prev.classList.remove('expanded');
                    const det = prev.querySelector('.logs-detail');
                    if (det) det.remove();
                }
            }
            if (wasExpanded) {
                row.classList.remove('expanded');
                const det = row.querySelector('.logs-detail');
                if (det) det.remove();
                this.logsExpandedKey = null;
            } else {
                row.classList.add('expanded');
                // Re-derive the entry's JSON from the dataset — we
                // stash it on the row at render time. Cheaper than
                // walking the model on every click.
                const raw = row.dataset.entryJson || '{}';
                row.insertAdjacentHTML('beforeend',
                    `<div class="logs-detail">${escapeHtml(
                        JSON.stringify(JSON.parse(raw), null, 2))}</div>`);
                this.logsExpandedKey = key;
            }
        });
    }

    /**
     * Insert one newly-arrived row at the top of the container. The
     * scroll-compensation block is the bit the user explicitly asked
     * for: if they're already scrolled away from the top to read
     * something, the new row must not push their view down. We do
     * this by measuring the row height after insert and bumping
     * scrollTop by that amount whenever the user wasn't at the very
     * top — so the visible content stays anchored.
     */
    _prependLogRow(entry, idx) {
        const container = document.getElementById('logs-rows');
        if (!container) return;
        if (!this._logEntryPassesFilter(entry, this._logFiltersAndRank())) {
            return;
        }
        // First message after the empty-state placeholder needs to
        // clear that placeholder before we insert.
        const placeholder = container.querySelector('.logs-empty');
        if (placeholder) placeholder.remove();

        this._ensureLogsContainerListener(container);

        const prevScrollTop = container.scrollTop;
        const wasAtTop = prevScrollTop <= 4;   // small tolerance
        const html = this._logRowHtml(entry, idx, this._fallbackLogSource());
        container.insertAdjacentHTML('afterbegin', html);
        const inserted = container.firstElementChild;
        if (inserted) {
            inserted.dataset.entryJson = JSON.stringify(entry);
        }
        if (!wasAtTop && inserted) {
            // Push scrollTop down by the inserted row's height so the
            // content the user is reading stays visually in place.
            container.scrollTop = prevScrollTop + inserted.offsetHeight;
        }
        // Cap the rendered DOM to a sensible size so long-running
        // sessions don't grow unbounded.
        while (container.childElementCount > 1000) {
            container.removeChild(container.lastElementChild);
        }
    }

    /**
     * Render the rows pane for the currently selected source. Called
     * on source-switch, filter change, and search change — NOT on
     * every incoming live event (that path is _prependLogRow above).
     */
    renderLogsPage() {
        const container = document.getElementById('logs-rows');
        if (!container) return;

        let entries = [];
        if (this.logsSelectedSource === 'server') {
            entries = this.activityLog;
        } else if (this.logsSelectedSource.startsWith('node:')) {
            const nodeId = this.logsSelectedSource.slice('node:'.length);
            entries = this.logsPerNode.get(nodeId) || [];
        }

        const ctx = this._logFiltersAndRank();
        const rows = entries.filter(e => this._logEntryPassesFilter(e, ctx));

        if (rows.length === 0) {
            container.innerHTML = '<p class="logs-empty">No logs match the current filters.</p>';
            return;
        }

        const fallbackSource = this._fallbackLogSource();
        container.innerHTML = rows.map((entry, idx) =>
            this._logRowHtml(entry, idx, fallbackSource)).join('');
        // Re-stash entry JSON on each row so click expansion can find it.
        container.querySelectorAll('.logs-row').forEach((row, i) => {
            row.dataset.entryJson = JSON.stringify(rows[i]);
        });
        this._ensureLogsContainerListener(container);
        // After a full render we reset to top so the freshest row is
        // visible. The incremental path is the one that preserves
        // mid-list scroll position.
        container.scrollTop = 0;
    }

    /**
     * Send the chosen global log level to the server. Persists to
     * config and applies live via apply_log_level().
     */
    async applyServerLogLevel(level) {
        if (!level) return;
        try {
            const ws = window.saintWS;
            await ws.management('set_settings', {
                settings: { logging: { level } }
            });
            this.addActivityLogEntry({
                text: `Log level set to ${level}`,
                level: 'info',
                timestamp: Date.now() / 1000,
            });
        } catch (err) {
            console.error('Failed to set log level:', err);
            alert(`Failed to set log level: ${err.message || err}`);
        }
    }

    /**
     * Clear the rows shown for the currently selected source. Server
     * source: clear the client-side activity ring only (the file log
     * is canonical history). Node source: also call clear_node_logs
     * on the server so the ring buffer drops the entries.
     */
    async clearLogsForCurrentSource() {
        if (this.logsSelectedSource === 'server') {
            this.activityLog = [];
            this.renderLogsPage();
            return;
        }
        if (!this.logsSelectedSource.startsWith('node:')) return;
        const nodeId = this.logsSelectedSource.slice('node:'.length);
        this.logsPerNode.set(nodeId, []);
        this.renderLogsPage();
        try {
            await window.saintWS.management('clear_node_logs', { node_id: nodeId });
        } catch (err) {
            console.error('clear_node_logs failed:', err);
        }
    }

    /**
     * Scan for new nodes.
     */
    async scanForNodes() {
        this.addActivityLogEntry({ text: 'Scanning for nodes...', level: 'info' });
        await this.loadNodesData();
    }

    /**
     * Latching emergency stop. First press engages (drives RoboClaw
     * estop_pins HIGH, sets motor duty to 0 on every adopted node, and
     * fires each peripheral driver's estop hook). Second press
     * releases the latch so motor commands flow again. The button's
     * visual state is driven by the server's broadcast on the 'estop'
     * topic — don't optimistically flip it here, otherwise two
     * dashboards racing would diverge from the canonical state.
     */
    async emergencyStop() {
        const ws = window.saintWS;
        // Toggle direction from the last broadcast we saw. If we
        // haven't received one yet (fresh page, server unreachable),
        // default to engage — the safer side of the unknown.
        const desired = this._estopActive ? 'release' : 'engage';
        try {
            const r = await ws.command('system', 'estop', {
                target: 'all', state: desired,
            });
            // The 'estop' state broadcast updates the button; here we
            // just log the user action and any server-side error.
            const detail = r?.data ? ` (${r.data.node_count} nodes)` : '';
            this.addActivityLogEntry({
                text: `Emergency stop ${desired}${detail}`,
                level: desired === 'engage' ? 'warn' : 'info',
            });
        } catch (error) {
            console.error('E-Stop failed:', error);
            this.addActivityLogEntry({ text: 'E-Stop command failed!', level: 'error' });
        }
    }

    /** Apply server-reported estop state to the button. Idempotent. */
    setEstopState(active) {
        this._estopActive = !!active;
        const btn = document.getElementById('btn-estop');
        const icon = document.getElementById('btn-estop-icon');
        const label = document.getElementById('btn-estop-label');
        if (!btn) return;
        btn.dataset.estopActive = this._estopActive ? 'true' : 'false';
        if (this._estopActive) {
            // Engaged: pulsing red, label flips to "Release". We
            // override btn-danger's hover/active styles with explicit
            // utility classes so the active state is unmistakable.
            btn.classList.add('ring-2', 'ring-red-400', 'animate-pulse');
            if (icon) icon.textContent = 'lock';
            if (label) label.textContent = 'E-Stop ENGAGED — click to release';
        } else {
            btn.classList.remove('ring-2', 'ring-red-400', 'animate-pulse');
            if (icon) icon.textContent = 'warning';
            if (label) label.textContent = 'E-Stop';
        }
    }

    /**
     * View node details.
     */
    viewNode(nodeId) {
        this.currentNodeId = nodeId;
        this.showPage('node-detail');
        window.location.hash = `node/${nodeId}`;
        this.addActivityLogEntry({ text: `Viewing node ${nodeId}`, level: 'info' });
    }

    /**
     * Reset a node (from nodes list).
     */
    async resetNode(nodeId) {
        if (!confirm(`Reset node ${nodeId}? This will return it to unadopted state.`)) {
            return;
        }

        const ws = window.saintWS;

        try {
            await ws.management('reset_node', { node_id: nodeId, factory_reset: true });
            this.addActivityLogEntry({ text: `Node ${nodeId} reset`, level: 'info' });
            await this.loadNodesData();
        } catch (error) {
            console.error('Reset failed:', error);
            this.addActivityLogEntry({ text: `Failed to reset ${nodeId}`, level: 'error' });
        }
    }

    /**
     * Restart current node.
     */
    async restartNode() {
        if (!this.currentNodeId) return;
        if (!confirm('Restart this node?')) return;

        const ws = window.saintWS;
        try {
            await ws.management('restart_node', { node_id: this.currentNodeId });
            this.addActivityLogEntry({ text: `Restarting node ${this.currentNodeId}`, level: 'info' });
        } catch (error) {
            console.error('Restart failed:', error);
            this.addActivityLogEntry({ text: `Failed to restart node`, level: 'error' });
        }
    }

    /**
     * Identify node (blink LED).
     */
    async identifyNode() {
        if (!this.currentNodeId) return;

        const ws = window.saintWS;
        try {
            await ws.management('identify_node', { node_id: this.currentNodeId });
            this.addActivityLogEntry({ text: `Identifying node ${this.currentNodeId}`, level: 'info' });
        } catch (error) {
            console.error('Identify failed:', error);
        }
    }

    /**
     * Emergency stop current node.
     */
    async estopNode() {
        if (!this.currentNodeId) return;

        const ws = window.saintWS;
        try {
            await ws.command(this.currentNodeId, 'estop', {});
            this.addActivityLogEntry({ text: `E-Stop sent to ${this.currentNodeId}`, level: 'warn' });
        } catch (error) {
            console.error('E-Stop failed:', error);
            this.addActivityLogEntry({ text: `E-Stop failed for ${this.currentNodeId}`, level: 'error' });
        }
    }

    /**
     * Factory reset current node.
     */
    async factoryResetNode() {
        if (!this.currentNodeId) return;
        if (!confirm('Factory reset this node? All configuration will be lost.')) return;

        const ws = window.saintWS;
        try {
            await ws.management('factory_reset_node', { node_id: this.currentNodeId });
            this.addActivityLogEntry({ text: `Factory reset ${this.currentNodeId}`, level: 'warn' });
        } catch (error) {
            console.error('Factory reset failed:', error);
            this.addActivityLogEntry({ text: `Factory reset failed`, level: 'error' });
        }
    }

    /**
     * Unadopt current node.
     */
    async unadoptNode() {
        if (!this.currentNodeId) return;
        if (!confirm('Unadopt this node? It will need to be re-adopted.')) return;

        const ws = window.saintWS;
        try {
            await ws.management('reset_node', { node_id: this.currentNodeId, factory_reset: false });
            this.addActivityLogEntry({ text: `Unadopted node ${this.currentNodeId}`, level: 'info' });
            this.showPage('nodes');
            window.location.hash = 'nodes';
        } catch (error) {
            console.error('Unadopt failed:', error);
            this.addActivityLogEntry({ text: `Failed to unadopt node`, level: 'error' });
        }
    }

    /**
     * Show/hide the "Update All" button on the Nodes page based on how many
     * adopted nodes have a newer firmware version available.
     */
    updateAllNodesButtonVisibility() {
        const button = document.getElementById('btn-update-all-nodes');
        const countEl = document.getElementById('update-all-count');
        if (!button) return;

        const updatable = (this.nodes?.adopted || []).filter(
            n => n.firmware_update_available === true && n.server_firmware_version
        );

        if (updatable.length === 0) {
            button.classList.add('hidden');
            return;
        }
        button.classList.remove('hidden');
        if (countEl) countEl.textContent = String(updatable.length);
    }

    /**
     * Update firmware on every adopted node that has a newer version
     * available. Skips nodes that are already up to date.
     */
    async updateAllNodes() {
        const updatable = (this.nodes?.adopted || []).filter(
            n => n.firmware_update_available === true && n.server_firmware_version
        );

        if (updatable.length === 0) {
            alert('All adopted nodes are already up to date.');
            return;
        }

        const lines = updatable.map(n =>
            `  • ${n.display_name || n.node_id}: ${n.firmware_version || '--'} → ${n.server_firmware_version}`
        ).join('\n');
        if (!confirm(
            `Update firmware on ${updatable.length} node${updatable.length === 1 ? '' : 's'}?\n\n` +
            `${lines}\n\n` +
            `Each node will restart during the update.`
        )) {
            return;
        }

        const ws = window.saintWS;
        const button = document.getElementById('btn-update-all-nodes');
        const textSpan = button?.querySelector('.update-all-text');
        const originalText = textSpan?.textContent;

        if (button) {
            button.disabled = true;
            button.classList.add('opacity-50', 'cursor-not-allowed');
        }

        let succeeded = 0;
        const failures = [];

        try {
            for (let i = 0; i < updatable.length; i++) {
                const node = updatable[i];
                if (textSpan) {
                    textSpan.textContent = `Updating ${i + 1}/${updatable.length}…`;
                }
                try {
                    await ws.management('update_firmware', { node_id: node.node_id });
                    succeeded++;
                    this.addActivityLogEntry({
                        text: `Firmware update initiated for ${node.display_name || node.node_id}: ${node.firmware_version} → ${node.server_firmware_version}`,
                        level: 'info',
                    });
                } catch (error) {
                    console.error(`Firmware update failed for ${node.node_id}:`, error);
                    failures.push({ node, error: error.message || String(error) });
                    this.addActivityLogEntry({
                        text: `Firmware update failed for ${node.display_name || node.node_id}: ${error.message || error}`,
                        level: 'error',
                    });
                }
            }
        } finally {
            if (button) {
                button.disabled = false;
                button.classList.remove('opacity-50', 'cursor-not-allowed');
            }
            if (textSpan && originalText) textSpan.textContent = originalText;
        }

        if (failures.length === 0) {
            alert(`Firmware update initiated on ${succeeded} node${succeeded === 1 ? '' : 's'}.\nEach node will restart and load the new firmware.`);
        } else {
            const failLines = failures.map(f => `  • ${f.node.display_name || f.node.node_id}: ${f.error}`).join('\n');
            alert(
                `Firmware update initiated on ${succeeded} of ${updatable.length} node${updatable.length === 1 ? '' : 's'}.\n\n` +
                `Failed:\n${failLines}`
            );
        }
    }

    /**
     * Update firmware on current node.
     */
    async updateFirmware() {
        if (!this.currentNodeId) return;

        const info = this.currentNodeInfo;
        if (!info) return;

        const serverVersion = info.server_firmware_version;
        const currentVersion = info.firmware_version;

        if (!confirm(
            `Update firmware on ${info.display_name || this.currentNodeId}?\n\n` +
            `Current version: ${currentVersion}\n` +
            `New version: ${serverVersion}\n\n` +
            `The node will restart during the update.`
        )) {
            return;
        }

        const ws = window.saintWS;
        const button = document.getElementById('btn-firmware-update');

        try {
            // Disable button and show updating state
            if (button) {
                button.disabled = true;
                button.classList.add('opacity-50', 'cursor-not-allowed');
                const textSpan = button.querySelector('.update-text');
                if (textSpan) {
                    textSpan.textContent = 'Updating...';
                }
            }

            const result = await ws.management('update_firmware', {
                node_id: this.currentNodeId
            });

            if (result.message) {
                this.addActivityLogEntry({
                    text: `Firmware update initiated: ${currentVersion} -> ${serverVersion}`,
                    level: 'info'
                });

                // Show success message
                alert(
                    `Firmware update initiated!\n\n` +
                    `The node will restart and load the new firmware.\n` +
                    `Version: ${currentVersion} -> ${serverVersion}`
                );

                // Hide the update button since update is in progress
                if (button) {
                    button.classList.add('hidden');
                }

                // Hide the badge
                const badge = document.getElementById('node-firmware-update-badge');
                if (badge) {
                    badge.classList.add('hidden');
                }
            }
        } catch (error) {
            console.error('Firmware update failed:', error);
            this.addActivityLogEntry({
                text: `Firmware update failed: ${error.message}`,
                level: 'error'
            });
            alert(`Firmware update failed: ${error.message}`);
        } finally {
            // Re-enable button
            if (button) {
                button.disabled = false;
                button.classList.remove('opacity-50', 'cursor-not-allowed');
                const textSpan = button.querySelector('.update-text');
                if (textSpan) {
                    textSpan.textContent = 'Update Firmware';
                }
            }
        }
    }

    /**
     * Show firmware update modal with build options.
     */
    async showFirmwareModal() {
        const modal = document.getElementById('firmware-update-modal');
        if (!modal) return;

        // Load firmware info for both build types
        const ws = window.saintWS;
        try {
            const result = await ws.management('get_firmware_builds', {});

            const simInfo = document.getElementById('fw-sim-info');
            const hwInfo = document.getElementById('fw-hw-info');
            const simBtn = document.getElementById('btn-fw-simulation');
            const hwBtn = document.getElementById('btn-fw-hardware');

            if (result.simulation?.available) {
                simInfo.textContent = `${result.simulation.version_full || result.simulation.version} - Built: ${result.simulation.build_date || 'unknown'}`;
                simBtn.disabled = false;
                simBtn.classList.remove('opacity-50');
            } else {
                simInfo.textContent = 'Not found - build with cmake -DSIMULATION=ON';
                simBtn.disabled = true;
                simBtn.classList.add('opacity-50');
            }

            if (result.hardware?.available) {
                hwInfo.textContent = `${result.hardware.version_full || result.hardware.version} - Built: ${result.hardware.build_date || 'unknown'}`;
                hwBtn.disabled = false;
                hwBtn.classList.remove('opacity-50');
            } else {
                hwInfo.textContent = 'Not found - build with cmake (no SIMULATION flag)';
                hwBtn.disabled = true;
                hwBtn.classList.add('opacity-50');
            }
        } catch (error) {
            console.error('Failed to load firmware info:', error);
        }

        modal.classList.remove('hidden');
    }

    /**
     * Close firmware update modal.
     */
    closeFirmwareModal() {
        const modal = document.getElementById('firmware-update-modal');
        if (modal) {
            modal.classList.add('hidden');
        }
    }

    /**
     * Force firmware update with specific build type.
     */
    async forceFirmwareUpdate(buildType) {
        if (!this.currentNodeId) return;

        const buildName = buildType === 'simulation' ? 'Simulation' : 'Hardware';
        if (!confirm(`Update firmware on this node with ${buildName} build?\n\nThe node will restart during the update.`)) {
            return;
        }

        this.closeFirmwareModal();

        const ws = window.saintWS;
        try {
            const result = await ws.management('force_firmware_update', {
                node_id: this.currentNodeId,
                build_type: buildType
            });

            if (result.message || result.success) {
                this.addActivityLogEntry({
                    text: `Firmware update initiated (${buildName} build)`,
                    level: 'info'
                });
                alert(`Firmware update initiated!\n\nThe node will restart and load the ${buildName} firmware.`);
            }
        } catch (error) {
            console.error('Firmware update failed:', error);
            this.addActivityLogEntry({
                text: `Firmware update failed: ${error.message}`,
                level: 'error'
            });
            alert(`Firmware update failed: ${error.message}`);
        }
    }

    /**
     * Remove a node completely from the server.
     */
    async removeNode(nodeId) {
        if (!confirm(`Remove node ${nodeId}?\n\nThis will remove the node from the server. If the node is still running, it will reappear when it announces itself.`)) {
            return;
        }

        const ws = window.saintWS;

        try {
            await ws.management('remove_node', { node_id: nodeId });
            this.addActivityLogEntry({ text: `Node ${nodeId} removed`, level: 'info' });
            await this.loadNodesData();
        } catch (error) {
            console.error('Remove failed:', error);
            this.addActivityLogEntry({ text: `Failed to remove ${nodeId}`, level: 'error' });
        }
    }

    /**
     * Show the adoption modal for a node.
     */
    async adoptNode(nodeId) {
        this._adoptingNodeId = nodeId;

        // Show modal
        const modal = document.getElementById('adopt-modal');
        modal.classList.remove('hidden');

        // Set node ID
        document.getElementById('adopt-node-id').textContent = nodeId;

        // Clear previous values
        document.getElementById('adopt-display-name').value = '';
        document.getElementById('adopt-role-description').textContent = '';

        // The chip + board pickers are populated from the server's
        // YAML catalog. The chip defaults to whatever the firmware
        // announced (when that maps to a known chip); otherwise the
        // operator picks. The board list is filtered by the current
        // chip selection and refreshes whenever the chip changes.
        const node = this.nodes.unadopted.find(n => n.node_id === nodeId);
        const announcedChip = node?.chip_family || '';

        const ws = window.saintWS;
        const chipSelect = document.getElementById('adopt-chip-select');
        const chipHint = document.getElementById('adopt-chip-detected');
        const boardSelect = document.getElementById('adopt-board-select');

        const repopulateBoards = async (chipFamily) => {
            try {
                const boardResult = await ws.management(
                    'list_boards', chipFamily ? { chip_family: chipFamily } : {}
                );
                const boards = boardResult?.boards || [];
                boardSelect.innerHTML = '<option value="">-- Select Board --</option>';
                for (const b of boards) {
                    const opt = document.createElement('option');
                    opt.value = b.board_id;
                    opt.textContent = `${b.display_name}${b.builtin ? '' : '  (custom)'}`;
                    boardSelect.appendChild(opt);
                }
                // Auto-select the only built-in match if there's exactly one.
                const builtins = boards.filter(b => b.builtin);
                if (builtins.length === 1) {
                    boardSelect.value = builtins[0].board_id;
                }
            } catch (e) {
                console.warn('list_boards failed:', e);
            }
        };

        try {
            const chipResult = await ws.management('list_chips', {});
            const chips = chipResult?.chips || [];
            chipSelect.innerHTML = '<option value="">-- Select Chip --</option>';
            for (const c of chips) {
                const opt = document.createElement('option');
                opt.value = c.chip_family;
                opt.textContent = `${c.display_name} (${c.chip_family})`;
                chipSelect.appendChild(opt);
            }

            // Default selection: announced chip if it's a recognized family.
            const announcedIsKnown = chips.some(c => c.chip_family === announcedChip);
            if (announcedIsKnown) {
                chipSelect.value = announcedChip;
                if (chipHint) {
                    chipHint.textContent = `Detected from firmware: ${announcedChip}`;
                    chipHint.classList.remove('text-amber-400');
                    chipHint.classList.add('text-slate-500');
                }
            } else {
                chipSelect.value = '';
                if (chipHint) {
                    chipHint.textContent = announcedChip
                        ? `Firmware reported '${announcedChip}' (not a known chip — please choose).`
                        : 'Firmware did not report a chip — please choose.';
                    chipHint.classList.remove('text-slate-500');
                    chipHint.classList.add('text-amber-400');
                }
            }
        } catch (e) {
            console.warn('list_chips failed:', e);
            chipSelect.innerHTML = '<option value="">-- (failed to load chips) --</option>';
        }

        // Wire up the chip → board cascade and load the initial board list.
        chipSelect.onchange = () => repopulateBoards(chipSelect.value);
        await repopulateBoards(chipSelect.value);

        const roleSelect = document.getElementById('adopt-role-select');

        try {
            const result = await ws.management('get_roles');
            const roles = result.roles || [];

            // Store roles for description lookup
            this._availableRoles = roles;

            // Populate dropdown
            roleSelect.innerHTML = '<option value="">-- Select Role --</option>';
            for (const role of roles) {
                const option = document.createElement('option');
                option.value = role.role;
                option.textContent = role.display_name || role.role;
                roleSelect.appendChild(option);
            }

            // Add change listener to show role description
            roleSelect.onchange = () => {
                const selectedRole = roles.find(r => r.role === roleSelect.value);
                const descEl = document.getElementById('adopt-role-description');
                if (selectedRole && selectedRole.description) {
                    descEl.textContent = selectedRole.description;
                } else {
                    descEl.textContent = '';
                }
            };
        } catch (error) {
            console.error('Failed to fetch roles:', error);
            // Fallback to hardcoded roles
            roleSelect.innerHTML = `
                <option value="">-- Select Role --</option>
                <option value="head">head</option>
                <option value="arms">arms</option>
                <option value="tracks">tracks</option>
                <option value="console">console</option>
            `;
        }
    }

    /**
     * Open the Edit Node modal pre-filled with the current node's
     * role / board / chip / display name. Mirrors adoptNode's
     * dropdown-loading pattern but doesn't move the node between
     * adopted/unadopted state — it just edits in-place via the
     * update_node management action.
     */
    async openEditNodeModal() {
        if (!this.currentNodeId || !this.currentNodeInfo) return;

        const node = this.currentNodeInfo;
        const ws = window.saintWS;
        const modal = document.getElementById('edit-node-modal');
        modal.classList.remove('hidden');

        document.getElementById('edit-node-id').textContent = node.node_id;
        document.getElementById('edit-node-display-name').value =
            node.display_name || '';

        const chipSelect = document.getElementById('edit-node-chip-select');
        const boardSelect = document.getElementById('edit-node-board-select');
        const roleSelect = document.getElementById('edit-node-role-select');
        const roleDesc = document.getElementById('edit-node-role-description');

        // Cascade: when the operator changes chip, re-populate board
        // dropdown filtered to that chip's boards. Identical pattern
        // to the adopt modal.
        const repopulateBoards = async (chipFamily, preselect) => {
            try {
                const boardResult = await ws.management(
                    'list_boards', chipFamily ? { chip_family: chipFamily } : {}
                );
                const boards = boardResult?.boards || [];
                boardSelect.innerHTML = '<option value="">-- Select Board --</option>';
                for (const b of boards) {
                    const opt = document.createElement('option');
                    opt.value = b.board_id;
                    opt.textContent = `${b.display_name}${b.builtin ? '' : '  (custom)'}`;
                    boardSelect.appendChild(opt);
                }
                if (preselect && boards.some(b => b.board_id === preselect)) {
                    boardSelect.value = preselect;
                }
            } catch (e) {
                console.warn('list_boards failed:', e);
            }
        };

        try {
            const chipResult = await ws.management('list_chips', {});
            const chips = chipResult?.chips || [];
            chipSelect.innerHTML = '<option value="">-- Select Chip --</option>';
            for (const c of chips) {
                const opt = document.createElement('option');
                opt.value = c.chip_family;
                opt.textContent = `${c.display_name} (${c.chip_family})`;
                chipSelect.appendChild(opt);
            }
            chipSelect.value = node.chip_family || '';
        } catch (e) {
            console.warn('list_chips failed:', e);
            chipSelect.innerHTML = '<option value="">-- (failed to load chips) --</option>';
        }

        // Wire the cascade AND pre-select the current board.
        chipSelect.onchange = () => repopulateBoards(chipSelect.value, null);
        await repopulateBoards(chipSelect.value, node.board_id);

        // Role dropdown — same loader as adopt modal.
        try {
            const result = await ws.management('get_roles');
            const roles = result.roles || [];
            roleSelect.innerHTML = '<option value="">-- Select Role --</option>';
            for (const role of roles) {
                const option = document.createElement('option');
                option.value = role.role;
                option.textContent = role.display_name || role.role;
                roleSelect.appendChild(option);
            }
            roleSelect.value = node.role || '';

            const renderDesc = () => {
                const selected = roles.find(r => r.role === roleSelect.value);
                roleDesc.textContent = selected?.description || '';
            };
            roleSelect.onchange = renderDesc;
            renderDesc();
        } catch (error) {
            console.error('Failed to fetch roles:', error);
            roleSelect.innerHTML = `
                <option value="">-- Select Role --</option>
                <option value="head">head</option>
                <option value="arms">arms</option>
                <option value="tracks">tracks</option>
                <option value="console">console</option>
            `;
            roleSelect.value = node.role || '';
        }
    }

    closeEditNodeModal() {
        const modal = document.getElementById('edit-node-modal');
        modal.classList.add('hidden');
    }

    async submitNodeEdit() {
        if (!this.currentNodeId || !this.currentNodeInfo) return;

        const role = document.getElementById('edit-node-role-select').value;
        const chipFamily = document.getElementById('edit-node-chip-select').value;
        const boardId = document.getElementById('edit-node-board-select').value;
        const displayName = document.getElementById('edit-node-display-name').value.trim();

        if (!role) { alert('Please select a role'); return; }
        if (!chipFamily) { alert('Please select a chip'); return; }
        if (!boardId) { alert('Please select a board'); return; }

        const params = {
            node_id: this.currentNodeId,
            role,
            chip_family: chipFamily,
            board_id: boardId,
            // Always send display_name even when empty — that lets the
            // operator clear a previously-set name. The server's
            // update_node only mutates a field when the incoming value
            // differs from current, so this is a no-op in the unchanged
            // case.
            display_name: displayName,
        };

        const ws = window.saintWS;
        try {
            const result = await ws.management('update_node', params);
            if (result && result.success === false) {
                alert(result.message || 'Update failed');
                return;
            }
            this.addActivityLogEntry({
                text: `Node ${this.currentNodeId} updated`,
                level: 'info',
            });
            this.closeEditNodeModal();
            // Reload the detail page so the new values appear in the
            // info card AND any board-change re-seed of builtin
            // peripherals lands in the Peripherals tab.
            await this.loadNodeDetailData();
        } catch (error) {
            console.error('Update failed:', error);
            this.addActivityLogEntry({
                text: `Failed to update ${this.currentNodeId}`,
                level: 'error',
            });
        }
    }

    /**
     * Close the adoption modal.
     */
    closeAdoptModal() {
        const modal = document.getElementById('adopt-modal');
        modal.classList.add('hidden');
        this._adoptingNodeId = null;
    }

    /**
     * Submit the adoption form.
     */
    async submitAdoption() {
        const nodeId = this._adoptingNodeId;
        if (!nodeId) return;

        const role = document.getElementById('adopt-role-select').value;
        const displayName = document.getElementById('adopt-display-name').value.trim();
        const chipFamily = document.getElementById('adopt-chip-select').value;
        const boardId = document.getElementById('adopt-board-select').value;

        if (!role) { alert('Please select a role'); return; }
        if (!chipFamily) { alert('Please select a chip'); return; }
        if (!boardId) { alert('Please select a board'); return; }

        const ws = window.saintWS;

        try {
            const params = {
                node_id: nodeId,
                role,
                chip_family: chipFamily,
                board_id: boardId,
            };
            if (displayName) {
                params.display_name = displayName;
            }

            const result = await ws.management('adopt_node', params);
            if (result && result.success === false) {
                alert(result.message || 'Adoption failed');
                return;
            }
            this.addActivityLogEntry({
                text: `Node ${nodeId} adopted as ${role} (board ${boardId})`,
                level: 'info'
            });
            this.closeAdoptModal();
            await this.loadNodesData();
        } catch (error) {
            console.error('Adoption failed:', error);
            this.addActivityLogEntry({ text: `Failed to adopt ${nodeId}`, level: 'error' });
        }
    }

    /**
     * Filter logs by level.
     */
    filterLogs(level) {
        // TODO: Implement log filtering
        console.log('Filter logs by:', level);
    }

}

// Initialize app
const app = new SaintApp();
document.addEventListener('DOMContentLoaded', () => app.init());
