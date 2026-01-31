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
        this.activityLog = [];
        this.currentNodeId = null;  // Currently viewed node
        this.currentNodeInfo = null;
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

        // Show initial page based on URL hash
        const hash = window.location.hash.slice(1) || 'dashboard';

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

        document.getElementById('btn-view-logs')?.addEventListener('click', () => {
            this.showPage('logs');
            window.location.hash = 'logs';
        });

        // Log filters
        document.getElementById('log-level')?.addEventListener('change', () => {
            this.renderLogsPage();
        });
        document.getElementById('log-search')?.addEventListener('input', () => {
            this.renderLogsPage();
        });

        // Node detail tab switching
        document.querySelectorAll('.node-tab').forEach(tab => {
            tab.addEventListener('click', (e) => {
                this.switchNodeTab(e.target.dataset.tab);
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
        // Load firmware build info
        try {
            const ws = window.saintWS;
            const result = await ws.management('get_firmware_builds', {});

            // Update simulation build info (RP2040)
            this.updateFirmwareBuildDisplay('sim', result.simulation);

            // Update hardware build info (RP2040)
            this.updateFirmwareBuildDisplay('hw', result.hardware);

            // Update Pi 5 firmware info
            this.updatePi5FirmwareDisplay(result.rpi5);
        } catch (error) {
            console.error('Failed to load firmware builds:', error);
        }
    }

    /**
     * Request initial data after connection.
     */
    async requestInitialData() {
        const ws = window.saintWS;

        try {
            // Subscribe to state updates
            await ws.subscribe(['system', 'nodes'], 1);

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
        }
    }

    /**
     * Update nodes-related UI elements based on current page.
     */
    updateNodesUI() {
        // Always update dashboard summary (it's in the sidebar)
        this.updateNodesSummary();

        // If on nodes page, refresh the list
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
     * Add entry to activity log.
     */
    addActivityLogEntry(message) {
        const logContainer = document.getElementById('activity-log');
        const entry = document.createElement('div');

        const level = message.level || 'info';
        entry.className = `log-entry ${level}`;

        // Use Unix timestamp (seconds) for consistency with server logs
        const timestamp = message.timestamp || (Date.now() / 1000);
        const timeStr = new Date(timestamp * 1000).toLocaleTimeString();
        entry.innerHTML = `<span class="text-slate-500">${timeStr}</span> ${message.text || message.message}`;

        // Remove "waiting" message if it exists
        const waiting = logContainer.querySelector('.text-slate-500:not(.log-entry span)');
        if (waiting && waiting.textContent.includes('Waiting')) {
            waiting.remove();
        }

        logContainer.insertBefore(entry, logContainer.firstChild);

        // Keep only last 100 entries
        while (logContainer.children.length > 100) {
            logContainer.removeChild(logContainer.lastChild);
        }

        // Add to activity log array with proper timestamp format
        this.activityLog.unshift({
            time: timestamp,
            text: message.text || message.message,
            level: level
        });

        // Trim activity log to reasonable size
        if (this.activityLog.length > 500) {
            this.activityLog = this.activityLog.slice(0, 500);
        }

        // Re-render logs page if currently viewing it
        if (this.currentPage === 'logs') {
            this.renderLogsPage();
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

            this.updateNodesSummary();
        } catch (error) {
            console.error('Failed to load dashboard data:', error);
        }
    }

    /**
     * Update nodes summary on dashboard.
     */
    updateNodesSummary() {
        const container = document.getElementById('nodes-summary');
        const adopted = this.nodes.adopted;
        const unadopted = this.nodes.unadopted;

        // Count online nodes
        const adoptedOnline = adopted.filter(n => n.online !== false).length;
        const unadoptedOnline = unadopted.filter(n => n.online !== false).length;

        let html = `
            <div class="flex items-center justify-between">
                <div class="flex items-center gap-2">
                    <span class="w-2 h-2 rounded-full bg-emerald-500"></span>
                    <span class="text-slate-300">Adopted</span>
                </div>
                <span class="text-xl font-bold text-white">${adoptedOnline}<span class="text-sm text-slate-500">/${adopted.length}</span></span>
            </div>
            <div class="flex items-center justify-between">
                <div class="flex items-center gap-2">
                    <span class="w-2 h-2 rounded-full bg-amber-500"></span>
                    <span class="text-slate-300">Unadopted</span>
                </div>
                <span class="text-xl font-bold text-white">${unadoptedOnline}<span class="text-sm text-slate-500">/${unadopted.length}</span></span>
            </div>
        `;

        // Show recent nodes (adopted first, then unadopted)
        const recentNodes = [...adopted.slice(0, 2), ...unadopted.filter(n => n.online !== false).slice(0, 2)];
        if (recentNodes.length > 0) {
            html += '<div class="mt-4 pt-4 border-t border-slate-700 space-y-2">';
            for (const node of recentNodes) {
                const isAdopted = node.role;
                const statusColor = node.online !== false
                    ? (isAdopted ? 'bg-emerald-500' : 'bg-amber-500')
                    : 'bg-slate-500';
                const label = isAdopted
                    ? (node.display_name || node.node_id)
                    : node.node_id;
                const badge = isAdopted
                    ? `<span class="text-slate-400">${node.role}</span>`
                    : '<span class="text-amber-400 text-xs">new</span>';
                html += `
                    <div class="flex items-center justify-between text-sm">
                        <div class="flex items-center gap-2">
                            <span class="w-2 h-2 rounded-full ${statusColor}"></span>
                            <span class="text-slate-200 truncate max-w-[150px]">${label}</span>
                        </div>
                        ${badge}
                    </div>
                `;
            }
            html += '</div>';
        }

        container.innerHTML = html;
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
            adoptedContainer.innerHTML = this.nodes.adopted.map(node => `
                <div class="node-card">
                    <div class="flex items-start justify-between mb-3">
                        <div class="flex items-center gap-2">
                            <span class="w-2.5 h-2.5 rounded-full ${node.online ? 'bg-emerald-500' : 'bg-slate-500'}"></span>
                            <span class="font-medium text-white">${node.display_name || node.node_id}</span>
                        </div>
                        <span class="px-2 py-0.5 text-xs rounded-full bg-cyan-500/20 text-cyan-400">${node.role}</span>
                    </div>
                    <div class="text-sm text-slate-400 mb-3">
                        <p>${node.hardware_model || 'Unknown hardware'}</p>
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
            `).join('');
        }

        // Unadopted nodes
        const unadoptedContainer = document.getElementById('unadopted-nodes');
        if (this.nodes.unadopted.length === 0) {
            unadoptedContainer.innerHTML = '<p class="text-slate-400 col-span-full">No unadopted nodes found</p>';
        } else {
            unadoptedContainer.innerHTML = this.nodes.unadopted.map(node => {
                const isOnline = node.online !== false;
                const statusDot = isOnline
                    ? 'bg-amber-500 animate-pulse'
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

            // Load pin config data
            if (typeof pinConfigManager !== 'undefined') {
                await pinConfigManager.loadNodeData(this.currentNodeId, this.currentNodeInfo);
            }
        } catch (error) {
            console.error('Failed to load node detail:', error);
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
        document.getElementById('node-info-ip').textContent = info.ip_address || '--';
        document.getElementById('node-info-uptime').textContent =
            info.uptime_seconds ? this.formatUptime(info.uptime_seconds) : '--';

        document.getElementById('node-stat-temp').textContent =
            info.cpu_temp ? `${info.cpu_temp.toFixed(1)}°C` : '--';
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
    }

    /**
     * Load pin controls for the State tab.
     */
    async loadStateTabControls() {
        console.log('Loading State tab controls for node:', this.currentNodeId);

        if (!this.currentNodeId || !this.currentNodeInfo) {
            console.warn('No current node selected');
            return;
        }

        // Set node for pin control manager
        if (typeof pinControlManager !== 'undefined') {
            await pinControlManager.setNode(this.currentNodeId);

            // Get configured pins from node info
            const ws = window.saintWS;
            try {
                console.log('Fetching pin config for', this.currentNodeId);
                const configResult = await ws.management('get_pin_config', {
                    node_id: this.currentNodeId
                });
                console.log('Pin config result:', configResult);

                // Convert config to pins array
                const pins = [];
                if (configResult?.pins) {
                    for (const [gpio, config] of Object.entries(configResult.pins)) {
                        if (config.mode && config.mode !== 'unconfigured') {
                            pins.push({
                                gpio: parseInt(gpio),
                                mode: config.mode,
                                logical_name: config.logical_name || ''
                            });
                        }
                    }
                }
                console.log('Extracted pins:', pins);

                // Render controls
                const container = document.getElementById('pin-controls-container');
                pinControlManager.renderControls(container, pins);

                // Update summary
                document.getElementById('state-node-status').textContent =
                    this.currentNodeInfo.online ? 'Online' : 'Offline';
                document.getElementById('state-pin-count').textContent =
                    pins.length.toString();

                // Setup quick action buttons
                this.setupQuickActionButtons(pins);
            } catch (e) {
                console.error('Failed to load pin config:', e);
                const container = document.getElementById('pin-controls-container');
                if (container) {
                    container.innerHTML = `
                        <div class="p-4 bg-red-900/20 border border-red-800/50 rounded-lg">
                            <p class="text-red-400">Failed to load pin configuration: ${e.message}</p>
                        </div>
                    `;
                }
            }
        } else {
            console.warn('pinControlManager not defined');
        }
    }

    /**
     * Setup quick action buttons for State tab.
     */
    setupQuickActionButtons(pins) {
        const pwmPins = pins.filter(p => p.mode === 'pwm');
        const servoPins = pins.filter(p => p.mode === 'servo');
        const digitalOutPins = pins.filter(p => p.mode === 'digital_out');

        // All PWM to 0%
        const pwmZeroBtn = document.getElementById('btn-all-pwm-zero');
        if (pwmZeroBtn) {
            pwmZeroBtn.onclick = async () => {
                for (const pin of pwmPins) {
                    await pinControlManager.sendControlValue(pin.gpio, 0);
                }
            };
        }

        // All Servos to 90 degrees
        const servoCenterBtn = document.getElementById('btn-all-servo-center');
        if (servoCenterBtn) {
            servoCenterBtn.onclick = async () => {
                for (const pin of servoPins) {
                    await pinControlManager.sendControlValue(pin.gpio, 90);
                }
            };
        }

        // All Digital OFF
        const digitalOffBtn = document.getElementById('btn-all-digital-off');
        if (digitalOffBtn) {
            digitalOffBtn.onclick = async () => {
                for (const pin of digitalOutPins) {
                    await pinControlManager.sendControlValue(pin.gpio, 0);
                }
            };
        }
    }

    /**
     * Load routes page data.
     */
    async loadRoutesData() {
        // TODO: Implement routes loading
    }

    /**
     * Load inputs page data.
     */
    async loadInputsData() {
        // Subscribe to LiveLink status updates
        const ws = window.saintWS;
        if (ws && ws.connected) {
            await ws.subscribe(['livelink']);
        }
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
     * Load logs page data.
     */
    async loadLogsData() {
        const ws = window.saintWS;
        if (!ws.connected) return;

        try {
            const result = await ws.management('get_logs', { limit: 200 });
            const logs = result?.logs || [];

            // Merge with local activity log (remove duplicates by time)
            const existingTimes = new Set(this.activityLog.map(e => e.time));
            for (const log of logs) {
                if (!existingTimes.has(log.time)) {
                    this.activityLog.push(log);
                }
            }

            // Sort by time (newest first)
            this.activityLog.sort((a, b) => b.time - a.time);

            // Trim to reasonable size
            if (this.activityLog.length > 500) {
                this.activityLog = this.activityLog.slice(0, 500);
            }

            // Render logs
            this.renderLogsPage();
        } catch (error) {
            console.error('Failed to load logs:', error);
        }
    }

    /**
     * Render the logs page.
     */
    renderLogsPage() {
        const container = document.getElementById('log-entries');
        if (!container) return;

        const levelFilter = document.getElementById('log-level')?.value || 'all';
        const searchFilter = document.getElementById('log-search')?.value?.toLowerCase() || '';

        let logs = this.activityLog;

        // Apply level filter
        if (levelFilter && levelFilter !== 'all') {
            logs = logs.filter(log => log.level === levelFilter);
        }

        // Apply search filter
        if (searchFilter) {
            logs = logs.filter(log => log.text?.toLowerCase().includes(searchFilter));
        }

        if (logs.length === 0) {
            container.innerHTML = '<p class="text-slate-500">No logs matching filters.</p>';
            return;
        }

        container.innerHTML = logs.map(log => {
            const time = new Date(log.time * 1000).toLocaleTimeString();
            const level = log.level || 'info';
            return `<div class="log-entry ${level}">[${time}] ${log.text || ''}</div>`;
        }).join('');
    }

    /**
     * Scan for new nodes.
     */
    async scanForNodes() {
        this.addActivityLogEntry({ text: 'Scanning for nodes...', level: 'info' });
        await this.loadNodesData();
    }

    /**
     * Emergency stop all motion.
     */
    async emergencyStop() {
        const ws = window.saintWS;

        try {
            await ws.command('system', 'estop', { target: 'all' });
            this.addActivityLogEntry({ text: 'Emergency stop activated', level: 'warn' });
        } catch (error) {
            console.error('E-Stop failed:', error);
            this.addActivityLogEntry({ text: 'E-Stop failed!', level: 'error' });
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

        // Fetch roles from server and populate dropdown
        const ws = window.saintWS;
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

        if (!role) {
            alert('Please select a role');
            return;
        }

        const ws = window.saintWS;

        try {
            const params = { node_id: nodeId, role };
            if (displayName) {
                params.display_name = displayName;
            }

            await ws.management('adopt_node', params);
            this.addActivityLogEntry({ text: `Node ${nodeId} adopted as ${role}`, level: 'info' });
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
