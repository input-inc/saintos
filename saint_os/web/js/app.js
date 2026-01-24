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
    }

    /**
     * Initialize the application.
     */
    init() {
        this.setupNavigation();
        this.setupWebSocket();
        this.setupEventListeners();

        // Show initial page based on URL hash
        const hash = window.location.hash.slice(1) || 'dashboard';
        this.showPage(hash);
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

        ws.on('connected', () => {
            this.updateConnectionStatus(true);
            this.requestInitialData();
        });

        ws.on('disconnected', () => {
            this.updateConnectionStatus(false);
        });

        ws.on('state', (message) => {
            this.handleStateUpdate(message);
        });

        ws.on('activity', (message) => {
            this.addActivityLogEntry(message);
        });

        // Connect
        ws.connect();
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
        document.getElementById('log-level')?.addEventListener('change', (e) => {
            this.filterLogs(e.target.value);
        });
    }

    /**
     * Show a page by ID.
     */
    showPage(pageId) {
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
        const ws = window.saintWS;
        if (!ws.connected) return;

        try {
            switch (pageId) {
                case 'dashboard':
                    await this.loadDashboardData();
                    break;
                case 'nodes':
                    await this.loadNodesData();
                    break;
                case 'routes':
                    await this.loadRoutesData();
                    break;
                case 'inputs':
                    await this.loadInputsData();
                    break;
                case 'logs':
                    await this.loadLogsData();
                    break;
            }
        } catch (error) {
            console.error(`Failed to load ${pageId} data:`, error);
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
    updateConnectionStatus(connected) {
        const statusDot = document.querySelector('.status-dot');
        const statusText = document.querySelector('.status-text');

        if (connected) {
            statusDot.classList.remove('bg-amber-500', 'bg-red-500', 'animate-pulse-dot');
            statusDot.classList.add('bg-emerald-500');
            statusText.textContent = 'Connected';
        } else {
            statusDot.classList.remove('bg-emerald-500', 'bg-amber-500');
            statusDot.classList.add('bg-red-500', 'animate-pulse-dot');
            statusText.textContent = 'Disconnected';
        }
    }

    /**
     * Handle state updates from server.
     */
    handleStateUpdate(message) {
        if (message.node === 'system') {
            this.systemStatus = message.data;
            this.updateSystemStatus();
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

        const time = new Date().toLocaleTimeString();
        entry.innerHTML = `<span class="text-slate-500">${time}</span> ${message.text || message.message}`;

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

        this.activityLog.unshift({ time, ...message });
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

        let html = `
            <div class="flex items-center justify-between">
                <div class="flex items-center gap-2">
                    <span class="w-2 h-2 rounded-full bg-emerald-500"></span>
                    <span class="text-slate-300">Adopted</span>
                </div>
                <span class="text-xl font-bold text-white">${adopted.length}</span>
            </div>
            <div class="flex items-center justify-between">
                <div class="flex items-center gap-2">
                    <span class="w-2 h-2 rounded-full bg-amber-500"></span>
                    <span class="text-slate-300">Unadopted</span>
                </div>
                <span class="text-xl font-bold text-white">${unadopted.length}</span>
            </div>
        `;

        if (adopted.length > 0) {
            html += '<div class="mt-4 pt-4 border-t border-slate-700 space-y-2">';
            for (const node of adopted.slice(0, 3)) {
                const statusColor = node.online ? 'bg-emerald-500' : 'bg-slate-500';
                html += `
                    <div class="flex items-center justify-between text-sm">
                        <div class="flex items-center gap-2">
                            <span class="w-2 h-2 rounded-full ${statusColor}"></span>
                            <span class="text-slate-200">${node.display_name || node.node_id}</span>
                        </div>
                        <span class="text-slate-400">${node.role}</span>
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
                    </div>
                </div>
            `).join('');
        }

        // Unadopted nodes
        const unadoptedContainer = document.getElementById('unadopted-nodes');
        if (this.nodes.unadopted.length === 0) {
            unadoptedContainer.innerHTML = '<p class="text-slate-400 col-span-full">No unadopted nodes found</p>';
        } else {
            unadoptedContainer.innerHTML = this.nodes.unadopted.map(node => `
                <div class="node-card border-amber-500/30">
                    <div class="flex items-start justify-between mb-3">
                        <div class="flex items-center gap-2">
                            <span class="w-2.5 h-2.5 rounded-full bg-amber-500 animate-pulse"></span>
                            <span class="font-medium text-white">${node.node_id}</span>
                        </div>
                        <span class="px-2 py-0.5 text-xs rounded-full bg-amber-500/20 text-amber-400">New</span>
                    </div>
                    <div class="text-sm text-slate-400 mb-3 space-y-1">
                        <p>${node.hardware_model || 'Unknown hardware'}</p>
                        <p class="font-mono text-xs">${node.ip_address || '--'}</p>
                    </div>
                    <div class="flex items-center gap-2 pt-3 border-t border-slate-700">
                        <button class="btn-sm bg-cyan-600 hover:bg-cyan-500 text-white flex-1" onclick="app.adoptNode('${node.node_id}')">
                            Adopt Node
                        </button>
                    </div>
                </div>
            `).join('');
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
        // TODO: Implement inputs loading
    }

    /**
     * Load logs page data.
     */
    async loadLogsData() {
        // TODO: Implement logs loading
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
        console.log('View node:', nodeId);
        this.addActivityLogEntry({ text: `Viewing node ${nodeId}`, level: 'info' });
        // TODO: Implement node detail view
    }

    /**
     * Reset a node.
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
     * Adopt a node.
     */
    async adoptNode(nodeId) {
        const roles = ['head', 'arms', 'tracks', 'console'];
        const role = prompt(`Enter role for node ${nodeId}:\n\nAvailable roles: ${roles.join(', ')}`);
        if (!role) return;

        if (!roles.includes(role)) {
            alert(`Invalid role. Please use one of: ${roles.join(', ')}`);
            return;
        }

        const ws = window.saintWS;

        try {
            await ws.management('adopt_node', { node_id: nodeId, role });
            this.addActivityLogEntry({ text: `Node ${nodeId} adopted as ${role}`, level: 'info' });
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
