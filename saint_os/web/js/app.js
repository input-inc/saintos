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
            link.classList.toggle('active', link.dataset.page === pageId);
        });

        // Show page
        document.querySelectorAll('.page').forEach(page => {
            page.classList.toggle('active', page.id === `page-${pageId}`);
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
            statusDot.classList.add('online');
            statusDot.classList.remove('offline');
            statusText.textContent = 'Connected';
        } else {
            statusDot.classList.remove('online');
            statusDot.classList.add('offline');
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

        document.getElementById('server-status').textContent =
            status.server_online ? 'Online' : 'Offline';
        document.getElementById('server-uptime').textContent =
            this.formatUptime(status.uptime_seconds);
        document.getElementById('server-cpu').textContent =
            `${status.cpu_usage?.toFixed(1) || '--'}%`;
        document.getElementById('server-memory').textContent =
            `${status.memory_usage?.toFixed(1) || '--'}%`;
        document.getElementById('server-name').textContent =
            status.server_name || '--';
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
        const entry = document.createElement('p');
        entry.className = `log-entry ${message.level || 'info'}`;

        const time = new Date().toLocaleTimeString();
        entry.textContent = `${time} ${message.text || message.message}`;

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

        let html = `<p><strong>Adopted:</strong> ${adopted.length}</p>`;
        html += `<p><strong>Unadopted:</strong> ${unadopted.length}</p>`;

        if (adopted.length > 0) {
            html += '<ul>';
            for (const node of adopted.slice(0, 5)) {
                const status = node.online ? '●' : '○';
                html += `<li>${status} ${node.display_name || node.node_id} (${node.role})</li>`;
            }
            html += '</ul>';
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
            adoptedContainer.innerHTML = '<p>No adopted nodes</p>';
        } else {
            adoptedContainer.innerHTML = this.nodes.adopted.map(node => `
                <div class="node-item">
                    <div class="node-status">
                        <span class="status-dot ${node.online ? 'online' : 'offline'}"></span>
                        <strong>${node.display_name || node.node_id}</strong>
                        <span>(${node.role})</span>
                    </div>
                    <div>
                        <button class="btn" onclick="app.viewNode('${node.node_id}')">View</button>
                        <button class="btn" onclick="app.resetNode('${node.node_id}')">Reset</button>
                    </div>
                </div>
            `).join('');
        }

        // Unadopted nodes
        const unadoptedContainer = document.getElementById('unadopted-nodes');
        if (this.nodes.unadopted.length === 0) {
            unadoptedContainer.innerHTML = '<p>No unadopted nodes found</p>';
        } else {
            unadoptedContainer.innerHTML = this.nodes.unadopted.map(node => `
                <div class="node-item">
                    <div class="node-status">
                        <span class="status-dot online"></span>
                        <strong>${node.node_id}</strong>
                        <span>${node.hardware_model || 'Unknown'}</span>
                    </div>
                    <div>
                        <button class="btn" onclick="app.adoptNode('${node.node_id}')">Adopt</button>
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
            await ws.management('reset_node', { node_id: nodeId, factory_reset: false });
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
        // TODO: Show adoption dialog with role selection
        const role = prompt('Enter role (head, arms, tracks, console):');
        if (!role) return;

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
    }
}

// Initialize app
const app = new SaintApp();
document.addEventListener('DOMContentLoaded', () => app.init());
