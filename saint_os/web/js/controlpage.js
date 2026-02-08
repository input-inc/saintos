/**
 * SAINT.OS Control Page
 *
 * Global control page with expandable node sections.
 * Each section uses PinControlManager for controls.
 */

console.log('controlpage.js: Script loading...');

class ControlPage {
    constructor() {
        this.nodes = [];
        this.expandedNodes = new Set();
        this.nodeManagers = {};  // node_id -> PinControlManager instance
        this.container = null;
        this.activeTab = 'nodes';  // 'nodes' or 'moods'
    }

    /**
     * Initialize the control page.
     */
    init() {
        this.container = document.getElementById('control-nodes-container');
        this.setupEventListeners();
    }

    /**
     * Switch between control sub-tabs.
     */
    switchTab(tabId) {
        console.log('ControlPage: Switching to tab:', tabId);
        this.activeTab = tabId;

        // Update tab buttons
        document.querySelectorAll('.control-tab').forEach(tab => {
            if (tab.dataset.tab === tabId) {
                tab.classList.add('active');
            } else {
                tab.classList.remove('active');
            }
        });

        // Update tab content
        document.querySelectorAll('.control-tab-content').forEach(content => {
            if (content.id === `control-tab-${tabId}`) {
                content.classList.remove('hidden');
            } else {
                content.classList.add('hidden');
            }
        });

        // Load tab-specific data
        if (tabId === 'moods' && window.moodsManager) {
            moodsManager.load();
        }
    }

    /**
     * Setup global event listeners.
     */
    setupEventListeners() {
        // E-Stop button
        const estopBtn = document.getElementById('btn-control-estop');
        if (estopBtn) {
            estopBtn.addEventListener('click', () => {
                this.emergencyStopAll();
            });
        }
    }

    /**
     * Load data when page becomes active.
     */
    async load() {
        console.log('ControlPage: Loading...');

        if (!this.container) {
            this.container = document.getElementById('control-nodes-container');
        }

        if (!this.container) {
            console.error('ControlPage: Container not found!');
            return;
        }

        const ws = window.saintWS;
        if (!ws) {
            console.error('ControlPage: WebSocket not initialized');
            this.container.innerHTML = `
                <div class="card">
                    <p class="text-red-400 text-sm">WebSocket not initialized.</p>
                </div>
            `;
            return;
        }

        if (!ws.connected) {
            console.warn('ControlPage: Not connected to server');
            this.container.innerHTML = `
                <div class="card">
                    <p class="text-slate-400 text-sm">Not connected to server.</p>
                </div>
            `;
            return;
        }

        try {
            // Get adopted nodes
            console.log('ControlPage: Fetching adopted nodes via WebSocket...');
            const result = await ws.management('list_adopted');
            console.log('ControlPage: Got result:', result);
            this.nodes = result?.nodes || [];
            console.log('ControlPage: Nodes count:', this.nodes.length);

            this.render();
        } catch (e) {
            console.error('ControlPage: Failed to load:', e);
            this.container.innerHTML = `
                <div class="card">
                    <p class="text-red-400 text-sm">Failed to load nodes: ${e.message}</p>
                </div>
            `;
        }
    }

    /**
     * Render the control page.
     */
    render() {
        if (!this.container) return;

        if (this.nodes.length === 0) {
            this.container.innerHTML = `
                <div class="card">
                    <div class="flex flex-col items-center justify-center py-8 text-center">
                        <span class="material-icons text-slate-600 mb-4" style="font-size: 48px;">dns</span>
                        <h3 class="text-lg font-medium text-slate-300 mb-2">No Adopted Nodes</h3>
                        <p class="text-slate-500">Adopt nodes from the Nodes page to control them here.</p>
                    </div>
                </div>
            `;
            return;
        }

        let html = '';

        for (const node of this.nodes) {
            const isExpanded = this.expandedNodes.has(node.node_id);
            const statusColor = node.online !== false ? 'bg-emerald-500' : 'bg-slate-500';
            const displayName = node.display_name || node.node_id;
            const role = node.role || 'Unknown';

            html += `
                <div class="control-node-panel card" data-node-id="${node.node_id}">
                    <button class="control-node-header w-full flex items-center justify-between p-2 -m-2 rounded-lg hover:bg-slate-800/50 transition-colors"
                            onclick="controlPage.toggleNode('${node.node_id}')">
                        <div class="flex items-center gap-3">
                            <span class="w-2.5 h-2.5 rounded-full ${statusColor}"></span>
                            <div class="text-left">
                                <span class="font-medium text-white">${displayName}</span>
                                <span class="text-sm text-slate-400 ml-2">${role}</span>
                            </div>
                        </div>
                        <div class="flex items-center gap-3">
                            <span class="node-pin-count text-sm text-slate-500"></span>
                            <span class="material-icons md-20 text-slate-400 transition-transform ${isExpanded ? 'rotate-180' : ''}">expand_more</span>
                        </div>
                    </button>

                    <div class="control-node-content mt-4 ${isExpanded ? '' : 'hidden'}"
                         id="control-content-${node.node_id}">
                        <div class="border-t border-slate-700 pt-4">
                            <div class="control-pins-container" id="control-pins-${node.node_id}">
                                <p class="text-slate-400 text-sm">Loading controls...</p>
                            </div>
                        </div>
                    </div>
                </div>
            `;
        }

        this.container.innerHTML = html;

        // Load expanded nodes
        for (const nodeId of this.expandedNodes) {
            this.loadNodeControls(nodeId);
        }
    }

    /**
     * Toggle node expansion.
     */
    async toggleNode(nodeId) {
        const panel = this.container.querySelector(`[data-node-id="${nodeId}"]`);
        const content = document.getElementById(`control-content-${nodeId}`);
        const arrow = panel.querySelector('.material-icons');

        if (this.expandedNodes.has(nodeId)) {
            // Collapse
            this.expandedNodes.delete(nodeId);
            content.classList.add('hidden');
            if (arrow) arrow.classList.remove('rotate-180');

            // Unsubscribe from state updates
            this.unsubscribeNode(nodeId);
        } else {
            // Expand
            this.expandedNodes.add(nodeId);
            content.classList.remove('hidden');
            if (arrow) arrow.classList.add('rotate-180');

            // Load controls
            await this.loadNodeControls(nodeId);
        }
    }

    /**
     * Load controls for a node.
     */
    async loadNodeControls(nodeId) {
        const container = document.getElementById(`control-pins-${nodeId}`);
        if (!container) return;

        // Find node info
        const node = this.nodes.find(n => n.node_id === nodeId);
        const isOffline = node && node.online === false;

        if (isOffline) {
            container.innerHTML = `
                <div class="flex items-center gap-3 p-3 bg-slate-800/50 rounded-lg border border-slate-700">
                    <span class="material-icons md-20 text-slate-500">wifi_off</span>
                    <div>
                        <p class="text-sm font-medium text-slate-300">Node Offline</p>
                        <p class="text-xs text-slate-500">Controls will be available when node reconnects</p>
                    </div>
                </div>
            `;
            return;
        }

        try {
            const ws = window.saintWS;

            // Get pin configuration
            const configResult = await ws.management('get_pin_config', {
                node_id: nodeId
            });

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

            // Update pin count
            const panel = this.container.querySelector(`[data-node-id="${nodeId}"]`);
            const countEl = panel.querySelector('.node-pin-count');
            if (countEl) {
                countEl.textContent = pins.length > 0 ? `${pins.length} pins` : '';
            }

            if (pins.length === 0) {
                container.innerHTML = `
                    <div class="flex items-center gap-3 p-3 bg-slate-800/50 rounded-lg border border-slate-700">
                        <span class="material-icons md-20 text-slate-500">add_circle_outline</span>
                        <div>
                            <p class="text-sm font-medium text-slate-300">No Pins Configured</p>
                            <p class="text-xs text-slate-500">Go to node detail page to configure pins</p>
                        </div>
                    </div>
                `;
                return;
            }

            // Create or get manager for this node
            if (!this.nodeManagers[nodeId]) {
                this.nodeManagers[nodeId] = new PinControlManager();
                this.nodeManagers[nodeId].init();
            }

            const manager = this.nodeManagers[nodeId];
            await manager.setNode(nodeId);

            // Render controls
            manager.renderControls(container, pins);

        } catch (e) {
            console.error(`Failed to load controls for ${nodeId}:`, e);
            container.innerHTML = `
                <div class="flex items-center gap-3 p-3 bg-red-900/20 rounded-lg border border-red-800/50">
                    <span class="material-icons md-20 text-red-400">error</span>
                    <div>
                        <p class="text-sm font-medium text-red-400">Failed to Load Controls</p>
                        <p class="text-xs text-red-300/70">${e.message || 'Unknown error'}</p>
                    </div>
                </div>
            `;
        }
    }

    /**
     * Unsubscribe from node state updates.
     */
    async unsubscribeNode(nodeId) {
        const manager = this.nodeManagers[nodeId];
        if (manager) {
            await manager.setNode(null);
        }
    }

    /**
     * Emergency stop all nodes.
     */
    async emergencyStopAll() {
        const ws = window.saintWS;

        try {
            await ws.command('system', 'estop', { target: 'all' });

            // Also set all PWM/servo to 0 and digital to off
            for (const nodeId of this.expandedNodes) {
                const manager = this.nodeManagers[nodeId];
                if (manager && manager.runtimeState?.pins) {
                    for (const pin of manager.runtimeState.pins) {
                        if (pin.mode === 'pwm' || pin.mode === 'servo' || pin.mode === 'digital_out') {
                            await manager.sendControlValue(pin.gpio, 0);
                        }
                    }
                }
            }

            // Show feedback
            if (typeof app !== 'undefined') {
                app.addActivityLogEntry({
                    text: 'Emergency stop activated - all outputs disabled',
                    level: 'warn'
                });
            }
        } catch (e) {
            console.error('E-Stop failed:', e);
        }
    }

    /**
     * Update node online status from broadcast data.
     */
    updateNodeStatus(adoptedNodes) {
        if (!adoptedNodes || !this.container) return;

        // Update cached nodes
        this.nodes = adoptedNodes;

        // Update status indicators for each node
        for (const node of adoptedNodes) {
            const panel = this.container.querySelector(`[data-node-id="${node.node_id}"]`);
            if (!panel) continue;

            // Update status dot
            const statusDot = panel.querySelector('.control-node-header .rounded-full');
            if (statusDot) {
                statusDot.className = statusDot.className.replace(
                    /bg-(emerald|slate)-500/,
                    node.online !== false ? 'bg-emerald-500' : 'bg-slate-500'
                );
            }

            // If node just came online and is expanded, reload controls
            const wasOffline = panel.dataset.wasOffline === 'true';
            const isOnline = node.online !== false;

            if (wasOffline && isOnline && this.expandedNodes.has(node.node_id)) {
                this.loadNodeControls(node.node_id);
            }

            panel.dataset.wasOffline = isOnline ? 'false' : 'true';
        }
    }

    /**
     * Cleanup when leaving page.
     */
    cleanup() {
        // Unsubscribe all expanded nodes
        for (const nodeId of this.expandedNodes) {
            this.unsubscribeNode(nodeId);
        }
    }
}

// Global instance
console.log('controlpage.js: Creating ControlPage instance...');
const controlPage = new ControlPage();
console.log('controlpage.js: ControlPage created, window.controlPage =', typeof window.controlPage);

// Make it globally accessible
window.controlPage = controlPage;

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    console.log('controlpage.js: DOMContentLoaded, initializing...');
    controlPage.init();
});
