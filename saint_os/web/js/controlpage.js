/**
 * SAINT.OS Control Page
 *
 * Global control page with expandable node sections.
 * Each section uses PinControlManager for controls.
 */

class ControlPage {
    constructor() {
        this.nodes = [];
        this.expandedNodes = new Set();
        this.nodeManagers = {};  // node_id -> PinControlManager instance
        this.container = null;
    }

    /**
     * Initialize the control page.
     */
    init() {
        this.container = document.getElementById('control-nodes-container');
        this.setupEventListeners();
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
        if (!this.container) {
            this.container = document.getElementById('control-nodes-container');
        }

        const ws = window.saintWS;
        if (!ws.connected) {
            this.container.innerHTML = `
                <div class="card">
                    <p class="text-slate-400 text-sm">Not connected to server.</p>
                </div>
            `;
            return;
        }

        try {
            // Get adopted nodes
            const result = await ws.management('list_adopted');
            this.nodes = result.nodes || [];

            this.render();
        } catch (e) {
            console.error('Failed to load control page:', e);
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
                        <svg class="w-12 h-12 text-slate-600 mb-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="1.5" d="M19 11H5m14 0a2 2 0 012 2v6a2 2 0 01-2 2H5a2 2 0 01-2-2v-6a2 2 0 012-2m14 0V9a2 2 0 00-2-2M5 11V9a2 2 0 012-2m0 0V5a2 2 0 012-2h6a2 2 0 012 2v2M7 7h10"/>
                        </svg>
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
                            <svg class="w-5 h-5 text-slate-400 transition-transform ${isExpanded ? 'rotate-180' : ''}"
                                 fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M19 9l-7 7-7-7"/>
                            </svg>
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
        const arrow = panel.querySelector('svg');

        if (this.expandedNodes.has(nodeId)) {
            // Collapse
            this.expandedNodes.delete(nodeId);
            content.classList.add('hidden');
            arrow.classList.remove('rotate-180');

            // Unsubscribe from state updates
            this.unsubscribeNode(nodeId);
        } else {
            // Expand
            this.expandedNodes.add(nodeId);
            content.classList.remove('hidden');
            arrow.classList.add('rotate-180');

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
                    <svg class="w-5 h-5 text-slate-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M18.364 5.636a9 9 0 010 12.728m0 0l-2.829-2.829m2.829 2.829L21 21M15.536 8.464a5 5 0 010 7.072m0 0l-2.829-2.829m-4.243 2.829a4.978 4.978 0 01-1.414-2.83m-1.414 5.658a9 9 0 01-2.167-9.238m7.824 2.167a1 1 0 111.414 1.414m-1.414-1.414L3 3m8.293 8.293l1.414 1.414"/>
                    </svg>
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
                        <svg class="w-5 h-5 text-slate-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 6v6m0 0v6m0-6h6m-6 0H6"/>
                        </svg>
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
                    <svg class="w-5 h-5 text-red-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z"/>
                    </svg>
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
const controlPage = new ControlPage();

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    controlPage.init();
});
