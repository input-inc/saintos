/**
 * SAINT.OS Pin Configuration Manager
 *
 * Handles pin configuration UI for mapping logical functions to physical GPIO pins.
 * Works within the node detail page context.
 */

class PinConfigManager {
    constructor() {
        this.selectedNode = null;
        this.nodeInfo = null;
        this.capabilities = null;
        this.roleDefinition = null;
        this.pinConfig = {};  // gpio -> config
        this.currentModalPin = null;

        // DOM elements (cached after init)
        this.elements = {};
    }

    /**
     * Initialize the pin config manager.
     */
    init() {
        this.cacheElements();
        this.setupEventListeners();
        this.setupWebSocketHandlers();
    }

    /**
     * Cache frequently accessed DOM elements.
     */
    cacheElements() {
        this.elements = {
            functions: document.getElementById('pinconfig-functions'),
            pinsGrid: document.getElementById('pinconfig-pins-grid'),
            summary: document.getElementById('pinconfig-summary'),
            syncStatus: document.getElementById('node-sync-status'),
            syncBtn: document.getElementById('btn-sync-config'),
            refreshBtn: document.getElementById('btn-refresh-capabilities'),
            modal: document.getElementById('pinconfig-modal'),
            modalPinName: document.getElementById('modal-pin-name'),
            modalPinGpio: document.getElementById('modal-pin-gpio'),
            modalPinCaps: document.getElementById('modal-pin-caps'),
            modalFunctionSelect: document.getElementById('modal-function-select'),
            modalModeSelect: document.getElementById('modal-mode-select'),
            modalPwmParams: document.getElementById('modal-pwm-params'),
            modalPwmFreq: document.getElementById('modal-pwm-freq'),
            modalDigitalInParams: document.getElementById('modal-digital-in-params'),
            modalPullUp: document.getElementById('modal-pull-up'),
            modalPullDown: document.getElementById('modal-pull-down'),
            modalMaestroParams: document.getElementById('modal-maestro-params'),
            modalMaestroMinPulse: document.getElementById('modal-maestro-min-pulse'),
            modalMaestroMaxPulse: document.getElementById('modal-maestro-max-pulse'),
            modalMaestroNeutral: document.getElementById('modal-maestro-neutral'),
            modalMaestroHome: document.getElementById('modal-maestro-home'),
            modalMaestroSpeed: document.getElementById('modal-maestro-speed'),
            modalMaestroAccel: document.getElementById('modal-maestro-accel'),
        };
    }

    /**
     * Setup event listeners.
     */
    setupEventListeners() {
        // Sync button
        this.elements.syncBtn?.addEventListener('click', () => {
            this.syncConfig();
        });

        // Refresh capabilities
        this.elements.refreshBtn?.addEventListener('click', () => {
            this.refreshCapabilities();
        });

        // Mode selection changes parameter visibility
        this.elements.modalModeSelect?.addEventListener('change', (e) => {
            this.updateModalParams(e.target.value);
        });
    }

    /**
     * Setup WebSocket message handlers.
     */
    setupWebSocketHandlers() {
        const ws = window.saintWS;

        // Listen for capabilities updates
        ws.on('state', (message) => {
            if (message.node && message.node.startsWith('node_capabilities/')) {
                const nodeId = message.node.replace('node_capabilities/', '');
                if (nodeId === this.selectedNode) {
                    this.capabilities = message.data;
                    this.renderPins();
                    this.renderFunctions();
                    this.renderSummary();
                    // Update sync status if included in capabilities
                    if (message.data.sync_status) {
                        this.updateSyncStatus(message.data.sync_status);
                    }
                }
            }
            // Listen for sync status updates
            if (message.node && message.node.startsWith('sync_status/')) {
                const nodeId = message.node.replace('sync_status/', '');
                if (nodeId === this.selectedNode) {
                    this.updateSyncStatus(message.data.sync_status);
                }
            }
        });
    }

    /**
     * Load node data - called from app.js when viewing a node.
     */
    async loadNodeData(nodeId, nodeInfo) {
        this.selectedNode = nodeId;
        this.nodeInfo = nodeInfo;
        this.capabilities = null;
        this.roleDefinition = null;
        this.pinConfig = {};

        const ws = window.saintWS;

        // Subscribe to capabilities updates for this node
        try {
            await ws.subscribe([`node_capabilities/${nodeId}`]);
            console.log(`Subscribed to capabilities for ${nodeId}`);
        } catch (error) {
            console.warn('Failed to subscribe to capabilities:', error);
        }

        try {
            // Load capabilities, role definition, and current config in parallel
            const [capsResult, configResult] = await Promise.all([
                ws.management('get_node_capabilities', { node_id: nodeId }),
                ws.management('get_pin_config', { node_id: nodeId }),
            ]);

            // Get capabilities (may be null if not yet received)
            this.capabilities = capsResult || null;

            // Load role definition
            if (nodeInfo?.role) {
                const roleResult = await ws.management('get_role_definition', { role: nodeInfo.role });
                this.roleDefinition = roleResult || null;
            }

            // Load current pin config
            if (configResult) {
                this.pinConfig = {};
                for (const [gpio, config] of Object.entries(configResult.pins || {})) {
                    this.pinConfig[parseInt(gpio)] = config;
                }
                this.updateSyncStatus(configResult.sync_status);
            }

            // Render UI
            this.renderFunctions();
            this.renderPins();
            this.renderSummary();

            // Request capabilities if not available
            if (!this.capabilities) {
                this.refreshCapabilities();
            }
        } catch (error) {
            console.error('Failed to load node pin config:', error);
        }
    }

    /**
     * Refresh capabilities from node.
     */
    async refreshCapabilities() {
        if (!this.selectedNode) return;

        const ws = window.saintWS;
        try {
            await ws.management('request_node_capabilities', { node_id: this.selectedNode });
            // Capabilities will arrive via WebSocket state update
        } catch (error) {
            console.error('Failed to request capabilities:', error);
        }
    }

    /**
     * Render logical functions list.
     */
    renderFunctions() {
        if (!this.elements.functions) return;

        if (!this.roleDefinition) {
            this.elements.functions.innerHTML = '<p class="text-slate-400 text-sm">Role definition not available.</p>';
            return;
        }

        const functions = this.roleDefinition.logical_functions || [];
        if (functions.length === 0) {
            this.elements.functions.innerHTML = '<p class="text-slate-400 text-sm">No functions defined for this role.</p>';
            return;
        }

        // Check which functions are assigned
        const assignedFunctions = new Set();
        for (const config of Object.values(this.pinConfig)) {
            if (config.logical_name) {
                assignedFunctions.add(config.logical_name);
            }
        }

        let html = '';
        for (const func of functions) {
            const isAssigned = assignedFunctions.has(func.name);
            const classes = [
                'function-item',
                func.required ? 'required' : '',
                isAssigned ? 'assigned' : '',
            ].filter(Boolean).join(' ');

            html += `
                <div class="${classes}">
                    <div>
                        <p class="text-sm font-medium text-white">${func.display_name}</p>
                        <p class="text-xs text-slate-400">${func.type}</p>
                    </div>
                    <div class="flex items-center gap-2">
                        ${func.required ? '<span class="text-xs text-cyan-400">Required</span>' : ''}
                        ${isAssigned
                            ? '<span class="w-2 h-2 rounded-full bg-emerald-500"></span>'
                            : '<span class="w-2 h-2 rounded-full bg-slate-600"></span>'}
                    </div>
                </div>
            `;
        }

        this.elements.functions.innerHTML = html;
    }

    /**
     * Render available pins grid.
     */
    renderPins() {
        if (!this.elements.pinsGrid) return;

        if (!this.capabilities) {
            this.elements.pinsGrid.innerHTML = `
                <div class="col-span-full text-center py-8">
                    <p class="text-slate-400 mb-2">Waiting for node capabilities...</p>
                    <button onclick="pinConfigManager.refreshCapabilities()" class="btn-secondary text-sm">
                        Request Capabilities
                    </button>
                </div>
            `;
            return;
        }

        const pins = this.capabilities.pins || [];
        if (pins.length === 0) {
            this.elements.pinsGrid.innerHTML = '<p class="text-slate-400 col-span-full text-center py-8">No configurable pins available.</p>';
            return;
        }

        // Get required functions that are not assigned
        const requiredMissing = new Set();
        if (this.roleDefinition) {
            const assignedFunctions = new Set(
                Object.values(this.pinConfig).map(c => c.logical_name).filter(Boolean)
            );
            for (const func of this.roleDefinition.logical_functions || []) {
                if (func.required && !assignedFunctions.has(func.name)) {
                    requiredMissing.add(func.name);
                }
            }
        }

        let html = '';
        for (const pin of pins) {
            const config = this.pinConfig[pin.gpio];
            const isConfigured = config && config.mode && config.mode !== 'unconfigured';

            // Check if this pin is compatible with any missing required function
            let isRequiredCandidate = false;
            if (this.roleDefinition && !isConfigured) {
                for (const func of this.roleDefinition.logical_functions || []) {
                    if (func.required && requiredMissing.has(func.name)) {
                        // Check mode compatibility
                        for (const mode of func.compatible_modes || []) {
                            if (this.pinSupportsMode(pin, mode)) {
                                isRequiredCandidate = true;
                                break;
                            }
                        }
                    }
                    if (isRequiredCandidate) break;
                }
            }

            const classes = [
                'pin-card',
                isConfigured ? 'configured' : '',
                isRequiredCandidate ? 'required-missing' : '',
            ].filter(Boolean).join(' ');

            const capsStr = (pin.capabilities || []).join(', ');
            const assignedTo = config?.logical_name || '';
            const modeStr = config?.mode || '';

            html += `
                <div class="${classes}" onclick="pinConfigManager.openPinModal(${pin.gpio})">
                    <div class="flex items-center justify-between mb-2">
                        <span class="font-mono font-bold text-cyan-400">${pin.name}</span>
                        <span class="text-xs text-slate-500">GPIO ${pin.gpio}</span>
                    </div>
                    <div class="text-xs text-slate-400 mb-2 truncate" title="${capsStr}">
                        ${capsStr || 'No capabilities'}
                    </div>
                    ${isConfigured ? `
                        <div class="pt-2 border-t border-slate-700">
                            <p class="text-xs text-emerald-400 truncate">${assignedTo}</p>
                            <p class="text-xs text-slate-500">${modeStr}</p>
                        </div>
                    ` : `
                        <div class="pt-2 border-t border-slate-700">
                            <p class="text-xs text-slate-500">Click to configure</p>
                        </div>
                    `}
                </div>
            `;
        }

        this.elements.pinsGrid.innerHTML = html;
    }

    /**
     * Check if pin supports a mode.
     */
    pinSupportsMode(pin, mode) {
        const caps = pin.capabilities || [];
        const modeMap = {
            'digital_in': 'digital_in',
            'digital_out': 'digital_out',
            'pwm': 'pwm',
            'servo': 'pwm',
            'adc': 'adc',
            'maestro_servo': 'maestro_servo',
        };
        const required = modeMap[mode] || mode;
        return caps.includes(required);
    }

    /**
     * Render configuration summary.
     */
    renderSummary() {
        if (!this.elements.summary) return;

        const configured = Object.entries(this.pinConfig)
            .filter(([_, cfg]) => cfg.mode && cfg.mode !== 'unconfigured');

        if (configured.length === 0) {
            this.elements.summary.innerHTML = '<p class="text-slate-400 text-sm">No pins configured.</p>';
            return;
        }

        let html = '<div class="space-y-2">';
        for (const [gpio, config] of configured) {
            const pin = this.capabilities?.pins?.find(p => p.gpio === parseInt(gpio));
            const pinName = pin?.name || `GPIO${gpio}`;

            html += `
                <div class="flex items-center justify-between p-2 bg-slate-900/50 rounded-lg">
                    <div class="flex items-center gap-3">
                        <span class="font-mono text-cyan-400">${pinName}</span>
                        <span class="text-slate-400">→</span>
                        <span class="text-white">${config.logical_name || 'Unnamed'}</span>
                    </div>
                    <span class="text-xs text-slate-500">${config.mode}</span>
                </div>
            `;
        }
        html += '</div>';

        this.elements.summary.innerHTML = html;
    }

    /**
     * Update sync status indicator.
     */
    updateSyncStatus(status) {
        const statusEl = this.elements.syncStatus;
        if (!statusEl) return;

        const dot = statusEl.querySelector('.sync-dot');
        const text = statusEl.querySelector('.sync-text');

        if (!dot || !text) return;

        const statusMap = {
            'unknown': { color: 'bg-slate-500', text: 'Unknown' },
            'unconfigured': { color: 'bg-slate-500', text: 'Not Configured' },
            'pending': { color: 'bg-amber-500 animate-pulse', text: 'Pending Sync' },
            'synced': { color: 'bg-emerald-500', text: 'Synced' },
            'error': { color: 'bg-red-500', text: 'Sync Error' },
        };

        const info = statusMap[status] || statusMap['unknown'];
        dot.className = `sync-dot w-2 h-2 rounded-full ${info.color}`;
        text.textContent = info.text;
    }

    /**
     * Open pin configuration modal.
     */
    openPinModal(gpio) {
        if (!this.capabilities) return;

        const pin = this.capabilities.pins?.find(p => p.gpio === gpio);
        if (!pin) return;

        this.currentModalPin = pin;
        const config = this.pinConfig[gpio] || {};

        // Update modal header
        this.elements.modalPinName.textContent = pin.name;
        this.elements.modalPinGpio.textContent = gpio;
        this.elements.modalPinCaps.textContent = (pin.capabilities || []).join(', ');

        // Populate function dropdown
        this.elements.modalFunctionSelect.innerHTML = '<option value="">-- Not Assigned --</option>';
        if (this.roleDefinition) {
            for (const func of this.roleDefinition.logical_functions || []) {
                // Check if function is compatible with this pin
                let compatible = false;
                for (const mode of func.compatible_modes || []) {
                    if (this.pinSupportsMode(pin, mode)) {
                        compatible = true;
                        break;
                    }
                }

                if (compatible) {
                    const option = document.createElement('option');
                    option.value = func.name;
                    option.textContent = `${func.display_name}${func.required ? ' *' : ''}`;
                    if (config.logical_name === func.name) {
                        option.selected = true;
                    }
                    this.elements.modalFunctionSelect.appendChild(option);
                }
            }
        }

        // Populate mode dropdown
        this.elements.modalModeSelect.innerHTML = '<option value="">-- Select Mode --</option>';
        const modeOptions = [
            { value: 'digital_in', label: 'Digital Input', cap: 'digital_in' },
            { value: 'digital_out', label: 'Digital Output', cap: 'digital_out' },
            { value: 'pwm', label: 'PWM Output', cap: 'pwm' },
            { value: 'servo', label: 'Servo', cap: 'pwm' },
            { value: 'adc', label: 'Analog Input', cap: 'adc' },
            { value: 'maestro_servo', label: 'Maestro Servo', cap: 'maestro_servo' },
        ];

        for (const opt of modeOptions) {
            if ((pin.capabilities || []).includes(opt.cap)) {
                const option = document.createElement('option');
                option.value = opt.value;
                option.textContent = opt.label;
                if (config.mode === opt.value) {
                    option.selected = true;
                }
                this.elements.modalModeSelect.appendChild(option);
            }
        }

        // Set parameter values
        this.elements.modalPwmFreq.value = config.pwm_frequency || 50;
        this.elements.modalPullUp.checked = config.pull_up || false;
        this.elements.modalPullDown.checked = config.pull_down || false;

        // Maestro params
        this.elements.modalMaestroMinPulse.value = config.min_pulse_us || 992;
        this.elements.modalMaestroMaxPulse.value = config.max_pulse_us || 2000;
        this.elements.modalMaestroNeutral.value = config.neutral_us || 1500;
        this.elements.modalMaestroHome.value = config.home_us || 0;
        this.elements.modalMaestroSpeed.value = config.speed || 0;
        this.elements.modalMaestroAccel.value = config.acceleration || 0;

        // Update parameter visibility
        this.updateModalParams(config.mode || '');

        // Show modal
        this.elements.modal.classList.remove('hidden');
    }

    /**
     * Close pin configuration modal.
     */
    closeModal() {
        this.elements.modal.classList.add('hidden');
        this.currentModalPin = null;
    }

    /**
     * Update modal parameter visibility based on mode.
     */
    updateModalParams(mode) {
        // Hide all param sections
        this.elements.modalPwmParams.classList.add('hidden');
        this.elements.modalDigitalInParams.classList.add('hidden');
        this.elements.modalMaestroParams.classList.add('hidden');

        // Show relevant section
        if (mode === 'pwm' || mode === 'servo') {
            this.elements.modalPwmParams.classList.remove('hidden');
        } else if (mode === 'digital_in') {
            this.elements.modalDigitalInParams.classList.remove('hidden');
        } else if (mode === 'maestro_servo') {
            this.elements.modalMaestroParams.classList.remove('hidden');
        }
    }

    /**
     * Save configuration from modal.
     */
    saveModalConfig() {
        if (!this.currentModalPin) return;

        const gpio = this.currentModalPin.gpio;
        const functionName = this.elements.modalFunctionSelect.value;
        const mode = this.elements.modalModeSelect.value;

        if (!mode) {
            // Clear configuration for this pin
            delete this.pinConfig[gpio];
        } else {
            // Build configuration
            const config = {
                mode: mode,
                logical_name: functionName,
            };

            // Add mode-specific params
            if (mode === 'pwm' || mode === 'servo') {
                config.pwm_frequency = parseInt(this.elements.modalPwmFreq.value) || 50;
            } else if (mode === 'digital_in') {
                config.pull_up = this.elements.modalPullUp.checked;
                config.pull_down = this.elements.modalPullDown.checked;
            } else if (mode === 'maestro_servo') {
                config.min_pulse_us = parseInt(this.elements.modalMaestroMinPulse.value) || 992;
                config.max_pulse_us = parseInt(this.elements.modalMaestroMaxPulse.value) || 2000;
                config.neutral_us = parseInt(this.elements.modalMaestroNeutral.value) || 1500;
                config.home_us = parseInt(this.elements.modalMaestroHome.value) || 0;
                config.speed = parseInt(this.elements.modalMaestroSpeed.value) || 0;
                config.acceleration = parseInt(this.elements.modalMaestroAccel.value) || 0;
            }

            this.pinConfig[gpio] = config;
        }

        // Update UI
        this.renderFunctions();
        this.renderPins();
        this.renderSummary();
        this.updateSyncStatus('pending');

        // Close modal
        this.closeModal();

        // Auto-save to server
        this.saveConfig();
    }

    /**
     * Save current configuration to server.
     */
    async saveConfig() {
        if (!this.selectedNode) return;

        const ws = window.saintWS;
        try {
            // Convert config to server format
            const pins = {};
            for (const [gpio, config] of Object.entries(this.pinConfig)) {
                pins[gpio.toString()] = config;
            }

            const result = await ws.management('save_pin_config', {
                node_id: this.selectedNode,
                pins: pins,
            });

            if (result.success) {
                console.log('Configuration saved, version:', result.version);
            } else {
                console.error('Failed to save config:', result.message);
            }
        } catch (error) {
            console.error('Failed to save configuration:', error);
        }
    }

    /**
     * Sync configuration to node.
     */
    async syncConfig() {
        if (!this.selectedNode) return;

        const ws = window.saintWS;
        try {
            // First validate
            const validateResult = await ws.management('validate_pin_config', {
                node_id: this.selectedNode,
                pins: Object.fromEntries(
                    Object.entries(this.pinConfig).map(([k, v]) => [k.toString(), v])
                ),
                role: this.nodeInfo?.role,
            });

            if (!validateResult.valid) {
                const errors = validateResult.errors || [];
                alert('Configuration validation failed:\n\n' + errors.join('\n'));
                return;
            }

            if (validateResult.warnings?.length > 0) {
                console.warn('Config warnings:', validateResult.warnings);
            }

            // Sync to node
            const result = await ws.management('sync_pin_config', { node_id: this.selectedNode });

            if (result?.success || result?.message) {
                this.updateSyncStatus('pending');
            } else {
                this.updateSyncStatus('pending');  // Still mark as pending since we sent the request
            }
        } catch (error) {
            console.error('Failed to sync configuration:', error);
            this.updateSyncStatus('error');
        }
    }
}

// Global instance
const pinConfigManager = new PinConfigManager();

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    pinConfigManager.init();
});
