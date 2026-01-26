/**
 * SAINT.OS Pin Control Manager
 *
 * Handles runtime pin value control (PWM sliders, servo sliders, digital toggles, ADC displays).
 * Separate from pinconfig.js which handles pin mode assignment.
 */

class PinControlManager {
    constructor() {
        this.nodeId = null;
        this.runtimeState = null;
        this.container = null;  // DOM container for this instance's controls
        this.controlThrottleMs = 50;  // Match server throttle
        this.lastSendTime = {};  // gpio -> timestamp

        // Bind methods
        this.handleStateUpdate = this.handleStateUpdate.bind(this);
    }

    /**
     * Initialize the pin control manager.
     */
    init() {
        this.setupWebSocketHandlers();
    }

    /**
     * Setup WebSocket message handlers.
     */
    setupWebSocketHandlers() {
        const ws = window.saintWS;

        // Listen for pin state updates
        ws.on('state', (message) => {
            if (message.node && message.node.startsWith('pin_state/')) {
                const nodeId = message.node.replace('pin_state/', '');
                if (nodeId === this.nodeId) {
                    this.handleStateUpdate(message.data);
                }
            }
        });
    }

    /**
     * Set the current node ID and subscribe to state updates.
     */
    async setNode(nodeId) {
        const isSameNode = (this.nodeId === nodeId);

        // If changing to a different node, unsubscribe from previous
        if (!isSameNode && this.nodeId) {
            try {
                await window.saintWS.unsubscribe([`pin_state/${this.nodeId}`]);
            } catch (e) {
                console.warn('Failed to unsubscribe:', e);
            }
        }

        this.nodeId = nodeId;

        if (!nodeId) {
            this.runtimeState = null;
            return;
        }

        // Subscribe to state updates (only if new node)
        if (!isSameNode) {
            try {
                await window.saintWS.subscribe([`pin_state/${nodeId}`]);
                console.log(`Subscribed to pin_state/${nodeId}`);
            } catch (e) {
                console.warn('Failed to subscribe to pin state:', e);
            }
        }

        // Always request current runtime state (even if same node - may have changed)
        await this.loadRuntimeState();
    }

    /**
     * Load current runtime state from server.
     */
    async loadRuntimeState() {
        if (!this.nodeId) return;

        try {
            const result = await window.saintWS.send({
                type: 'control',
                action: 'get_runtime_state',
                params: { node_id: this.nodeId }
            });

            console.log('PinControlManager: loadRuntimeState result:', result);

            if (result) {
                this.handleStateUpdate(result);
            }
        } catch (e) {
            console.warn('Failed to load runtime state:', e);
        }
    }

    /**
     * Handle state update from server.
     */
    handleStateUpdate(data) {
        console.log('PinControlManager: handleStateUpdate:', data);
        this.runtimeState = data;
        this.updateUI();
    }

    /**
     * Render controls for a container element.
     * @param {HTMLElement} container - Container to render into
     * @param {Array} pins - Array of pin configs from node pin_config
     */
    renderControls(container, pins) {
        if (!container) {
            console.warn('PinControlManager: No container provided');
            return;
        }

        // Store container reference for scoped queries in updateUI
        this.container = container;

        console.log('PinControlManager: Rendering controls for', pins?.length || 0, 'pins', pins);

        if (!pins || pins.length === 0) {
            container.innerHTML = `
                <div class="flex flex-col items-center justify-center py-8 text-center">
                    <span class="material-icons text-slate-600 mb-4" style="font-size: 48px;">tune</span>
                    <h3 class="text-lg font-medium text-slate-300 mb-2">No Pins Configured</h3>
                    <p class="text-slate-500 text-sm mb-4">Configure pins in the Pin Config tab to control them here.</p>
                    <button onclick="app.switchNodeTab('pinconfig')" class="btn-secondary text-sm">
                        Go to Pin Config
                    </button>
                </div>
            `;
            return;
        }

        // Group pins by type
        const pwmPins = pins.filter(p => p.mode === 'pwm');
        const servoPins = pins.filter(p => p.mode === 'servo');
        const digitalOutPins = pins.filter(p => p.mode === 'digital_out');
        const digitalInPins = pins.filter(p => p.mode === 'digital_in');
        const adcPins = pins.filter(p => p.mode === 'adc');

        let html = '';

        // PWM Controls
        if (pwmPins.length > 0) {
            html += `
                <div class="control-section mb-6">
                    <h4 class="text-sm font-semibold text-slate-300 mb-3 flex items-center gap-2">
                        <span class="material-icons icon-sm text-cyan-400">bolt</span>
                        PWM Outputs
                    </h4>
                    <div class="space-y-3">
                        ${pwmPins.map(p => this.renderPWMSlider(p)).join('')}
                    </div>
                </div>
            `;
        }

        // Servo Controls
        if (servoPins.length > 0) {
            html += `
                <div class="control-section mb-6">
                    <h4 class="text-sm font-semibold text-slate-300 mb-3 flex items-center gap-2">
                        <span class="material-icons icon-sm text-violet-400">tune</span>
                        Servo Controls
                    </h4>
                    <div class="space-y-3">
                        ${servoPins.map(p => this.renderServoSlider(p)).join('')}
                    </div>
                </div>
            `;
        }

        // Digital Output Controls
        if (digitalOutPins.length > 0) {
            html += `
                <div class="control-section mb-6">
                    <h4 class="text-sm font-semibold text-slate-300 mb-3 flex items-center gap-2">
                        <span class="material-icons icon-sm text-emerald-400">memory</span>
                        Digital Outputs
                    </h4>
                    <div class="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-3">
                        ${digitalOutPins.map(p => this.renderDigitalToggle(p)).join('')}
                    </div>
                </div>
            `;
        }

        // Digital Input Displays
        if (digitalInPins.length > 0) {
            html += `
                <div class="control-section mb-6">
                    <h4 class="text-sm font-semibold text-slate-300 mb-3 flex items-center gap-2">
                        <span class="material-icons icon-sm text-amber-400">visibility</span>
                        Digital Inputs
                    </h4>
                    <div class="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-3">
                        ${digitalInPins.map(p => this.renderDigitalInput(p)).join('')}
                    </div>
                </div>
            `;
        }

        // ADC Displays
        if (adcPins.length > 0) {
            html += `
                <div class="control-section mb-6">
                    <h4 class="text-sm font-semibold text-slate-300 mb-3 flex items-center gap-2">
                        <span class="material-icons icon-sm text-rose-400">bar_chart</span>
                        Analog Inputs (ADC)
                    </h4>
                    <div class="space-y-3">
                        ${adcPins.map(p => this.renderADCDisplay(p)).join('')}
                    </div>
                </div>
            `;
        }

        container.innerHTML = html;

        // Attach event listeners
        this.attachEventListeners(container);

        // Update with current values
        this.updateUI();
    }

    /**
     * Render PWM slider control.
     */
    renderPWMSlider(pin) {
        const gpio = pin.gpio;
        const name = pin.logical_name || `GPIO ${gpio}`;

        return `
            <div class="control-item bg-slate-900/50 rounded-lg p-3" data-gpio="${gpio}" data-mode="pwm">
                <div class="flex items-center justify-between mb-2">
                    <span class="text-sm font-medium text-white">${name}</span>
                    <div class="flex items-center gap-2">
                        <span class="pin-sync-indicator w-2 h-2 rounded-full bg-slate-500" title="Sync status"></span>
                        <span class="pin-value-display text-sm text-cyan-400 font-mono w-12 text-right">0%</span>
                    </div>
                </div>
                <input type="range" min="0" max="100" value="0" step="1"
                       class="pin-slider pwm-slider w-full"
                       data-gpio="${gpio}">
                <div class="flex justify-between mt-1">
                    <span class="text-xs text-slate-500">0%</span>
                    <span class="text-xs text-slate-500">GPIO ${gpio}</span>
                    <span class="text-xs text-slate-500">100%</span>
                </div>
            </div>
        `;
    }

    /**
     * Render servo slider control.
     */
    renderServoSlider(pin) {
        const gpio = pin.gpio;
        const name = pin.logical_name || `GPIO ${gpio}`;

        return `
            <div class="control-item bg-slate-900/50 rounded-lg p-3" data-gpio="${gpio}" data-mode="servo">
                <div class="flex items-center justify-between mb-2">
                    <span class="text-sm font-medium text-white">${name}</span>
                    <div class="flex items-center gap-2">
                        <span class="pin-sync-indicator w-2 h-2 rounded-full bg-slate-500" title="Sync status"></span>
                        <span class="pin-value-display text-sm text-violet-400 font-mono w-12 text-right">90&deg;</span>
                    </div>
                </div>
                <input type="range" min="0" max="180" value="90" step="1"
                       class="pin-slider servo-slider w-full"
                       data-gpio="${gpio}">
                <div class="flex justify-between mt-1">
                    <span class="text-xs text-slate-500">0&deg;</span>
                    <span class="text-xs text-slate-500">GPIO ${gpio}</span>
                    <span class="text-xs text-slate-500">180&deg;</span>
                </div>
            </div>
        `;
    }

    /**
     * Render digital output toggle.
     */
    renderDigitalToggle(pin) {
        const gpio = pin.gpio;
        const name = pin.logical_name || `GPIO ${gpio}`;

        return `
            <div class="control-item bg-slate-900/50 rounded-lg p-3" data-gpio="${gpio}" data-mode="digital_out">
                <div class="flex items-center justify-between">
                    <div>
                        <span class="text-sm font-medium text-white">${name}</span>
                        <span class="text-xs text-slate-500 block">GPIO ${gpio}</span>
                    </div>
                    <button class="digital-toggle relative inline-flex h-6 w-11 items-center rounded-full bg-slate-700 transition-colors"
                            data-gpio="${gpio}">
                        <span class="toggle-dot inline-block h-4 w-4 transform rounded-full bg-slate-400 transition-transform translate-x-1"></span>
                    </button>
                </div>
            </div>
        `;
    }

    /**
     * Render digital input display.
     */
    renderDigitalInput(pin) {
        const gpio = pin.gpio;
        const name = pin.logical_name || `GPIO ${gpio}`;

        return `
            <div class="control-item bg-slate-900/50 rounded-lg p-3" data-gpio="${gpio}" data-mode="digital_in">
                <div class="flex items-center justify-between">
                    <div>
                        <span class="text-sm font-medium text-white">${name}</span>
                        <span class="text-xs text-slate-500 block">GPIO ${gpio}</span>
                    </div>
                    <div class="digital-input-indicator w-8 h-8 rounded-full bg-slate-700 flex items-center justify-center">
                        <span class="digital-input-state text-xs font-mono text-slate-400">OFF</span>
                    </div>
                </div>
            </div>
        `;
    }

    /**
     * Render ADC display.
     */
    renderADCDisplay(pin) {
        const gpio = pin.gpio;
        const name = pin.logical_name || `A${gpio - 26}`;

        return `
            <div class="control-item bg-slate-900/50 rounded-lg p-3" data-gpio="${gpio}" data-mode="adc">
                <div class="flex items-center justify-between mb-2">
                    <span class="text-sm font-medium text-white">${name}</span>
                    <div class="flex items-center gap-2">
                        <span class="adc-voltage text-sm text-rose-400 font-mono">0.00V</span>
                    </div>
                </div>
                <div class="adc-bar-container h-3 bg-slate-700 rounded-full overflow-hidden">
                    <div class="adc-bar h-full bg-gradient-to-r from-rose-500 to-amber-500 transition-all duration-100" style="width: 0%"></div>
                </div>
                <div class="flex justify-between mt-1">
                    <span class="text-xs text-slate-500">0V</span>
                    <span class="text-xs text-slate-500">GPIO ${gpio}</span>
                    <span class="text-xs text-slate-500">3.3V</span>
                </div>
            </div>
        `;
    }

    /**
     * Attach event listeners to controls.
     */
    attachEventListeners(container) {
        // Slider inputs
        container.querySelectorAll('.pin-slider').forEach(slider => {
            slider.addEventListener('input', (e) => {
                this.onSliderChange(e.target);
            });
        });

        // Digital toggles
        container.querySelectorAll('.digital-toggle').forEach(toggle => {
            toggle.addEventListener('click', (e) => {
                this.onToggleClick(e.currentTarget);
            });
        });
    }

    /**
     * Handle slider value change.
     */
    onSliderChange(slider) {
        const gpio = parseInt(slider.dataset.gpio);
        const value = parseFloat(slider.value);

        // Update local display immediately
        const item = slider.closest('.control-item');
        const display = item.querySelector('.pin-value-display');
        const mode = item.dataset.mode;

        if (display) {
            if (mode === 'servo') {
                display.textContent = `${value}\u00B0`;
            } else {
                display.textContent = `${value}%`;
            }
        }

        // Send to server (throttled)
        this.throttledSend(gpio, value);
    }

    /**
     * Handle toggle click.
     */
    onToggleClick(toggle) {
        const gpio = parseInt(toggle.dataset.gpio);
        const isOn = toggle.classList.contains('active');
        const newValue = isOn ? 0 : 1;

        // Update UI immediately
        this.setToggleState(toggle, newValue === 1);

        // Send to server
        this.sendControlValue(gpio, newValue);
    }

    /**
     * Set toggle visual state.
     */
    setToggleState(toggle, isOn) {
        const dot = toggle.querySelector('.toggle-dot');
        if (isOn) {
            toggle.classList.add('active', 'bg-emerald-500');
            toggle.classList.remove('bg-slate-700');
            dot.classList.add('translate-x-6', 'bg-white');
            dot.classList.remove('translate-x-1', 'bg-slate-400');
        } else {
            toggle.classList.remove('active', 'bg-emerald-500');
            toggle.classList.add('bg-slate-700');
            dot.classList.remove('translate-x-6', 'bg-white');
            dot.classList.add('translate-x-1', 'bg-slate-400');
        }
    }

    /**
     * Send control value with throttling.
     */
    throttledSend(gpio, value) {
        const now = Date.now();
        const lastSend = this.lastSendTime[gpio] || 0;

        if (now - lastSend < this.controlThrottleMs) {
            // Schedule a delayed send
            if (this._throttleTimers?.[gpio]) {
                clearTimeout(this._throttleTimers[gpio]);
            }
            if (!this._throttleTimers) this._throttleTimers = {};
            this._throttleTimers[gpio] = setTimeout(() => {
                this.sendControlValue(gpio, value);
            }, this.controlThrottleMs - (now - lastSend));
            return;
        }

        this.sendControlValue(gpio, value);
    }

    /**
     * Send control value to server.
     */
    async sendControlValue(gpio, value) {
        if (!this.nodeId) return;

        this.lastSendTime[gpio] = Date.now();

        try {
            await window.saintWS.send({
                type: 'control',
                action: 'set_pin_value',
                params: {
                    node_id: this.nodeId,
                    gpio: gpio,
                    value: value
                }
            });
        } catch (e) {
            console.warn('Failed to send control value:', e);
        }
    }

    /**
     * Update UI with current runtime state.
     */
    updateUI() {
        if (!this.runtimeState || !this.runtimeState.pins) return;

        // Use scoped container if available, otherwise fall back to document
        const searchRoot = this.container || document;

        // Update stale indicator (only in node detail page, not control page)
        const staleIndicator = document.getElementById('state-stale-indicator');
        if (staleIndicator) {
            if (this.runtimeState.stale) {
                staleIndicator.classList.remove('hidden');
            } else {
                staleIndicator.classList.add('hidden');
            }
        }

        // Update last update time (only in node detail page)
        const lastUpdateEl = document.getElementById('state-last-update');
        if (lastUpdateEl && this.runtimeState.last_feedback) {
            const lastFeedback = this.runtimeState.last_feedback;
            const age = Math.round((Date.now() / 1000) - lastFeedback);
            if (age < 60) {
                lastUpdateEl.textContent = age < 2 ? 'Just now' : `${age}s ago`;
            } else {
                lastUpdateEl.textContent = new Date(lastFeedback * 1000).toLocaleTimeString();
            }
        }

        for (const pin of this.runtimeState.pins) {
            // Use scoped search to find controls only within this instance's container
            const item = searchRoot.querySelector(`.control-item[data-gpio="${pin.gpio}"]`);
            if (!item) continue;

            const mode = item.dataset.mode;
            const syncIndicator = item.querySelector('.pin-sync-indicator');

            // Update sync indicator
            if (syncIndicator) {
                if (pin.synced) {
                    syncIndicator.classList.remove('bg-amber-500', 'animate-pulse');
                    syncIndicator.classList.add('bg-emerald-500');
                    syncIndicator.title = 'Synced';
                } else {
                    syncIndicator.classList.remove('bg-emerald-500');
                    syncIndicator.classList.add('bg-amber-500', 'animate-pulse');
                    syncIndicator.title = 'Pending sync';
                }
            }

            // Check for stale pin data (no firmware feedback > 2s)
            // Only mark as stale if we have actual values - if only desired values,
            // firmware may not support feedback (e.g., simulation mode)
            const pinAge = (Date.now() / 1000) - (pin.last_updated || 0);
            const hasActualFeedback = pin.actual !== undefined;
            const isStale = hasActualFeedback && pinAge > 2.0;

            if (isStale) {
                item.classList.add('opacity-60');
            } else {
                item.classList.remove('opacity-60');
            }

            // Update value based on mode
            const actual = pin.actual !== undefined ? pin.actual : pin.desired;
            if (actual === undefined) continue;

            switch (mode) {
                case 'pwm': {
                    const slider = item.querySelector('.pin-slider');
                    const display = item.querySelector('.pin-value-display');
                    // Only update if not being dragged
                    if (slider && document.activeElement !== slider) {
                        slider.value = actual;
                    }
                    if (display) {
                        display.textContent = `${Math.round(actual)}%`;
                    }
                    break;
                }
                case 'servo': {
                    const slider = item.querySelector('.pin-slider');
                    const display = item.querySelector('.pin-value-display');
                    if (slider && document.activeElement !== slider) {
                        slider.value = actual;
                    }
                    if (display) {
                        display.textContent = `${Math.round(actual)}\u00B0`;
                    }
                    break;
                }
                case 'digital_out': {
                    const toggle = item.querySelector('.digital-toggle');
                    if (toggle) {
                        this.setToggleState(toggle, actual >= 0.5);
                    }
                    break;
                }
                case 'digital_in': {
                    const indicator = item.querySelector('.digital-input-indicator');
                    const state = item.querySelector('.digital-input-state');
                    const isHigh = actual >= 0.5;
                    if (indicator) {
                        if (isHigh) {
                            indicator.classList.remove('bg-slate-700');
                            indicator.classList.add('bg-emerald-500');
                        } else {
                            indicator.classList.add('bg-slate-700');
                            indicator.classList.remove('bg-emerald-500');
                        }
                    }
                    if (state) {
                        state.textContent = isHigh ? 'ON' : 'OFF';
                        state.classList.toggle('text-white', isHigh);
                        state.classList.toggle('text-slate-400', !isHigh);
                    }
                    break;
                }
                case 'adc': {
                    const voltage = pin.voltage !== undefined ? pin.voltage : actual;
                    const voltageDisplay = item.querySelector('.adc-voltage');
                    const bar = item.querySelector('.adc-bar');
                    if (voltageDisplay) {
                        voltageDisplay.textContent = `${voltage.toFixed(2)}V`;
                    }
                    if (bar) {
                        const percent = (voltage / 3.3) * 100;
                        bar.style.width = `${Math.min(100, percent)}%`;
                    }
                    break;
                }
            }
        }
    }

    /**
     * Cleanup when no longer viewing this node.
     */
    cleanup() {
        if (this.nodeId) {
            window.saintWS.unsubscribe([`pin_state/${this.nodeId}`]).catch(() => {});
            this.nodeId = null;
            this.runtimeState = null;
        }
    }
}

// Global instance
const pinControlManager = new PinControlManager();

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    pinControlManager.init();
});
