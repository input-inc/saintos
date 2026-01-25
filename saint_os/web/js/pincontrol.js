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
        if (this.nodeId === nodeId) return;

        // Unsubscribe from previous node
        if (this.nodeId) {
            try {
                await window.saintWS.unsubscribe([`pin_state/${this.nodeId}`]);
            } catch (e) {
                console.warn('Failed to unsubscribe:', e);
            }
        }

        this.nodeId = nodeId;
        this.runtimeState = null;

        if (!nodeId) return;

        // Subscribe to state updates
        try {
            await window.saintWS.subscribe([`pin_state/${nodeId}`]);
            console.log(`Subscribed to pin_state/${nodeId}`);
        } catch (e) {
            console.warn('Failed to subscribe to pin state:', e);
        }

        // Request current runtime state
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

            if (result.data) {
                this.handleStateUpdate(result.data);
            }
        } catch (e) {
            console.warn('Failed to load runtime state:', e);
        }
    }

    /**
     * Handle state update from server.
     */
    handleStateUpdate(data) {
        this.runtimeState = data;
        this.updateUI();
    }

    /**
     * Render controls for a container element.
     * @param {HTMLElement} container - Container to render into
     * @param {Array} pins - Array of pin configs from node pin_config
     */
    renderControls(container, pins) {
        if (!container) return;

        if (!pins || pins.length === 0) {
            container.innerHTML = `
                <p class="text-slate-400 text-sm">
                    No pins configured. Go to Pin Configuration to assign pins.
                </p>
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
                        <svg class="w-4 h-4 text-cyan-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 10V3L4 14h7v7l9-11h-7z"/>
                        </svg>
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
                        <svg class="w-4 h-4 text-violet-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4"/>
                        </svg>
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
                        <svg class="w-4 h-4 text-emerald-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z"/>
                        </svg>
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
                        <svg class="w-4 h-4 text-amber-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"/>
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z"/>
                        </svg>
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
                        <svg class="w-4 h-4 text-rose-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z"/>
                        </svg>
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

        // Update stale indicator
        const staleIndicator = document.getElementById('state-stale-indicator');
        if (staleIndicator) {
            if (this.runtimeState.stale) {
                staleIndicator.classList.remove('hidden');
            } else {
                staleIndicator.classList.add('hidden');
            }
        }

        // Update last update time
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
            const item = document.querySelector(`.control-item[data-gpio="${pin.gpio}"]`);
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

            // Check for stale pin data (no update > 1.5s)
            const pinAge = (Date.now() / 1000) - (pin.last_updated || 0);
            const isStale = pinAge > 1.5;

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
