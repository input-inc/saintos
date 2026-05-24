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
        this.visualizationModes = {};  // gpio -> 'slider' | 'stick'
        this.stickPairings = {};  // Store X/Y pairings for stick mode

        // UI update throttling - prevents DOM thrashing from high-frequency updates
        this._uiUpdatePending = false;
        this._uiUpdateThrottleMs = 16;  // ~60fps max
        this._lastUiUpdate = 0;
        this._pendingPinUpdates = new Map();  // gpio -> pin data (only update changed pins)

        // DOM element cache for frequently updated elements (cleared on renderControls)
        this._elementCache = new Map();  // gpio -> {item, slider, display, syncIndicator, ...}

        // Active stick references for drag handling
        this._activeStick = null;
        this._activeTrackStick = null;

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
     * Uses throttled updates to prevent DOM thrashing from high-frequency messages.
     */
    handleStateUpdate(data) {
        this.runtimeState = data;

        // Queue pins for update
        if (data && data.pins) {
            for (const pin of data.pins) {
                this._pendingPinUpdates.set(pin.gpio, pin);
            }
        }

        // Throttle UI updates using requestAnimationFrame
        this._scheduleUiUpdate();
    }

    /**
     * Schedule a throttled UI update.
     */
    _scheduleUiUpdate() {
        if (this._uiUpdatePending) return;

        const now = performance.now();
        const elapsed = now - this._lastUiUpdate;

        if (elapsed >= this._uiUpdateThrottleMs) {
            // Enough time has passed, update immediately via rAF
            this._uiUpdatePending = true;
            requestAnimationFrame(() => {
                this._performUiUpdate();
            });
        } else {
            // Schedule update after remaining throttle time
            this._uiUpdatePending = true;
            setTimeout(() => {
                requestAnimationFrame(() => {
                    this._performUiUpdate();
                });
            }, this._uiUpdateThrottleMs - elapsed);
        }
    }

    /**
     * Perform the actual UI update (called via requestAnimationFrame).
     */
    _performUiUpdate() {
        this._uiUpdatePending = false;
        this._lastUiUpdate = performance.now();

        // Update only the pins that changed
        if (this._pendingPinUpdates.size > 0) {
            this._updateChangedPins();

            // Check if any track drive controls need updating
            this._updateTrackDriveControls();

            this._pendingPinUpdates.clear();
        }

        // Update stale indicator and last update time (less frequent)
        this._updateStatusIndicators();
    }

    /**
     * Update any track drive controls based on pending pin updates.
     */
    _updateTrackDriveControls() {
        if (!this.container) return;

        // Find all track drive controls
        const trackControls = this.container.querySelectorAll('.track-drive-control');
        for (const control of trackControls) {
            const leftGpio = parseInt(control.dataset.leftGpio);
            const rightGpio = parseInt(control.dataset.rightGpio);

            // Check if either track pin was updated
            if (this._pendingPinUpdates.has(leftGpio) || this._pendingPinUpdates.has(rightGpio)) {
                this.updateTrackDriveFromState(leftGpio, rightGpio);
            }
        }
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

        // Clear element cache since we're re-rendering
        this._clearElementCache();

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

        // Detect track drive pairs (left_velocity + right_velocity)
        const leftTrack = pwmPins.find(p => p.logical_name === 'left_velocity');
        const rightTrack = pwmPins.find(p => p.logical_name === 'right_velocity');
        const hasTrackDrive = leftTrack && rightTrack;

        // Filter out track pins from regular PWM if we have a pair
        const regularPwmPins = hasTrackDrive
            ? pwmPins.filter(p => p.logical_name !== 'left_velocity' && p.logical_name !== 'right_velocity')
            : pwmPins;

        let html = '';

        // Track Drive Control (combined left/right with mixing)
        if (hasTrackDrive) {
            html += `
                <div class="control-section mb-6">
                    <h4 class="text-sm font-semibold text-slate-300 mb-3 flex items-center gap-2">
                        <span class="material-icons icon-sm text-cyan-400">swap_horiz</span>
                        Track Drive
                    </h4>
                    ${this.renderTrackDriveControl(leftTrack, rightTrack)}
                </div>
            `;
        }

        // PWM Controls (non-track)
        if (regularPwmPins.length > 0) {
            html += `
                <div class="control-section mb-6">
                    <h4 class="text-sm font-semibold text-slate-300 mb-3 flex items-center gap-2">
                        <span class="material-icons icon-sm text-cyan-400">bolt</span>
                        PWM Outputs
                    </h4>
                    <div class="space-y-3">
                        ${regularPwmPins.map(p => this.renderPWMSlider(p)).join('')}
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
        const mode = this.visualizationModes[gpio] || 'slider';

        return `
            <div class="control-item bg-slate-900/50 rounded-lg p-3" data-gpio="${gpio}" data-mode="pwm">
                <div class="flex items-center justify-between mb-2">
                    <span class="text-sm font-medium text-white">${name}</span>
                    <div class="flex items-center gap-2">
                        <select class="viz-mode-select text-xs bg-slate-800 border border-slate-600 rounded px-1 py-0.5 text-slate-300"
                                data-gpio="${gpio}" title="Visualization mode">
                            <option value="slider" ${mode === 'slider' ? 'selected' : ''}>Slider</option>
                            <option value="stick" ${mode === 'stick' ? 'selected' : ''}>Stick</option>
                        </select>
                        <span class="pin-sync-indicator w-2 h-2 rounded-full bg-slate-500" title="Sync status"></span>
                        <span class="pin-value-display text-sm text-cyan-400 font-mono w-16 text-right">50%</span>
                    </div>
                </div>
                <div class="viz-slider-container" data-gpio="${gpio}" ${mode !== 'slider' ? 'style="display:none"' : ''}>
                    <input type="range" min="0" max="100" value="50" step="1"
                           class="pin-slider pwm-slider w-full"
                           data-gpio="${gpio}">
                    <div class="flex justify-between mt-1">
                        <span class="text-xs text-slate-500">0%</span>
                        <span class="text-xs text-slate-500">50% (center)</span>
                        <span class="text-xs text-slate-500">100%</span>
                    </div>
                </div>
                <div class="viz-stick-container flex justify-center" data-gpio="${gpio}" ${mode !== 'stick' ? 'style="display:none"' : ''}>
                    ${this.renderStickVisualization(gpio, 'pwm')}
                </div>
            </div>
        `;
    }

    /**
     * Render stick visualization for a control.
     * Shows current value as a position on a 2D circle.
     * For single-axis controls, X represents the value (0-100 maps to -1 to 1).
     */
    renderStickVisualization(gpio, mode) {
        const maxVal = mode === 'servo' ? 180 : 100;
        const midVal = maxVal / 2;

        return `
            <div class="stick-wrapper flex flex-col items-center gap-2">
                <div class="stick-container" data-gpio="${gpio}" data-max="${maxVal}">
                    <div class="stick-crosshair horizontal"></div>
                    <div class="stick-crosshair vertical"></div>
                    <div class="stick-knob" data-gpio="${gpio}" style="left: 50%; top: 50%;"></div>
                    <span class="stick-label top">${maxVal}</span>
                    <span class="stick-label bottom">0</span>
                    <span class="stick-label left">-</span>
                    <span class="stick-label right">+</span>
                </div>
                <div class="text-xs text-slate-400">
                    <span>X: <span class="stick-x-value font-mono text-cyan-400" data-gpio="${gpio}">0</span></span>
                    <span class="ml-2">Y: <span class="stick-y-value font-mono text-violet-400" data-gpio="${gpio}">0</span></span>
                </div>
                <div class="text-xs text-slate-500">Click/drag to control</div>
            </div>
        `;
    }

    /**
     * Render servo slider control.
     */
    renderServoSlider(pin) {
        const gpio = pin.gpio;
        const name = pin.logical_name || `GPIO ${gpio}`;
        const mode = this.visualizationModes[gpio] || 'slider';

        return `
            <div class="control-item bg-slate-900/50 rounded-lg p-3" data-gpio="${gpio}" data-mode="servo">
                <div class="flex items-center justify-between mb-2">
                    <span class="text-sm font-medium text-white">${name}</span>
                    <div class="flex items-center gap-2">
                        <select class="viz-mode-select text-xs bg-slate-800 border border-slate-600 rounded px-1 py-0.5 text-slate-300"
                                data-gpio="${gpio}" title="Visualization mode">
                            <option value="slider" ${mode === 'slider' ? 'selected' : ''}>Slider</option>
                            <option value="stick" ${mode === 'stick' ? 'selected' : ''}>Stick</option>
                        </select>
                        <span class="pin-sync-indicator w-2 h-2 rounded-full bg-slate-500" title="Sync status"></span>
                        <span class="pin-value-display text-sm text-violet-400 font-mono w-16 text-right">90&deg;</span>
                    </div>
                </div>
                <div class="viz-slider-container" data-gpio="${gpio}" ${mode !== 'slider' ? 'style="display:none"' : ''}>
                    <input type="range" min="0" max="180" value="90" step="1"
                           class="pin-slider servo-slider w-full"
                           data-gpio="${gpio}">
                    <div class="flex justify-between mt-1">
                        <span class="text-xs text-slate-500">0&deg;</span>
                        <span class="text-xs text-slate-500">90&deg; (center)</span>
                        <span class="text-xs text-slate-500">180&deg;</span>
                    </div>
                </div>
                <div class="viz-stick-container flex justify-center" data-gpio="${gpio}" ${mode !== 'stick' ? 'style="display:none"' : ''}>
                    ${this.renderStickVisualization(gpio, 'servo')}
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
     * Render Track Drive control with joystick and channel mixing.
     * Shows a 2D stick for direction input and vertical bars for left/right track output.
     */
    renderTrackDriveControl(leftPin, rightPin) {
        const leftGpio = leftPin.gpio;
        const rightGpio = rightPin.gpio;

        return `
            <div class="track-drive-control bg-slate-900/50 rounded-lg p-4"
                 data-left-gpio="${leftGpio}" data-right-gpio="${rightGpio}">
                <div class="flex items-center gap-6">
                    <!-- Left Track Bar -->
                    <div class="track-bar-container flex flex-col items-center gap-1">
                        <span class="text-xs text-slate-400 font-medium">L</span>
                        <div class="track-bar-wrapper relative h-32 w-8 bg-slate-800 rounded-lg overflow-hidden border border-slate-700">
                            <div class="track-bar-fill absolute bottom-1/2 left-0 right-0 bg-cyan-500 transition-all duration-75"
                                 data-track="left" style="height: 0%; transform: translateY(50%);"></div>
                            <div class="track-bar-center absolute left-0 right-0 top-1/2 h-px bg-slate-600"></div>
                        </div>
                        <span class="track-bar-value text-xs font-mono text-cyan-400" data-track="left">0%</span>
                    </div>

                    <!-- Joystick Control -->
                    <div class="track-stick-wrapper flex flex-col items-center gap-2">
                        <div class="track-stick-container" data-left-gpio="${leftGpio}" data-right-gpio="${rightGpio}">
                            <div class="track-stick-crosshair horizontal"></div>
                            <div class="track-stick-crosshair vertical"></div>
                            <div class="track-stick-knob"></div>
                        </div>
                        <div class="text-xs text-slate-400 flex gap-4">
                            <span>Throttle: <span class="track-throttle-value font-mono text-violet-400">0.00</span></span>
                            <span>Turn: <span class="track-turn-value font-mono text-amber-400">0.00</span></span>
                        </div>
                    </div>

                    <!-- Right Track Bar -->
                    <div class="track-bar-container flex flex-col items-center gap-1">
                        <span class="text-xs text-slate-400 font-medium">R</span>
                        <div class="track-bar-wrapper relative h-32 w-8 bg-slate-800 rounded-lg overflow-hidden border border-slate-700">
                            <div class="track-bar-fill absolute bottom-1/2 left-0 right-0 bg-cyan-500 transition-all duration-75"
                                 data-track="right" style="height: 0%; transform: translateY(50%);"></div>
                            <div class="track-bar-center absolute left-0 right-0 top-1/2 h-px bg-slate-600"></div>
                        </div>
                        <span class="track-bar-value text-xs font-mono text-cyan-400" data-track="right">0%</span>
                    </div>
                </div>
                <p class="text-xs text-slate-500 mt-3 text-center">Drag stick to control. Y = throttle, X = turn.</p>
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

        // Track drive stick controls
        container.querySelectorAll('.track-stick-container').forEach(stick => {
            stick.addEventListener('mousedown', (e) => {
                this.onTrackStickStart(stick, e);
            });
            stick.addEventListener('touchstart', (e) => {
                e.preventDefault();
                this.onTrackStickStart(stick, e.touches[0]);
            });
        });

        // Visualization mode dropdowns
        container.querySelectorAll('.viz-mode-select').forEach(select => {
            select.addEventListener('change', (e) => {
                this.onVisualizationModeChange(e.target);
            });
        });

        // Stick containers - mouse events
        container.querySelectorAll('.stick-container').forEach(stick => {
            stick.addEventListener('mousedown', (e) => {
                this.onStickStart(stick, e);
            });
        });

        // Global mouse events for stick dragging
        document.addEventListener('mousemove', (e) => {
            if (this._activeStick) {
                this.onStickMove(this._activeStick, e);
            }
            if (this._activeTrackStick) {
                this.onTrackStickMove(this._activeTrackStick, e);
            }
        });
        document.addEventListener('mouseup', () => {
            if (this._activeTrackStick) {
                this.onTrackStickEnd(this._activeTrackStick);
            }
            this._activeStick = null;
            this._activeTrackStick = null;
        });

        // Global touch events for track stick
        document.addEventListener('touchmove', (e) => {
            if (this._activeTrackStick) {
                this.onTrackStickMove(this._activeTrackStick, e.touches[0]);
            }
        });
        document.addEventListener('touchend', () => {
            if (this._activeTrackStick) {
                this.onTrackStickEnd(this._activeTrackStick);
            }
            this._activeTrackStick = null;
        });
    }

    /**
     * Handle visualization mode change.
     */
    onVisualizationModeChange(select) {
        const gpio = parseInt(select.dataset.gpio);
        const mode = select.value;
        this.visualizationModes[gpio] = mode;

        // Find the container and toggle visibility
        const container = select.closest('.control-item');
        const sliderContainer = container.querySelector(`.viz-slider-container[data-gpio="${gpio}"]`);
        const stickContainer = container.querySelector(`.viz-stick-container[data-gpio="${gpio}"]`);

        if (mode === 'slider') {
            sliderContainer.style.display = '';
            stickContainer.style.display = 'none';
        } else {
            sliderContainer.style.display = 'none';
            stickContainer.style.display = '';
            // Update stick position with current value
            this.updateStickPosition(gpio);
        }
    }

    /**
     * Handle stick mousedown.
     */
    onStickStart(stick, event) {
        event.preventDefault();
        this._activeStick = stick;
        stick.classList.add('active');
        stick.querySelector('.stick-knob').classList.add('active');
        this.onStickMove(stick, event);
    }

    /**
     * Handle stick mousemove.
     */
    onStickMove(stick, event) {
        const rect = stick.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;
        const maxRadius = rect.width / 2 - 20; // Account for knob size

        let deltaX = event.clientX - centerX;
        let deltaY = centerY - event.clientY; // Invert Y so up is positive

        // Clamp to circle
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        if (distance > maxRadius) {
            deltaX = (deltaX / distance) * maxRadius;
            deltaY = (deltaY / distance) * maxRadius;
        }

        // Calculate normalized values (-1 to 1)
        const normalizedX = deltaX / maxRadius;
        const normalizedY = deltaY / maxRadius;

        // Update knob position
        const knob = stick.querySelector('.stick-knob');
        knob.style.left = `${50 + (normalizedX * 40)}%`;
        knob.style.top = `${50 - (normalizedY * 40)}%`;

        // Get max value and convert to native range
        const gpio = parseInt(stick.dataset.gpio);
        const maxVal = parseInt(stick.dataset.max) || 100;
        const midVal = maxVal / 2;

        // For stick mode: X controls the value
        // Map -1 to 1 -> 0 to maxVal
        const value = midVal + (normalizedX * midVal);

        // Update value displays
        const xDisplay = stick.parentElement.querySelector(`.stick-x-value[data-gpio="${gpio}"]`);
        const yDisplay = stick.parentElement.querySelector(`.stick-y-value[data-gpio="${gpio}"]`);
        if (xDisplay) xDisplay.textContent = normalizedX.toFixed(2);
        if (yDisplay) yDisplay.textContent = normalizedY.toFixed(2);

        // Update the main value display
        const item = stick.closest('.control-item');
        const display = item.querySelector('.pin-value-display');
        const mode = item.dataset.mode;
        if (display) {
            if (mode === 'servo') {
                display.textContent = `${Math.round(value)}°`;
            } else {
                display.textContent = `${Math.round(value)}%`;
            }
        }

        // Also update the slider to stay in sync
        const slider = item.querySelector('.pin-slider');
        if (slider) {
            slider.value = value;
        }

        // Send to server
        this.throttledSend(gpio, value);
    }

    /**
     * Update stick position based on current value.
     */
    updateStickPosition(gpio) {
        const searchRoot = this.container || document;
        const stick = searchRoot.querySelector(`.stick-container[data-gpio="${gpio}"]`);
        if (!stick) return;

        const maxVal = parseInt(stick.dataset.max) || 100;
        const midVal = maxVal / 2;

        // Get current value from runtime state
        let value = midVal;
        if (this.runtimeState && this.runtimeState.pins) {
            const pin = this.runtimeState.pins.find(p => p.gpio === gpio);
            if (pin) {
                value = pin.actual !== undefined ? pin.actual : (pin.desired || midVal);
            }
        }

        // Convert value to normalized X (-1 to 1)
        const normalizedX = (value - midVal) / midVal;

        // Update knob position
        const knob = stick.querySelector('.stick-knob');
        if (knob) {
            knob.style.left = `${50 + (normalizedX * 40)}%`;
            knob.style.top = '50%'; // Y stays centered for single-axis control
        }

        // Update value displays
        const xDisplay = stick.parentElement?.querySelector(`.stick-x-value[data-gpio="${gpio}"]`);
        if (xDisplay) xDisplay.textContent = normalizedX.toFixed(2);
    }

    /**
     * Handle track stick mousedown/touchstart.
     */
    onTrackStickStart(stick, event) {
        event.preventDefault();
        this._activeTrackStick = stick;
        stick.classList.add('active');
        stick.querySelector('.track-stick-knob').classList.add('active');
        this.onTrackStickMove(stick, event);
    }

    /**
     * Handle track stick move with channel mixing.
     * Y axis = throttle (forward/back), X axis = turn (left/right)
     * Mixing: left = throttle + turn, right = throttle - turn
     */
    onTrackStickMove(stick, event) {
        const rect = stick.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;
        const maxRadius = rect.width / 2 - 16;

        let deltaX = event.clientX - centerX;
        let deltaY = centerY - event.clientY; // Invert Y so up is positive (forward)

        // Clamp to circle
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        if (distance > maxRadius) {
            deltaX = (deltaX / distance) * maxRadius;
            deltaY = (deltaY / distance) * maxRadius;
        }

        // Calculate normalized values (-1 to 1)
        const turn = deltaX / maxRadius;      // X = turn (right is positive)
        const throttle = deltaY / maxRadius;  // Y = throttle (forward is positive)

        // Update knob position
        const knob = stick.querySelector('.track-stick-knob');
        knob.style.left = `${50 + (turn * 40)}%`;
        knob.style.top = `${50 - (throttle * 40)}%`;

        // Channel mixing: left = throttle + turn, right = throttle - turn
        // Clamp to -1 to 1 range
        let leftMix = throttle + turn;
        let rightMix = throttle - turn;

        // Normalize if either exceeds bounds (proportional scaling)
        const maxMix = Math.max(Math.abs(leftMix), Math.abs(rightMix));
        if (maxMix > 1) {
            leftMix /= maxMix;
            rightMix /= maxMix;
        }

        // Convert to PWM percentage (0-100, where 50 is neutral)
        // -1 = 0%, 0 = 50%, 1 = 100%
        const leftPwm = (leftMix + 1) * 50;
        const rightPwm = (rightMix + 1) * 50;

        // Update UI displays
        const wrapper = stick.closest('.track-drive-control');
        this.updateTrackBarDisplay(wrapper, 'left', leftMix, leftPwm);
        this.updateTrackBarDisplay(wrapper, 'right', rightMix, rightPwm);

        // Update throttle/turn displays
        const throttleDisplay = wrapper.querySelector('.track-throttle-value');
        const turnDisplay = wrapper.querySelector('.track-turn-value');
        if (throttleDisplay) throttleDisplay.textContent = throttle.toFixed(2);
        if (turnDisplay) turnDisplay.textContent = turn.toFixed(2);

        // Send values to server
        const leftGpio = parseInt(stick.dataset.leftGpio);
        const rightGpio = parseInt(stick.dataset.rightGpio);
        this.throttledSend(leftGpio, leftPwm);
        this.throttledSend(rightGpio, rightPwm);
    }

    /**
     * Handle track stick release - return to center (stop).
     */
    onTrackStickEnd(stick) {
        stick.classList.remove('active');
        const knob = stick.querySelector('.track-stick-knob');
        if (knob) {
            knob.classList.remove('active');
            // Animate back to center
            knob.style.left = '50%';
            knob.style.top = '50%';
        }

        // Send neutral (stop) values
        const leftGpio = parseInt(stick.dataset.leftGpio);
        const rightGpio = parseInt(stick.dataset.rightGpio);
        this.sendControlValue(leftGpio, 50);  // 50% = neutral
        this.sendControlValue(rightGpio, 50);

        // Update UI to show stopped
        const wrapper = stick.closest('.track-drive-control');
        this.updateTrackBarDisplay(wrapper, 'left', 0, 50);
        this.updateTrackBarDisplay(wrapper, 'right', 0, 50);

        const throttleDisplay = wrapper.querySelector('.track-throttle-value');
        const turnDisplay = wrapper.querySelector('.track-turn-value');
        if (throttleDisplay) throttleDisplay.textContent = '0.00';
        if (turnDisplay) turnDisplay.textContent = '0.00';
    }

    /**
     * Update track bar display (vertical progress bar showing power/direction).
     */
    updateTrackBarDisplay(wrapper, track, normalizedValue, pwmValue) {
        const bar = wrapper.querySelector(`.track-bar-fill[data-track="${track}"]`);
        const valueDisplay = wrapper.querySelector(`.track-bar-value[data-track="${track}"]`);

        if (bar) {
            // Bar height represents magnitude, position represents direction
            const magnitude = Math.abs(normalizedValue) * 50; // 0-50% of container height
            const isForward = normalizedValue >= 0;

            if (isForward) {
                // Bar grows upward from center
                bar.style.height = `${magnitude}%`;
                bar.style.bottom = '50%';
                bar.style.top = 'auto';
                bar.classList.remove('bg-rose-500');
                bar.classList.add('bg-cyan-500');
            } else {
                // Bar grows downward from center
                bar.style.height = `${magnitude}%`;
                bar.style.top = '50%';
                bar.style.bottom = 'auto';
                bar.classList.remove('bg-cyan-500');
                bar.classList.add('bg-rose-500');
            }
        }

        if (valueDisplay) {
            // Show signed percentage from center (e.g., +25%, -30%)
            const signedPercent = Math.round((pwmValue - 50) * 2);
            const sign = signedPercent >= 0 ? '+' : '';
            valueDisplay.textContent = `${sign}${signedPercent}%`;
        }
    }

    /**
     * Update track drive display from runtime state.
     */
    updateTrackDriveFromState(leftGpio, rightGpio) {
        const searchRoot = this.container || document;
        const wrapper = searchRoot.querySelector(`.track-drive-control[data-left-gpio="${leftGpio}"]`);
        if (!wrapper || this._activeTrackStick) return; // Don't update while dragging

        let leftValue = 50, rightValue = 50;
        if (this.runtimeState && this.runtimeState.pins) {
            const leftPin = this.runtimeState.pins.find(p => p.gpio === leftGpio);
            const rightPin = this.runtimeState.pins.find(p => p.gpio === rightGpio);
            if (leftPin) leftValue = leftPin.actual !== undefined ? leftPin.actual : (leftPin.desired || 50);
            if (rightPin) rightValue = rightPin.actual !== undefined ? rightPin.actual : (rightPin.desired || 50);
        }

        // Convert PWM (0-100) to normalized (-1 to 1)
        const leftNorm = (leftValue - 50) / 50;
        const rightNorm = (rightValue - 50) / 50;

        this.updateTrackBarDisplay(wrapper, 'left', leftNorm, leftValue);
        this.updateTrackBarDisplay(wrapper, 'right', rightNorm, rightValue);

        // Reverse the mixing to show stick position
        // left = throttle + turn, right = throttle - turn
        // throttle = (left + right) / 2, turn = (left - right) / 2
        const throttle = (leftNorm + rightNorm) / 2;
        const turn = (leftNorm - rightNorm) / 2;

        // Update stick knob position
        const stick = wrapper.querySelector('.track-stick-container');
        const knob = stick?.querySelector('.track-stick-knob');
        if (knob) {
            knob.style.left = `${50 + (turn * 40)}%`;
            knob.style.top = `${50 - (throttle * 40)}%`;
        }

        const throttleDisplay = wrapper.querySelector('.track-throttle-value');
        const turnDisplay = wrapper.querySelector('.track-turn-value');
        if (throttleDisplay) throttleDisplay.textContent = throttle.toFixed(2);
        if (turnDisplay) turnDisplay.textContent = turn.toFixed(2);
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
     * Get cached DOM elements for a pin, creating cache entry if needed.
     */
    _getCachedElements(gpio) {
        if (this._elementCache.has(gpio)) {
            return this._elementCache.get(gpio);
        }

        const searchRoot = this.container || document;
        const item = searchRoot.querySelector(`.control-item[data-gpio="${gpio}"]`);
        if (!item) return null;

        // Cache all frequently-accessed elements for this pin
        const elements = {
            item,
            mode: item.dataset.mode,
            slider: item.querySelector('.pin-slider'),
            display: item.querySelector('.pin-value-display'),
            syncIndicator: item.querySelector('.pin-sync-indicator'),
            toggle: item.querySelector('.digital-toggle'),
            indicator: item.querySelector('.digital-input-indicator'),
            state: item.querySelector('.digital-input-state'),
            voltageDisplay: item.querySelector('.adc-voltage'),
            bar: item.querySelector('.adc-bar'),
        };

        this._elementCache.set(gpio, elements);
        return elements;
    }

    /**
     * Clear the DOM element cache.
     */
    _clearElementCache() {
        this._elementCache.clear();
    }

    /**
     * Update only the pins that have changed (optimized for high-frequency updates).
     */
    _updateChangedPins() {
        for (const [gpio, pin] of this._pendingPinUpdates) {
            const elements = this._getCachedElements(gpio);
            if (!elements) continue;

            this._updateSinglePinFast(elements, pin);
        }
    }

    /**
     * Update a single pin's UI elements using cached elements (fast path).
     */
    _updateSinglePinFast(elements, pin) {
        const { item, mode, slider, display, syncIndicator, toggle, indicator, state, voltageDisplay, bar } = elements;

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

        // Check for stale pin data
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
        if (actual === undefined) return;

        switch (mode) {
            case 'pwm':
                if (slider && document.activeElement !== slider) {
                    slider.value = actual;
                }
                if (display) {
                    display.textContent = `${Math.round(actual)}%`;
                }
                if (this.visualizationModes[pin.gpio] === 'stick') {
                    this.updateStickFromValue(pin.gpio, actual, 100);
                }
                break;
            case 'servo':
                if (slider && document.activeElement !== slider) {
                    slider.value = actual;
                }
                if (display) {
                    display.textContent = `${Math.round(actual)}\u00B0`;
                }
                if (this.visualizationModes[pin.gpio] === 'stick') {
                    this.updateStickFromValue(pin.gpio, actual, 180);
                }
                break;
            case 'digital_out':
                if (toggle) {
                    this.setToggleState(toggle, actual >= 0.5);
                }
                break;
            case 'digital_in': {
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

    /**
     * Update a single pin's UI elements (fallback for initial render).
     */
    _updateSinglePin(item, pin) {
        const elements = this._getCachedElements(pin.gpio);
        if (elements) {
            this._updateSinglePinFast(elements, pin);
        }
    }

    /**
     * Update status indicators (stale indicator, last update time).
     * Called less frequently as part of the throttled update cycle.
     */
    _updateStatusIndicators() {
        if (!this.runtimeState) return;

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
    }

    /**
     * Update UI with current runtime state (full update, used for initial render).
     */
    updateUI() {
        if (!this.runtimeState || !this.runtimeState.pins) return;

        // Use scoped container if available, otherwise fall back to document
        const searchRoot = this.container || document;

        // Update status indicators
        this._updateStatusIndicators();

        // Update all pins
        for (const pin of this.runtimeState.pins) {
            const item = searchRoot.querySelector(`.control-item[data-gpio="${pin.gpio}"]`);
            if (!item) continue;

            this._updateSinglePin(item, pin);
        }
    }

    /**
     * Update stick visualization from a value.
     */
    updateStickFromValue(gpio, value, maxVal) {
        const searchRoot = this.container || document;
        const stick = searchRoot.querySelector(`.stick-container[data-gpio="${gpio}"]`);
        if (!stick || this._activeStick === stick) return; // Don't update while dragging

        const midVal = maxVal / 2;

        // Convert value to normalized X (-1 to 1)
        const normalizedX = (value - midVal) / midVal;

        // Update knob position
        const knob = stick.querySelector('.stick-knob');
        if (knob) {
            knob.style.left = `${50 + (normalizedX * 40)}%`;
            knob.style.top = '50%';
        }

        // Update value displays
        const xDisplay = stick.parentElement?.querySelector(`.stick-x-value[data-gpio="${gpio}"]`);
        if (xDisplay) xDisplay.textContent = normalizedX.toFixed(2);
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
        this._activeStick = null;
        this._activeTrackStick = null;
        this._clearElementCache();
        this._pendingPinUpdates.clear();
    }
}

// Global instance
const pinControlManager = new PinControlManager();

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    pinControlManager.init();
});
