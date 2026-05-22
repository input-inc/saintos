/**
 * SAINT.OS Peripheral Manager
 *
 * Replaces the old Pin Configuration UI. Operators add peripherals
 * (FAS100, Button, NeoPixel, ...) which own pins and expose named
 * channels. Routing to widgets / signals happens on a separate page.
 */

class PeripheralManager {
    constructor() {
        this.selectedNode = null;
        this.nodeInfo = null;
        this.capabilities = null;
        this.peripherals = [];
        this.syncStatus = 'unknown';

        // Cached catalog (peripheral types + widget types) — fetched once.
        this.catalog = null;
        this._catalogPromise = null;

        // Modal state
        this.modalMode = 'add';   // 'add' | 'edit'
        this.modalEditingId = null;
    }

    init() {
        this.setupEventListeners();
        this.setupWebSocketHandlers();
    }

    // ─── Event wiring ───────────────────────────────────────────────────

    setupEventListeners() {
        const addBtn = document.getElementById('btn-add-peripheral');
        if (addBtn) addBtn.addEventListener('click', () => this.openAddModal());

        const syncBtn = document.getElementById('btn-sync-config');
        if (syncBtn) syncBtn.addEventListener('click', () => this.syncToNode());

        const refreshBtn = document.getElementById('btn-refresh-capabilities');
        if (refreshBtn) refreshBtn.addEventListener('click', () => this.refreshCapabilities());
    }

    setupWebSocketHandlers() {
        const ws = window.saintWS;
        if (!ws) return;

        ws.on('state', (message) => {
            if (!message || !message.node) return;
            const topic = message.node;

            if (topic.startsWith('node_capabilities/')) {
                const nodeId = topic.slice('node_capabilities/'.length);
                if (nodeId !== this.selectedNode) return;
                this.capabilities = message.data;
                if (message.data && message.data.sync_status) {
                    this.updateSyncStatus(message.data.sync_status);
                }
                this.renderAll();
                // If the Add/Edit modal happens to be open and was
                // showing the "no capabilities yet" warning, re-render
                // its body now that pins are available — so the user
                // doesn't have to close + reopen.
                const modal = document.getElementById('peripheral-modal');
                if (modal && !modal.classList.contains('hidden')) {
                    const err = document.getElementById('peripheral-modal-error');
                    if (err) { err.textContent = ''; err.classList.add('hidden'); }
                    const existing = this.modalEditingId
                        ? this.peripherals.find(p => p.id === this.modalEditingId)
                        : null;
                    this.renderModalBody(existing);
                }
            } else if (topic.startsWith('sync_status/')) {
                const nodeId = topic.slice('sync_status/'.length);
                if (nodeId !== this.selectedNode) return;
                if (message.data && message.data.sync_status) {
                    this.updateSyncStatus(message.data.sync_status);
                }
            }
        });
    }

    // ─── Loading data ───────────────────────────────────────────────────

    async ensureCatalog() {
        if (this.catalog) return this.catalog;
        if (this._catalogPromise) return this._catalogPromise;
        const ws = window.saintWS;
        this._catalogPromise = ws.management('get_peripheral_catalog', {})
            .then(result => {
                this.catalog = result || { peripheral_types: [], widget_types: [] };
                return this.catalog;
            })
            .catch(err => {
                console.error('Failed to load peripheral catalog:', err);
                this.catalog = { peripheral_types: [], widget_types: [] };
                return this.catalog;
            });
        return this._catalogPromise;
    }

    async loadNodeData(nodeId, nodeInfo) {
        this.selectedNode = nodeId;
        this.nodeInfo = nodeInfo;
        this.capabilities = null;
        this.peripherals = [];

        const ws = window.saintWS;

        try {
            await ws.subscribe([`node_capabilities/${nodeId}`, `sync_status/${nodeId}`]);
        } catch (e) {
            console.warn('Failed to subscribe to node updates:', e);
        }

        try {
            const [catalog, caps, periph] = await Promise.all([
                this.ensureCatalog(),
                ws.management('get_node_capabilities', { node_id: nodeId }),
                ws.management('get_node_peripherals', { node_id: nodeId }),
            ]);

            this.capabilities = caps || null;
            this.peripherals = (periph && periph.peripherals) ? periph.peripherals : [];
            this.updateSyncStatus(periph?.sync_status || 'unknown');

            this.renderAll();

            if (!this.capabilities) {
                this.refreshCapabilities();
            }
        } catch (err) {
            console.error('Failed to load node peripherals:', err);
            const list = document.getElementById('peripheral-list');
            if (list) list.innerHTML =
                `<p class="text-red-300 text-sm">Failed to load peripherals: ${escapeHtml(err.message || err)}</p>`;
        }
    }

    async refreshCapabilities() {
        if (!this.selectedNode) return;
        const btn = document.getElementById('btn-refresh-capabilities');
        const origHtml = btn ? btn.innerHTML : null;
        if (btn) {
            btn.disabled = true;
            btn.innerHTML = '<span class="material-icons icon-sm animate-spin">refresh</span> Refreshing…';
        }
        try {
            // Ask the firmware to re-publish capabilities. The host
            // controller has no firmware backing it, so this is a no-op
            // for that case — the synthetic state on the server is
            // already current. We still always re-fetch below so the
            // UI rerenders even when the broadcast path hasn't fired.
            try {
                await window.saintWS.management('request_node_capabilities', {
                    node_id: this.selectedNode,
                });
            } catch (e) {
                console.warn('request_node_capabilities failed:', e);
            }

            // Give the firmware a moment to publish, then re-fetch
            // straight from the server's cached state. This makes
            // Refresh feel responsive regardless of whether the
            // broadcast arrives.
            await new Promise(r => setTimeout(r, 500));
            const ws = window.saintWS;
            const [caps, periph] = await Promise.all([
                ws.management('get_node_capabilities', { node_id: this.selectedNode }),
                ws.management('get_node_peripherals',  { node_id: this.selectedNode }),
            ]);
            this.capabilities = caps || this.capabilities;
            this.peripherals = (periph && periph.peripherals) ? periph.peripherals : this.peripherals;
            this.updateSyncStatus(periph?.sync_status || this.syncStatus);
            this.renderAll();
        } finally {
            if (btn) {
                btn.disabled = false;
                btn.innerHTML = origHtml;
            }
        }
    }

    // ─── Rendering ─────────────────────────────────────────────────────

    renderAll() {
        this.renderList();
        this.renderPinSidebar();
    }

    renderList() {
        const el = document.getElementById('peripheral-list');
        if (!el) return;
        if (!this.catalog) {
            el.innerHTML = '<p class="text-slate-400 text-sm">Loading catalog...</p>';
            return;
        }
        if (this.peripherals.length === 0) {
            el.innerHTML = `
                <div class="text-sm text-slate-500 italic p-4 text-center border border-dashed border-slate-700 rounded">
                    No peripherals yet. Click <span class="font-semibold">Add peripheral</span> to start.
                </div>`;
            return;
        }
        // Built-in peripherals first so they're visually grouped.
        const sorted = [...this.peripherals].sort((a, b) =>
            (b.builtin ? 1 : 0) - (a.builtin ? 1 : 0));

        el.innerHTML = sorted.map(p => this.renderRow(p)).join('');
    }

    renderRow(p) {
        const type = this.typeFor(p.type);
        const typeLabel = type ? type.label : p.type;
        const pinDesc = this.describePins(p, type);
        const channelChips = type
            ? type.channels.map(c =>
                `<span class="px-2 py-0.5 bg-slate-700 text-slate-300 rounded text-xs font-mono"
                       title="${escapeAttr(c.display)} (${c.dir})">
                    ${escapeHtml(c.id)} ${c.dir === 'in' ? '↑' : '↓'}
                 </span>`).join(' ')
            : '';
        const builtinBadge = p.builtin
            ? '<span class="px-2 py-0.5 rounded-full text-xs bg-slate-700 text-slate-300 ml-2" title="Hardwired on this node">Built-in</span>'
            : '';
        const removeBtn = p.builtin
            ? `<button class="btn-secondary text-xs opacity-30 cursor-not-allowed" disabled
                       title="Built-in peripherals can't be removed">Remove</button>`
            : `<button class="btn-secondary text-xs hover:bg-red-900/40 hover:border-red-700"
                       onclick="peripheralManager.removePeripheral('${escapeAttr(p.id)}')">Remove</button>`;

        // Logging toggle. Disabled when the peripheral has no input
        // channels — nothing to record.
        const hasInputs = type ? type.channels.some(c => c.dir === 'in') : false;
        const logOn = !!p.log_enabled;
        const logBtn = hasInputs
            ? `<button class="btn-secondary text-xs ${logOn ? 'bg-emerald-900/40 border-emerald-700 text-emerald-200' : ''}"
                       onclick="peripheralManager.toggleLog('${escapeAttr(p.id)}', ${!logOn})"
                       title="${logOn ? 'Disable' : 'Enable'} 30s/60s history + daily NDJSON log">
                    Log ${logOn ? 'on' : 'off'}
               </button>`
            : `<button class="btn-secondary text-xs opacity-30 cursor-not-allowed" disabled
                       title="No input channels to log">Log —</button>`;

        const border = p.builtin ? 'border-slate-600 bg-slate-800/40' : 'border-slate-700 bg-slate-900/40';

        return `
            <div class="border ${border} rounded p-3">
                <div class="flex items-center justify-between gap-2">
                    <div class="min-w-0">
                        <div class="font-semibold text-white truncate">
                            ${escapeHtml(p.label || p.id)}
                            <span class="text-xs text-slate-500 ml-2">${escapeHtml(typeLabel)}</span>
                            ${builtinBadge}
                        </div>
                        <div class="text-xs text-slate-400 mt-1">${pinDesc}</div>
                        <div class="mt-2 flex flex-wrap gap-1">${channelChips}</div>
                    </div>
                    <div class="flex items-center gap-1 shrink-0">
                        ${logBtn}
                        <button class="btn-secondary text-xs"
                                onclick="peripheralManager.openEditModal('${escapeAttr(p.id)}')">Edit</button>
                        ${removeBtn}
                    </div>
                </div>
            </div>`;
    }

    async toggleLog(peripheralId, enabled) {
        try {
            const result = await window.saintWS.management('set_peripheral_log_enabled', {
                node_id: this.selectedNode,
                peripheral_id: peripheralId,
                enabled,
            });
            if (!result || result.success === false) {
                alert(result?.message || 'Failed to toggle logging');
                return;
            }
            // Optimistically flip in-memory + re-render so the button
            // updates without a full round-trip to get_node_peripherals.
            const p = this.peripherals.find(x => x.id === peripheralId);
            if (p) p.log_enabled = !!enabled;
            this.renderList();
            // Notify the live-readings tab so it can backfill/clear its
            // sparkline for this peripheral.
            if (window.nodeLiveManager && typeof window.nodeLiveManager.onLogEnabledChanged === 'function') {
                window.nodeLiveManager.onLogEnabledChanged(this.selectedNode, peripheralId, !!enabled);
            }
        } catch (err) {
            alert('Failed to toggle logging: ' + (err.message || err));
        }
    }

    describePins(p, type) {
        if (!type || !p.pins) return '';
        if (type.pin_kind === 'uart') {
            return `UART: <span class="font-mono">TX=GP${p.pins.uart_tx ?? '?'}, RX=GP${p.pins.uart_rx ?? '?'}</span>`;
        }
        if (type.pin_kind === 'i2c') {
            return `I²C: <span class="font-mono">SDA=GP${p.pins.sda ?? '?'}, SCL=GP${p.pins.scl ?? '?'}</span>`;
        }
        if (type.pin_kind === 'builtin') {
            const pin = Object.values(p.pins)[0];
            return pin !== undefined ? `Hardwired: <span class="font-mono">GP${pin}</span>` : 'Hardwired';
        }
        const pin = p.pins.gpio ?? Object.values(p.pins)[0];
        return pin !== undefined ? `Pin: <span class="font-mono">GP${pin}</span>` : '';
    }

    renderPinSidebar() {
        const freeEl = document.getElementById('peripheral-free-pins');
        const claimedEl = document.getElementById('peripheral-claimed-pins');
        if (!freeEl || !claimedEl) return;

        const pins = (this.capabilities && this.capabilities.pins) || [];
        const claimed = this.buildPinClaimIndex();

        if (pins.length === 0) {
            // Distinguish "haven't heard from the firmware yet" from
            // "this node has no operator-configurable pins" — the host
            // controller is the latter even though it has channels via
            // built-in peripherals.
            const msg = this.capabilities
                ? 'No operator-configurable pins on this node'
                : 'No capabilities yet — click Refresh';
            freeEl.innerHTML = `<span class="text-slate-500 text-xs italic">${msg}</span>`;
            claimedEl.innerHTML = '';
            return;
        }

        const freeHtml = pins
            .filter(pin => !claimed[pin.gpio])
            .map(pin => `<span class="inline-block px-1.5 py-0.5 rounded text-xs font-mono mr-1 mb-1 bg-teal-900/40 text-teal-200">GP${pin.gpio}<span class="text-slate-500 ml-1">${escapeHtml(pin.name)}</span></span>`)
            .join('');
        freeEl.innerHTML = freeHtml || '<span class="text-slate-500 text-xs italic">All pins claimed</span>';

        const grouped = {};
        for (const pin of pins) {
            const owner = claimed[pin.gpio];
            if (!owner) continue;
            grouped[owner.id] = grouped[owner.id] || { label: owner.label, gpios: [] };
            grouped[owner.id].gpios.push(`GP${pin.gpio}`);
        }
        claimedEl.innerHTML = Object.values(grouped)
            .map(g => `<div><span class="text-slate-300">${escapeHtml(g.label)}</span> <span class="font-mono text-slate-500">${g.gpios.join(', ')}</span></div>`)
            .join('') || '<span class="text-slate-500 text-xs italic">None</span>';
    }

    buildPinClaimIndex() {
        // Mirror of PeripheralInstance.claimed_gpios() on the server:
        // walk both `pins` AND any `params` whose catalog schema declares
        // type === "gpio" (e.g. RoboClaw estop_pin). A value of 0 is the
        // "no pin assigned" sentinel and is skipped.
        const claimed = {};
        const types = this.catalog?.peripheral_types || [];
        const gpioParamsByType = {};
        for (const t of types) {
            gpioParamsByType[t.id] = (t.params || [])
                .filter(p => p.type === 'gpio')
                .map(p => p.id);
        }
        for (const p of this.peripherals) {
            for (const v of Object.values(p.pins || {})) {
                if (typeof v === 'number' && v > 0) claimed[v] = p;
            }
            const gpioParams = gpioParamsByType[p.type] || [];
            for (const paramId of gpioParams) {
                const v = p.params?.[paramId];
                if (typeof v === 'number' && v > 0) claimed[v] = p;
            }
        }
        return claimed;
    }

    updateSyncStatus(status) {
        this.syncStatus = status || 'unknown';
        const el = document.getElementById('node-sync-status');
        if (!el) return;
        const map = {
            synced:       ['Synced',       'bg-emerald-500'],
            pending:      ['Pending',      'bg-amber-500'],
            error:        ['Error',        'bg-red-500'],
            unknown:      ['Unknown',      'bg-slate-500'],
            unconfigured: ['Unconfigured', 'bg-slate-500'],
        };
        const [label, dotCls] = map[this.syncStatus] || map.unknown;
        const dot = el.querySelector('.sync-dot');
        const text = el.querySelector('.sync-text');
        if (dot) dot.className = `sync-dot w-2 h-2 rounded-full ${dotCls}`;
        if (text) text.textContent = label;
    }

    // ─── Modal: add / edit peripheral ───────────────────────────────────

    openAddModal() {
        if (!this.catalog) return;
        this.modalMode = 'add';
        this.modalEditingId = null;
        this.populateTypeSelect(null);
        this.openModal('Add peripheral');

        // If the firmware hasn't reported its capabilities yet, the pin
        // selectors will be empty and the user can't pick a valid pin.
        // Surface an explicit warning + kick off a refresh — the modal
        // re-renders when the broadcast arrives via setupWebSocketHandlers.
        const hasPins = this.capabilities
            && Array.isArray(this.capabilities.pins)
            && this.capabilities.pins.length > 0;
        const hasUart = this.capabilities
            && Array.isArray(this.capabilities.uart_pairs)
            && this.capabilities.uart_pairs.length > 0;
        if (!hasPins && !hasUart) {
            const err = document.getElementById('peripheral-modal-error');
            if (err) {
                err.textContent = 'Capabilities not loaded yet — refreshing from firmware. '
                                + 'Wait a moment then re-open this dialog, or close and reopen the Peripherals tab.';
                err.classList.remove('hidden');
            }
            this.refreshCapabilities();
        }

        this.renderModalBody();
    }

    openEditModal(peripheralId) {
        if (!this.catalog) return;
        const p = this.peripherals.find(x => x.id === peripheralId);
        if (!p) return;
        this.modalMode = 'edit';
        this.modalEditingId = peripheralId;
        this.populateTypeSelect(p.type, p.builtin);
        this.openModal(p.builtin ? 'Edit built-in peripheral' : 'Edit peripheral');
        document.getElementById('peripheral-modal-label').value = p.label || '';
        this.renderModalBody(p);
    }

    openModal(title) {
        const modal = document.getElementById('peripheral-modal');
        if (!modal) return;
        document.getElementById('peripheral-modal-title').textContent = title;
        const err = document.getElementById('peripheral-modal-error');
        if (err) { err.classList.add('hidden'); err.textContent = ''; }
        modal.classList.remove('hidden');
    }

    closeModal() {
        const modal = document.getElementById('peripheral-modal');
        if (modal) modal.classList.add('hidden');
    }

    populateTypeSelect(selectedType, isBuiltin = false) {
        const sel = document.getElementById('peripheral-modal-type');
        if (!sel) return;
        // Filter out builtin_only types from the add-list (they're seeded by firmware)
        const types = (this.catalog?.peripheral_types || []).filter(t =>
            isBuiltin ? t.id === selectedType : !t.builtin_only);
        sel.innerHTML = types
            .map(t => `<option value="${escapeAttr(t.id)}">${escapeHtml(t.label)}</option>`)
            .join('');
        if (selectedType) sel.value = selectedType;
        sel.disabled = isBuiltin || this.modalMode === 'edit';
        sel.onchange = () => this.renderModalBody();
    }

    renderModalBody(existing = null) {
        const sel = document.getElementById('peripheral-modal-type');
        const typeId = sel ? sel.value : null;
        if (!typeId) return;
        const type = this.typeFor(typeId);
        if (!type) return;

        document.getElementById('peripheral-modal-type-desc').textContent = type.description || '';

        // Default label
        const labelEl = document.getElementById('peripheral-modal-label');
        if (!existing && !labelEl.value) labelEl.value = type.label;

        // Pin section
        const pinSection = document.getElementById('peripheral-modal-pin-section');
        pinSection.innerHTML = this.renderPinSelector(type, existing);
        this.wirePinSelector(type, existing);

        // Params section
        const paramsSection = document.getElementById('peripheral-modal-params-section');
        paramsSection.innerHTML = this.renderParamsFor(type, existing);
    }

    renderPinSelector(type, existing) {
        const claimed = this.buildClaimedExcluding(existing?.id);

        if (type.pin_kind === 'builtin') {
            const pin = existing ? Object.values(existing.pins)[0] : '?';
            return `<div class="text-xs text-slate-400">Hardwired pin: <span class="font-mono">GP${pin}</span> (cannot be changed)</div>`;
        }

        if (type.pin_kind === 'uart') {
            const uartPairs = (this.capabilities && this.capabilities.uart_pairs) || [];
            const options = uartPairs.map(pair => {
                const conflict = claimed[pair.tx] || claimed[pair.rx];
                const note = conflict ? ` — in use by ${conflict.label}` : '';
                return `<option value="${pair.tx}:${pair.rx}" ${conflict ? 'disabled' : ''}>UART${pair.uart}: TX=GP${pair.tx}, RX=GP${pair.rx}${note}</option>`;
            }).join('');
            return `
                <label class="block text-sm font-medium text-slate-300 mb-1">UART pins</label>
                <select id="peripheral-modal-pin-uart" class="input-field w-full">${options}</select>
                <p class="text-xs text-slate-500 mt-1">Selects which hardware UART and pin pair this peripheral uses.</p>`;
        }

        // gpio / pwm / adc — single pin selector filtered by capability
        const capNeeded = (
            type.pin_kind === 'pwm' ? 'pwm' :
            type.pin_kind === 'adc' ? 'adc' :
            null);
        const pinList = ((this.capabilities && this.capabilities.pins) || [])
            .filter(p => !capNeeded || (p.capabilities || []).includes(capNeeded));
        const options = pinList.map(p => {
            const conflict = claimed[p.gpio];
            const note = conflict ? ` — in use by ${conflict.label}` : '';
            return `<option value="${p.gpio}" ${conflict ? 'disabled' : ''}>GP${p.gpio} (${escapeHtml(p.name)})${note}</option>`;
        }).join('');
        const label = type.pin_kind === 'pwm' ? 'PWM pin'
                    : type.pin_kind === 'adc' ? 'Analog pin'
                    : 'Pin';
        return `
            <label class="block text-sm font-medium text-slate-300 mb-1">${label}</label>
            <select id="peripheral-modal-pin-single" class="input-field w-full">${options || '<option value="">No compatible pins</option>'}</select>`;
    }

    wirePinSelector(type, existing) {
        if (type.pin_kind === 'uart') {
            const sel = document.getElementById('peripheral-modal-pin-uart');
            if (sel && existing && existing.pins.uart_tx !== undefined) {
                sel.value = `${existing.pins.uart_tx}:${existing.pins.uart_rx}`;
            }
        } else if (type.pin_kind !== 'builtin') {
            const sel = document.getElementById('peripheral-modal-pin-single');
            if (sel && existing && existing.pins) {
                const v = existing.pins.gpio ?? Object.values(existing.pins)[0];
                if (v !== undefined) sel.value = String(v);
            }
        }
    }

    renderParamsFor(type, existing) {
        if (!type.params || type.params.length === 0) return '';
        // Compute "what pins are already claimed by other peripherals"
        // once, so every gpio-typed param on this peripheral shares
        // the same exclusion set. excludeId = the peripheral being
        // edited, so its own currently-selected pins/params don't
        // show up disabled in its own dropdowns.
        const claimed = this.buildClaimedExcluding(existing ? existing.id : null);
        const rows = type.params.map(p => {
            const value = (existing && existing.params && existing.params[p.id] !== undefined)
                ? existing.params[p.id]
                : p.default;
            const id = `peripheral-modal-param-${p.id}`;
            if (p.type === 'bool') {
                return `
                    <label class="flex items-center gap-2 text-sm text-slate-300">
                        <input type="checkbox" id="${escapeAttr(id)}" data-param="${escapeAttr(p.id)}" data-ptype="bool"
                               ${value ? 'checked' : ''} class="rounded bg-slate-700 border-slate-600">
                        ${escapeHtml(p.label)}
                    </label>`;
            }
            if (p.type === 'gpio') {
                // Dropdown of the controller's GPIO pins, excluding
                // those already claimed by another peripheral. A 0
                // "— None —" option lets the operator clear it; the
                // firmware treats estop_pin=0 as "no E-stop wired".
                const pinList = (this.capabilities && this.capabilities.pins) || [];
                const currentValue = (typeof value === 'number') ? value : 0;
                const options = pinList.map(pin => {
                    const isSelf = pin.gpio === currentValue;
                    const conflict = !isSelf && claimed[pin.gpio];
                    const note = conflict ? ` — in use by ${conflict.label}` : '';
                    return `<option value="${pin.gpio}" ${conflict ? 'disabled' : ''} ${isSelf ? 'selected' : ''}>GP${pin.gpio} (${escapeHtml(pin.name)})${note}</option>`;
                }).join('');
                return `
                    <div>
                        <label class="block text-xs text-slate-400 mb-1">${escapeHtml(p.label)}</label>
                        <select id="${escapeAttr(id)}" data-param="${escapeAttr(p.id)}" data-ptype="gpio" class="input-field w-full">
                            <option value="0" ${currentValue === 0 ? 'selected' : ''}>— None —</option>
                            ${options}
                        </select>
                    </div>`;
            }
            if (p.type === 'int' || p.type === 'float') {
                const step = p.type === 'int' ? '1' : '0.01';
                const min = (p.min !== undefined) ? `min="${p.min}"` : '';
                const max = (p.max !== undefined) ? `max="${p.max}"` : '';
                return `
                    <div>
                        <label class="block text-xs text-slate-400 mb-1">${escapeHtml(p.label)}</label>
                        <input type="number" id="${escapeAttr(id)}" data-param="${escapeAttr(p.id)}" data-ptype="${p.type}"
                               value="${value}" step="${step}" ${min} ${max} class="input-field w-full">
                    </div>`;
            }
            // string
            return `
                <div>
                    <label class="block text-xs text-slate-400 mb-1">${escapeHtml(p.label)}</label>
                    <input type="text" id="${escapeAttr(id)}" data-param="${escapeAttr(p.id)}" data-ptype="string"
                           value="${escapeAttr(String(value ?? ''))}" class="input-field w-full">
                </div>`;
        }).join('');
        return `<div class="space-y-3 pt-2 border-t border-slate-700">${rows}</div>`;
    }

    buildClaimedExcluding(excludeId) {
        // Index of "GPIO number → peripheral that owns it." Covers
        // both the pin_kind selection (uart_tx, uart_rx, gpio) AND
        // params declared with type="gpio" (E-stop pin and any
        // future side-channel pins), so the modal's pin and gpio-
        // param dropdowns mark the same pin as in-use no matter
        // which slot it was assigned through.
        const claimed = {};
        for (const p of this.peripherals) {
            if (p.id === excludeId) continue;
            for (const v of Object.values(p.pins || {})) {
                if (typeof v === 'number') claimed[v] = p;
            }
            const type = this.typeFor(p.type);
            if (type && type.params) {
                for (const param of type.params) {
                    if (param.type !== 'gpio') continue;
                    const v = p.params ? p.params[param.id] : undefined;
                    // 0 is the "unset" sentinel — a real GPIO pick
                    // is 1..29 on RP2040, so 0 never collides with
                    // a legitimate selection.
                    if (typeof v === 'number' && v > 0) claimed[v] = p;
                }
            }
        }
        return claimed;
    }

    async saveModal() {
        const sel = document.getElementById('peripheral-modal-type');
        if (!sel) return;
        const typeId = sel.value;
        const type = this.typeFor(typeId);
        if (!type) return;

        // Collect pins
        let pins = {};
        if (type.pin_kind === 'uart') {
            const v = document.getElementById('peripheral-modal-pin-uart').value;
            if (!v) return this.showModalError('Pick a UART pin pair');
            const [tx, rx] = v.split(':').map(n => parseInt(n, 10));
            pins = { uart_tx: tx, uart_rx: rx };
        } else if (type.pin_kind === 'builtin') {
            // Built-in pins are hardwired; preserve existing.
            const existing = this.modalEditingId
                ? this.peripherals.find(p => p.id === this.modalEditingId)
                : null;
            pins = existing ? { ...existing.pins } : {};
        } else {
            const v = document.getElementById('peripheral-modal-pin-single').value;
            if (!v) return this.showModalError('Pick a pin');
            pins = { gpio: parseInt(v, 10) };
        }

        // Collect params
        const params = {};
        const section = document.getElementById('peripheral-modal-params-section');
        if (section) {
            section.querySelectorAll('[data-param]').forEach(el => {
                const id = el.getAttribute('data-param');
                const ptype = el.getAttribute('data-ptype');
                if (ptype === 'bool') params[id] = el.checked;
                else if (ptype === 'int' || ptype === 'gpio')
                    params[id] = parseInt(el.value, 10) || 0;
                else if (ptype === 'float') params[id] = parseFloat(el.value) || 0;
                else params[id] = el.value;
            });
        }

        const label = document.getElementById('peripheral-modal-label').value.trim() || type.label;

        const peripheral = {
            type: typeId,
            label,
            pins,
            params,
        };
        if (this.modalEditingId) peripheral.id = this.modalEditingId;

        try {
            const result = await window.saintWS.management('save_node_peripheral', {
                node_id: this.selectedNode,
                peripheral,
            });
            if (!result || result.success === false) {
                this.showModalError(result?.message || 'Save failed');
                return;
            }
            // Refresh and close
            await this.reload();
            this.closeModal();
        } catch (err) {
            this.showModalError(err.message || String(err));
        }
    }

    showModalError(msg) {
        const el = document.getElementById('peripheral-modal-error');
        if (!el) return;
        el.textContent = msg;
        el.classList.remove('hidden');
    }

    async removePeripheral(peripheralId) {
        if (!confirm('Remove this peripheral? Any routes touching it will be deleted too.')) return;
        try {
            const result = await window.saintWS.management('remove_node_peripheral', {
                node_id: this.selectedNode,
                peripheral_id: peripheralId,
            });
            if (!result || result.success === false) {
                alert(result?.message || 'Remove failed');
                return;
            }
            await this.reload();
        } catch (err) {
            alert('Failed to remove peripheral: ' + (err.message || err));
        }
    }

    async reload() {
        const ws = window.saintWS;
        const periph = await ws.management('get_node_peripherals', { node_id: this.selectedNode });
        this.peripherals = (periph && periph.peripherals) ? periph.peripherals : [];
        this.updateSyncStatus(periph?.sync_status || 'unknown');
        this.renderAll();
    }

    async syncToNode() {
        if (!this.selectedNode) {
            alert('No node selected — open a node first.');
            return;
        }
        try {
            const result = await window.saintWS.management('sync_node_peripherals', {
                node_id: this.selectedNode,
            });
            if (result && result.success === false) {
                alert(result.message || 'Sync failed');
                return;
            }
            this.updateSyncStatus('pending');
        } catch (err) {
            console.error('Sync failed:', err);
            this.updateSyncStatus('error');
            alert('Sync failed: ' + (err.message || err));
        }
    }

    typeFor(typeId) {
        return (this.catalog?.peripheral_types || []).find(t => t.id === typeId);
    }
}

// ─── escape helpers (kept local since other JS files have their own) ────

function escapeHtml(s) {
    if (s === null || s === undefined) return '';
    return String(s)
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#39;');
}
function escapeAttr(s) { return escapeHtml(s); }

const peripheralManager = new PeripheralManager();

document.addEventListener('DOMContentLoaded', () => {
    peripheralManager.init();
});
