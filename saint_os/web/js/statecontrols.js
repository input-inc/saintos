/**
 * SAINT.OS State tab — per-peripheral controls.
 *
 * Enumerates the node's peripherals from the server and renders one
 * card per peripheral, with a widget for every `dir: "out"` channel.
 * Slider for analog, toggle for digital, color picker for rgb. Values
 * round-trip via `control / set_channel_value`, keyed on
 * (peripheral_id, channel_id) — the server translates to firmware GPIO.
 *
 * The card list is built from the configured catalog, not from runtime
 * hardware probes, so controls are present even before the peripheral
 * is wired up. The connection state is shown as a badge on the header.
 */

class StateControlManager {
    constructor() {
        this.nodeId = null;
        this.catalog = null;            // { peripheral_types: [...] }
        this.peripherals = [];          // PeripheralInstance[]
        this.peripheralConnected = {};  // peripheral_id -> bool
        this.values = new Map();        // "peripheral_id/channel_id" -> {value, ts}
        this._activeControl = null;     // suppress live updates while dragging
        this._subscribed = null;
        this._throttleTimers = {};
        this._throttleMs = 50;
        this._lastSendAt = {};          // gpio-ish key -> ms
    }

    init() {
        const ws = window.saintWS;
        if (!ws) return;
        ws.on('state', (msg) => {
            if (!msg || !msg.node) return;
            if (!msg.node.startsWith('pin_state/')) return;
            const nodeId = msg.node.slice('pin_state/'.length);
            if (nodeId !== this.nodeId) return;
            this._ingest(msg.data);
        });
    }

    async setNode(nodeId, nodeInfo) {
        await this._unsubscribe();
        this.nodeId = nodeId || null;
        this.values.clear();
        this.peripheralConnected = (nodeInfo && nodeInfo.peripheral_connected) || {};

        const container = document.getElementById('peripheral-controls-container');
        if (container) {
            container.innerHTML = '<p class="text-slate-400 text-sm">Loading controls…</p>';
        }
        if (!nodeId) return;

        const ws = window.saintWS;
        try {
            await ws.subscribe([`pin_state/${nodeId}`], 5);
            this._subscribed = `pin_state/${nodeId}`;
            const [catalog, periph] = await Promise.all([
                this._ensureCatalog(),
                ws.management('get_node_peripherals', { node_id: nodeId }),
            ]);
            this.catalog = catalog;
            this.peripherals = (periph && periph.peripherals) || [];
            this._render();
        } catch (err) {
            console.error('StateControlManager: failed to load:', err);
            if (container) {
                container.innerHTML =
                    `<p class="text-red-300 text-sm">Failed to load: ${escapeHtml(err.message || err)}</p>`;
            }
        }
    }

    async _unsubscribe() {
        if (!this._subscribed) return;
        try { await window.saintWS.unsubscribe([this._subscribed]); } catch (_) {}
        this._subscribed = null;
    }

    async _ensureCatalog() {
        if (this.catalog) return this.catalog;
        try {
            return await window.saintWS.management('get_peripheral_catalog', {}) || { peripheral_types: [] };
        } catch (_) {
            return { peripheral_types: [] };
        }
    }

    _ingest(data) {
        if (!data) return;
        const channels = Array.isArray(data.channels) ? data.channels : [];
        for (const ch of channels) {
            if (!ch.peripheral_id || !ch.channel_id) continue;
            const key = `${ch.peripheral_id}/${ch.channel_id}`;
            this.values.set(key, { value: ch.value, ts: ch.last_updated || Date.now() / 1000 });
            this._reflectValue(ch.peripheral_id, ch.channel_id, ch.value);
        }
    }

    _typeFor(typeId) {
        const types = (this.catalog && this.catalog.peripheral_types) || [];
        return types.find(t => t.id === typeId);
    }

    _render() {
        const container = document.getElementById('peripheral-controls-container');
        if (!container) return;
        if (!this.peripherals.length) {
            container.innerHTML = `
                <div class="p-4 bg-slate-900/40 border border-slate-700 rounded-lg text-sm text-slate-300">
                    No peripherals configured on this node yet.
                    Add some in the <button class="text-cyan-400 underline" onclick="app.switchNodeTab('peripherals')">Peripherals</button> tab.
                </div>`;
            return;
        }

        container.innerHTML = this.peripherals
            .map(p => this._renderPeripheralCard(p)).join('');
        this._wireEvents();
        // Reflect any values already received.
        for (const [key, v] of this.values.entries()) {
            const [pid, cid] = key.split('/');
            this._reflectValue(pid, cid, v.value);
        }
    }

    _renderPeripheralCard(peripheral) {
        const type = this._typeFor(peripheral.type);
        const typeLabel = type ? type.label : peripheral.type;
        const channels = (type && type.channels) || [];
        const writable = channels.filter(c => c.dir === 'out');
        const readable = channels.filter(c => c.dir === 'in');

        const connected = this.peripheralConnected[peripheral.id];
        const badge = (connected === false)
            ? '<span class="px-2 py-0.5 rounded-full text-xs bg-rose-900/40 text-rose-300 ml-2">Disconnected</span>'
            : (connected === true
                ? '<span class="px-2 py-0.5 rounded-full text-xs bg-emerald-900/40 text-emerald-300 ml-2">Connected</span>'
                : '');
        const builtinBadge = peripheral.builtin
            ? '<span class="px-2 py-0.5 rounded-full text-xs bg-slate-700 text-slate-300 ml-2">Built-in</span>'
            : '';

        let body;
        if (writable.length === 0) {
            body = `<p class="text-xs text-slate-500 italic">No writable channels — this peripheral only emits readings.</p>`;
        } else {
            body = writable.map(c => this._renderChannel(peripheral, c)).join('');
        }

        const readSummary = readable.length
            ? `<div class="mt-3 pt-3 border-t border-slate-800 text-xs text-slate-500">
                 Readings: ${readable.map(c => escapeHtml(c.display || c.id)).join(', ')}
                 — see the <button class="text-cyan-400 underline" onclick="app.switchNodeTab('live')">Live</button> tab.
               </div>`
            : '';

        return `
            <div class="card peripheral-control-card" data-peripheral-id="${escapeAttr(peripheral.id)}">
                <div class="flex items-center justify-between mb-3">
                    <div>
                        <h4 class="text-base font-semibold text-white">${escapeHtml(peripheral.label || peripheral.id)}</h4>
                        <div class="text-xs text-slate-500">${escapeHtml(typeLabel)} · <span class="font-mono">${escapeHtml(peripheral.id)}</span>${builtinBadge}${badge}</div>
                    </div>
                </div>
                <div class="space-y-3">${body}</div>
                ${readSummary}
            </div>`;
    }

    _channelSpec(peripheralType, channel) {
        // Per-(type, channel) widget hints. Falls back to channel.cap.
        const key = `${peripheralType}/${channel.id}`;
        const overrides = {
            'roboclaw/motor': { kind: 'slider', min: -1,  max: 1,   step: 0.01, neutral: 0, fmt: v => `${(v * 100).toFixed(0)}%`, hint: '-100% reverse · 0 stop · +100% forward' },
            'syren/motor':    { kind: 'slider', min: -1,  max: 1,   step: 0.01, neutral: 0, fmt: v => `${(v * 100).toFixed(0)}%`, hint: '-100% reverse · 0 stop · +100% forward' },
            'servo/angle':    { kind: 'slider', min: 0,   max: 180, step: 1,    neutral: 90, fmt: v => `${v.toFixed(0)}°` },
            'led/on':         { kind: 'toggle' },
            'neopixel/color':      { kind: 'color',  unsupported: true, note: 'Not yet routable through firmware control path.' },
            'neopixel/brightness': { kind: 'slider', min: 0, max: 1, step: 0.01, neutral: 0.5, fmt: v => `${(v * 100).toFixed(0)}%`, unsupported: true, note: 'Not yet routable through firmware control path.' },
        };
        if (overrides[key]) return overrides[key];
        // Fallbacks by capability tag
        if (channel.cap === 'digital_out') return { kind: 'toggle' };
        if (channel.cap === 'rgb')         return { kind: 'color', unsupported: true, note: 'No firmware control path yet.' };
        // Generic analog
        return { kind: 'slider', min: 0, max: 1, step: 0.01, neutral: 0, fmt: v => v.toFixed(2) };
    }

    _renderChannel(peripheral, channel) {
        const spec = this._channelSpec(peripheral.type, channel);
        const idAttr = escapeAttr(`${peripheral.id}--${channel.id}`);
        const label = escapeHtml(channel.display || channel.id);
        const unsupported = spec.unsupported
            ? `<div class="text-xs text-amber-400/80 mt-1">${escapeHtml(spec.note || 'Not yet supported.')}</div>`
            : '';

        if (spec.kind === 'slider') {
            const init = spec.neutral !== undefined ? spec.neutral : (spec.min + spec.max) / 2;
            const initFmt = spec.fmt ? spec.fmt(init) : String(init);
            const hintRow = spec.hint
                ? `<div class="text-xs text-slate-500 mt-1">${escapeHtml(spec.hint)}</div>`
                : '';
            return `
                <div class="control-channel" data-peripheral-id="${escapeAttr(peripheral.id)}" data-channel-id="${escapeAttr(channel.id)}" data-kind="slider">
                    <div class="flex items-center justify-between mb-2">
                        <span class="text-sm font-medium text-white">${label}</span>
                        <div class="flex items-center gap-2">
                            <span class="channel-value-display text-sm text-cyan-400 font-mono w-20 text-right">${escapeHtml(initFmt)}</span>
                            <button class="channel-neutral-btn text-xs text-slate-400 hover:text-slate-200 underline" title="Reset to ${initFmt}">reset</button>
                        </div>
                    </div>
                    <input type="range" min="${spec.min}" max="${spec.max}" step="${spec.step}" value="${init}"
                           class="channel-slider w-full" data-spec='${escapeAttr(JSON.stringify({min: spec.min, max: spec.max, neutral: init}))}' />
                    <div class="flex justify-between mt-1">
                        <span class="text-xs text-slate-500">${spec.fmt ? spec.fmt(spec.min) : spec.min}</span>
                        <span class="text-xs text-slate-500">${spec.fmt ? spec.fmt(init) : init}</span>
                        <span class="text-xs text-slate-500">${spec.fmt ? spec.fmt(spec.max) : spec.max}</span>
                    </div>
                    ${hintRow}
                    ${unsupported}
                </div>`;
        }
        if (spec.kind === 'toggle') {
            return `
                <div class="control-channel flex items-center justify-between" data-peripheral-id="${escapeAttr(peripheral.id)}" data-channel-id="${escapeAttr(channel.id)}" data-kind="toggle">
                    <span class="text-sm font-medium text-white">${label}</span>
                    <button class="channel-toggle relative inline-flex h-6 w-11 items-center rounded-full bg-slate-700 transition-colors" data-state="off">
                        <span class="toggle-dot inline-block h-4 w-4 transform rounded-full bg-slate-400 transition-transform translate-x-1"></span>
                    </button>
                </div>`;
        }
        if (spec.kind === 'color') {
            return `
                <div class="control-channel" data-peripheral-id="${escapeAttr(peripheral.id)}" data-channel-id="${escapeAttr(channel.id)}" data-kind="color">
                    <div class="flex items-center justify-between mb-2">
                        <span class="text-sm font-medium text-white">${label}</span>
                        <input type="color" class="channel-color h-8 w-16 cursor-pointer bg-slate-900 border border-slate-700 rounded" value="#1e293b" />
                    </div>
                    ${unsupported}
                </div>`;
        }
        return `<div class="text-xs text-slate-500">Unsupported channel kind: ${escapeHtml(spec.kind)}</div>`;
    }

    _wireEvents() {
        const root = document.getElementById('peripheral-controls-container');
        if (!root) return;

        root.querySelectorAll('.control-channel[data-kind="slider"]').forEach(el => {
            const slider = el.querySelector('.channel-slider');
            const display = el.querySelector('.channel-value-display');
            const reset = el.querySelector('.channel-neutral-btn');
            const peripheralId = el.dataset.peripheralId;
            const channelId = el.dataset.channelId;
            const spec = JSON.parse(slider.dataset.spec);

            const sendNow = (v) => this._sendChannelValue(peripheralId, channelId, v);

            slider.addEventListener('input', () => {
                this._activeControl = `${peripheralId}/${channelId}`;
                const v = parseFloat(slider.value);
                if (display) display.textContent = this._formatForSpec(spec, v);
                this._throttledSend(peripheralId, channelId, v);
            });
            slider.addEventListener('change', () => {
                this._activeControl = null;
            });
            if (reset) {
                reset.addEventListener('click', () => {
                    slider.value = spec.neutral;
                    if (display) display.textContent = this._formatForSpec(spec, spec.neutral);
                    sendNow(spec.neutral);
                });
            }
        });

        root.querySelectorAll('.control-channel[data-kind="toggle"]').forEach(el => {
            const btn = el.querySelector('.channel-toggle');
            const peripheralId = el.dataset.peripheralId;
            const channelId = el.dataset.channelId;
            btn.addEventListener('click', () => {
                const newOn = btn.dataset.state !== 'on';
                this._setToggleVisualState(btn, newOn);
                this._sendChannelValue(peripheralId, channelId, newOn ? 1 : 0);
            });
        });

        root.querySelectorAll('.control-channel[data-kind="color"]').forEach(el => {
            const picker = el.querySelector('.channel-color');
            const peripheralId = el.dataset.peripheralId;
            const channelId = el.dataset.channelId;
            picker.addEventListener('change', () => {
                // Pack #RRGGBB into a uint24 carried in a float. Firmware
                // path doesn't consume this yet — we send anyway so the
                // wire format is exercised end-to-end.
                const hex = picker.value.replace('#', '');
                const packed = parseInt(hex, 16);
                this._sendChannelValue(peripheralId, channelId, packed);
            });
        });
    }

    _formatForSpec(spec, v) {
        // The override fmt isn't carried through dataset (function),
        // so re-derive a reasonable default from min/max.
        if (spec.max - spec.min <= 2.0 + 1e-9) return `${(v * 100).toFixed(0)}%`;
        if (spec.max - spec.min <= 180.5)      return `${v.toFixed(0)}°`;
        return v.toFixed(2);
    }

    _setToggleVisualState(btn, isOn) {
        btn.dataset.state = isOn ? 'on' : 'off';
        const dot = btn.querySelector('.toggle-dot');
        if (isOn) {
            btn.classList.add('bg-emerald-500'); btn.classList.remove('bg-slate-700');
            if (dot) { dot.classList.add('translate-x-6', 'bg-white'); dot.classList.remove('translate-x-1', 'bg-slate-400'); }
        } else {
            btn.classList.remove('bg-emerald-500'); btn.classList.add('bg-slate-700');
            if (dot) { dot.classList.remove('translate-x-6', 'bg-white'); dot.classList.add('translate-x-1', 'bg-slate-400'); }
        }
    }

    _throttledSend(peripheralId, channelId, value) {
        const key = `${peripheralId}/${channelId}`;
        const now = Date.now();
        const last = this._lastSendAt[key] || 0;
        if (now - last >= this._throttleMs) {
            this._lastSendAt[key] = now;
            this._sendChannelValue(peripheralId, channelId, value);
            return;
        }
        if (this._throttleTimers[key]) clearTimeout(this._throttleTimers[key]);
        this._throttleTimers[key] = setTimeout(() => {
            this._lastSendAt[key] = Date.now();
            this._sendChannelValue(peripheralId, channelId, value);
        }, this._throttleMs - (now - last));
    }

    async _sendChannelValue(peripheralId, channelId, value) {
        if (!this.nodeId) return;
        try {
            await window.saintWS.send({
                type: 'control',
                action: 'set_channel_value',
                params: {
                    node_id: this.nodeId,
                    peripheral_id: peripheralId,
                    channel_id: channelId,
                    value: value,
                },
            });
        } catch (e) {
            console.warn('set_channel_value failed:', e);
        }
    }

    _reflectValue(peripheralId, channelId, value) {
        // Don't fight the user's hand while they're dragging this control.
        if (this._activeControl === `${peripheralId}/${channelId}`) return;
        if (value === null || value === undefined) return;

        const root = document.getElementById('peripheral-controls-container');
        if (!root) return;
        const el = root.querySelector(
            `.control-channel[data-peripheral-id="${cssEscape(peripheralId)}"][data-channel-id="${cssEscape(channelId)}"]`
        );
        if (!el) return;

        const kind = el.dataset.kind;
        if (kind === 'slider') {
            const slider = el.querySelector('.channel-slider');
            const display = el.querySelector('.channel-value-display');
            if (slider) {
                slider.value = value;
                if (display) {
                    const spec = JSON.parse(slider.dataset.spec);
                    display.textContent = this._formatForSpec(spec, parseFloat(value));
                }
            }
        } else if (kind === 'toggle') {
            const btn = el.querySelector('.channel-toggle');
            if (btn) this._setToggleVisualState(btn, value >= 0.5);
        }
    }
}

function cssEscape(s) {
    return String(s).replace(/["\\]/g, '\\$&');
}

const stateControlManager = new StateControlManager();
window.stateControlManager = stateControlManager;
document.addEventListener('DOMContentLoaded', () => stateControlManager.init());
