/**
 * SAINT.OS Dashboard Widgets
 *
 * Renders one card per operator-configured widget (see Routing page).
 * Each card shows its inputs and the route source feeding each input;
 * live values are pulled from per-channel telemetry the firmware emits
 * via the routing engine.
 *
 * Until the firmware-side routing engine ships, live values render as
 * "—". The widget structure, labels, and routing summary are correct
 * — only the per-input numbers are stubbed.
 */

class WidgetsDashboard {
    constructor() {
        this.catalog = null;
        this.routing = { version: 0, routes: [], widgets: [] };
        this.nodesById = new Map();           // node_id -> {peripherals:[], display_name}
        this._lastValues = new Map();         // "widget_id/input_id" -> {value, lastUpdate}
        this._pinStateSubs = new Set();
        this._refreshInterval = null;
        this._stateHandler = null;
    }

    init() {
        const btn = document.getElementById('btn-dashboard-add-widget');
        if (btn) btn.addEventListener('click', () => window.app.showPage('routes'));

        // Listen for system_routing broadcasts (a remote operator edits
        // a widget, we want to reflect it).
        const ws = window.saintWS;
        if (ws) {
            this._stateHandler = (msg) => {
                if (!msg || !msg.node) return;
                if (msg.node === 'system_routing') {
                    this.routing = msg.data || { version: 0, routes: [], widgets: [] };
                    if (this._active) this.render();
                } else if (msg.node.startsWith('pin_state/')) {
                    const nodeId = msg.node.slice('pin_state/'.length);
                    this._ingestPinState(nodeId, msg.data);
                }
            };
            ws.on('state', this._stateHandler);
        }
    }

    async activate(adoptedNodes) {
        this._active = true;
        const ws = window.saintWS;

        try {
            await ws.subscribe(['system_routing']);
        } catch (e) {
            console.warn('Failed to subscribe to system_routing:', e);
        }

        try {
            const [catalog, routing] = await Promise.all([
                this._ensureCatalog(),
                ws.management('get_system_routing', {}),
            ]);
            this.catalog = catalog;
            this.routing = routing || { version: 0, routes: [], widgets: [] };
            await this._refreshNodes(adoptedNodes);
            await this._subscribePinStateForAllNodes();
            this.render();
        } catch (err) {
            console.error('Widgets activate failed:', err);
        }
    }

    deactivate() {
        this._active = false;
        // Drop pin_state subscriptions so we don't keep the broadcast
        // pipe busy when nobody's watching the dashboard.
        if (this._pinStateSubs.size > 0) {
            const topics = [...this._pinStateSubs];
            this._pinStateSubs.clear();
            window.saintWS?.unsubscribe(topics).catch(() => {});
        }
    }

    async _ensureCatalog() {
        if (this.catalog) return this.catalog;
        const r = await window.saintWS.management('get_peripheral_catalog', {});
        this.catalog = r || { peripheral_types: [], widget_types: [] };
        return this.catalog;
    }

    async _refreshNodes(adoptedNodes) {
        const ws = window.saintWS;
        const list = adoptedNodes || [];
        this.nodesById = new Map();
        await Promise.all(list.map(async (n) => {
            try {
                const r = await ws.management('get_node_peripherals', { node_id: n.node_id });
                this.nodesById.set(n.node_id, {
                    display_name: n.display_name || n.node_id,
                    peripherals: r?.peripherals || [],
                });
            } catch (e) {
                this.nodesById.set(n.node_id, { display_name: n.display_name || n.node_id, peripherals: [] });
            }
        }));
    }

    async _subscribePinStateForAllNodes() {
        const ws = window.saintWS;
        if (!ws) return;
        const desired = new Set(
            [...this.nodesById.keys()].map(id => `pin_state/${id}`)
        );
        const toAdd = [...desired].filter(t => !this._pinStateSubs.has(t));
        if (toAdd.length === 0) return;
        try {
            await ws.subscribe(toAdd, 5);
            for (const t of toAdd) this._pinStateSubs.add(t);
        } catch (e) {
            console.warn('pin_state subscribe failed:', e);
        }
    }

    // ─── Routing — given a widget input, find the source endpoint ────

    sourceForInput(widgetId, inputId) {
        for (const r of (this.routing.routes || [])) {
            if (r.sink?.kind === 'widget'
                && r.sink.parts[0] === widgetId
                && r.sink.parts[1] === inputId) {
                return r.source;
            }
        }
        return null;
    }

    sourceLabel(source) {
        if (!source) return 'unrouted';
        if (source.kind === 'peripheral') {
            const [nodeId, periphId, channelId] = source.parts;
            const n = this.nodesById.get(nodeId);
            const p = n?.peripherals.find(x => x.id === periphId);
            const label = p?.label || periphId;
            return `${label}.${channelId} on ${n?.display_name || nodeId}`;
        }
        if (source.kind === 'signal') return source.parts[0];
        if (source.kind === 'peripheral') return source.parts.join('/');
        return source.parts.join('/');
    }

    // ─── Live value ingestion ────────────────────────────────────────

    _ingestPinState(nodeId, data) {
        if (!data) return;
        const node = this.nodesById.get(nodeId);
        if (!node) return;
        let touched = false;

        // Preferred path: the server emits per-channel readings directly
        // (peripheral_id + channel_id), so route resolution is a direct
        // match — no virtual-GPIO indirection.
        if (Array.isArray(data.channels)) {
            for (const ch of data.channels) {
                if (ch.value === undefined || ch.value === null) continue;
                if (this._dispatchChannelValue(nodeId, ch.peripheral_id, ch.channel_id, ch.value)) {
                    touched = true;
                }
            }
        }

        // Legacy path: firmware nodes (RP2040 FAS100) still report on
        // virtual GPIOs. Translate to (peripheral_id, channel_id) via
        // a small per-type map and reuse the same dispatch.
        if (Array.isArray(data.pins)) {
            const legacyMap = {};
            for (const p of node.peripherals) {
                if (p.type === 'fas100') {
                    legacyMap[232] = { peripheralId: p.id, channelId: 'amps'  };
                    legacyMap[233] = { peripheralId: p.id, channelId: 'volts' };
                    legacyMap[234] = { peripheralId: p.id, channelId: 'temp1' };
                    legacyMap[235] = { peripheralId: p.id, channelId: 'temp2' };
                }
            }
            for (const pin of data.pins) {
                if (pin.actual === undefined || pin.actual === null) continue;
                const m = legacyMap[pin.gpio];
                if (!m) continue;
                if (this._dispatchChannelValue(nodeId, m.peripheralId, m.channelId, pin.actual)) {
                    touched = true;
                }
            }
        }

        if (touched && this._active) this.renderValues();
    }

    /** Push a channel reading to every widget input that routes from it. */
    _dispatchChannelValue(nodeId, peripheralId, channelId, value) {
        let any = false;
        for (const r of (this.routing.routes || [])) {
            if (r.source?.kind !== 'peripheral') continue;
            if (r.source.parts[0] !== nodeId) continue;
            if (r.source.parts[1] !== peripheralId) continue;
            if (r.source.parts[2] !== channelId) continue;
            if (r.sink?.kind !== 'widget') continue;
            const key = `${r.sink.parts[0]}/${r.sink.parts[1]}`;
            this._lastValues.set(key, { value, ts: Date.now() });
            any = true;
        }
        return any;
    }

    // ─── Rendering ───────────────────────────────────────────────────

    render() {
        const grid = document.getElementById('widget-grid');
        if (!grid) return;
        const widgets = this.routing.widgets || [];
        if (widgets.length === 0) {
            grid.innerHTML = `
                <p class="text-slate-500 col-span-full italic">
                    No widgets yet — add one on the <a href="#routes" class="text-cyan-400 hover:underline">Routing page</a>.
                </p>`;
            return;
        }
        grid.innerHTML = widgets.map(w => this._renderWidgetCard(w)).join('');
        this.renderValues();
    }

    _renderWidgetCard(w) {
        const type = this._widgetType(w.type);
        if (!type) {
            return `<div class="card">
                <div class="text-sm text-slate-400">Unknown widget type: ${escapeHtml(w.type)}</div>
            </div>`;
        }

        const inputsHtml = (type.inputs || []).map(input => {
            const src = this.sourceForInput(w.id, input.id);
            const srcLabel = this.sourceLabel(src);
            const valId = `widget-${w.id}-${input.id}`;
            const valueHtml = this._renderInputValue(type.id, input, valId);
            return `
                <div class="border-t border-slate-700/60 pt-2 mt-2 first:border-t-0 first:pt-0 first:mt-0">
                    <div class="flex items-center justify-between">
                        <span class="stat-label">${escapeHtml(input.display)}</span>
                        ${valueHtml}
                    </div>
                    <div class="text-[0.65rem] text-slate-500 mt-1 truncate" title="${escapeAttr(srcLabel)}">
                        ${src ? '← ' : ''}${escapeHtml(srcLabel)}
                    </div>
                </div>`;
        }).join('');

        const iconMap = {
            battery_monitor: 'bolt',
            estop_indicator: 'warning',
            single_gauge: 'speed',
            status_led_indicator: 'fiber_manual_record',
        };
        const icon = iconMap[type.id] || 'widgets';

        return `
            <div class="card" data-widget-id="${escapeAttr(w.id)}">
                <div class="flex items-center justify-between mb-3">
                    <div class="flex items-center gap-2">
                        <span class="material-icons text-cyan-400 icon-md">${icon}</span>
                        <h4 class="text-base font-semibold text-white">${escapeHtml(w.label)}</h4>
                    </div>
                    <span class="px-2 py-0.5 text-xs font-medium rounded-full bg-slate-700 text-slate-400">
                        ${escapeHtml(type.label)}
                    </span>
                </div>
                <div>${inputsHtml || '<p class="text-xs text-slate-500 italic">No inputs configured.</p>'}</div>
            </div>`;
    }

    _renderInputValue(typeId, input, valId) {
        // Most inputs render the value as a stat. Status indicators get a
        // colored dot. Battery monitor's current/voltage/temp use the
        // default stat path.
        if (typeId === 'estop_indicator' || typeId === 'status_led_indicator') {
            return `<span id="${valId}" class="inline-block w-3 h-3 rounded-full bg-slate-600" data-empty="1"></span>`;
        }
        return `<span id="${valId}" class="stat-value">—</span>`;
    }

    renderValues() {
        for (const w of (this.routing.widgets || [])) {
            const type = this._widgetType(w.type);
            if (!type) continue;
            for (const input of (type.inputs || [])) {
                const key = `${w.id}/${input.id}`;
                const last = this._lastValues.get(key);
                const valEl = document.getElementById(`widget-${w.id}-${input.id}`);
                if (!valEl) continue;
                if (!last || (Date.now() - last.ts) > 15000) {
                    if (valEl.dataset.empty === '1') {
                        valEl.className = 'inline-block w-3 h-3 rounded-full bg-slate-600';
                    } else {
                        valEl.textContent = '—';
                    }
                    continue;
                }
                if (valEl.dataset.empty === '1') {
                    const triggered = !!last.value;
                    valEl.className = 'inline-block w-3 h-3 rounded-full ' +
                        (triggered ? 'bg-red-500 shadow-[0_0_8px_rgba(239,68,68,0.7)]' : 'bg-emerald-500');
                } else {
                    valEl.textContent = this._formatValue(input, last.value);
                }
            }
        }
    }

    _formatValue(input, value) {
        if (typeof value !== 'number') return String(value);
        // Heuristic by cap tag — close enough for the MVP.
        if (input.cap === 'digital_in' || input.cap === 'digital_out') {
            return value ? 'On' : 'Off';
        }
        if (Math.abs(value) >= 100) return value.toFixed(0);
        if (Math.abs(value) >= 10)  return value.toFixed(1);
        return value.toFixed(2);
    }

    _widgetType(id) {
        return (this.catalog?.widget_types || []).find(t => t.id === id);
    }
}

// Make `escapeHtml`/`escapeAttr` available — peripherals.js declares them
// as top-level functions on the same script "realm", so they're already
// global by the time this file loads. Re-declare as a guard for any
// load order shuffles.
if (typeof escapeHtml === 'undefined') {
    window.escapeHtml = function (s) {
        if (s === null || s === undefined) return '';
        return String(s)
            .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
    };
}
if (typeof escapeAttr === 'undefined') window.escapeAttr = window.escapeHtml;

const widgetsDashboard = new WidgetsDashboard();
window.widgetsDashboard = widgetsDashboard;

document.addEventListener('DOMContentLoaded', () => {
    widgetsDashboard.init();
});
