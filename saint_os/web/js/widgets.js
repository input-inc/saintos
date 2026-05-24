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

// Sparkline geometry — matches nodelive.js so both views look identical.
const WIDGET_SPARK_WINDOW_S = 30;
const WIDGET_SPARK_W = 140;
const WIDGET_SPARK_H = 24;

class WidgetsDashboard {
    constructor() {
        this.catalog = null;
        this.routing = { version: 0, sheets: {} };
        this.nodesById = new Map();           // node_id -> {peripherals:[], display_name}
        this._lastValues = new Map();         // "widget_id/input_id" -> {value, lastUpdate}
        // Rolling 30s history per (widget_id, input_id) for the inline
        // sparkline. Browser-clock timestamps only — see comment in
        // nodelive.js _ingest for why we don't mix in the server's
        // last_updated.
        this._history = new Map();            // "widget_id/input_id" -> [[ts, v], ...]
        this._pinStateSubs = new Set();
        this._refreshInterval = null;
        this._stateHandler = null;
    }

    init() {
        const btn = document.getElementById('btn-dashboard-add-widget');
        if (btn) btn.addEventListener('click', () => app.showPage('routes'));

        // Listen for system_routing broadcasts (a remote operator edits
        // a widget, we want to reflect it).
        const ws = window.saintWS;
        if (ws) {
            this._stateHandler = (msg) => {
                if (!msg || !msg.node) return;
                if (msg.node === 'system_routing') {
                    this.routing = msg.data || { version: 0, sheets: {} };
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
            this.routing = routing || { version: 0, sheets: {} };
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

    /** Flatten widgets across every controller sheet, tagging each with
     *  its owning sheet so wire lookups can stay scoped. */
    _allWidgets() {
        const out = [];
        const sheets = this.routing.sheets || {};
        for (const sheetId of Object.keys(sheets)) {
            const sheet = sheets[sheetId];
            for (const w of (sheet.widgets || [])) {
                out.push({ widget: w, sheetId, sheet });
            }
        }
        return out;
    }

    _findWidgetSheet(widgetId) {
        const sheets = this.routing.sheets || {};
        for (const sheetId of Object.keys(sheets)) {
            const sheet = sheets[sheetId];
            if ((sheet.widgets || []).some(w => w.id === widgetId)) {
                return sheet;
            }
        }
        return null;
    }

    sourceForInput(widgetId, inputId) {
        const sheet = this._findWidgetSheet(widgetId);
        if (!sheet) return null;
        for (const w of (sheet.wires || [])) {
            if (w.sink?.kind === 'widget'
                && w.sink.parts[0] === widgetId
                && w.sink.parts[1] === inputId) {
                return w.source;
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
        if (source.kind === 'input')    return source.parts[0];
        if (source.kind === 'operator') return source.parts.join('/');
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

        // (Legacy `data.pins` path retired — the server now translates
        // firmware virtual GPIOs to (peripheral_id, channel_id) and
        // emits them through `data.channels` above. The pin_state
        // broadcast still includes a `pins` array for the State tab
        // and inspection, but widgets sink only from `channels`.)

        if (touched && this._active) this.renderValues();
    }

    /** Push a channel reading to every widget input that wires from it.
     *  Walks each controller sheet's wires; only direct peripheral→widget
     *  connections light up here. Wires that pass through an operator
     *  are computed server-side by the routing evaluator and don't yet
     *  surface to the dashboard. */
    _dispatchChannelValue(nodeId, peripheralId, channelId, value) {
        let any = false;
        const tsMs = Date.now();
        const sheets = this.routing.sheets || {};
        for (const sheetId of Object.keys(sheets)) {
            for (const w of (sheets[sheetId].wires || [])) {
                if (w.source?.kind !== 'peripheral') continue;
                if (w.source.parts[0] !== nodeId) continue;
                if (w.source.parts[1] !== peripheralId) continue;
                if (w.source.parts[2] !== channelId) continue;
                if (w.sink?.kind !== 'widget') continue;
                const key = `${w.sink.parts[0]}/${w.sink.parts[1]}`;
                this._lastValues.set(key, { value, ts: tsMs });
                if (typeof value === 'number') {
                    this._appendHistory(key, tsMs / 1000, value);
                }
                any = true;
            }
        }
        return any;
    }

    _appendHistory(key, ts, value) {
        let arr = this._history.get(key);
        if (!arr) {
            arr = [];
            this._history.set(key, arr);
        }
        arr.push([ts, value]);
        const cutoff = (Date.now() / 1000) - WIDGET_SPARK_WINDOW_S;
        while (arr.length && arr[0][0] < cutoff) arr.shift();
    }

    // ─── Rendering ───────────────────────────────────────────────────

    render() {
        const grid = document.getElementById('widget-grid');
        if (!grid) return;
        const widgets = this._allWidgets().map(entry => entry.widget);
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
        // FAS100 Power Monitor has a distinct mocked-up appearance —
        // amber-accented header, per-input progress bars alongside the
        // sparkline. Other widget types fall through to the generic
        // renderer below.
        if (type.id === 'battery_monitor') {
            return this._renderFas100Card(w, type);
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

    /** Mockup-faithful rendering for the FAS100 Power Monitor widget.
     *  Color-coded per input (amber for voltage, cyan for current,
     *  rose for temps), with an inline 30s sparkline + a thin progress
     *  bar showing the value's position within its expected range. */
    _renderFas100Card(w, type) {
        // Per-input range + accent color. Ranges are generous defaults
        // — they're only used to scale the progress fill, never as
        // clamps on the displayed value. Temperature unit follows the
        // operator's preference via SaintApp.temperatureUnitSymbol().
        //
        // Use bare `app`, not `window.app`: the page has
        // `<div id="app">` which the browser auto-exposes on window,
        // shadowing the SaintApp instance. Bare `app` resolves through
        // the script's lexical environment to the `const app` in app.js.
        const a = (typeof app !== 'undefined') ? app : null;
        const tempUnit = (a && a.temperatureUnitSymbol)
            ? a.temperatureUnitSymbol() : '°C';
        const meta = {
            voltage: { max: 30,  unit: 'V',  color: 'bg-amber-500',
                       barText: 'text-amber-400' },
            current: { max: 100, unit: 'A',  color: 'bg-cyan-500',
                       barText: 'text-cyan-400' },
            temp1:   { max: 80,  unit: tempUnit, color: 'bg-rose-500',
                       barText: 'text-rose-300' },
            temp2:   { max: 80,  unit: tempUnit, color: 'bg-rose-500',
                       barText: 'text-rose-300' },
        };

        const rowsHtml = (type.inputs || []).map(input => {
            const src = this.sourceForInput(w.id, input.id);
            const srcLabel = this.sourceLabel(src);
            const valId = `widget-${w.id}-${input.id}`;
            const m = meta[input.id] || meta.voltage;
            const showBar = input.id === 'voltage' || input.id === 'current';
            const barHtml = showBar ? `
                <div class="flex-1 h-1 bg-slate-700 rounded-full overflow-hidden ml-2">
                    <div id="${valId}-bar" class="${m.color} h-full transition-all"
                         style="width:0%"></div>
                </div>` : '';
            return `
                <div class="border-t border-slate-700/60 pt-2 mt-2 first:border-t-0 first:pt-0 first:mt-0">
                    <div class="flex items-center justify-between">
                        <span class="stat-label">${escapeHtml(input.display)}</span>
                        <span class="inline-flex items-center gap-2">
                            <span id="${valId}-spark" class="inline-flex items-center"
                                  style="width:${WIDGET_SPARK_W}px;height:${WIDGET_SPARK_H}px"></span>
                            <span id="${valId}" class="stat-value ${m.barText}">—</span>
                            <span class="text-xs text-slate-500">${m.unit}</span>
                        </span>
                    </div>
                    <div class="flex items-center mt-1">
                        <div class="text-[0.65rem] text-slate-500 truncate flex-1" title="${escapeAttr(srcLabel)}">
                            ${src ? '← ' : ''}${escapeHtml(srcLabel)}
                        </div>
                        ${barHtml}
                    </div>
                </div>`;
        }).join('');

        return `
            <div class="card" data-widget-id="${escapeAttr(w.id)}">
                <div class="flex items-center justify-between mb-3">
                    <div class="flex items-center gap-2">
                        <span class="material-icons text-amber-400 icon-md">bolt</span>
                        <h4 class="text-base font-semibold text-white">${escapeHtml(w.label)}</h4>
                    </div>
                    <span class="px-2 py-0.5 text-xs font-medium rounded-full bg-amber-900/40 text-amber-200">
                        FAS100
                    </span>
                </div>
                <div class="h-0.5 bg-amber-500 rounded-full mb-3"></div>
                <div>${rowsHtml || '<p class="text-xs text-slate-500 italic">No inputs configured.</p>'}</div>
            </div>`;
    }

    _renderInputValue(typeId, input, valId) {
        // Status indicators get a colored dot. Analog inputs render the
        // value plus an inline 30s sparkline that the live pin_state
        // stream feeds. Digital inputs skip the sparkline (an on/off
        // line doesn't graph meaningfully).
        if (typeId === 'estop_indicator' || typeId === 'status_led_indicator') {
            return `<span id="${valId}" class="inline-block w-3 h-3 rounded-full bg-slate-600" data-empty="1"></span>`;
        }
        const isDigital = input.cap === 'digital_in' || input.cap === 'digital_out';
        const sparkSpan = isDigital ? '' :
            `<span id="${valId}-spark" class="inline-flex items-center"
                   style="width:${WIDGET_SPARK_W}px;height:${WIDGET_SPARK_H}px"></span>`;
        return `<span class="inline-flex items-center gap-2">
                    ${sparkSpan}
                    <span id="${valId}" class="stat-value">—</span>
                </span>`;
    }

    renderValues() {
        // Per-input ranges for the FAS100 progress bars. Kept in sync
        // with the same table in _renderFas100Card.
        const fas100Max = { voltage: 30, current: 100, temp1: 80, temp2: 80 };
        for (const { widget: w } of this._allWidgets()) {
            const type = this._widgetType(w.type);
            if (!type) continue;
            for (const input of (type.inputs || [])) {
                const key = `${w.id}/${input.id}`;
                const last = this._lastValues.get(key);
                const valId = `widget-${w.id}-${input.id}`;
                const valEl = document.getElementById(valId);
                const sparkEl = document.getElementById(`${valId}-spark`);
                const barEl = document.getElementById(`${valId}-bar`);
                if (!valEl) continue;
                if (!last || (Date.now() - last.ts) > 15000) {
                    if (valEl.dataset.empty === '1') {
                        valEl.className = 'inline-block w-3 h-3 rounded-full bg-slate-600';
                    } else {
                        valEl.textContent = '—';
                    }
                    if (sparkEl) sparkEl.innerHTML = '';
                    if (barEl) barEl.style.width = '0%';
                    continue;
                }
                if (valEl.dataset.empty === '1') {
                    const triggered = !!last.value;
                    valEl.className = 'inline-block w-3 h-3 rounded-full ' +
                        (triggered ? 'bg-red-500 shadow-[0_0_8px_rgba(239,68,68,0.7)]' : 'bg-emerald-500');
                } else {
                    valEl.textContent = this._formatValueForInput(input, last.value);
                }
                if (sparkEl) {
                    sparkEl.innerHTML = this._sparkline(this._history.get(key));
                }
                if (barEl && type.id === 'battery_monitor'
                        && typeof last.value === 'number') {
                    const max = fas100Max[input.id] || 1;
                    const pct = Math.max(0, Math.min(100, (last.value / max) * 100));
                    barEl.style.width = `${pct.toFixed(1)}%`;
                }
            }
        }
    }

    /** Inline SVG sparkline of the 30s window. Same shape as nodelive's. */
    _sparkline(samples) {
        if (!samples || samples.length < 2) return '';
        const t0 = (Date.now() / 1000) - WIDGET_SPARK_WINDOW_S;
        let lo = Infinity, hi = -Infinity;
        for (const [, v] of samples) {
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (!isFinite(lo) || !isFinite(hi)) return '';
        const span = hi - lo || 1;
        const pts = samples.map(([ts, v]) => {
            const x = Math.max(0, Math.min(WIDGET_SPARK_W,
                ((ts - t0) / WIDGET_SPARK_WINDOW_S) * WIDGET_SPARK_W));
            const y = (WIDGET_SPARK_H - 2) - ((v - lo) / span) * (WIDGET_SPARK_H - 4) + 1;
            return `${x.toFixed(1)},${y.toFixed(1)}`;
        }).join(' ');
        const last = samples[samples.length - 1];
        const lx = Math.max(0, Math.min(WIDGET_SPARK_W,
            ((last[0] - t0) / WIDGET_SPARK_WINDOW_S) * WIDGET_SPARK_W));
        const ly = (WIDGET_SPARK_H - 2) - ((last[1] - lo) / span) * (WIDGET_SPARK_H - 4) + 1;
        return `<svg width="${WIDGET_SPARK_W}" height="${WIDGET_SPARK_H}"
                     viewBox="0 0 ${WIDGET_SPARK_W} ${WIDGET_SPARK_H}"
                     class="text-cyan-400">
                    <polyline fill="none" stroke="currentColor" stroke-width="1.25"
                              stroke-linejoin="round" stroke-linecap="round"
                              points="${pts}"/>
                    <circle cx="${lx.toFixed(1)}" cy="${ly.toFixed(1)}" r="1.5"
                            fill="currentColor"/>
                </svg>`;
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

    /** Input-aware formatter. Routes temperature inputs through
     *  SaintApp.formatTemperatureValue (which converts to the
     *  operator's preferred unit and returns the number only — the
     *  unit symbol is rendered in a sibling span by _renderFas100Card,
     *  using SaintApp.temperatureUnitSymbol() so the two stay in
     *  sync). Falls back to the generic numeric format otherwise.
     *
     *  Bare `app`, not `window.app`: see _renderFas100Card for why
     *  (`<div id="app">` shadows the SaintApp instance on window). */
    _formatValueForInput(input, value) {
        const a = (typeof app !== 'undefined') ? app : null;
        if (typeof value === 'number' && a && a.isTemperatureChannel(input)) {
            return a.formatTemperatureValue(value);
        }
        return this._formatValue(input, value);
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
