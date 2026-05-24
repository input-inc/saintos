/**
 * SAINT.OS Node-detail Live Readings tab.
 *
 * Renders one card per peripheral on this node, listing each channel's
 * latest value coming from the server's pin_state broadcast. The
 * server translates the firmware's virtual-GPIO readings into
 * (peripheral_id, channel_id) form (see _firmware_channel_id in
 * state_manager.py), so we just consume data.channels[] directly —
 * no per-type mapping logic on the client.
 *
 * Purpose: visibility for debugging. If a widget on the dashboard
 * isn't updating, this tab tells you whether the underlying channel
 * is even producing data.
 */

// Sparkline window in seconds — matches PeripheralLogger's 30s deque.
const SPARK_WINDOW_S = 30;
const SPARK_W = 140;
const SPARK_H = 24;

class NodeLiveManager {
    constructor() {
        this.selectedNode = null;
        this._subscribed = null;
        this._catalog = null;
        this._peripherals = [];
        this._values = new Map();          // "peripheral_id/channel_id" -> {value, ts}
        // 30s rolling window per channel, populated for log-enabled
        // peripherals only. Array of [ts_seconds, value] tuples, oldest
        // first; trimmed in _appendHistory.
        this._history = new Map();         // "peripheral_id/channel_id" -> [[ts, v], ...]
        this._lastFeedback = 0;
        this._rerenderTimer = null;
    }

    init() {
        const ws = window.saintWS;
        if (!ws) return;
        ws.on('state', (msg) => {
            if (!msg || !msg.node) return;
            if (!msg.node.startsWith('pin_state/')) return;
            const nodeId = msg.node.slice('pin_state/'.length);
            if (nodeId !== this.selectedNode) return;
            this._ingest(msg.data);
        });
    }

    /** Activate the tab for a node. Re-fetches peripherals and subscribes. */
    async setNode(nodeId) {
        await this._unsubscribe();
        this.selectedNode = nodeId || null;
        this._values.clear();
        this._history.clear();
        this._lastFeedback = 0;

        const grid = document.getElementById('node-live-grid');
        if (grid) grid.innerHTML = '<p class="text-slate-500 italic text-sm">Loading…</p>';

        if (!nodeId) return;

        const ws = window.saintWS;
        try {
            await ws.subscribe([`pin_state/${nodeId}`], 5);
            this._subscribed = `pin_state/${nodeId}`;

            const [catalog, periph] = await Promise.all([
                this._ensureCatalog(),
                ws.management('get_node_peripherals', { node_id: nodeId }),
            ]);
            this._catalog = catalog;
            this._peripherals = (periph && periph.peripherals) || [];
            // Pull the server's 30s ring buffer for every input channel
            // of a log-enabled peripheral. Done in parallel; renders the
            // grid immediately and then re-renders once the histories
            // arrive (typically <100 ms for a small fleet).
            this._render();
            await this._backfillHistory();
            this._scheduleRender();
        } catch (err) {
            console.error('Failed to load live readings:', err);
            if (grid) grid.innerHTML =
                `<p class="text-red-300 text-sm">Failed to load: ${escapeHtml(err.message || err)}</p>`;
        }
    }

    /** Called by peripheralManager when the operator flips the Log toggle. */
    async onLogEnabledChanged(nodeId, peripheralId, enabled) {
        if (nodeId !== this.selectedNode) return;
        const p = this._peripherals.find(x => x.id === peripheralId);
        if (p) p.log_enabled = !!enabled;
        if (enabled) {
            await this._backfillPeripheralHistory(peripheralId);
        } else {
            // Drop the local rings for this peripheral — they'd otherwise
            // keep filling from the pin_state stream below.
            for (const k of [...this._history.keys()]) {
                if (k.startsWith(`${peripheralId}/`)) this._history.delete(k);
            }
        }
        this._scheduleRender();
    }

    async _backfillHistory() {
        const enabled = this._peripherals.filter(p => p.log_enabled);
        await Promise.all(enabled.map(p => this._backfillPeripheralHistory(p.id)));
    }

    async _backfillPeripheralHistory(peripheralId) {
        const typesById = this._typesById();
        const p = this._peripherals.find(x => x.id === peripheralId);
        if (!p) return;
        const type = typesById[p.type];
        if (!type) return;
        const ws = window.saintWS;
        const inputs = type.channels.filter(c => c.dir === 'in');
        // Fire all channel requests for this peripheral in parallel.
        await Promise.all(inputs.map(async (ch) => {
            try {
                const r = await ws.management('get_peripheral_history', {
                    node_id: this.selectedNode,
                    peripheral_id: peripheralId,
                    channel_id: ch.id,
                    window: '30s',
                });
                if (!r || !Array.isArray(r.samples)) return;
                const key = `${peripheralId}/${ch.id}`;
                this._history.set(key, r.samples.map(s => [s.ts, s.v]));
            } catch (e) {
                // Fall through — sparkline just won't render until live
                // samples arrive. Don't spam the console on transient errs.
            }
        }));
    }

    async _unsubscribe() {
        if (!this._subscribed) return;
        const ws = window.saintWS;
        try { await ws.unsubscribe([this._subscribed]); } catch (_) {}
        this._subscribed = null;
    }

    async _ensureCatalog() {
        if (this._catalog) return this._catalog;
        const ws = window.saintWS;
        try {
            const r = await ws.management('get_peripheral_catalog', {});
            return r || { peripheral_types: [] };
        } catch (e) {
            return { peripheral_types: [] };
        }
    }

    _ingest(data) {
        if (!data) return;
        if (data.last_feedback) this._lastFeedback = data.last_feedback;
        const channels = Array.isArray(data.channels) ? data.channels : [];
        let touched = false;
        for (const ch of channels) {
            if (!ch.peripheral_id || !ch.channel_id) continue;
            const key = `${ch.peripheral_id}/${ch.channel_id}`;
            // ALWAYS use the browser's wall-clock for the in-RAM ring
            // buffer. The server's `last_updated` is set from the Pi's
            // time.time() — which, on a Pi without an RTC or NTP sync,
            // can be off from the browser by hours. Mixing the two
            // timebases means every fresh sample lands "older than 30s"
            // by the browser-clock cutoff and gets shifted out of the
            // ring immediately. Stamping with the browser clock makes
            // the sparkline robust to whatever the Pi reports.
            const ts = Date.now() / 1000;
            this._values.set(key, { value: ch.value, ts });
            // Sparkline ring buffer is always-on for numeric input
            // channels — the disk-logging toggle is independent.
            if (typeof ch.value === 'number') {
                this._appendHistory(key, ts, ch.value);
            }
            touched = true;
        }
        if (touched) this._scheduleRender();
        this._updateStatus(data.stale);
    }

    _appendHistory(key, ts, value) {
        let arr = this._history.get(key);
        if (!arr) {
            arr = [];
            this._history.set(key, arr);
        }
        arr.push([ts, value]);
        // Trim entries older than the window. Cheap to do inline since
        // samples arrive monotonically — usually shifts 0–1 items.
        const cutoff = (Date.now() / 1000) - SPARK_WINDOW_S;
        while (arr.length && arr[0][0] < cutoff) arr.shift();
    }

    _typesById() {
        const out = {};
        for (const t of (this._catalog?.peripheral_types || [])) out[t.id] = t;
        return out;
    }

    _updateStatus(stale) {
        const el = document.getElementById('node-live-status');
        if (!el) return;
        if (!this._lastFeedback) {
            el.textContent = 'No feedback yet';
            el.className = 'text-xs text-slate-500';
            return;
        }
        const age = Math.max(0, (Date.now() / 1000) - this._lastFeedback);
        const ageStr = age < 2 ? `${age.toFixed(2)}s` : `${age.toFixed(1)}s`;
        if (stale) {
            el.textContent = `Stale (last update ${ageStr} ago)`;
            el.className = 'text-xs text-amber-400';
        } else {
            el.textContent = `Live (last update ${ageStr} ago)`;
            el.className = 'text-xs text-emerald-400';
        }
    }

    /** Coalesce renders — pin_state arrives at 10 Hz, no need to re-DOM that fast. */
    _scheduleRender() {
        if (this._rerenderTimer) return;
        this._rerenderTimer = setTimeout(() => {
            this._rerenderTimer = null;
            this._render();
        }, 250);
    }

    _render() {
        const grid = document.getElementById('node-live-grid');
        if (!grid) return;

        if (!this._peripherals.length) {
            grid.innerHTML = '<p class="text-slate-500 italic text-sm">No peripherals on this node yet.</p>';
            return;
        }

        const typesById = this._typesById();

        grid.innerHTML = this._peripherals.map(p => {
            const type = typesById[p.type];
            const typeLabel = type ? type.label : p.type;
            const channels = (type?.channels) || [];

            const rows = channels.length
                ? channels.map(ch => this._channelRow(p, ch)).join('')
                : '<div class="text-xs text-slate-500 italic">No declared channels</div>';

            const builtinBadge = p.builtin
                ? '<span class="px-2 py-0.5 rounded-full text-xs bg-slate-700 text-slate-300 ml-2">Built-in</span>'
                : '';
            const logBadge = p.log_enabled
                ? '<span class="px-2 py-0.5 rounded-full text-xs bg-emerald-900/40 text-emerald-200 ml-2" title="Recording to peripherals.log">Logging</span>'
                : '';

            return `
                <div class="card">
                    <div class="flex items-center justify-between mb-3">
                        <div>
                            <h4 class="text-base font-semibold text-white">${escapeHtml(p.label || p.id)}</h4>
                            <div class="text-xs text-slate-500">${escapeHtml(typeLabel)} · <span class="font-mono">${escapeHtml(p.id)}</span>${builtinBadge}${logBadge}</div>
                        </div>
                    </div>
                    <div class="space-y-1">${rows}</div>
                </div>
            `;
        }).join('');
    }

    _channelRow(p, ch) {
        const key = `${p.id}/${ch.id}`;
        const v = this._values.get(key);
        const valStr = (v === undefined || v.value === null || v.value === undefined)
            ? '<span class="text-slate-500">—</span>'
            : `<span class="${ch.dir === 'in' ? 'text-cyan-300' : 'text-amber-300'}">${this._formatValueForChannel(ch, v.value)}</span>`;
        // v.ts has already been corrected in _ingest to a browser-clock
        // value when the Pi has no RTC. Cap the printed age at "stale"
        // so a few-second drift doesn't look like hours.
        const age = v ? Math.max(0, (Date.now() / 1000) - v.ts) : 0;
        const ageStr = v
            ? (age > 600 ? 'stale' : `${age.toFixed(1)}s ago`)
            : '';
        // Sparkline for every input channel; output channels' setpoints
        // are not interesting to graph. The disk-logging toggle is
        // separate from the visual ring buffer — this draws as soon as
        // samples arrive from pin_state.
        const spark = (ch.dir === 'in')
            ? this._sparkline(this._history.get(key), ch)
            : '';
        return `
            <div class="flex items-center justify-between text-sm font-mono py-1 border-b border-slate-800 last:border-b-0">
                <span class="text-slate-400">${escapeHtml(ch.display || ch.id)}</span>
                <div class="flex items-center gap-3">
                    ${spark}
                    ${valStr}
                    <span class="text-xs text-slate-600 w-20 text-right">${ageStr}</span>
                </div>
            </div>
        `;
    }

    /** Inline SVG sparkline of the 30s window. Returns empty when no data.
     *  `ch` is the optional channel descriptor — used so the tooltip
     *  shows min/max in the operator's preferred unit (matters for
     *  temperature channels where the raw values are in Celsius but
     *  the operator may have Fahrenheit selected). */
    _sparkline(samples, ch) {
        if (!samples || samples.length < 2) {
            return `<span class="inline-block" style="width:${SPARK_W}px;height:${SPARK_H}px"></span>`;
        }
        const nowS = Date.now() / 1000;
        const t0 = nowS - SPARK_WINDOW_S;
        // Build x/y arrays, mapping ts to [0..SPARK_W] and value to
        // [SPARK_H-1..1] (inverted because SVG y grows downward).
        let lo = Infinity, hi = -Infinity;
        for (const [, v] of samples) {
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (!isFinite(lo) || !isFinite(hi)) {
            return `<span class="inline-block" style="width:${SPARK_W}px;height:${SPARK_H}px"></span>`;
        }
        const span = hi - lo || 1;
        const pts = samples.map(([ts, v]) => {
            const x = Math.max(0, Math.min(SPARK_W,
                ((ts - t0) / SPARK_WINDOW_S) * SPARK_W));
            const y = (SPARK_H - 2) - ((v - lo) / span) * (SPARK_H - 4) + 1;
            return `${x.toFixed(1)},${y.toFixed(1)}`;
        }).join(' ');
        const last = samples[samples.length - 1];
        const lx = Math.max(0, Math.min(SPARK_W,
            ((last[0] - t0) / SPARK_WINDOW_S) * SPARK_W));
        const ly = (SPARK_H - 2) - ((last[1] - lo) / span) * (SPARK_H - 4) + 1;
        const titleAttr = `min=${this._formatValueForChannel(ch, lo)} max=${this._formatValueForChannel(ch, hi)} (30s)`;
        return `<svg width="${SPARK_W}" height="${SPARK_H}"
                     viewBox="0 0 ${SPARK_W} ${SPARK_H}"
                     class="text-cyan-400" title="${escapeAttr(titleAttr)}"
                     aria-label="${escapeAttr(titleAttr)}">
                    <polyline fill="none" stroke="currentColor" stroke-width="1.25"
                              stroke-linejoin="round" stroke-linecap="round"
                              points="${pts}"/>
                    <circle cx="${lx.toFixed(1)}" cy="${ly.toFixed(1)}" r="1.5"
                            fill="currentColor"/>
                </svg>`;
    }

    _formatValue(v) {
        if (typeof v !== 'number') return escapeHtml(String(v));
        if (Number.isInteger(v)) return v.toString();
        return v.toFixed(3);
    }

    /** Channel-aware formatter. For temperature channels routes through
     *  window.app.formatTemperature so the displayed unit follows the
     *  operator's preference; for everything else falls back to the
     *  generic numeric format. `ch` may be undefined (sparkline tooltip
     *  callers before this was threaded through), in which case we
     *  also fall back to the generic format. */
    _formatValueForChannel(ch, v) {
        if (ch && window.app && window.app.isTemperatureChannel(ch)
            && typeof v === 'number') {
            return window.app.formatTemperature(v);
        }
        return this._formatValue(v);
    }

    /** Public re-entry point for re-rendering after a settings change
     *  (e.g. the operator toggled Celsius/Fahrenheit). Just re-runs
     *  the grid render against the current data — no refetch. */
    renderGrid() {
        if (this.selectedNode) {
            this._render();
        }
    }
}

const nodeLiveManager = new NodeLiveManager();
window.nodeLiveManager = nodeLiveManager;

document.addEventListener('DOMContentLoaded', () => nodeLiveManager.init());
