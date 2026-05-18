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

class NodeLiveManager {
    constructor() {
        this.selectedNode = null;
        this._subscribed = null;
        this._catalog = null;
        this._peripherals = [];
        this._values = new Map();          // "peripheral_id/channel_id" -> {value, ts}
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
            this._render();
        } catch (err) {
            console.error('Failed to load live readings:', err);
            if (grid) grid.innerHTML =
                `<p class="text-red-300 text-sm">Failed to load: ${escapeHtml(err.message || err)}</p>`;
        }
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
            this._values.set(key, {
                value: ch.value,
                ts: ch.last_updated || (Date.now() / 1000),
            });
            touched = true;
        }
        if (touched) this._scheduleRender();
        this._updateStatus(data.stale);
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

        const typesById = {};
        for (const t of (this._catalog?.peripheral_types || [])) typesById[t.id] = t;

        grid.innerHTML = this._peripherals.map(p => {
            const type = typesById[p.type];
            const typeLabel = type ? type.label : p.type;
            const channels = (type?.channels) || [];

            const rows = channels.length
                ? channels.map(ch => this._channelRow(p.id, ch)).join('')
                : '<div class="text-xs text-slate-500 italic">No declared channels</div>';

            const builtinBadge = p.builtin
                ? '<span class="px-2 py-0.5 rounded-full text-xs bg-slate-700 text-slate-300 ml-2">Built-in</span>'
                : '';

            return `
                <div class="card">
                    <div class="flex items-center justify-between mb-3">
                        <div>
                            <h4 class="text-base font-semibold text-white">${escapeHtml(p.label || p.id)}</h4>
                            <div class="text-xs text-slate-500">${escapeHtml(typeLabel)} · <span class="font-mono">${escapeHtml(p.id)}</span>${builtinBadge}</div>
                        </div>
                    </div>
                    <div class="space-y-1">${rows}</div>
                </div>
            `;
        }).join('');
    }

    _channelRow(peripheralId, ch) {
        const key = `${peripheralId}/${ch.id}`;
        const v = this._values.get(key);
        const valStr = (v === undefined || v.value === null || v.value === undefined)
            ? '<span class="text-slate-500">—</span>'
            : `<span class="${ch.dir === 'in' ? 'text-cyan-300' : 'text-amber-300'}">${this._formatValue(v.value)}</span>`;
        const ageStr = v
            ? `${Math.max(0, (Date.now() / 1000 - v.ts)).toFixed(1)}s ago`
            : '';
        return `
            <div class="flex items-center justify-between text-sm font-mono py-1 border-b border-slate-800 last:border-b-0">
                <span class="text-slate-400">${escapeHtml(ch.display || ch.id)}</span>
                <div class="flex items-center gap-3">
                    ${valStr}
                    <span class="text-xs text-slate-600 w-20 text-right">${ageStr}</span>
                </div>
            </div>
        `;
    }

    _formatValue(v) {
        if (typeof v !== 'number') return escapeHtml(String(v));
        if (Number.isInteger(v)) return v.toString();
        return v.toFixed(3);
    }
}

const nodeLiveManager = new NodeLiveManager();
window.nodeLiveManager = nodeLiveManager;

document.addEventListener('DOMContentLoaded', () => nodeLiveManager.init());
