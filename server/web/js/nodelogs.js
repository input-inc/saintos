/**
 * SAINT.OS Node-detail Logs tab.
 *
 * Renders per-node events that the server records in NodeInfo's
 * log_entries ring buffer. On open, fetches history via WS action
 * `get_node_logs` and subscribes to `node_log/<id>` for live updates.
 * The Clear button wipes the local view (use the server action to
 * also clear the buffer).
 */

class NodeLogsManager {
    constructor() {
        this.selectedNode = null;
        this._subscribed = null;     // currently subscribed node_log topic
        this._maxRows = 500;         // cap rendered rows (server caps to 200; some headroom)
    }

    init() {
        const clearBtn = document.getElementById('btn-clear-node-logs');
        if (clearBtn) clearBtn.addEventListener('click', () => this.clear());

        const ws = window.saintWS;
        if (!ws) return;

        // Single 'state' handler dispatches to whichever node_log topic is current
        ws.on('state', (message) => {
            if (!message || !message.node) return;
            if (!message.node.startsWith('node_log/')) return;
            const nodeId = message.node.slice('node_log/'.length);
            if (nodeId !== this.selectedNode) return;
            this._append(message.data);
        });
    }

    /** Called by app.js when entering a node-detail page. */
    async setNode(nodeId) {
        // Unsubscribe from previous
        await this._unsubscribe();
        this.selectedNode = nodeId || null;
        const list = document.getElementById('node-logs');
        if (list) list.innerHTML = '<p class="text-slate-500 italic text-sm">Loading…</p>';
        if (!nodeId) return;

        const ws = window.saintWS;
        try {
            await ws.subscribe([`node_log/${nodeId}`]);
            this._subscribed = `node_log/${nodeId}`;
            const result = await ws.management('get_node_logs', { node_id: nodeId });
            const entries = (result && result.entries) || [];
            this._renderHistory(entries);
        } catch (err) {
            console.error('Failed to load node logs:', err);
            if (list) list.innerHTML =
                `<p class="text-red-300 text-sm">Failed to load logs: ${escapeHtml(err.message || err)}</p>`;
        }
    }

    async _unsubscribe() {
        if (!this._subscribed) return;
        const ws = window.saintWS;
        try { await ws.unsubscribe([this._subscribed]); } catch (_) { /* ignore */ }
        this._subscribed = null;
    }

    _renderHistory(entries) {
        const list = document.getElementById('node-logs');
        if (!list) return;
        if (!entries.length) {
            list.innerHTML = '<p class="text-slate-500 italic text-sm">No logs yet.</p>';
            return;
        }
        list.innerHTML = entries.map(e => this._rowHtml(e)).join('');
        list.scrollTop = list.scrollHeight;
    }

    _append(entry) {
        const list = document.getElementById('node-logs');
        if (!list) return;
        // First entry: replace placeholder
        if (list.querySelector('p.italic')) list.innerHTML = '';
        list.insertAdjacentHTML('beforeend', this._rowHtml(entry));
        // Trim if we're over the cap
        while (list.childElementCount > this._maxRows) {
            list.removeChild(list.firstElementChild);
        }
        // Stick to bottom if user wasn't scrolling up
        const nearBottom = (list.scrollHeight - list.scrollTop - list.clientHeight) < 80;
        if (nearBottom) list.scrollTop = list.scrollHeight;
    }

    _rowHtml(entry) {
        const t = entry.time ? new Date(entry.time * 1000) : new Date();
        const ts = t.toLocaleTimeString([], { hour12: false }) +
                   '.' + String(t.getMilliseconds()).padStart(3, '0');
        const level = (entry.level || 'info').toLowerCase();
        const levelClass = {
            error: 'text-red-300',
            warn:  'text-amber-300',
            info:  'text-slate-200',
            debug: 'text-slate-500',
        }[level] || 'text-slate-200';
        return `<div class="${levelClass}">` +
               `<span class="text-slate-500 mr-2">${ts}</span>` +
               escapeHtml(entry.text || '') +
               `</div>`;
    }

    async clear() {
        const list = document.getElementById('node-logs');
        if (list) list.innerHTML = '<p class="text-slate-500 italic text-sm">No logs yet.</p>';
        if (!this.selectedNode) return;
        try {
            await window.saintWS.management('clear_node_logs', { node_id: this.selectedNode });
        } catch (err) {
            console.error('clear_node_logs failed:', err);
        }
    }
}

const nodeLogsManager = new NodeLogsManager();
window.nodeLogsManager = nodeLogsManager;

document.addEventListener('DOMContentLoaded', () => nodeLogsManager.init());
