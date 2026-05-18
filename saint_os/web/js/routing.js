/**
 * SAINT.OS Routing Page
 *
 * System-wide graph editor for connecting peripheral channels to logical
 * signals and dashboard widgets. Each graph node represents one of:
 *   - a peripheral instance on a node      (handle: p::<node>::<periph>)
 *   - a logical signal (operator-created)  (handle: s::<path>)
 *   - a dashboard widget                   (handle: w::<widget_id>)
 *
 * Wires correspond to server-side Route objects. Positions live in
 * localStorage and are layout-only (not part of the wire semantics).
 */

class RoutingPage {
    constructor() {
        this.catalog = null;          // peripheral_types + widget_types
        this.nodes = [];              // adopted nodes (with peripherals)
        this.routing = { version: 0, routes: [], widgets: [] };

        // Graph state — built from server data on each refresh.
        this.graphNodes = [];
        // Map graphNodeId -> {x, y}  (persisted in localStorage)
        this.positions = this._loadPositions();
        // Operator-created signal labels that don't yet have any routes.
        // Persisted in localStorage so they survive page reloads.
        this.pendingSignals = this._loadPendingSignals();

        // Interaction state
        this._connecting = null;
        this._draggingNode = null;
        this._dragOffset = { x: 0, y: 0 };
        this._mouseHandler = null;
        this._upHandler = null;
        this._canvasClickHandler = null;
    }

    init() {
        document.getElementById('btn-routing-add-signal')
            ?.addEventListener('click', () => this.promptAddSignal());
        document.getElementById('btn-routing-add-widget')
            ?.addEventListener('click', () => this.openWidgetModal());
        document.getElementById('btn-routing-auto-layout')
            ?.addEventListener('click', () => this.autoLayout());

        // Listen for system_routing broadcasts so the graph keeps in sync
        // when other clients edit it.
        const ws = window.saintWS;
        if (ws) {
            ws.on('state', (msg) => {
                if (!msg || msg.node !== 'system_routing') return;
                this.routing = msg.data || { version: 0, routes: [], widgets: [] };
                if (this._active) this.rebuildGraph(/* keepPositions= */ true);
            });
        }
    }

    /**
     * Called when the Routing page becomes active. Lazy-fetches catalog
     * + node list + routing graph and renders.
     */
    async activate() {
        this._active = true;
        const ws = window.saintWS;
        if (!ws) return;

        try {
            await ws.subscribe(['system_routing']);
        } catch (e) {
            console.warn('Failed to subscribe to system_routing:', e);
        }

        try {
            const [catalog, nodes, routing] = await Promise.all([
                this.ensureCatalog(),
                ws.management('list_adopted', {}),
                ws.management('get_system_routing', {}),
            ]);
            this.catalog = catalog;
            this.nodes = await this._enrichNodesWithPeripherals(nodes?.nodes || []);
            this.routing = routing || { version: 0, routes: [], widgets: [] };

            this.rebuildGraph();
            this._installMouseHandlers();
        } catch (err) {
            console.error('Failed to activate routing page:', err);
        }
    }

    async ensureCatalog() {
        if (this.catalog) return this.catalog;
        const result = await window.saintWS.management('get_peripheral_catalog', {});
        this.catalog = result || { peripheral_types: [], widget_types: [] };
        return this.catalog;
    }

    async _enrichNodesWithPeripherals(adoptedNodes) {
        // `get_adopted_nodes` returns identity + counts but not peripherals.
        // Fetch peripherals per-node in parallel.
        const ws = window.saintWS;
        const enriched = await Promise.all(adoptedNodes.map(async (n) => {
            try {
                const result = await ws.management('get_node_peripherals', { node_id: n.node_id });
                return { ...n, peripherals: result?.peripherals || [] };
            } catch (e) {
                return { ...n, peripherals: [] };
            }
        }));
        return enriched;
    }

    // ─── Build the graph from server state ──────────────────────────────

    rebuildGraph(keepPositions = false) {
        const oldPositions = keepPositions ? new Map(
            this.graphNodes.map(n => [n.id, { x: n.x, y: n.y }])
        ) : new Map();

        const graphNodes = [];

        // Peripheral nodes
        for (const node of this.nodes) {
            for (const p of (node.peripherals || [])) {
                const id = `p::${node.node_id}::${p.id}`;
                const stored = this.positions[id] || oldPositions.get(id);
                graphNodes.push({
                    id, kind: 'peripheral',
                    nodeId: node.node_id,
                    nodeLabel: node.display_name || node.node_id,
                    peripheral: p,
                    x: stored?.x ?? 0, y: stored?.y ?? 0,
                });
            }
        }

        // Widget nodes
        for (const w of (this.routing.widgets || [])) {
            const id = `w::${w.id}`;
            const stored = this.positions[id] || oldPositions.get(id);
            graphNodes.push({
                id, kind: 'widget', widget: w,
                x: stored?.x ?? 0, y: stored?.y ?? 0,
            });
        }

        // Signal nodes — union of (paths referenced by any route) + (pending signals)
        const sigPaths = new Set(this.pendingSignals);
        for (const r of (this.routing.routes || [])) {
            if (r.source?.kind === 'signal') sigPaths.add(r.source.parts[0]);
            if (r.sink?.kind === 'signal')   sigPaths.add(r.sink.parts[0]);
        }
        for (const path of sigPaths) {
            const id = `s::${path}`;
            const stored = this.positions[id] || oldPositions.get(id);
            graphNodes.push({
                id, kind: 'signal', signal: path,
                x: stored?.x ?? 0, y: stored?.y ?? 0,
            });
        }

        // Auto-layout any nodes that don't have a saved position.
        this._layoutMissing(graphNodes);

        this.graphNodes = graphNodes;
        this.render();
    }

    _layoutMissing(graphNodes) {
        // Snap missing nodes into a tidy three-column grid:
        //   peripherals on the left, signals in the middle, widgets on the right.
        const cols = { peripheral: 30, signal: 360, widget: 720 };
        const cursorY = { peripheral: 20, signal: 20, widget: 20 };
        const SPACING = 130;
        for (const g of graphNodes) {
            if (g.x === 0 && g.y === 0) {
                g.x = cols[g.kind] ?? 30;
                g.y = cursorY[g.kind] ?? 20;
                cursorY[g.kind] = (cursorY[g.kind] ?? 20) + SPACING;
            }
        }
    }

    autoLayout() {
        // Wipe stored positions and let _layoutMissing place everything.
        this.positions = {};
        this._savePositions();
        for (const g of this.graphNodes) { g.x = 0; g.y = 0; }
        this._layoutMissing(this.graphNodes);
        this.render();
    }

    // ─── Pin metadata: inputs/outputs for each graph node ───────────────

    getNodeMeta(g) {
        if (g.kind === 'peripheral') {
            const p = g.peripheral;
            const type = this.peripheralType(p.type);
            // channel.dir == 'in'  → data flows out of the peripheral (source pin on the right)
            // channel.dir == 'out' → data flows into the peripheral (sink pin on the left)
            const channels = type?.channels || [];
            return {
                title: p.label || p.id,
                subtitle: `${type?.label || p.type} on ${g.nodeLabel}`,
                builtin: !!p.builtin,
                removable: false,
                inputs:  channels.filter(c => c.dir === 'out')
                                 .map(c => ({ id: c.id, label: c.display, cap: c.cap })),
                outputs: channels.filter(c => c.dir === 'in')
                                 .map(c => ({ id: c.id, label: c.display, cap: c.cap })),
            };
        }
        if (g.kind === 'signal') {
            return {
                title: g.signal, subtitle: 'logical signal',
                builtin: false, removable: true,
                inputs:  [{ id: 'in',  label: 'in',  cap: 'any' }],
                outputs: [{ id: 'out', label: 'out', cap: 'any' }],
            };
        }
        if (g.kind === 'widget') {
            const w = g.widget;
            const type = this.widgetType(w.type);
            return {
                title: w.label || w.id,
                subtitle: `${type?.label || w.type} widget`,
                builtin: false, removable: true,
                inputs:  (type?.inputs || []).map(i => ({ id: i.id, label: i.display, cap: i.cap })),
                outputs: [],
            };
        }
        return null;
    }

    // ─── Rendering ──────────────────────────────────────────────────────

    render() {
        const layer = document.getElementById('routing-node-layer');
        if (!layer) return;
        layer.innerHTML = this.graphNodes.map(g => this._renderNodeHtml(g)).join('');
        // Draw wires after the DOM updates so we can read pin positions.
        requestAnimationFrame(() => this.renderWires());
        this._attachNodeInteractions();
    }

    _renderNodeHtml(g) {
        const meta = this.getNodeMeta(g);
        if (!meta) return '';
        const inputsHtml = meta.inputs.map(p => `
            <div class="routing-pin-row input">
                <div class="routing-pin dir-in" data-node="${escapeAttr(g.id)}"
                     data-pin="${escapeAttr(p.id)}" data-dir="in"></div>
                <span class="routing-pin-label-in">${escapeHtml(p.label)}</span>
            </div>`).join('');
        const outputsHtml = meta.outputs.map(p => `
            <div class="routing-pin-row output">
                <span class="routing-pin-label-out">${escapeHtml(p.label)}</span>
                <div class="routing-pin dir-out" data-node="${escapeAttr(g.id)}"
                     data-pin="${escapeAttr(p.id)}" data-dir="out"></div>
            </div>`).join('');
        const removeBtn = meta.removable
            ? `<span class="text-slate-400 hover:text-red-400 cursor-pointer text-base"
                     onclick="routingPage.removeGraphNode('${escapeAttr(g.id)}')"
                     title="Remove from graph">✕</span>`
            : '';
        return `
            <div class="routing-node" data-kind="${g.kind}"
                 data-builtin="${meta.builtin ? 'true' : 'false'}"
                 id="${escapeAttr(g.id)}" style="left:${g.x}px; top:${g.y}px;">
                <div class="routing-node-header">
                    <div class="min-w-0">
                        <div class="truncate">${escapeHtml(meta.title)}</div>
                        <div class="routing-node-kind">${escapeHtml(meta.subtitle)}</div>
                    </div>
                    ${removeBtn}
                </div>
                <div class="routing-node-body">
                    ${inputsHtml}
                    ${outputsHtml}
                </div>
            </div>`;
    }

    renderWires() {
        const layer = document.getElementById('routing-wire-layer');
        if (!layer) return;
        const paths = [];
        for (const route of (this.routing.routes || [])) {
            const fromHandle = this.endpointToHandle(route.source, /* dir= */ 'out');
            const toHandle   = this.endpointToHandle(route.sink,   /* dir= */ 'in');
            if (!fromHandle || !toHandle) continue;
            const a = this._pinPosition(fromHandle);
            const b = this._pinPosition(toHandle);
            if (!a || !b) continue;
            paths.push(`<path d="${this._bezier(a, b)}" fill="none" stroke="#06b6d4" stroke-width="2"
                              data-route-id="${escapeAttr(route.id)}"
                              onclick="routingPage.removeRoute('${escapeAttr(route.id)}')"/>`);
        }
        if (this._connecting?.cursor) {
            const a = this._pinPosition(this._connecting.fromHandle);
            if (a) {
                paths.push(`<path d="${this._bezier(a, this._connecting.cursor)}"
                                  fill="none" stroke="#34d399" stroke-width="2"
                                  stroke-dasharray="4,3"/>`);
            }
        }
        layer.innerHTML = paths.join('');
    }

    _pinPosition(handle) {
        const [nodeId, pinId] = handle.split('/');
        const pin = document.querySelector(
            `.routing-pin[data-node="${cssEscape(nodeId)}"][data-pin="${cssEscape(pinId)}"]`
        );
        if (!pin) return null;
        const canvas = document.getElementById('routing-canvas');
        const cr = canvas.getBoundingClientRect();
        const pr = pin.getBoundingClientRect();
        return {
            x: pr.left + pr.width / 2 - cr.left,
            y: pr.top + pr.height / 2 - cr.top,
        };
    }

    _bezier(a, b) {
        const dx = Math.max(Math.abs(b.x - a.x) * 0.5, 30);
        return `M ${a.x},${a.y} C ${a.x + dx},${a.y} ${b.x - dx},${b.y} ${b.x},${b.y}`;
    }

    // ─── Endpoint <-> graph-handle translation ──────────────────────────

    // graph handle = "<graphNodeId>/<pinId>"
    //   graphNodeId starts with "p::", "s::", or "w::"
    //
    // Map a Route source/sink endpoint to the handle on the graph. The
    // dir tells us which side of a signal node to point at (signals have
    // both an `in` and an `out` pin).
    endpointToHandle(ep, dir) {
        if (!ep) return null;
        if (ep.kind === 'peripheral') {
            const [nodeId, peripheralId, channelId] = ep.parts;
            return `p::${nodeId}::${peripheralId}/${channelId}`;
        }
        if (ep.kind === 'signal') {
            return `s::${ep.parts[0]}/${dir}`;
        }
        if (ep.kind === 'widget') {
            return `w::${ep.parts[0]}/${ep.parts[1]}`;
        }
        return null;
    }

    handleToEndpoint(handle) {
        const [graphNodeId, pinId] = handle.split('/');
        if (graphNodeId.startsWith('p::')) {
            const rest = graphNodeId.slice(3);
            const idx = rest.lastIndexOf('::');
            const nodeId = rest.slice(0, idx);
            const peripheralId = rest.slice(idx + 2);
            return { kind: 'peripheral', parts: [nodeId, peripheralId, pinId] };
        }
        if (graphNodeId.startsWith('s::')) {
            return { kind: 'signal', parts: [graphNodeId.slice(3)] };
        }
        if (graphNodeId.startsWith('w::')) {
            return { kind: 'widget', parts: [graphNodeId.slice(3), pinId] };
        }
        return null;
    }

    // ─── Mouse / pin interactions ───────────────────────────────────────

    _attachNodeInteractions() {
        document.querySelectorAll('.routing-node-header').forEach(h => {
            h.onmousedown = (e) => {
                if (e.target.tagName === 'SPAN' && e.target.onclick) return;
                const node = h.parentElement;
                const g = this.graphNodes.find(n => n.id === node.id);
                if (!g) return;
                this._draggingNode = node.id;
                node.classList.add('dragging');
                this._dragOffset = { x: e.clientX - g.x, y: e.clientY - g.y };
                e.preventDefault();
            };
        });
        document.querySelectorAll('.routing-pin').forEach(p => {
            p.onmousedown = (e) => {
                e.stopPropagation();
                const handle = `${p.dataset.node}/${p.dataset.pin}`;
                const dir = p.dataset.dir;
                if (!this._connecting) {
                    this._connecting = { fromHandle: handle, fromDir: dir };
                    p.classList.add('connecting');
                } else {
                    const a = this._connecting;
                    const b = { handle, dir };
                    let from, to;
                    if (a.fromDir === 'out' && b.dir === 'in') {
                        from = a.fromHandle; to = b.handle;
                    } else if (a.fromDir === 'in' && b.dir === 'out') {
                        from = b.handle; to = a.fromHandle;
                    } else {
                        this._cancelConnecting();
                        return;
                    }
                    if (from === to) { this._cancelConnecting(); return; }
                    this._cancelConnecting();
                    this._submitWire(from, to);
                }
            };
        });
    }

    _installMouseHandlers() {
        if (this._mouseHandler) return;
        this._mouseHandler = (e) => {
            if (this._draggingNode) {
                const g = this.graphNodes.find(n => n.id === this._draggingNode);
                if (g) {
                    g.x = Math.max(0, e.clientX - this._dragOffset.x);
                    g.y = Math.max(0, e.clientY - this._dragOffset.y);
                    const node = document.getElementById(this._draggingNode);
                    if (node) {
                        node.style.left = g.x + 'px';
                        node.style.top  = g.y + 'px';
                    }
                    this.renderWires();
                }
            }
            if (this._connecting) {
                const canvas = document.getElementById('routing-canvas');
                if (canvas) {
                    const cr = canvas.getBoundingClientRect();
                    this._connecting.cursor = { x: e.clientX - cr.left, y: e.clientY - cr.top };
                    this.renderWires();
                }
            }
        };
        this._upHandler = () => {
            if (this._draggingNode) {
                const node = document.getElementById(this._draggingNode);
                if (node) node.classList.remove('dragging');
                const g = this.graphNodes.find(n => n.id === this._draggingNode);
                if (g) {
                    this.positions[g.id] = { x: g.x, y: g.y };
                    this._savePositions();
                }
                this._draggingNode = null;
            }
        };
        this._canvasClickHandler = (e) => {
            if (e.target.id === 'routing-canvas' || e.target.tagName === 'svg') {
                this._cancelConnecting();
            }
        };
        document.addEventListener('mousemove', this._mouseHandler);
        document.addEventListener('mouseup',   this._upHandler);
        document.getElementById('routing-canvas')
            ?.addEventListener('click', this._canvasClickHandler);
    }

    _cancelConnecting() {
        document.querySelectorAll('.routing-pin').forEach(p => p.classList.remove('connecting'));
        this._connecting = null;
        this.renderWires();
    }

    // ─── Graph mutations (server round-trip) ────────────────────────────

    async _submitWire(fromHandle, toHandle) {
        const source = this.handleToEndpoint(fromHandle);
        const sink   = this.handleToEndpoint(toHandle);
        if (!source || !sink) return;

        try {
            const result = await window.saintWS.management('add_route', { source, sink });
            if (result && result.success === false) {
                alert('Cannot add route: ' + (result.message || 'unknown error'));
                return;
            }
            // Server will broadcast system_routing; if we don't get it, refresh.
            await this._refreshRouting();
        } catch (err) {
            console.error('Add route failed:', err);
        }
    }

    async removeRoute(routeId) {
        if (!confirm('Remove this route?')) return;
        try {
            await window.saintWS.management('remove_route', { route_id: routeId });
            await this._refreshRouting();
        } catch (err) {
            console.error('Remove route failed:', err);
        }
    }

    async removeGraphNode(graphNodeId) {
        if (graphNodeId.startsWith('s::')) {
            // Signals only exist as long as routes reference them; remove
            // any pending entry + drop routes touching this signal.
            if (!confirm('Remove this signal and all routes touching it?')) return;
            const path = graphNodeId.slice(3);
            this.pendingSignals = this.pendingSignals.filter(s => s !== path);
            this._savePendingSignals();
            // Drop touching routes server-side.
            const touching = (this.routing.routes || []).filter(r =>
                (r.source.kind === 'signal' && r.source.parts[0] === path) ||
                (r.sink.kind === 'signal'   && r.sink.parts[0] === path));
            for (const r of touching) {
                try { await window.saintWS.management('remove_route', { route_id: r.id }); }
                catch (e) { console.warn('Failed to drop route', r.id, e); }
            }
            await this._refreshRouting();
            return;
        }
        if (graphNodeId.startsWith('w::')) {
            if (!confirm('Remove this widget? Any routes wired to it will be deleted too.')) return;
            const widgetId = graphNodeId.slice(3);
            try {
                await window.saintWS.management('remove_widget', { widget_id: widgetId });
            } catch (e) {
                console.error('Remove widget failed:', e);
                return;
            }
            await this._refreshRouting();
            return;
        }
        // Peripheral nodes can't be removed from the graph — they're
        // removed from the Peripherals tab instead.
    }

    async _refreshRouting() {
        try {
            const routing = await window.saintWS.management('get_system_routing', {});
            this.routing = routing || { version: 0, routes: [], widgets: [] };
            this.rebuildGraph(/* keepPositions= */ true);
        } catch (e) {
            console.warn('Failed to refresh routing:', e);
        }
    }

    // ─── Add signal / widget ───────────────────────────────────────────

    promptAddSignal() {
        const name = prompt('Logical signal name (e.g. /battery/main/current):');
        if (!name) return;
        const path = name.trim();
        if (!path) return;
        if (this.pendingSignals.includes(path)) return;
        // Verify it's not already implied by a route — that means the node
        // is already on the graph; nothing to do.
        const inUse = (this.routing.routes || []).some(r =>
            (r.source.kind === 'signal' && r.source.parts[0] === path) ||
            (r.sink.kind   === 'signal' && r.sink.parts[0]   === path));
        if (!inUse) {
            this.pendingSignals.push(path);
            this._savePendingSignals();
        }
        this.rebuildGraph(/* keepPositions= */ true);
    }

    openWidgetModal() {
        if (!this.catalog) return;
        const sel = document.getElementById('routing-widget-type');
        const desc = document.getElementById('routing-widget-type-desc');
        const labelInput = document.getElementById('routing-widget-label');
        sel.innerHTML = (this.catalog.widget_types || [])
            .map(t => `<option value="${escapeAttr(t.id)}">${escapeHtml(t.label)}</option>`).join('');
        sel.onchange = () => {
            const t = this.widgetType(sel.value);
            desc.textContent = t?.description || '';
        };
        sel.onchange();
        labelInput.value = '';
        document.getElementById('routing-widget-error').classList.add('hidden');
        document.getElementById('routing-widget-modal').classList.remove('hidden');
    }

    closeWidgetModal() {
        document.getElementById('routing-widget-modal')?.classList.add('hidden');
    }

    async saveWidgetModal() {
        const sel = document.getElementById('routing-widget-type');
        const labelEl = document.getElementById('routing-widget-label');
        const type = sel.value;
        const label = labelEl.value.trim() || this.widgetType(type)?.label || type;
        const errEl = document.getElementById('routing-widget-error');
        errEl.classList.add('hidden');
        try {
            const result = await window.saintWS.management('add_widget', {
                type, label, position: [720, 20],
            });
            if (result && result.success === false) {
                errEl.textContent = result.message || 'Failed to add widget';
                errEl.classList.remove('hidden');
                return;
            }
            this.closeWidgetModal();
            await this._refreshRouting();
        } catch (err) {
            errEl.textContent = String(err.message || err);
            errEl.classList.remove('hidden');
        }
    }

    // ─── localStorage helpers ──────────────────────────────────────────

    _loadPositions() {
        try { return JSON.parse(localStorage.getItem('saint.routing.positions') || '{}'); }
        catch (e) { return {}; }
    }
    _savePositions() {
        try { localStorage.setItem('saint.routing.positions', JSON.stringify(this.positions)); }
        catch (e) { /* ignore quota errors */ }
    }
    _loadPendingSignals() {
        try { return JSON.parse(localStorage.getItem('saint.routing.signals') || '[]'); }
        catch (e) { return []; }
    }
    _savePendingSignals() {
        try { localStorage.setItem('saint.routing.signals', JSON.stringify(this.pendingSignals)); }
        catch (e) { /* ignore */ }
    }

    peripheralType(id) {
        return (this.catalog?.peripheral_types || []).find(t => t.id === id);
    }
    widgetType(id) {
        return (this.catalog?.widget_types || []).find(t => t.id === id);
    }
}

// ─── escape helpers (peripherals.js declares these too; keep guarded) ───
if (typeof escapeHtml === 'undefined') {
    window.escapeHtml = function (s) {
        if (s === null || s === undefined) return '';
        return String(s)
            .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
    };
}
if (typeof escapeAttr === 'undefined') {
    window.escapeAttr = window.escapeHtml;
}
function cssEscape(s) {
    // CSS attribute selector escape. Browser's CSS.escape is widely
    // available, but our identifiers can contain `::`, `/`, etc.
    return (window.CSS && window.CSS.escape) ? CSS.escape(s) : String(s).replace(/(["'\\])/g, '\\$1');
}

const routingPage = new RoutingPage();
window.routingPage = routingPage;

document.addEventListener('DOMContentLoaded', () => {
    routingPage.init();
});
