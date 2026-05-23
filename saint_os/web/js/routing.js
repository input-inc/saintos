/**
 * SAINT.OS Routing Page
 *
 * Per-controller graph editor. One sheet per adopted controller node
 * plus a dashboard sheet for widgets. Each sheet contains:
 *   - Input nodes     (ROS topic + scalar channel)
 *   - Operator nodes  (server-evaluated transforms: Max/Min/Clamp/…)
 *   - Wires           (input/operator output → operator pin / peripheral
 *                      channel / widget input)
 *
 * Layout-only state (node positions, currently-selected sheet) lives in
 * localStorage; the canonical graph lives in `system_routing.yaml`
 * server-side.
 */

class RoutingPage {
    constructor() {
        this.catalog = null;          // peripheral_types + widget_types + operator_types
        this.topicCatalog = [];       // list of {topic, state_type, channels[]}
        this.nodes = [];              // adopted controller nodes (with peripherals)
        this.routing = { version: 0, sheets: {} };

        this.activeSheetId = this._loadActiveSheet();
        // Map "sheetId:nodeId" → {x, y}
        this.positions = this._loadPositions();
        this.graphNodes = [];
        // Live per-sheet values pushed from the server's routing
        // evaluator (`routing_values` topic). Shape:
        //   { sheets: { <sheet_id>: { inputs: {id: value},
        //                              operators: {id: value},
        //                              outputs: {id: value},
        //                              widgets: {"wid/inp": value} } } }
        this.values = { sheets: {} };

        // Interaction state
        this._connecting = null;
        this._draggingNode = null;
        this._dragOffset = { x: 0, y: 0 };
        this._mouseHandler = null;
        this._upHandler = null;
        this._canvasClickHandler = null;
        this._active = false;
    }

    init() {
        document.getElementById('btn-routing-add-input')
            ?.addEventListener('click', () => this.openInputModal());
        document.getElementById('btn-routing-add-output')
            ?.addEventListener('click', () => this.openOutputModal());
        document.getElementById('btn-routing-add-operator')
            ?.addEventListener('click', () => this.openOperatorModal());
        document.getElementById('btn-routing-add-widget')
            ?.addEventListener('click', () => this.openWidgetModal());
        document.getElementById('btn-routing-auto-layout')
            ?.addEventListener('click', () => this.autoLayout());

        const ws = window.saintWS;
        if (ws) {
            ws.on('state', (msg) => {
                if (!msg) return;
                if (msg.node === 'system_routing') {
                    this.routing = msg.data || { version: 0, sheets: {} };
                    if (this._active) this.render();
                } else if (msg.node === 'routing_values') {
                    this.values = msg.data || { sheets: {} };
                    if (this._active) this._renderValueOverlays();
                }
            });
        }
    }

    async activate() {
        this._active = true;
        const ws = window.saintWS;
        if (!ws) return;

        try {
            await ws.subscribe(['system_routing', 'routing_values']);
        } catch (e) {
            console.warn('Failed to subscribe to system_routing:', e);
        }

        try {
            const [catalog, nodes, routing, topics] = await Promise.all([
                this.ensureCatalog(),
                ws.management('list_adopted', {}),
                ws.management('get_system_routing', {}),
                ws.send('ros', 'list_topic_channels', {}).catch(() => ({ topics: [] })),
            ]);
            this.catalog = catalog;
            this.nodes = await this._enrichNodesWithPeripherals(nodes?.nodes || []);
            this.routing = routing || { version: 0, sheets: {} };
            this.topicCatalog = (topics?.topics) || [];

            this._ensureActiveSheet();
            this._installMouseHandlers();
            this.render();
        } catch (err) {
            console.error('Failed to activate routing page:', err);
        }
    }

    deactivate() { this._active = false; }

    async ensureCatalog() {
        if (this.catalog) return this.catalog;
        const result = await window.saintWS.management('get_peripheral_catalog', {});
        this.catalog = result || { peripheral_types: [], widget_types: [], operator_types: [] };
        return this.catalog;
    }

    async _enrichNodesWithPeripherals(adoptedNodes) {
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

    // ── Sheet selection ───────────────────────────────────────────────

    _ensureActiveSheet() {
        const sheetIds = this._sheetIds();
        if (!sheetIds.includes(this.activeSheetId)) {
            this.activeSheetId = sheetIds[0] || null;
            this._saveActiveSheet();
        }
    }

    _sheetIds() {
        // Sheets in display order: one per adopted controller node.
        return this.nodes.map(n => n.node_id);
    }

    selectSheet(sheetId) {
        this.activeSheetId = sheetId;
        this._saveActiveSheet();
        this._cancelConnecting();
        this.render();
    }

    _sheetData(sheetId) {
        return this.routing.sheets?.[sheetId]
            || { node_id: sheetId, inputs: [], ws_inputs: [],
                 outputs: [], operators: [], widgets: [], wires: [] };
    }

    _activeSheet() {
        return this._sheetData(this.activeSheetId);
    }

    _activeNode() {
        return this.nodes.find(n => n.node_id === this.activeSheetId) || null;
    }

    // ── Rendering ─────────────────────────────────────────────────────

    render() {
        this._renderSheetList();
        this._renderToolbar();
        this._renderCanvas();
    }

    _renderSheetList() {
        const list = document.getElementById('routing-sheet-list');
        if (!list) return;
        const html = [];
        for (const node of this.nodes) {
            const sheet = this._sheetData(node.node_id);
            const count = (sheet.wires || []).length;
            const isActive = node.node_id === this.activeSheetId;
            const label = node.display_name || node.node_id;
            html.push(`
                <div class="routing-sheet-item ${isActive ? 'active' : ''}"
                     onclick="routingPage.selectSheet('${escapeAttr(node.node_id)}')">
                    <span class="material-icons">memory</span>
                    <span class="truncate">${escapeHtml(label)}</span>
                    ${count ? `<span class="routing-sheet-count">${count}</span>` : ''}
                </div>
            `);
        }
        list.innerHTML = html.join('') || '<div class="text-xs text-slate-500 p-2">No controller nodes adopted yet.</div>';
    }

    _renderToolbar() {
        const title = document.getElementById('routing-sheet-title');
        const subtitle = document.getElementById('routing-sheet-subtitle');
        const addInput = document.getElementById('btn-routing-add-input');
        const addOutput = document.getElementById('btn-routing-add-output');
        const addOp = document.getElementById('btn-routing-add-operator');
        const addWidget = document.getElementById('btn-routing-add-widget');
        const autoBtn = document.getElementById('btn-routing-auto-layout');

        const hasSheet = !!this.activeSheetId;
        if (addInput)  addInput.disabled  = !hasSheet;
        if (addOutput) addOutput.disabled = !hasSheet;
        if (addOp)     addOp.disabled     = !hasSheet;
        if (addWidget) addWidget.disabled = !hasSheet;
        if (autoBtn)   autoBtn.disabled   = !hasSheet;

        const node = this._activeNode();
        if (!node) {
            if (title)    title.textContent    = 'Routing';
            if (subtitle) subtitle.textContent = 'Adopt a controller node to start.';
            return;
        }
        if (title) title.textContent = node.display_name || node.node_id;
        if (subtitle) {
            const sheet = this._sheetData(node.node_id);
            const periphCount = (node.peripherals || []).length;
            const wsCount = (sheet.ws_inputs || []).length;
            subtitle.textContent =
                `${periphCount} peripheral${periphCount === 1 ? '' : 's'} · `
              + `${sheet.inputs.length} in · `
              + `${wsCount} ws-in · `
              + `${sheet.outputs.length} out · `
              + `${sheet.operators.length} op · `
              + `${sheet.widgets.length} widget${sheet.widgets.length === 1 ? '' : 's'} · `
              + `${sheet.wires.length} wire${sheet.wires.length === 1 ? '' : 's'}`;
        }
    }

    _renderCanvas() {
        const layer = document.getElementById('routing-node-layer');
        const empty = document.getElementById('routing-empty');
        if (!layer) return;
        if (!this.activeSheetId) {
            layer.innerHTML = '';
            if (empty) empty.classList.remove('hidden');
            this._renderWires();
            return;
        }
        if (empty) empty.classList.add('hidden');
        this.graphNodes = this._buildGraphNodes();
        this._layoutMissing(this.graphNodes);
        layer.innerHTML = this.graphNodes.map(g => this._renderNodeHtml(g)).join('');
        requestAnimationFrame(() => {
            this._renderWires();
            this._renderValueOverlays();
        });
        this._attachNodeInteractions();
    }

    /** Update the per-pin value pills from the latest `routing_values`
     *  snapshot. Cheap to call on every update — only mutates the
     *  textContent of pre-rendered <span>s, no DOM rebuild. */
    _renderValueOverlays() {
        if (!this.activeSheetId) return;
        const sheetVals = this.values?.sheets?.[this.activeSheetId];
        if (!sheetVals) return;
        const idFor = (graphNodeId, pinId, dir) =>
            `val-${dir}-${graphNodeId.replace(/[^a-zA-Z0-9_-]/g, '_')}-${pinId}`;
        const fmt = (v) => {
            if (typeof v !== 'number' || !Number.isFinite(v)) return '';
            // Three sig figs is plenty for a live overlay.
            const a = Math.abs(v);
            if (a === 0)    return '0';
            if (a < 0.01)   return v.toExponential(1);
            if (a < 100)    return v.toFixed(2);
            return v.toFixed(0);
        };
        const set = (elId, value) => {
            const el = document.getElementById(elId);
            if (!el) return;
            el.textContent = value === undefined || value === null ? '' : fmt(value);
        };

        for (const g of this.graphNodes) {
            if (g.kind === 'input') {
                set(idFor(g.id, 'out', 'out'), sheetVals.inputs?.[g.sheetNodeId]);
            } else if (g.kind === 'ws_input') {
                set(idFor(g.id, 'out', 'out'), sheetVals.ws_inputs?.[g.sheetNodeId]);
            } else if (g.kind === 'operator') {
                set(idFor(g.id, 'out', 'out'), sheetVals.operators?.[g.sheetNodeId]);
            } else if (g.kind === 'output') {
                // Same value flows through both pins (sink in, tap out).
                set(idFor(g.id, 'value', 'in'),  sheetVals.outputs?.[g.sheetNodeId]);
                set(idFor(g.id, 'out',   'out'), sheetVals.outputs?.[g.sheetNodeId]);
            } else if (g.kind === 'widget' && g.widget) {
                const wType = this._widgetType(g.widget.type);
                for (const inp of (wType?.inputs || [])) {
                    const key = `${g.widget.id}/${inp.id}`;
                    set(idFor(g.id, inp.id, 'in'), sheetVals.widgets?.[key]);
                }
            }
        }
    }

    _buildGraphNodes() {
        const out = [];
        const sheet = this._activeSheet();
        // Input nodes (left column) — subscribed to a real ROS topic.
        for (const n of (sheet.inputs || [])) {
            const id = `in::${n.id}`;
            const pos = this._positionFor(id, n.position);
            out.push({ id, kind: 'input', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y });
        }
        // WebSocket-input nodes (left column) — controller writes the value.
        for (const n of (sheet.ws_inputs || [])) {
            const id = `ws::${n.id}`;
            const pos = this._positionFor(id, n.position);
            out.push({ id, kind: 'ws_input', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y });
        }
        // Operator nodes (middle column).
        for (const n of (sheet.operators || [])) {
            const id = `op::${n.id}`;
            const pos = this._positionFor(id, n.position);
            out.push({ id, kind: 'operator', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y });
        }
        // Output nodes (right column) — publish back into ROS topics.
        for (const n of (sheet.outputs || [])) {
            const id = `out::${n.id}`;
            const pos = this._positionFor(id, n.position);
            out.push({ id, kind: 'output', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y });
        }
        // Widget sinks — attached to this controller's dashboard surface.
        for (const w of (sheet.widgets || [])) {
            const id = `w::${w.id}`;
            const pos = this._positionFor(id, w.position);
            out.push({ id, kind: 'widget', sheetNodeId: w.id, widget: w, x: pos.x, y: pos.y });
        }
        // Peripheral sinks (right-most column) — this controller's peripherals.
        const node = this._activeNode();
        for (const p of ((node && node.peripherals) || [])) {
            const id = `p::${node.node_id}::${p.id}`;
            const pos = this._positionFor(id, null);
            out.push({ id, kind: 'peripheral', nodeId: node.node_id,
                       nodeLabel: node.display_name || node.node_id,
                       peripheral: p, x: pos.x, y: pos.y });
        }
        return out;
    }

    _positionFor(id, fallback) {
        const key = `${this.activeSheetId}:${id}`;
        const saved = this.positions[key];
        if (saved && Number.isFinite(saved.x) && Number.isFinite(saved.y)) {
            return { x: saved.x, y: saved.y };
        }
        if (Array.isArray(fallback) && fallback.length >= 2
            && (fallback[0] || fallback[1])) {
            return { x: fallback[0], y: fallback[1] };
        }
        return { x: 0, y: 0 };
    }

    _layoutMissing(graphNodes) {
        // Default columns: inputs on the left, operators middle, sinks
        // (output topics, widgets, peripherals) on the right.
        const cols = { input: 30, ws_input: 30, operator: 320, output: 640, widget: 640, peripheral: 900 };
        const cursorY = { input: 30, ws_input: 30, operator: 30, output: 30, widget: 30, peripheral: 30 };
        const SPACING = 140;
        for (const g of graphNodes) {
            if (g.x === 0 && g.y === 0) {
                g.x = cols[g.kind] ?? 30;
                g.y = cursorY[g.kind] ?? 30;
                cursorY[g.kind] = (cursorY[g.kind] ?? 30) + SPACING;
            }
        }
    }

    autoLayout() {
        const sheet = this.activeSheetId;
        if (!sheet) return;
        // Wipe positions for this sheet only.
        const prefix = `${sheet}:`;
        for (const key of Object.keys(this.positions)) {
            if (key.startsWith(prefix)) delete this.positions[key];
        }
        this._savePositions();
        this.render();
    }

    _renderNodeHtml(g) {
        const meta = this._getNodeMeta(g);
        if (!meta) return '';
        const valId = (pinId, dir) =>
            `val-${dir}-${g.id.replace(/[^a-zA-Z0-9_-]/g, '_')}-${pinId}`;
        const inputsHtml = meta.inputs.map(p => `
            <div class="routing-pin-row input">
                <div class="routing-pin dir-in" data-node="${escapeAttr(g.id)}"
                     data-pin="${escapeAttr(p.id)}" data-dir="in"></div>
                <span class="routing-pin-label-in">${escapeHtml(p.label)}</span>
                <span id="${valId(p.id, 'in')}" class="routing-pin-value"></span>
                ${this._renderDefaultInput(g, p)}
            </div>`).join('');
        const outputsHtml = meta.outputs.map(p => `
            <div class="routing-pin-row output">
                <span id="${valId(p.id, 'out')}" class="routing-pin-value"></span>
                <span class="routing-pin-label-out">${escapeHtml(p.label)}</span>
                <div class="routing-pin dir-out" data-node="${escapeAttr(g.id)}"
                     data-pin="${escapeAttr(p.id)}" data-dir="out"></div>
            </div>`).join('');
        const removeBtn = meta.removable
            ? `<span class="text-slate-400 hover:text-red-400 cursor-pointer text-base"
                     onclick="routingPage.removeSheetNode('${escapeAttr(g.id)}')"
                     title="Remove from sheet">✕</span>`
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

    _renderDefaultInput(g, pin) {
        // Operator inputs get an inline default-constant field, used as
        // the input value when no wire is connected.
        if (g.kind !== 'operator' || !pin.editable) return '';
        const wired = this._isPinWired(g.id, pin.id);
        if (wired) return '';
        const op = g.data;
        const cur = (op.defaults && op.defaults[pin.id] !== undefined)
            ? op.defaults[pin.id]
            : (pin.default ?? 0);
        return `<input type="number" step="0.01"
                       class="routing-pin-default"
                       value="${escapeAttr(String(cur))}"
                       onclick="event.stopPropagation()"
                       onmousedown="event.stopPropagation()"
                       onchange="routingPage.setOperatorDefault('${escapeAttr(op.id)}', '${escapeAttr(pin.id)}', this.value)">`;
    }

    _isPinWired(graphNodeId, pinId) {
        const sheet = this._activeSheet();
        for (const w of (sheet.wires || [])) {
            if (w.sink.kind === 'operator'
                && w.sink.parts && w.sink.parts.length >= 2
                && `op::${w.sink.parts[0]}` === graphNodeId
                && w.sink.parts[1] === pinId) {
                return true;
            }
        }
        return false;
    }

    _getNodeMeta(g) {
        if (g.kind === 'input') {
            const inp = g.data;
            const topicMeta = this.topicCatalog.find(t => t.topic === inp.topic);
            return {
                title: inp.label || `${inp.topic}${inp.field ? '.' + inp.field : ''}`,
                subtitle: `ROS · ${inp.topic}${inp.field ? ' · ' + inp.field : ''}`
                          + (topicMeta ? ` (${topicMeta.state_type})` : ''),
                builtin: false, removable: true,
                inputs: [],
                outputs: [{ id: 'out', label: 'value' }],
            };
        }
        if (g.kind === 'ws_input') {
            const inp = g.data;
            return {
                title: inp.label || inp.id,
                subtitle: `WebSocket input · ${inp.id}`,
                builtin: false, removable: true,
                inputs: [],
                outputs: [{ id: 'out', label: 'value' }],
            };
        }
        if (g.kind === 'output') {
            const o = g.data;
            const topicMeta = this.topicCatalog.find(t => t.topic === o.topic);
            return {
                title: o.label || `${o.topic}${o.field ? '.' + o.field : ''}`,
                subtitle: `→ ${o.topic}${o.field ? ' · ' + o.field : ''}`
                          + (topicMeta ? ` (${topicMeta.state_type})` : ''),
                builtin: false, removable: true,
                inputs: [{ id: 'value', label: 'value' }],
                // Tap pin: a wire from this pin re-emits the same value
                // that was published to ROS, so the same sheet can chain
                // ROS output → peripheral on the same value.
                outputs: [{ id: 'out', label: 'tap' }],
            };
        }
        if (g.kind === 'operator') {
            const op = g.data;
            const type = this._operatorType(op.op);
            const ins = (type?.inputs || []).map(spec => ({
                id: spec.id, label: spec.label,
                editable: true, default: spec.default,
            }));
            return {
                title: op.label || (type?.label || op.op),
                subtitle: type ? type.description : `operator: ${op.op}`,
                builtin: false, removable: true,
                inputs:  ins,
                outputs: [{ id: 'out', label: 'out' }],
            };
        }
        if (g.kind === 'peripheral') {
            const p = g.peripheral;
            const type = this._peripheralType(p.type);
            const channels = type?.channels || [];
            return {
                title: p.label || p.id,
                subtitle: `${type?.label || p.type} on ${g.nodeLabel}`,
                builtin: !!p.builtin, removable: false,
                inputs:  channels.filter(c => c.dir === 'out').map(c => ({ id: c.id, label: c.display })),
                outputs: channels.filter(c => c.dir === 'in').map(c => ({ id: c.id, label: c.display })),
            };
        }
        if (g.kind === 'widget') {
            const w = g.widget;
            const type = this._widgetType(w.type);
            return {
                title: w.label || w.id,
                subtitle: `${type?.label || w.type} widget`,
                builtin: false, removable: true,
                inputs:  (type?.inputs || []).map(i => ({ id: i.id, label: i.display })),
                outputs: [],
            };
        }
        return null;
    }

    _renderWires() {
        const layer = document.getElementById('routing-wire-layer');
        if (!layer) return;
        const paths = [];
        const sheet = this._activeSheet();
        for (const w of (sheet.wires || [])) {
            const a = this._pinPositionFromEndpoint(w.source, 'out');
            const b = this._pinPositionFromEndpoint(w.sink, 'in');
            if (!a || !b) continue;
            paths.push(`<path d="${this._bezier(a, b)}" fill="none" stroke="#06b6d4" stroke-width="2"
                              data-wire-id="${escapeAttr(w.id)}"
                              onclick="routingPage.removeWire('${escapeAttr(w.id)}')"/>`);
        }
        if (this._connecting?.cursor) {
            const a = this._pinPositionByHandle(this._connecting.fromHandle);
            if (a) {
                paths.push(`<path d="${this._bezier(a, this._connecting.cursor)}"
                                  fill="none" stroke="#34d399" stroke-width="2"
                                  stroke-dasharray="4,3"/>`);
            }
        }
        layer.innerHTML = paths.join('');
    }

    _pinPositionFromEndpoint(ep, dirHint) {
        const handle = this._endpointToHandle(ep, dirHint);
        return handle ? this._pinPositionByHandle(handle) : null;
    }

    _pinPositionByHandle(handle) {
        const [nodeId, pinId] = handle.split('/');
        const pin = document.querySelector(
            `.routing-pin[data-node="${cssEscape(nodeId)}"][data-pin="${cssEscape(pinId)}"]`
        );
        if (!pin) return null;
        const canvas = document.getElementById('routing-canvas');
        const cr = canvas.getBoundingClientRect();
        const pr = pin.getBoundingClientRect();
        return {
            x: pr.left + pr.width / 2 - cr.left + canvas.scrollLeft,
            y: pr.top + pr.height / 2 - cr.top + canvas.scrollTop,
        };
    }

    _bezier(a, b) {
        const dx = Math.max(Math.abs(b.x - a.x) * 0.5, 30);
        return `M ${a.x},${a.y} C ${a.x + dx},${a.y} ${b.x - dx},${b.y} ${b.x},${b.y}`;
    }

    // ── Endpoint ↔ DOM-handle translation ────────────────────────────

    _endpointToHandle(ep, dirHint) {
        if (!ep) return null;
        if (ep.kind === 'input')      return `in::${ep.parts[0]}/out`;
        if (ep.kind === 'ws_input')   return `ws::${ep.parts[0]}/out`;
        if (ep.kind === 'output') {
            // Output is both a sink (left pin "value") and a tap source
            // (right pin "out"). Pick by direction so wires render to
            // the right side of the card.
            return dirHint === 'out'
                ? `out::${ep.parts[0]}/out`
                : `out::${ep.parts[0]}/value`;
        }
        if (ep.kind === 'operator')   return `op::${ep.parts[0]}/${ep.parts[1] || 'out'}`;
        if (ep.kind === 'peripheral') return `p::${ep.parts[0]}::${ep.parts[1]}/${ep.parts[2]}`;
        if (ep.kind === 'widget')     return `w::${ep.parts[0]}/${ep.parts[1]}`;
        return null;
    }

    _handleToEndpoint(handle) {
        const [graphNodeId, pinId] = handle.split('/');
        if (graphNodeId.startsWith('in::')) {
            return { kind: 'input', parts: [graphNodeId.slice(4)] };
        }
        if (graphNodeId.startsWith('ws::')) {
            return { kind: 'ws_input', parts: [graphNodeId.slice(4)] };
        }
        if (graphNodeId.startsWith('out::')) {
            return { kind: 'output', parts: [graphNodeId.slice(5)] };
        }
        if (graphNodeId.startsWith('op::')) {
            return { kind: 'operator', parts: [graphNodeId.slice(4), pinId] };
        }
        if (graphNodeId.startsWith('p::')) {
            const rest = graphNodeId.slice(3);
            const idx = rest.lastIndexOf('::');
            return { kind: 'peripheral',
                     parts: [rest.slice(0, idx), rest.slice(idx + 2), pinId] };
        }
        if (graphNodeId.startsWith('w::')) {
            return { kind: 'widget', parts: [graphNodeId.slice(3), pinId] };
        }
        return null;
    }

    // ── Interactions ─────────────────────────────────────────────────

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
                    this._renderWires();
                }
            }
            if (this._connecting) {
                const canvas = document.getElementById('routing-canvas');
                if (canvas) {
                    const cr = canvas.getBoundingClientRect();
                    this._connecting.cursor = {
                        x: e.clientX - cr.left + canvas.scrollLeft,
                        y: e.clientY - cr.top + canvas.scrollTop,
                    };
                    this._renderWires();
                }
            }
        };
        this._upHandler = () => {
            if (this._draggingNode) {
                const node = document.getElementById(this._draggingNode);
                if (node) node.classList.remove('dragging');
                const g = this.graphNodes.find(n => n.id === this._draggingNode);
                if (g) {
                    const key = `${this.activeSheetId}:${g.id}`;
                    this.positions[key] = { x: g.x, y: g.y };
                    this._savePositions();
                    // Persist sheet-owned node positions to the server too so
                    // they're shared across clients. Peripherals aren't
                    // sheet-owned (they belong to the controller).
                    if (g.sheetNodeId) {
                        window.saintWS.management('update_sheet_node', {
                            node_id: this.activeSheetId,
                            sheet_node_id: g.sheetNodeId,
                            position: [g.x, g.y],
                        }).catch(() => {});
                    }
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
        this._renderWires();
    }

    // ── Graph mutations (server round-trip) ──────────────────────────

    async _submitWire(fromHandle, toHandle) {
        const source = this._handleToEndpoint(fromHandle);
        const sink   = this._handleToEndpoint(toHandle);
        if (!source || !sink) return;
        try {
            const result = await window.saintWS.management('add_routing_wire', {
                node_id: this.activeSheetId, source, sink,
            });
            if (result && result.success === false) {
                alert('Cannot add wire: ' + (result.message || 'unknown error'));
            }
        } catch (err) {
            console.error('Add wire failed:', err);
        }
    }

    async removeWire(wireId) {
        if (!confirm('Remove this wire?')) return;
        try {
            await window.saintWS.management('remove_routing_wire', {
                node_id: this.activeSheetId, wire_id: wireId,
            });
        } catch (err) {
            console.error('Remove wire failed:', err);
        }
    }

    async removeSheetNode(graphNodeId) {
        // Every sheet-owned node (input, output, operator, widget) is
        // removed through the same remove_sheet_node action — the server
        // dispatches by id across the four lists.
        let sheetNodeId = null;
        if (graphNodeId.startsWith('in::'))   sheetNodeId = graphNodeId.slice(4);
        else if (graphNodeId.startsWith('ws::'))  sheetNodeId = graphNodeId.slice(4);
        else if (graphNodeId.startsWith('out::')) sheetNodeId = graphNodeId.slice(5);
        else if (graphNodeId.startsWith('op::'))  sheetNodeId = graphNodeId.slice(4);
        else if (graphNodeId.startsWith('w::'))   sheetNodeId = graphNodeId.slice(3);
        if (!sheetNodeId) return;
        if (!confirm('Remove this node? Wires touching it will be deleted.')) return;
        try {
            await window.saintWS.management('remove_sheet_node', {
                node_id: this.activeSheetId, sheet_node_id: sheetNodeId,
            });
        } catch (e) { console.error('Remove sheet node failed:', e); }
    }

    async setOperatorDefault(opId, pinId, valueStr) {
        const value = Number(valueStr);
        if (!Number.isFinite(value)) return;
        const sheet = this._activeSheet();
        const op = (sheet.operators || []).find(o => o.id === opId);
        if (!op) return;
        const defaults = { ...(op.defaults || {}), [pinId]: value };
        try {
            await window.saintWS.management('update_sheet_node', {
                node_id: this.activeSheetId, sheet_node_id: opId, defaults,
            });
        } catch (e) { console.error('Update operator defaults failed:', e); }
    }

    // ── Add Input modal ──────────────────────────────────────────────

    openInputModal() {
        if (!this.activeSheetId) return;
        const kindSel = document.getElementById('routing-input-kind');
        const topicSel = document.getElementById('routing-input-topic');
        const fieldSel = document.getElementById('routing-input-field');
        const desc = document.getElementById('routing-input-topic-desc');
        const labelInput = document.getElementById('routing-input-label');
        const topicRow = document.getElementById('routing-input-topic-row');
        const fieldRow = document.getElementById('routing-input-field-row');
        if (!topicSel || !fieldSel || !kindSel) return;

        topicSel.innerHTML = this.topicCatalog.map(t =>
            `<option value="${escapeAttr(t.topic)}">${escapeHtml(t.topic)}</option>`
        ).join('') || '<option value="">(no ROS topics discovered)</option>';

        const refreshFields = () => {
            const t = this.topicCatalog.find(x => x.topic === topicSel.value);
            desc.textContent = t ? `${t.state_type} · ${t.channels.length} channel(s)` : '';
            fieldSel.innerHTML = (t?.channels || []).map(c =>
                `<option value="${escapeAttr(c.field)}">${escapeHtml(c.label)} (${escapeHtml(c.type || 'num')})</option>`
            ).join('') || '<option value="">(message has no scalar fields)</option>';
        };
        topicSel.onchange = refreshFields;
        refreshFields();

        const refreshKindUI = () => {
            const isWs = kindSel.value === 'ws_input';
            topicRow.classList.toggle('hidden', isWs);
            fieldRow.classList.toggle('hidden', isWs);
            labelInput.placeholder = isWs
                ? 'e.g. "Left stick X"'
                : 'Defaults to topic.channel';
        };
        kindSel.onchange = refreshKindUI;
        // Default to WS input — that's the path the controller bindings
        // now use; ROS-state subscriptions are the secondary option.
        kindSel.value = 'ws_input';
        refreshKindUI();

        labelInput.value = '';
        document.getElementById('routing-input-error').classList.add('hidden');
        document.getElementById('routing-input-modal').classList.remove('hidden');
    }

    closeInputModal() {
        document.getElementById('routing-input-modal')?.classList.add('hidden');
    }

    async saveInputModal() {
        const kind = document.getElementById('routing-input-kind').value;
        const label = document.getElementById('routing-input-label').value.trim();
        const errEl = document.getElementById('routing-input-error');
        errEl.classList.add('hidden');
        try {
            let result;
            if (kind === 'ws_input') {
                result = await window.saintWS.management('add_routing_ws_input', {
                    node_id: this.activeSheetId, label,
                });
            } else {
                const topic = document.getElementById('routing-input-topic').value;
                const field = document.getElementById('routing-input-field').value;
                if (!topic) {
                    errEl.textContent = 'Pick a ROS topic';
                    errEl.classList.remove('hidden');
                    return;
                }
                result = await window.saintWS.management('add_routing_input', {
                    node_id: this.activeSheetId, topic, field, label,
                });
            }
            if (result && result.success === false) {
                errEl.textContent = result.message || 'Failed to add input';
                errEl.classList.remove('hidden');
                return;
            }
            this.closeInputModal();
        } catch (err) {
            errEl.textContent = String(err.message || err);
            errEl.classList.remove('hidden');
        }
    }

    // ── Add Output modal ─────────────────────────────────────────────

    openOutputModal() {
        if (!this.activeSheetId) return;
        const topicSel = document.getElementById('routing-output-topic');
        const fieldSel = document.getElementById('routing-output-field');
        const desc = document.getElementById('routing-output-topic-desc');
        const labelInput = document.getElementById('routing-output-label');
        if (!topicSel || !fieldSel) return;

        // Only show topics that have a publishable command type. The
        // server's list_topic_channels currently returns every endpoint
        // with a state type; treat the same set as candidates and let
        // the server reject if the endpoint isn't publishable.
        topicSel.innerHTML = this.topicCatalog.map(t =>
            `<option value="${escapeAttr(t.topic)}">${escapeHtml(t.topic)}</option>`
        ).join('') || '<option value="">(no ROS topics discovered)</option>';

        const refreshFields = () => {
            const t = this.topicCatalog.find(x => x.topic === topicSel.value);
            desc.textContent = t ? `${t.state_type} · ${t.channels.length} channel(s)` : '';
            fieldSel.innerHTML = (t?.channels || []).map(c =>
                `<option value="${escapeAttr(c.field)}">${escapeHtml(c.label)} (${escapeHtml(c.type || 'num')})</option>`
            ).join('') || '<option value="">(message has no scalar fields)</option>';
        };
        topicSel.onchange = refreshFields;
        refreshFields();

        labelInput.value = '';
        document.getElementById('routing-output-error').classList.add('hidden');
        document.getElementById('routing-output-modal').classList.remove('hidden');
    }

    closeOutputModal() {
        document.getElementById('routing-output-modal')?.classList.add('hidden');
    }

    async saveOutputModal() {
        const topic = document.getElementById('routing-output-topic').value;
        const field = document.getElementById('routing-output-field').value;
        const label = document.getElementById('routing-output-label').value.trim();
        const errEl = document.getElementById('routing-output-error');
        errEl.classList.add('hidden');
        if (!topic) {
            errEl.textContent = 'Pick a ROS topic';
            errEl.classList.remove('hidden');
            return;
        }
        try {
            const result = await window.saintWS.management('add_routing_output', {
                node_id: this.activeSheetId, topic, field, label,
            });
            if (result && result.success === false) {
                errEl.textContent = result.message || 'Failed to add output';
                errEl.classList.remove('hidden');
                return;
            }
            this.closeOutputModal();
        } catch (err) {
            errEl.textContent = String(err.message || err);
            errEl.classList.remove('hidden');
        }
    }

    // ── Add Operator modal ───────────────────────────────────────────

    openOperatorModal() {
        if (!this.activeSheetId) return;
        const sel = document.getElementById('routing-operator-type');
        const desc = document.getElementById('routing-operator-type-desc');
        const labelInput = document.getElementById('routing-operator-label');
        sel.innerHTML = (this.catalog?.operator_types || [])
            .map(t => `<option value="${escapeAttr(t.id)}">${escapeHtml(t.label)}</option>`)
            .join('');
        sel.onchange = () => {
            const t = this._operatorType(sel.value);
            desc.textContent = t?.description || '';
        };
        sel.onchange();
        labelInput.value = '';
        document.getElementById('routing-operator-error').classList.add('hidden');
        document.getElementById('routing-operator-modal').classList.remove('hidden');
    }

    closeOperatorModal() {
        document.getElementById('routing-operator-modal')?.classList.add('hidden');
    }

    async saveOperatorModal() {
        const op = document.getElementById('routing-operator-type').value;
        const label = document.getElementById('routing-operator-label').value.trim();
        const errEl = document.getElementById('routing-operator-error');
        errEl.classList.add('hidden');
        if (!op) {
            errEl.textContent = 'Pick an operator';
            errEl.classList.remove('hidden');
            return;
        }
        try {
            const result = await window.saintWS.management('add_routing_operator', {
                node_id: this.activeSheetId, op, label,
            });
            if (result && result.success === false) {
                errEl.textContent = result.message || 'Failed to add operator';
                errEl.classList.remove('hidden');
                return;
            }
            this.closeOperatorModal();
        } catch (err) {
            errEl.textContent = String(err.message || err);
            errEl.classList.remove('hidden');
        }
    }

    // ── Add Widget modal (dashboard only) ────────────────────────────

    openWidgetModal() {
        if (!this.catalog) return;
        // Reuse the legacy widget modal markup if available; otherwise
        // a prompt() fallback so the dashboard sheet stays usable.
        const modal = document.getElementById('routing-widget-modal');
        if (modal) {
            const sel = document.getElementById('routing-widget-type');
            const desc = document.getElementById('routing-widget-type-desc');
            const labelInput = document.getElementById('routing-widget-label');
            sel.innerHTML = (this.catalog.widget_types || [])
                .map(t => `<option value="${escapeAttr(t.id)}">${escapeHtml(t.label)}</option>`).join('');
            sel.onchange = () => {
                const t = this._widgetType(sel.value);
                desc.textContent = t?.description || '';
            };
            sel.onchange();
            labelInput.value = '';
            document.getElementById('routing-widget-error').classList.add('hidden');
            modal.classList.remove('hidden');
        }
    }

    closeWidgetModal() {
        document.getElementById('routing-widget-modal')?.classList.add('hidden');
    }

    async saveWidgetModal() {
        if (!this.activeSheetId) return;
        const sel = document.getElementById('routing-widget-type');
        const labelEl = document.getElementById('routing-widget-label');
        const type = sel.value;
        const label = labelEl.value.trim() || this._widgetType(type)?.label || type;
        const errEl = document.getElementById('routing-widget-error');
        errEl.classList.add('hidden');
        try {
            const result = await window.saintWS.management('add_widget', {
                node_id: this.activeSheetId, type, label, position: [640, 20],
            });
            if (result && result.success === false) {
                errEl.textContent = result.message || 'Failed to add widget';
                errEl.classList.remove('hidden');
                return;
            }
            this.closeWidgetModal();
        } catch (err) {
            errEl.textContent = String(err.message || err);
            errEl.classList.remove('hidden');
        }
    }

    // ── Catalog helpers ─────────────────────────────────────────────

    _peripheralType(id) {
        return (this.catalog?.peripheral_types || []).find(t => t.id === id);
    }
    _widgetType(id) {
        return (this.catalog?.widget_types || []).find(t => t.id === id);
    }
    _operatorType(id) {
        return (this.catalog?.operator_types || []).find(t => t.id === id);
    }

    // ── localStorage helpers ────────────────────────────────────────

    _loadPositions() {
        try { return JSON.parse(localStorage.getItem('saint.routing.positions') || '{}'); }
        catch (e) { return {}; }
    }
    _savePositions() {
        try { localStorage.setItem('saint.routing.positions', JSON.stringify(this.positions)); }
        catch (e) { /* ignore */ }
    }
    _loadActiveSheet() {
        try { return localStorage.getItem('saint.routing.active_sheet') || null; }
        catch (e) { return null; }
    }
    _saveActiveSheet() {
        try { localStorage.setItem('saint.routing.active_sheet', this.activeSheetId || ''); }
        catch (e) { /* ignore */ }
    }
}

// ── escape helpers (peripherals.js declares these too; keep guarded) ──
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
    return (window.CSS && window.CSS.escape) ? CSS.escape(s) : String(s).replace(/(["'\\])/g, '\\$1');
}

const routingPage = new RoutingPage();
window.routingPage = routingPage;

document.addEventListener('DOMContentLoaded', () => {
    routingPage.init();
});
