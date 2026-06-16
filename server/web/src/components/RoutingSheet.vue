<script setup>
// Per-sheet routing canvas. Owned by Routes.vue, which keys this with
// `:key="sheetId"` so all internal state (positions, drag, connecting,
// graph nodes) is naturally scoped to the active sheet.
//
// Ported from main's js/routing.js. Each sheet is hard-scoped to one
// controller node — its peripherals live on the canvas, widgets here
// surface on that controller's dashboard, and wires/inputs/operators/
// outputs all live in `routing.sheets[sheetId]`.
import { computed, onMounted, onBeforeUnmount, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import AppModal from '@/components/AppModal.vue'

const props = defineProps({
  sheetId:       { type: String, required: true },
  sheet:         { type: Object, required: true }, // {node_id, inputs, ws_inputs, outputs, operators, widgets, wires}
  node:          { type: Object, default: null },  // adopted node (null if orphan)
  peripherals:   { type: Array, default: () => [] }, // node peripherals (when not orphan)
  topicCatalog:  { type: Array, default: () => [] },
  values:        { type: Object, default: () => ({}) }, // per-sheet values bucket
  isDashboard:   { type: Boolean, default: false },
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()

// localStorage positions are keyed `<sheetId>:<graphNodeId>` — matches legacy.
const POSITIONS_KEY = 'saint.routing.positions'
function loadPositions () {
  try { return JSON.parse(localStorage.getItem(POSITIONS_KEY) || '{}') } catch (_) { return {} }
}
function savePositions () {
  try { localStorage.setItem(POSITIONS_KEY, JSON.stringify(positions.value)) } catch (_) {}
}
const positions = ref(loadPositions())

// ── Interaction state — sheet-scoped (component is re-keyed per sheet) ──
const canvasEl = ref(null)
const connecting = ref(null)        // { fromHandle, fromDir, cursor }
const dragging = ref(null)          // { nodeId, offsetX, offsetY }

const NODE_WIDTH    = 220
const HEADER_HEIGHT = 48
const PIN_ROW       = 22
const PIN_OFFSET_X  = 6

// ── Catalog helpers ─────────────────────────────────────────────────────
// All three type lists live in the shared peripheralCatalog store — one
// fetch per page load, populated by Routes.vue's loadAll().
const peripheralType = (id) => catalog.byType(id)
const widgetType = (id) => catalog.widgetType(id)
const operatorType = (id) => catalog.operatorType(id)

// ── Graph node assembly ─────────────────────────────────────────────────
function positionFor (id, fallback) {
  const key = `${props.sheetId}:${id}`
  const saved = positions.value[key]
  if (saved && Number.isFinite(saved.x) && Number.isFinite(saved.y)) return { x: saved.x, y: saved.y }
  if (Array.isArray(fallback) && fallback.length >= 2 && (fallback[0] || fallback[1])) {
    return { x: fallback[0], y: fallback[1] }
  }
  return { x: 0, y: 0 }
}

const graphNodes = computed(() => {
  const out = []
  const seen = new Set()
  const push = (entry) => {
    // Dedup by graph node id — a buggy migration or wire pointing at the
    // same peripheral channel via two wires must not produce two nodes.
    if (seen.has(entry.id)) return
    seen.add(entry.id)
    out.push(entry)
  }
  const s = props.sheet || {}
  for (const n of (s.inputs || [])) {
    const id = `in::${n.id}`
    const pos = positionFor(id, n.position)
    push({ id, kind: 'input', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y })
  }
  for (const n of (s.ws_inputs || [])) {
    const id = `ws::${n.id}`
    const pos = positionFor(id, n.position)
    push({ id, kind: 'ws_input', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y })
  }
  for (const n of (s.operators || [])) {
    const id = `op::${n.id}`
    const pos = positionFor(id, n.position)
    push({ id, kind: 'operator', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y })
  }
  for (const n of (s.outputs || [])) {
    const id = `out::${n.id}`
    const pos = positionFor(id, n.position)
    push({ id, kind: 'output', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y })
  }
  for (const w of (s.widgets || [])) {
    const id = `w::${w.id}`
    const pos = positionFor(id, w.position)
    push({ id, kind: 'widget', sheetNodeId: w.id, widget: w, x: pos.x, y: pos.y })
  }
  for (const n of (s.signals || [])) {
    const id = `sig::${n.id}`
    const pos = positionFor(id, n.position)
    push({ id, kind: 'signal', sheetNodeId: n.id, data: n, x: pos.x, y: pos.y })
  }
  // Peripheral sinks — only on controller sheets, never on dashboard.
  if (props.node && !props.isDashboard) {
    for (const p of (props.peripherals || [])) {
      const id = `p::${props.node.node_id}::${p.id}`
      const pos = positionFor(id, null)
      push({
        id, kind: 'peripheral',
        nodeId: props.node.node_id,
        nodeLabel: props.node.display_name || props.node.node_id,
        peripheral: p, x: pos.x, y: pos.y,
      })
    }
  }
  // Auto-layout pass for any zero/zero entries.
  const cols     = { input: 30, ws_input: 30, operator: 320, output: 640, widget: 640, peripheral: 900, signal: 480 }
  const cursorY  = { input: 30, ws_input: 30, operator: 30, output: 30, widget: 30, peripheral: 30, signal: 30 }
  const SPACING  = 140
  for (const g of out) {
    if (g.x === 0 && g.y === 0) {
      g.x = cols[g.kind] ?? 30
      g.y = cursorY[g.kind] ?? 30
      cursorY[g.kind] = (cursorY[g.kind] ?? 30) + SPACING
    }
  }
  return out
})

// ── Per-node metadata: pins, title, removable ─────────────────────────
function meta (g) {
  if (g.kind === 'input') {
    const inp = g.data
    if (inp.kind === 'urdf_joint') {
      return {
        title: inp.label || inp.joint || 'URDF joint',
        subtitle: `URDF · joint ${inp.joint || '?'}`,
        builtin: false, removable: true,
        inputs: [],
        outputs: [{ id: 'out', label: 'value' }],
      }
    }
    const topicMeta = props.topicCatalog.find(t => t.topic === inp.topic)
    return {
      title: inp.label || `${inp.topic}${inp.field ? '.' + inp.field : ''}`,
      subtitle: `ROS · ${inp.topic}${inp.field ? ' · ' + inp.field : ''}`
              + (topicMeta ? ` (${topicMeta.state_type})` : ''),
      builtin: false, removable: true,
      inputs: [],
      outputs: [{ id: 'out', label: 'value' }],
    }
  }
  if (g.kind === 'ws_input') {
    const inp = g.data
    return {
      title: inp.label || inp.id,
      subtitle: `WebSocket input · ${inp.id}`,
      builtin: false, removable: true,
      inputs: [],
      outputs: [{ id: 'out', label: 'value' }],
    }
  }
  if (g.kind === 'output') {
    const o = g.data
    const topicMeta = props.topicCatalog.find(t => t.topic === o.topic)
    return {
      title: o.label || `${o.topic}${o.field ? '.' + o.field : ''}`,
      subtitle: `→ ${o.topic}${o.field ? ' · ' + o.field : ''}`
              + (topicMeta ? ` (${topicMeta.state_type})` : ''),
      builtin: false, removable: true,
      inputs: [{ id: 'value', label: 'value' }],
      outputs: [{ id: 'out', label: 'tap' }],
    }
  }
  if (g.kind === 'operator') {
    const op = g.data
    const type = operatorType(op.op)
    const ins = (type?.inputs || []).map(spec => ({
      id: spec.id, label: spec.label, editable: true, default: spec.default,
    }))
    return {
      title: op.label || (type?.label || op.op),
      subtitle: type ? type.description : `operator: ${op.op}`,
      builtin: false, removable: true,
      inputs: ins,
      outputs: [{ id: 'out', label: 'out' }],
      // Static config knobs (bool / int / float) declared by the
      // operator's catalog entry. Rendered below the pin list on the
      // node card.
      params: (type?.params || []).map(spec => ({
        id: spec.id, label: spec.label, type: spec.type, default: spec.default,
      })),
    }
  }
  if (g.kind === 'peripheral') {
    const p = g.peripheral
    const type = peripheralType(p.type)
    let channels = type?.channels || []
    // Maestro: one catalog entry, 24 channels declared, instance
    // picks how many to expose via params.channel_count.
    if (p.type === 'maestro') {
      const n = Number(p.params?.channel_count) || 6
      channels = channels.slice(0, n)
    }
    return {
      title: p.label || p.id,
      subtitle: `${type?.label || p.type} on ${g.nodeLabel}`,
      builtin: !!p.builtin, removable: false,
      // Legacy convention: channel.dir === 'out' means peripheral consumes
      // the value (a sink — left side); 'in' means peripheral produces it.
      inputs:  channels.filter(c => c.dir === 'out').map(c => ({ id: c.id, label: c.display })),
      outputs: channels.filter(c => c.dir === 'in').map(c => ({ id: c.id, label: c.display })),
    }
  }
  if (g.kind === 'widget') {
    const w = g.widget
    const type = widgetType(w.type)
    return {
      title: w.label || w.id,
      subtitle: `${type?.label || w.type} widget`,
      builtin: false, removable: true,
      inputs: (type?.inputs || []).map(i => ({ id: i.id, label: i.display })),
      outputs: [],
    }
  }
  if (g.kind === 'signal') {
    const n = g.data
    // Signals are bidirectional: one card carries an input pin (write
    // into the named global) AND an output pin (read from it). Other
    // sheets that hold a SignalNode with the same name reference the
    // same value — that's the cross-sheet point.
    return {
      title: n.label || n.name,
      subtitle: `Signal · ${n.name}`,
      builtin: false, removable: true,
      inputs:  [{ id: 'in',  label: 'in'  }],
      outputs: [{ id: 'out', label: 'out' }],
    }
  }
  return null
}

// ── Wires + pin geometry ────────────────────────────────────────────
function pinPosition (handle) {
  if (!handle) return null
  const slash = handle.indexOf('/')
  const nodeId = handle.slice(0, slash)
  const pinId  = handle.slice(slash + 1)
  const g = graphNodes.value.find(n => n.id === nodeId)
  if (!g) return null
  const m = meta(g)
  if (!m) return null
  const inIdx  = m.inputs.findIndex(p => p.id === pinId)
  if (inIdx >= 0) {
    return {
      x: g.x - PIN_OFFSET_X + 6,
      y: g.y + HEADER_HEIGHT + inIdx * PIN_ROW + PIN_ROW / 2,
    }
  }
  const outIdx = m.outputs.findIndex(p => p.id === pinId)
  if (outIdx >= 0) {
    const row = m.inputs.length + outIdx
    return {
      x: g.x + NODE_WIDTH + PIN_OFFSET_X - 6,
      y: g.y + HEADER_HEIGHT + row * PIN_ROW + PIN_ROW / 2,
    }
  }
  return null
}

function handleToEndpoint (handle) {
  const slash = handle.indexOf('/')
  const graphNodeId = handle.slice(0, slash)
  const pinId  = handle.slice(slash + 1)
  if (graphNodeId.startsWith('in::')) return { kind: 'input', parts: [graphNodeId.slice(4)] }
  if (graphNodeId.startsWith('ws::')) return { kind: 'ws_input', parts: [graphNodeId.slice(4)] }
  if (graphNodeId.startsWith('out::')) return { kind: 'output', parts: [graphNodeId.slice(5)] }
  if (graphNodeId.startsWith('op::')) return { kind: 'operator', parts: [graphNodeId.slice(4), pinId] }
  if (graphNodeId.startsWith('p::')) {
    const rest = graphNodeId.slice(3)
    const idx = rest.lastIndexOf('::')
    return { kind: 'peripheral', parts: [rest.slice(0, idx), rest.slice(idx + 2), pinId] }
  }
  if (graphNodeId.startsWith('w::')) return { kind: 'widget', parts: [graphNodeId.slice(3), pinId] }
  if (graphNodeId.startsWith('sig::')) {
    // Wire stores the GLOBAL signal name in parts, not the per-sheet
    // node id — that's how the same signal name on two sheets binds
    // both wires to the same global value. Look up the SignalNode
    // here to get its name.
    const sigId = graphNodeId.slice(5)
    const node = (props.sheet?.signals || []).find(n => n.id === sigId)
    return node ? { kind: 'signal', parts: [node.name] } : null
  }
  return null
}
function endpointToHandle (ep, dirHint) {
  if (!ep) return null
  if (ep.kind === 'input')      return `in::${ep.parts[0]}/out`
  if (ep.kind === 'ws_input')   return `ws::${ep.parts[0]}/out`
  if (ep.kind === 'output')     return dirHint === 'out' ? `out::${ep.parts[0]}/out` : `out::${ep.parts[0]}/value`
  if (ep.kind === 'operator')   return `op::${ep.parts[0]}/${ep.parts[1] || 'out'}`
  if (ep.kind === 'peripheral') return `p::${ep.parts[0]}::${ep.parts[1]}/${ep.parts[2]}`
  if (ep.kind === 'widget')     return `w::${ep.parts[0]}/${ep.parts[1]}`
  if (ep.kind === 'signal') {
    // The wire references a signal by name; this sheet may or may not
    // hold a local SignalNode for it. Pick the first matching one to
    // draw against — if none, the wire is "dangling" on this sheet
    // (still resolves on the server, just doesn't render here).
    const node = (props.sheet?.signals || []).find(n => n.name === ep.parts[0])
    if (!node) return null
    return `sig::${node.id}/${dirHint === 'out' ? 'out' : 'in'}`
  }
  return null
}

function bezier (a, b) {
  const dx = Math.max(Math.abs(b.x - a.x) * 0.5, 30)
  return `M ${a.x},${a.y} C ${a.x + dx},${a.y} ${b.x - dx},${b.y} ${b.x},${b.y}`
}

const wires = computed(() => {
  const out = []
  for (const w of (props.sheet?.wires || [])) {
    const a = pinPosition(endpointToHandle(w.source, 'out'))
    const b = pinPosition(endpointToHandle(w.sink, 'in'))
    if (a && b) out.push({ id: w.id, d: bezier(a, b), pending: false })
  }
  if (connecting.value?.cursor) {
    const a = pinPosition(connecting.value.fromHandle)
    if (a) out.push({ id: '_pending', d: bezier(a, connecting.value.cursor), pending: true })
  }
  return out
})

// ── Drag / connect mouse handling ─────────────────────────────────────
function startDrag (evt, g) {
  if (evt.target.closest('.routing-pin')) return
  if (evt.target.closest('button')) return
  if (evt.target.closest('input')) return
  dragging.value = { nodeId: g.id, offsetX: evt.clientX - g.x, offsetY: evt.clientY - g.y }
  evt.preventDefault()
}
function onMouseMove (evt) {
  if (dragging.value) {
    const id = dragging.value.nodeId
    positions.value = {
      ...positions.value,
      [`${props.sheetId}:${id}`]: {
        x: Math.max(0, evt.clientX - dragging.value.offsetX),
        y: Math.max(0, evt.clientY - dragging.value.offsetY),
      },
    }
  }
  if (connecting.value && canvasEl.value) {
    const cr = canvasEl.value.getBoundingClientRect()
    connecting.value = {
      ...connecting.value,
      cursor: {
        x: evt.clientX - cr.left + canvasEl.value.scrollLeft,
        y: evt.clientY - cr.top + canvasEl.value.scrollTop,
      },
    }
  }
}
function onMouseUp () {
  if (dragging.value) {
    savePositions()
    // Mirror to server for sheet-owned nodes only (peripherals belong to
    // the controller, not the sheet — they're rendered but not stored).
    const id = dragging.value.nodeId
    const g = graphNodes.value.find(n => n.id === id)
    if (g && g.sheetNodeId && !props.isDashboard) {
      ws.management('update_sheet_node', {
        node_id: props.sheetId,
        sheet_node_id: g.sheetNodeId,
        position: [g.x, g.y],
      }).catch(() => {})
    }
    dragging.value = null
  }
}
onMounted(() => {
  window.addEventListener('mousemove', onMouseMove)
  window.addEventListener('mouseup',   onMouseUp)
})
onBeforeUnmount(() => {
  window.removeEventListener('mousemove', onMouseMove)
  window.removeEventListener('mouseup',   onMouseUp)
})

function isConnectingHandle (gid, pinId) {
  return connecting.value?.fromHandle === `${gid}/${pinId}`
}
function cancelConnecting () { connecting.value = null }

function pinMouseDown (evt, g, pinId, dir) {
  evt.stopPropagation()
  const handle = `${g.id}/${pinId}`
  if (connecting.value && connecting.value.fromHandle !== handle) {
    completeConnection(handle, dir); return
  }
  connecting.value = { fromHandle: handle, fromDir: dir }
}
function pinMouseUp (evt, g, pinId, dir) {
  evt.stopPropagation()
  const handle = `${g.id}/${pinId}`
  if (connecting.value && connecting.value.fromHandle !== handle) {
    completeConnection(handle, dir)
  }
}

async function completeConnection (toHandle, toDir) {
  if (!connecting.value) return
  let from, to
  if (connecting.value.fromDir === 'out' && toDir === 'in') {
    from = connecting.value.fromHandle; to = toHandle
  } else if (connecting.value.fromDir === 'in' && toDir === 'out') {
    from = toHandle; to = connecting.value.fromHandle
  } else {
    cancelConnecting(); return
  }
  cancelConnecting()
  const source = handleToEndpoint(from)
  const sink   = handleToEndpoint(to)
  if (!source || !sink) return
  try {
    const r = await ws.management('add_routing_wire', {
      node_id: props.sheetId, source, sink,
    })
    if (r && r.success === false) alert('Cannot add wire: ' + (r.message || 'unknown error'))
  } catch (e) {
    console.warn('add_routing_wire failed:', e)
  }
}

async function clickWire (id) {
  if (!confirm('Remove this wire?')) return
  try { await ws.management('remove_routing_wire', { node_id: props.sheetId, wire_id: id }) }
  catch (e) { console.warn('remove_routing_wire failed:', e) }
}

// ── Sheet-node removal (input / ws_input / operator / output / widget) ─
async function removeSheetNode (graphNodeId) {
  let sheetNodeId = null
  let isSignal = false
  if      (graphNodeId.startsWith('in::'))  sheetNodeId = graphNodeId.slice(4)
  else if (graphNodeId.startsWith('ws::'))  sheetNodeId = graphNodeId.slice(4)
  else if (graphNodeId.startsWith('out::')) sheetNodeId = graphNodeId.slice(5)
  else if (graphNodeId.startsWith('op::'))  sheetNodeId = graphNodeId.slice(4)
  else if (graphNodeId.startsWith('w::'))   sheetNodeId = graphNodeId.slice(3)
  else if (graphNodeId.startsWith('sig::')) { sheetNodeId = graphNodeId.slice(5); isSignal = true }
  if (!sheetNodeId) return
  if (!confirm('Remove this node? Wires touching it will be deleted.')) return
  try {
    if (isSignal) {
      // remove_sheet_node doesn't know about signals; route through
      // the dedicated action.
      await ws.management('remove_routing_signal', {
        node_id: props.sheetId, signal_id: sheetNodeId,
      })
    } else {
      await ws.management('remove_sheet_node', {
        node_id: props.sheetId, sheet_node_id: sheetNodeId,
      })
    }
  } catch (e) { console.warn('remove_sheet_node failed:', e) }
}

// ── Operator default-constant editor ─────────────────────────────────
function operatorWired (opId, pinId) {
  for (const w of (props.sheet?.wires || [])) {
    if (w.sink.kind === 'operator'
        && w.sink.parts?.[0] === opId
        && w.sink.parts?.[1] === pinId) return true
  }
  return false
}
function operatorDefault (op, pin) {
  if (op.defaults && op.defaults[pin.id] !== undefined) return op.defaults[pin.id]
  return pin.default ?? 0
}
async function setOperatorDefault (opId, pinId, valStr) {
  const value = Number(valStr)
  if (!Number.isFinite(value)) return
  const op = (props.sheet?.operators || []).find(o => o.id === opId)
  if (!op) return
  const defaults = { ...(op.defaults || {}), [pinId]: value }
  try {
    await ws.management('update_sheet_node', {
      node_id: props.sheetId, sheet_node_id: opId, defaults,
    })
  } catch (e) { console.warn('update_sheet_node failed:', e) }
}

function operatorParam (op, param) {
  if (op.params && op.params[param.id] !== undefined) return op.params[param.id]
  return param.default
}
async function setOperatorParam (opId, param, value) {
  const op = (props.sheet?.operators || []).find(o => o.id === opId)
  if (!op) return
  // Coerce by declared type so we don't accidentally save a bool as
  // the string "false" or an int as a float.
  let coerced = value
  if (param.type === 'bool') coerced = !!value
  else if (param.type === 'int') coerced = Math.trunc(Number(value))
  else if (param.type === 'float') coerced = Number(value)
  const params = { ...(op.params || {}), [param.id]: coerced }
  try {
    await ws.management('update_sheet_node', {
      node_id: props.sheetId, sheet_node_id: opId, params,
    })
  } catch (e) { console.warn('update_sheet_node failed:', e) }
}

// ── Inline node edit (rename labels, retarget topics, etc.) ──────────
//
// One modal handles every editable kind. The fields shown vary by
// kind (a SignalNode exposes `name`; an Input exposes topic/field
// or joint depending on its kind; an Output exposes topic/field;
// everything else just exposes label). Saves go through
// update_sheet_node — the server-side handler is permissive about
// extra keys, so we just dispatch whatever fields the form mutated.
//
// Peripheral nodes intentionally have no edit affordance: the
// peripheral's own state lives on the node's Peripherals tab, not
// on the routing sheet.
const editModal = ref({ open: false, sheetNodeId: '', kind: '', fields: {}, error: '' })

function nodeIsEditable (g) {
  return g && g.kind && g.kind !== 'peripheral'
}

function openEditNode (g) {
  if (!nodeIsEditable(g)) return
  const d = g.data || g.widget || {}
  const fields = { label: d.label || '' }
  if (g.kind === 'input') {
    // Distinguish topic vs urdf_joint InputNodes — they expose
    // different fields downstream.
    fields.kind  = d.kind || 'topic'
    fields.topic = d.topic || ''
    fields.field = d.field || ''
    fields.joint = d.joint || ''
  } else if (g.kind === 'output') {
    fields.topic = d.topic || ''
    fields.field = d.field || ''
  } else if (g.kind === 'signal') {
    // Renaming a signal rebinds it to a different (or new) global
    // value — wires keyed by the old name stop resolving until
    // rewired. Surfaced to the operator via the help line in the
    // modal so this isn't a surprise.
    fields.name = d.name || ''
  }
  editModal.value = {
    open: true, sheetNodeId: g.sheetNodeId, kind: g.kind, fields, error: '',
  }
}

async function saveEditNode () {
  const m = editModal.value
  m.error = ''
  // Only ship keys that were actually populated AND that the server
  // accepts for this kind. Empty strings are valid (clearing a
  // field), so we don't filter falsy; we just match against the
  // server's accept list per kind.
  const keyMap = {
    input:    ['label', 'kind', 'topic', 'field', 'joint'],
    ws_input: ['label'],
    output:   ['label', 'topic', 'field'],
    operator: ['label'],
    widget:   ['label'],
    signal:   ['label', 'name'],
  }
  const allowed = keyMap[m.kind] || ['label']
  const payload = { node_id: props.sheetId, sheet_node_id: m.sheetNodeId }
  for (const k of allowed) {
    if (m.fields[k] !== undefined) payload[k] = m.fields[k]
  }
  if (m.kind === 'signal' && !(payload.name || '').trim()) {
    m.error = 'Signal name cannot be empty'
    return
  }
  try {
    const r = await ws.management('update_sheet_node', payload)
    if (r && r.success === false) {
      m.error = r.message || 'Save failed'
      return
    }
    editModal.value.open = false
  } catch (e) {
    m.error = e.message || String(e)
  }
}

// ── Auto-layout (this sheet only) ────────────────────────────────────
function autoLayout () {
  const prefix = `${props.sheetId}:`
  const next = { ...positions.value }
  for (const k of Object.keys(next)) if (k.startsWith(prefix)) delete next[k]
  positions.value = next
  savePositions()
}
defineExpose({ autoLayout })

// ── Live value surfacing — peripheral sinks, operator outputs, etc. ──
function fmtVal (v) {
  if (typeof v !== 'number' || !Number.isFinite(v)) return ''
  const a = Math.abs(v)
  if (a < 0.005) return '0'
  if (a < 100)   return v.toFixed(2)
  return v.toFixed(0)
}
function pinValueFor (g, pin, dir) {
  const buckets = props.values || {}
  if (g.kind === 'input'    && dir === 'out') return fmtVal(buckets.inputs?.[g.sheetNodeId])
  if (g.kind === 'ws_input' && dir === 'out') return fmtVal(buckets.ws_inputs?.[g.sheetNodeId])
  if (g.kind === 'operator' && dir === 'out') return fmtVal(buckets.operators?.[g.sheetNodeId])
  if (g.kind === 'output') {
    // same value flows through sink-in and tap-out
    return fmtVal(buckets.outputs?.[g.sheetNodeId])
  }
  if (g.kind === 'widget' && dir === 'in') {
    return fmtVal(buckets.widgets?.[`${g.widget.id}/${pin.id}`])
  }
  if (g.kind === 'peripheral' && dir === 'in') {
    return fmtVal(buckets.peripherals?.[`${g.nodeId}/${g.peripheral.id}/${pin.id}`])
  }
  if (g.kind === 'signal') {
    // Same value reads on both pins — the global signal is what got
    // written and what gets read this tick.
    return fmtVal(buckets.signals?.[g.sheetNodeId])
  }
  return ''
}

// Reset transient state when sheet identity changes. Even though Routes
// keys this component by sheetId, defensive watcher keeps prop-only
// switches clean.
watch(() => props.sheetId, () => { connecting.value = null; dragging.value = null })
</script>

<template>
  <div
    ref="canvasEl"
    class="routing-canvas"
    @click.self="cancelConnecting"
  >
    <svg class="routing-wires" width="100%" height="100%" preserveAspectRatio="none">
      <path
        v-for="w in wires"
        :key="w.id"
        :d="w.d"
        fill="none"
        :stroke="w.pending ? '#34d399' : '#06b6d4'"
        stroke-width="2"
        :stroke-dasharray="w.pending ? '4,3' : ''"
        :class="['routing-wire', w.pending ? 'pending' : '']"
        @click.stop="!w.pending && clickWire(w.id)"
      />
    </svg>

    <template v-for="g in graphNodes" :key="g.id">
      <div
        :class="['routing-node', dragging?.nodeId === g.id ? 'dragging' : '']"
        :data-kind="g.kind"
        :data-builtin="meta(g)?.builtin ? 'true' : 'false'"
        :style="{ left: g.x + 'px', top: g.y + 'px', width: NODE_WIDTH + 'px' }"
      >
        <div class="routing-node-header" @mousedown="startDrag($event, g)">
          <div class="min-w-0">
            <div class="truncate">{{ meta(g)?.title }}</div>
            <div class="routing-node-kind truncate">{{ meta(g)?.subtitle }}</div>
          </div>
          <div class="flex items-center gap-1 ml-1">
            <!-- Edit (rename / retarget / etc.) is offered for every
                 sheet-owned node kind. Peripherals have no edit
                 affordance — their state belongs on the node's
                 Peripherals tab, not the routing sheet. -->
            <button
              v-if="nodeIsEditable(g)"
              class="text-fg-muted hover:text-cyan-300 text-sm leading-none"
              title="Edit this node"
              @click.stop="openEditNode(g)"
            >✎</button>
            <button
              v-if="meta(g)?.removable"
              class="text-fg-strong hover:text-rose-400 text-base leading-none"
              title="Remove from sheet"
              @click.stop="removeSheetNode(g.id)"
            >✕</button>
          </div>
        </div>
        <div class="routing-node-body">
          <div
            v-for="p in (meta(g)?.inputs || [])"
            :key="'i-' + p.id"
            class="routing-pin-row input"
          >
            <div
              :class="['routing-pin dir-in', isConnectingHandle(g.id, p.id) ? 'connecting' : '']"
              :title="`${p.label} (in)`"
              @mousedown.stop="pinMouseDown($event, g, p.id, 'in')"
              @mouseup.stop="pinMouseUp($event, g, p.id, 'in')"
            />
            <span class="routing-pin-label-in">{{ p.label }}</span>
            <span class="routing-pin-value">{{ pinValueFor(g, p, 'in') }}</span>
            <input
              v-if="g.kind === 'operator' && p.editable && !operatorWired(g.sheetNodeId, p.id)"
              type="number"
              step="0.01"
              class="routing-pin-default"
              :value="operatorDefault(g.data, p)"
              @click.stop
              @mousedown.stop
              @change="setOperatorDefault(g.sheetNodeId, p.id, $event.target.value)"
            />
          </div>
          <div
            v-for="p in (meta(g)?.outputs || [])"
            :key="'o-' + p.id"
            class="routing-pin-row output"
          >
            <span class="routing-pin-value">{{ pinValueFor(g, p, 'out') }}</span>
            <span class="routing-pin-label-out">{{ p.label }}</span>
            <div
              :class="['routing-pin dir-out', isConnectingHandle(g.id, p.id) ? 'connecting' : '']"
              :title="`${p.label} (out)`"
              @mousedown.stop="pinMouseDown($event, g, p.id, 'out')"
              @mouseup.stop="pinMouseUp($event, g, p.id, 'out')"
            />
          </div>
          <!-- Operator static-config knobs. Sits below the pin list
               so they read as "settings on the node" rather than pins.
               Booleans → checkbox; int/float → number input. -->
          <div
            v-if="g.kind === 'operator' && (meta(g)?.params || []).length"
            class="routing-node-params"
          >
            <label
              v-for="param in meta(g).params"
              :key="'pr-' + param.id"
              class="routing-param-row"
              @mousedown.stop
            >
              <template v-if="param.type === 'bool'">
                <input
                  type="checkbox"
                  :checked="!!operatorParam(g.data, param)"
                  @click.stop
                  @change="setOperatorParam(g.sheetNodeId, param, $event.target.checked)"
                />
                <span class="routing-param-label">{{ param.label }}</span>
              </template>
              <template v-else>
                <span class="routing-param-label">{{ param.label }}</span>
                <input
                  type="number"
                  :step="param.type === 'int' ? 1 : 0.01"
                  class="routing-param-input"
                  :value="operatorParam(g.data, param)"
                  @click.stop
                  @mousedown.stop
                  @change="setOperatorParam(g.sheetNodeId, param, $event.target.value)"
                />
              </template>
            </label>
          </div>
        </div>
      </div>
    </template>

    <!-- Per-node edit modal. Field set varies by kind; everything
         routes through update_sheet_node on the server. -->
    <AppModal
      v-if="editModal.open"
      :title="`Edit ${editModal.kind} · ${editModal.sheetNodeId}`"
      @close="editModal.open = false"
    >
      <div v-if="editModal.error" class="mb-3 p-2 rounded bg-red-500/20 border border-red-500/40 text-sm text-red-300">
        {{ editModal.error }}
      </div>

      <div class="space-y-3">
        <!-- Label is shared across every editable kind. -->
        <div>
          <label class="block text-sm font-medium text-fg mb-1">Label</label>
          <input v-model="editModal.fields.label" type="text" class="input-field w-full" placeholder="Leave blank to fall back to ID / topic" />
        </div>

        <!-- Signal: editable name binds to a different global value. -->
        <template v-if="editModal.kind === 'signal'">
          <div>
            <label class="block text-sm font-medium text-fg mb-1">Signal name</label>
            <input v-model="editModal.fields.name" type="text" class="input-field w-full font-mono" placeholder="shoulder_safe" />
            <p class="text-xs text-fg-faint mt-1">
              Renaming rebinds this node to a different (or new) global signal.
              Wires keyed by the old name stop resolving until rewired; other
              sheets holding a SignalNode by the new name will then share the
              same value with this one.
            </p>
          </div>
        </template>

        <!-- Output: ROS topic + field. -->
        <template v-else-if="editModal.kind === 'output'">
          <div>
            <label class="block text-sm font-medium text-fg mb-1">ROS topic</label>
            <input v-model="editModal.fields.topic" type="text" class="input-field w-full font-mono" placeholder="/saint/foo" />
          </div>
          <div>
            <label class="block text-sm font-medium text-fg mb-1">Field</label>
            <input v-model="editModal.fields.field" type="text" class="input-field w-full font-mono" placeholder="value" />
          </div>
        </template>

        <!-- Input: branches on kind (topic vs urdf_joint). -->
        <template v-else-if="editModal.kind === 'input'">
          <div>
            <label class="block text-sm font-medium text-fg mb-1">Input kind</label>
            <select v-model="editModal.fields.kind" class="input-field w-full">
              <option value="topic">ROS topic</option>
              <option value="urdf_joint">URDF joint (animation)</option>
            </select>
          </div>
          <template v-if="editModal.fields.kind === 'urdf_joint'">
            <div>
              <label class="block text-sm font-medium text-fg mb-1">URDF joint</label>
              <input v-model="editModal.fields.joint" type="text" class="input-field w-full font-mono" placeholder="shoulder_pan_joint" />
            </div>
          </template>
          <template v-else>
            <div>
              <label class="block text-sm font-medium text-fg mb-1">ROS topic</label>
              <input v-model="editModal.fields.topic" type="text" class="input-field w-full font-mono" placeholder="/joy" />
            </div>
            <div>
              <label class="block text-sm font-medium text-fg mb-1">Field</label>
              <input v-model="editModal.fields.field" type="text" class="input-field w-full font-mono" placeholder="axes[0]" />
            </div>
          </template>
        </template>
      </div>

      <template #actions>
        <button class="btn-secondary" @click="editModal.open = false">Cancel</button>
        <button class="btn-primary" @click="saveEditNode">Save</button>
      </template>
    </AppModal>
  </div>
</template>

<style scoped>
.routing-canvas {
  position: relative;
  height: 640px;
  background:
    radial-gradient(circle at 1px 1px, var(--color-surface) 1px, transparent 1px) 0 0 / 24px 24px,
    var(--color-canvas);
  border: 1px solid var(--color-surface);
  border-radius: 0.5rem;
  overflow: auto;
  user-select: none;
}
.routing-canvas svg.routing-wires {
  position: absolute; inset: 0; pointer-events: none;
  z-index: 1; width: 100%; height: 100%; overflow: visible;
}
.routing-canvas svg.routing-wires path { pointer-events: stroke; cursor: pointer; }
.routing-canvas svg.routing-wires path:hover { stroke: #f87171 !important; }
.routing-canvas svg.routing-wires path.pending { pointer-events: none; }

.routing-node {
  position: absolute; min-width: 180px;
  background: var(--color-panel); border: 1px solid var(--color-surface-2); border-radius: 0.5rem;
  z-index: 2; box-shadow: 0 4px 12px rgba(0,0,0,0.4); color: var(--color-fg);
}
.routing-node.dragging { z-index: 10; opacity: 0.9; }
.routing-node-header {
  height: 48px; box-sizing: border-box;
  padding: 0.5rem 0.75rem;
  border-bottom: 1px solid var(--color-surface);
  font-size: 0.75rem; font-weight: 600;
  cursor: move; display: flex; align-items: center;
  justify-content: space-between; gap: 0.25rem;
}
.routing-node-kind { font-size: 0.65rem; color: var(--color-fg-muted); font-weight: 400; }
.routing-node-body { padding: 0; }
.routing-pin-row {
  display: flex; align-items: center; padding: 0 0.5rem;
  font-size: 0.7rem; gap: 0.35rem; position: relative;
  height: 22px;
}
.routing-pin-row.input  { justify-content: flex-start; }
.routing-pin-row.output { justify-content: flex-end; }
.routing-pin {
  width: 12px; height: 12px; border-radius: 50%;
  background: var(--color-surface-2); border: 2px solid var(--color-panel);
  cursor: pointer; position: absolute;
  top: 50%; transform: translateY(-50%);
}
.routing-pin-row.input  .routing-pin { left: -6px; }
.routing-pin-row.output .routing-pin { right: -6px; }
.routing-pin.dir-in  { background: #06b6d4; }
.routing-pin.dir-out { background: #f59e0b; }
.routing-pin:hover    { transform: translateY(-50%) scale(1.3); }
.routing-pin.connecting { background: #34d399; box-shadow: 0 0 8px #34d399; }
.routing-pin-label-in  { margin-left: 0.75rem; color: var(--color-fg); }
.routing-pin-label-out { margin-right: 0.75rem; color: var(--color-fg); }
.routing-pin-value {
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 0.65rem; color: #fbbf24; margin: 0 0.25rem;
  min-width: 2ch; text-align: right;
}
.routing-pin-default {
  width: 56px;
  margin-left: 0.5rem;
  padding: 0 0.25rem;
  font-size: 0.65rem;
  background: var(--color-canvas);
  border: 1px solid var(--color-surface);
  border-radius: 3px;
  color: var(--color-fg);
}
.routing-pin-default:focus { outline: 1px solid #06b6d4; }

/* Node header tints — defined in src/style.css with theme-aware
 * color-mix tints so they read on both light and dark backgrounds.
 * (Previously duplicated here with hardcoded dark hexes; removed
 * so the global rules win the cascade.) */
</style>