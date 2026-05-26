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
const connecting = ref(null)        // { fromHandle, fromDir }
const dragging = ref(null)          // { nodeId, offsetX, offsetY }
const cursor = ref({ x: 0, y: 0 })

const NODE_WIDTH    = 220
const HEADER_HEIGHT = 48
const PIN_ROW       = 22
const PIN_OFFSET_X  = 6

// ── Catalog helpers ─────────────────────────────────────────────────────
const peripheralType = (id) => catalog.byType(id)
const widgetType = (id) => catalog.widgetType(id)
// Operator type catalog isn't on the shared catalog store yet — fetch
// once per RoutingSheet mount. Cheap (already cached server-side).
const operatorTypes = ref([])
const operatorType = (id) => operatorTypes.value.find(t => t.id === id) || null
async function ensureOperatorTypes () {
  if (operatorTypes.value.length) return
  try {
    const r = await ws.management('get_peripheral_catalog', {})
    operatorTypes.value = r?.operator_types || []
  } catch (_) {}
}
onMounted(ensureOperatorTypes)

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
  const cols     = { input: 30, ws_input: 30, operator: 320, output: 640, widget: 640, peripheral: 900 }
  const cursorY  = { input: 30, ws_input: 30, operator: 30, output: 30, widget: 30, peripheral: 30 }
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
    }
  }
  if (g.kind === 'peripheral') {
    const p = g.peripheral
    const type = peripheralType(p.type)
    const channels = type?.channels || []
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
  if (connecting.value) {
    const a = pinPosition(connecting.value.fromHandle)
    if (a) out.push({ id: '_pending', d: bezier(a, cursor.value), pending: true })
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
  const rect = canvasEl.value?.getBoundingClientRect()
  if (rect) cursor.value = {
    x: evt.clientX - rect.left + (canvasEl.value?.scrollLeft || 0),
    y: evt.clientY - rect.top  + (canvasEl.value?.scrollTop  || 0),
  }
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
  if      (graphNodeId.startsWith('in::'))  sheetNodeId = graphNodeId.slice(4)
  else if (graphNodeId.startsWith('ws::'))  sheetNodeId = graphNodeId.slice(4)
  else if (graphNodeId.startsWith('out::')) sheetNodeId = graphNodeId.slice(5)
  else if (graphNodeId.startsWith('op::'))  sheetNodeId = graphNodeId.slice(4)
  else if (graphNodeId.startsWith('w::'))   sheetNodeId = graphNodeId.slice(3)
  if (!sheetNodeId) return
  if (!confirm('Remove this node? Wires touching it will be deleted.')) return
  try {
    await ws.management('remove_sheet_node', {
      node_id: props.sheetId, sheet_node_id: sheetNodeId,
    })
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
          <button
            v-if="meta(g)?.removable"
            class="text-slate-200 hover:text-rose-400 text-base leading-none ml-1"
            title="Remove from sheet"
            @click.stop="removeSheetNode(g.id)"
          >✕</button>
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
        </div>
      </div>
    </template>
  </div>
</template>

<style scoped>
.routing-canvas {
  position: relative;
  height: 640px;
  background:
    radial-gradient(circle at 1px 1px, #334155 1px, transparent 1px) 0 0 / 24px 24px,
    #0b1220;
  border: 1px solid #334155;
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
  background: #1e293b; border: 1px solid #475569; border-radius: 0.5rem;
  z-index: 2; box-shadow: 0 4px 12px rgba(0,0,0,0.4); color: #e2e8f0;
}
.routing-node.dragging { z-index: 10; opacity: 0.9; }
.routing-node-header {
  height: 48px; box-sizing: border-box;
  padding: 0.5rem 0.75rem;
  border-bottom: 1px solid #334155;
  font-size: 0.75rem; font-weight: 600;
  cursor: move; display: flex; align-items: center;
  justify-content: space-between; gap: 0.25rem;
}
.routing-node-kind { font-size: 0.65rem; color: #94a3b8; font-weight: 400; }
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
  background: #475569; border: 2px solid #1e293b;
  cursor: pointer; position: absolute;
  top: 50%; transform: translateY(-50%);
}
.routing-pin-row.input  .routing-pin { left: -6px; }
.routing-pin-row.output .routing-pin { right: -6px; }
.routing-pin.dir-in  { background: #06b6d4; }
.routing-pin.dir-out { background: #f59e0b; }
.routing-pin:hover    { transform: translateY(-50%) scale(1.3); }
.routing-pin.connecting { background: #34d399; box-shadow: 0 0 8px #34d399; }
.routing-pin-label-in  { margin-left: 0.75rem; color: #cbd5e1; }
.routing-pin-label-out { margin-right: 0.75rem; color: #cbd5e1; }
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
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 3px;
  color: #e2e8f0;
}
.routing-pin-default:focus { outline: 1px solid #06b6d4; }

.routing-node[data-kind="peripheral"] .routing-node-header { background: #0f4c5c; }
.routing-node[data-kind="input"]      .routing-node-header { background: #1e3a4a; }
.routing-node[data-kind="ws_input"]   .routing-node-header { background: #1e3a5c; }
.routing-node[data-kind="operator"]   .routing-node-header { background: #3a3f5c; }
.routing-node[data-kind="output"]     .routing-node-header { background: #4a3b5c; }
.routing-node[data-kind="widget"]     .routing-node-header { background: #5c4a0f; }
.routing-node[data-builtin="true"]    .routing-node-header { background: #1e3a4a; }
</style>