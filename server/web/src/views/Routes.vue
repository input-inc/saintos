<script setup>
import { computed, onMounted, onBeforeUnmount, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useNodesStore } from '@/stores/nodes'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useWsTopic } from '@/composables/useWsTopic'
import AppModal from '@/components/AppModal.vue'

const ws = useWsStore()
const nodes = useNodesStore()
const catalog = usePeripheralCatalog()
const live = useWsTopic(() => 'system_routing')

const routing = ref({ version: 0, routes: [], widgets: [] })
const nodePeripherals = ref({})       // node_id -> peripherals[]
const widgetCatalog = ref([])

const positionsKey = 'saint_routing_positions'
const pendingSignalsKey = 'saint_routing_pending_signals'

const positions = ref(loadJson(positionsKey, {}))
const pendingSignals = ref(loadJson(pendingSignalsKey, []))

const canvas = ref(null)
const connecting = ref(null)          // { fromHandle, fromDir }
const dragging = ref(null)            // { nodeId, offsetX, offsetY }
const cursor = ref({ x: 0, y: 0 })

// Deterministic pin geometry — must match the CSS rules below.
const NODE_WIDTH    = 220
const HEADER_HEIGHT = 48
const PIN_ROW       = 22
const PIN_OFFSET_X  = 6

function loadJson (key, fallback) {
  try { return JSON.parse(localStorage.getItem(key)) || fallback }
  catch (_) { return fallback }
}
function saveJson (key, val) {
  try { localStorage.setItem(key, JSON.stringify(val)) } catch (_) {}
}

async function loadAll () {
  try {
    const r = await ws.management('get_system_routing', {})
    routing.value = r || { version: 0, routes: [], widgets: [] }
  } catch (e) { console.warn('routing load failed:', e) }
  await catalog.ensureLoaded()
  widgetCatalog.value = catalog.widgetTypes
  await nodes.fetchAll().catch(() => {})
  for (const n of nodes.all) {
    if (nodePeripherals.value[n.node_id]) continue
    try {
      const r = await ws.management('get_node_peripherals', { node_id: n.node_id })
      nodePeripherals.value[n.node_id] = r?.peripherals || []
    } catch (_) {}
  }
}
onMounted(() => { catalog.ensureLoaded(); loadAll() })

const currentRouting = computed(() => live.value || routing.value)
const routes  = computed(() => currentRouting.value.routes  || [])
const widgets = computed(() => currentRouting.value.widgets || [])

// ── Graph node assembly ──────────────────────────────────────────────────

function toPin (c) { return { id: c.id, label: c.display || c.id, cap: c.cap } }
function widgetType (id) { return widgetCatalog.value.find(t => t.id === id) }

const graphNodes = computed(() => {
  const out = []
  let pCount = 0
  for (const n of nodes.all) {
    for (const p of (nodePeripherals.value[n.node_id] || [])) {
      const id = `p::${n.node_id}::${p.id}`
      const type = catalog.byType(p.type)
      const channels = type?.channels || []
      // Legacy convention: channel.dir === 'in' means the peripheral
      // *produces* this value (e.g. a sensor reading), so it becomes a
      // graph OUTPUT (right side, amber). dir === 'out' means the
      // peripheral *consumes* this value (e.g. an actuator), so it
      // becomes a graph INPUT (left side, cyan).
      const inputs  = channels.filter(c => c.dir === 'out').map(toPin)
      const outputs = channels.filter(c => c.dir === 'in').map(toPin)
      const pos = positions.value[id] || autoPos('peripheral', pCount++)
      out.push({
        id, kind: 'peripheral',
        nodeId: n.node_id, nodeLabel: n.display_name || n.node_id,
        peripheral: p, typeLabel: type?.label || p.type,
        builtin: !!p.builtin,
        inputs, outputs, x: pos.x, y: pos.y,
      })
    }
  }
  let wCount = 0
  for (const w of widgets.value) {
    const id = `w::${w.id}`
    const type = widgetType(w.type)
    const inputs = (type?.inputs || []).map(i => ({ id: i.id, label: i.display || i.id, cap: i.cap }))
    const pos = positions.value[id] || autoPos('widget', wCount++)
    out.push({ id, kind: 'widget', widget: w, typeLabel: type?.label || w.type,
               inputs, outputs: [], x: pos.x, y: pos.y })
  }
  const sigPaths = new Set(pendingSignals.value)
  for (const r of routes.value) {
    if (r.source?.kind === 'signal') sigPaths.add(r.source.parts[0])
    if (r.sink?.kind   === 'signal') sigPaths.add(r.sink.parts[0])
  }
  let sCount = 0
  for (const path of sigPaths) {
    const id = `s::${path}`
    const pos = positions.value[id] || autoPos('signal', sCount++)
    out.push({
      id, kind: 'signal', signal: path,
      inputs:  [{ id: 'in',  label: 'in',  cap: 'any' }],
      outputs: [{ id: 'out', label: 'out', cap: 'any' }],
      x: pos.x, y: pos.y,
    })
  }
  return out
})

function autoPos (kind, n) {
  const cols = { peripheral: 30, signal: 380, widget: 730 }
  return { x: cols[kind], y: 30 + n * 140 }
}

// ── Pin geometry — matches CSS row heights ───────────────────────────────

function pinPosition (handle) {
  if (!handle) return null
  const slash = handle.indexOf('/')
  const nodeId = handle.slice(0, slash)
  const pinId  = handle.slice(slash + 1)
  const g = graphNodes.value.find(n => n.id === nodeId)
  if (!g) return null
  const inIdx  = g.inputs.findIndex(p => p.id === pinId)
  if (inIdx >= 0) {
    return {
      x: g.x - PIN_OFFSET_X + 6,                          // pin is 12px wide, centered on -6px left
      y: g.y + HEADER_HEIGHT + inIdx * PIN_ROW + PIN_ROW / 2,
    }
  }
  const outIdx = g.outputs.findIndex(p => p.id === pinId)
  if (outIdx >= 0) {
    const row = g.inputs.length + outIdx
    return {
      x: g.x + NODE_WIDTH + PIN_OFFSET_X - 6,
      y: g.y + HEADER_HEIGHT + row * PIN_ROW + PIN_ROW / 2,
    }
  }
  return null
}

// ── Endpoint <-> handle translation ──────────────────────────────────────

function handleToEndpoint (handle) {
  const slash = handle.indexOf('/')
  const nodeId = handle.slice(0, slash)
  const pinId  = handle.slice(slash + 1)
  if (nodeId.startsWith('p::')) {
    const rest = nodeId.slice(3)
    const idx = rest.lastIndexOf('::')
    return { kind: 'peripheral', parts: [rest.slice(0, idx), rest.slice(idx + 2), pinId] }
  }
  if (nodeId.startsWith('s::')) return { kind: 'signal', parts: [nodeId.slice(3)] }
  if (nodeId.startsWith('w::')) return { kind: 'widget', parts: [nodeId.slice(3), pinId] }
  return null
}

// `dir` disambiguates the signal-node side: routes point to its 'in' pin on
// the sink end and its 'out' pin on the source end.
function endpointToHandle (ep, dir) {
  if (!ep) return null
  if (ep.kind === 'peripheral') return `p::${ep.parts[0]}::${ep.parts[1]}/${ep.parts[2]}`
  if (ep.kind === 'signal')     return `s::${ep.parts[0]}/${dir}`
  if (ep.kind === 'widget')     return `w::${ep.parts[0]}/${ep.parts[1]}`
  return null
}

// ── Wires ────────────────────────────────────────────────────────────────

function bezier (a, b) {
  const dx = Math.max(Math.abs(b.x - a.x) * 0.5, 30)
  return `M ${a.x},${a.y} C ${a.x + dx},${a.y} ${b.x - dx},${b.y} ${b.x},${b.y}`
}

const wires = computed(() => {
  const out = []
  for (const r of routes.value) {
    const a = pinPosition(endpointToHandle(r.source, 'out'))
    const b = pinPosition(endpointToHandle(r.sink,   'in'))
    if (a && b) out.push({ id: r.id, d: bezier(a, b), pending: false })
  }
  if (connecting.value) {
    const a = pinPosition(connecting.value.fromHandle)
    if (a) out.push({ id: '_pending', d: bezier(a, cursor.value), pending: true })
  }
  return out
})

// ── Drag handling (header only) ──────────────────────────────────────────

function startDrag (evt, g) {
  if (evt.target.closest('.routing-pin')) return
  if (evt.target.closest('button')) return
  dragging.value = {
    nodeId: g.id,
    offsetX: evt.clientX - g.x,
    offsetY: evt.clientY - g.y,
  }
  evt.preventDefault()
}
function onMouseMove (evt) {
  const rect = canvas.value?.getBoundingClientRect()
  if (rect) cursor.value = { x: evt.clientX - rect.left, y: evt.clientY - rect.top }
  if (dragging.value) {
    const id = dragging.value.nodeId
    positions.value = {
      ...positions.value,
      [id]: {
        x: Math.max(0, evt.clientX - dragging.value.offsetX),
        y: Math.max(0, evt.clientY - dragging.value.offsetY),
      },
    }
  }
}
function onMouseUp () {
  if (dragging.value) {
    saveJson(positionsKey, positions.value)
    dragging.value = null
  }
}
onMounted(() => {
  window.addEventListener('mousemove', onMouseMove)
  window.addEventListener('mouseup', onMouseUp)
})
onBeforeUnmount(() => {
  window.removeEventListener('mousemove', onMouseMove)
  window.removeEventListener('mouseup', onMouseUp)
})

// ── Pin click / drag / wire creation ─────────────────────────────────────
//
// Two interaction styles are supported:
//   1. Click pin A, click pin B (legacy behavior — connecting state persists
//      between separate clicks).
//   2. Press pin A, drag, release on pin B (node-editor UX).
// Both end in `completeConnection(toHandle, toDir)` which submits add_route.

function isConnectingHandle (nodeId, pinId) {
  return connecting.value?.fromHandle === `${nodeId}/${pinId}`
}

function cancelConnecting () { connecting.value = null }

function pinMouseDown (evt, g, pinId, dir) {
  evt.stopPropagation()
  const handle = `${g.id}/${pinId}`
  // Click-click finalize: a second mousedown on a *different* pin completes.
  if (connecting.value && connecting.value.fromHandle !== handle) {
    completeConnection(handle, dir)
    return
  }
  // Start (or restart) a connection from this pin.
  connecting.value = { fromHandle: handle, fromDir: dir }
}

function pinMouseUp (evt, g, pinId, dir) {
  evt.stopPropagation()
  const handle = `${g.id}/${pinId}`
  // Drag-release finalize: only fires when the mouseup pin is different
  // from where the connecting state started (the same-pin mouseup at the
  // end of a click should not auto-cancel the in-progress connection).
  if (connecting.value && connecting.value.fromHandle !== handle) {
    completeConnection(handle, dir)
  }
}

async function completeConnection (toHandle, toDir) {
  if (!connecting.value) return
  let source, sink
  if (connecting.value.fromDir === 'out' && toDir === 'in') {
    source = connecting.value.fromHandle; sink = toHandle
  } else if (connecting.value.fromDir === 'in' && toDir === 'out') {
    source = toHandle; sink = connecting.value.fromHandle
  } else {
    cancelConnecting(); return            // can't connect in→in or out→out
  }
  cancelConnecting()
  try {
    await ws.management('add_route', {
      source: handleToEndpoint(source),
      sink:   handleToEndpoint(sink),
    })
  } catch (e) { console.warn('add_route failed:', e) }
}

async function clickWire (id) {
  if (!confirm('Remove this route?')) return
  try { await ws.management('remove_route', { route_id: id }) }
  catch (e) { console.warn('remove_route failed:', e) }
}

// ── Add / remove signals + widgets ───────────────────────────────────────

function promptAddSignal () {
  const name = prompt('Logical signal path (e.g. /battery/main/current):')
  if (!name) return
  const path = name.trim()
  if (!path) return
  if (!pendingSignals.value.includes(path)) {
    pendingSignals.value = [...pendingSignals.value, path]
    saveJson(pendingSignalsKey, pendingSignals.value)
  }
}

async function removeGraphNode (graphId) {
  if (graphId.startsWith('s::')) {
    const path = graphId.slice(3)
    if (!confirm(`Remove signal "${path}" and all routes touching it?`)) return
    pendingSignals.value = pendingSignals.value.filter(s => s !== path)
    saveJson(pendingSignalsKey, pendingSignals.value)
    const touching = routes.value.filter(r =>
      (r.source?.kind === 'signal' && r.source.parts[0] === path) ||
      (r.sink?.kind   === 'signal' && r.sink.parts[0]   === path))
    for (const r of touching) {
      try { await ws.management('remove_route', { route_id: r.id }) }
      catch (e) { console.warn('remove_route failed:', e) }
    }
    return
  }
  if (graphId.startsWith('w::')) {
    const widgetId = graphId.slice(3)
    if (!confirm('Remove this widget? Any routes wired to it will be deleted too.')) return
    try { await ws.management('remove_widget', { widget_id: widgetId }) }
    catch (e) { console.warn('remove_widget failed:', e) }
  }
  // Peripheral graph nodes can't be removed here — operator must edit the
  // node's Peripherals tab to do that.
}

const widgetModal = ref({ open: false, type: '', label: '', error: '' })
function openAddWidget () {
  const first = widgetCatalog.value?.[0]?.id || ''
  widgetModal.value = { open: true, type: first, label: '', error: '' }
}
const widgetTypeDesc = computed(() =>
  widgetCatalog.value?.find(t => t.id === widgetModal.value.type)?.description || '',
)
async function saveAddWidget () {
  const type = widgetModal.value.type
  if (!type) { widgetModal.value.error = 'Pick a widget type.'; return }
  const fallbackLabel = widgetType(type)?.label || type
  const label = (widgetModal.value.label || '').trim() || fallbackLabel
  widgetModal.value.error = ''
  try {
    await ws.management('add_widget', { type, label, position: [720, 20] })
    widgetModal.value.open = false
  } catch (e) {
    widgetModal.value.error = e.message || String(e)
  }
}

function autoLayout () {
  positions.value = {}
  saveJson(positionsKey, positions.value)
}

// Title / subtitle for the node header.
function titleFor (g) {
  if (g.kind === 'peripheral') return g.peripheral.label || g.peripheral.id
  if (g.kind === 'signal')     return g.signal
  return g.widget.label || g.widget.id
}
function subtitleFor (g) {
  if (g.kind === 'peripheral') return `${g.typeLabel} on ${g.nodeLabel}`
  if (g.kind === 'signal')     return 'logical signal'
  return `${g.typeLabel} widget`
}
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-4">
      <div>
        <h2 class="text-2xl font-bold text-white">Routing</h2>
        <p class="text-sm text-slate-400">Connect peripheral channels to widgets and logical signals.</p>
      </div>
      <div class="flex items-center gap-2">
        <button class="btn-secondary text-sm" @click="promptAddSignal">
          <span class="material-icons icon-sm">add</span> Signal
        </button>
        <button class="btn-secondary text-sm" @click="openAddWidget">
          <span class="material-icons icon-sm">widgets</span> Widget
        </button>
        <button class="btn-secondary text-sm" title="Auto-arrange nodes" @click="autoLayout">
          <span class="material-icons icon-sm">grid_view</span> Auto-layout
        </button>
      </div>
    </div>

    <div class="card">
      <p class="text-xs text-slate-400 mb-3">
        Drag node headers to reposition. Drag from an
        <span class="inline-block align-middle" style="width:10px;height:10px;background:#f59e0b;border-radius:50%;"></span>
        output to an
        <span class="inline-block align-middle" style="width:10px;height:10px;background:#06b6d4;border-radius:50%;"></span>
        input to wire them (or click each in turn). Click a wire to delete it.
      </p>

      <div
        ref="canvas"
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

        <div
          v-for="g in graphNodes"
          :key="g.id"
          :class="['routing-node', dragging?.nodeId === g.id ? 'dragging' : '']"
          :data-kind="g.kind"
          :data-builtin="g.builtin ? 'true' : 'false'"
          :style="{ left: g.x + 'px', top: g.y + 'px', width: NODE_WIDTH + 'px' }"
        >
          <div class="routing-node-header" @mousedown="startDrag($event, g)">
            <div class="min-w-0">
              <div class="truncate">{{ titleFor(g) }}</div>
              <div class="routing-node-kind truncate">{{ subtitleFor(g) }}</div>
            </div>
            <button
              v-if="g.kind !== 'peripheral'"
              class="text-slate-200 hover:text-rose-400 text-base leading-none ml-1"
              title="Remove from graph"
              @click.stop="removeGraphNode(g.id)"
            >✕</button>
          </div>
          <div class="routing-node-body">
            <div
              v-for="p in g.inputs"
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
            </div>
            <div
              v-for="p in g.outputs"
              :key="'o-' + p.id"
              class="routing-pin-row output"
            >
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
      </div>
    </div>

    <div class="card mt-4">
      <div class="flex items-center justify-between mb-3">
        <h3 class="text-sm font-semibold text-white">All routes ({{ routes.length }})</h3>
      </div>
      <ul v-if="routes.length" class="divide-y divide-slate-700/50 text-xs font-mono">
        <li v-for="r in routes" :key="r.id" class="flex items-center gap-2 py-1.5">
          <span class="text-slate-500 w-10">{{ r.id }}</span>
          <span class="text-amber-300">{{ (r.source?.parts || []).join('/') }}</span>
          <span class="material-icons icon-sm text-slate-500">arrow_forward</span>
          <span class="text-cyan-300">{{ (r.sink?.parts || []).join('/') }}</span>
          <span class="flex-1" />
          <button class="text-slate-500 hover:text-rose-400" @click="clickWire(r.id)">
            <span class="material-icons icon-sm">delete</span>
          </button>
        </li>
      </ul>
      <p v-else class="text-sm text-slate-400 italic">No routes yet.</p>
    </div>

    <AppModal v-if="widgetModal.open" title="Add widget" @close="widgetModal.open = false">
      <div class="space-y-3">
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Widget type</label>
          <select v-model="widgetModal.type" class="input-field w-full">
            <option v-for="t in widgetCatalog" :key="t.id" :value="t.id">{{ t.label }}</option>
          </select>
          <p v-if="widgetTypeDesc" class="text-xs text-slate-500 mt-1">{{ widgetTypeDesc }}</p>
        </div>
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Label</label>
          <input v-model="widgetModal.label" type="text" class="input-field w-full" placeholder="Auto-generated if blank" />
        </div>
        <p v-if="widgetModal.error" class="text-sm text-red-300">{{ widgetModal.error }}</p>
      </div>
      <template #actions>
        <button class="btn-secondary" @click="widgetModal.open = false">Cancel</button>
        <button class="btn-primary" @click="saveAddWidget">Add</button>
      </template>
    </AppModal>
  </section>
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
  overflow: hidden;
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

.routing-node[data-kind="peripheral"] .routing-node-header { background: #0f4c5c; }
.routing-node[data-kind="signal"]     .routing-node-header { background: #4a3b5c; }
.routing-node[data-kind="widget"]     .routing-node-header { background: #5c4a0f; }
.routing-node[data-builtin="true"]    .routing-node-header { background: #1e3a4a; }
</style>
