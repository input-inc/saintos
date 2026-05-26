<script setup>
// Per-sheet routing editor. One sheet per adopted controller node, plus
// an optional dashboard sheet for cross-controller widgets.
//
// Tab bar lives here. The canvas + per-sheet graph state lives in
// RoutingSheet.vue, re-keyed by sheetId so all transient state
// (positions ref, drag/connect, value pills) is naturally isolated.
//
// Live `routing_values` snapshot is subscribed here and passed down to
// the active sheet so it can light up its peripheral sinks / operator
// outputs / widget input pills.
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useNodesStore } from '@/stores/nodes'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useWsTopic } from '@/composables/useWsTopic'
import AppModal from '@/components/AppModal.vue'
import RoutingSheet from '@/components/RoutingSheet.vue'

const DASHBOARD_SHEET_ID = '_dashboard'
const ACTIVE_SHEET_KEY   = 'routing_active_sheet_id'

const ws = useWsStore()
const nodes = useNodesStore()
const catalog = usePeripheralCatalog()

const systemRouting = useWsTopic(() => 'system_routing')
const routingValues = useWsTopic(() => 'routing_values')

const routing = ref({ version: 0, sheets: {} })
const topicCatalog = ref([])
const operatorTypes = ref([])
const nodePeripherals = ref({})  // node_id -> peripherals[]

const activeSheetId = ref(loadActiveSheet())
const sheetRef = ref(null)

function loadActiveSheet () {
  try { return localStorage.getItem(ACTIVE_SHEET_KEY) || null } catch (_) { return null }
}
function saveActiveSheet () {
  try { localStorage.setItem(ACTIVE_SHEET_KEY, activeSheetId.value || '') } catch (_) {}
}

// ── Live routing snapshot ────────────────────────────────────────────
const liveRouting = computed(() => systemRouting.value || routing.value)
const sheets = computed(() => liveRouting.value?.sheets || {})

// All controllers (adopted nodes whose role is "controller") + any
// sheet on disk that isn't backed by an adopted node ("orphan"). The
// dashboard sheet always shows up if it exists on disk OR if anyone
// has widgets they'd like to host outside a controller sheet.
const sheetEntries = computed(() => {
  const entries = []
  const seen = new Set()
  for (const n of nodes.all) {
    if (n.role && n.role !== 'controller') continue
    entries.push({
      id: n.node_id,
      label: n.display_name || n.node_id,
      kind: 'controller',
      node: n,
      orphan: false,
    })
    seen.add(n.node_id)
  }
  for (const sid of Object.keys(sheets.value)) {
    if (seen.has(sid)) continue
    if (sid === DASHBOARD_SHEET_ID) continue
    entries.push({
      id: sid,
      label: `${sid} (no node)`,
      kind: 'controller',
      node: null,
      orphan: true,
    })
    seen.add(sid)
  }
  // Always offer a dashboard tab — it's where global widgets live.
  entries.push({
    id: DASHBOARD_SHEET_ID,
    label: 'Dashboard',
    kind: 'dashboard',
    node: null,
    orphan: false,
  })
  return entries
})

function ensureActiveSheet () {
  const ids = sheetEntries.value.map(e => e.id)
  if (!ids.includes(activeSheetId.value)) {
    activeSheetId.value = ids[0] || null
    saveActiveSheet()
  }
}

const activeEntry = computed(() =>
  sheetEntries.value.find(e => e.id === activeSheetId.value) || null,
)
const activeSheet = computed(() => {
  const id = activeSheetId.value
  if (!id) return null
  return sheets.value[id] || {
    node_id: id, inputs: [], ws_inputs: [], outputs: [],
    operators: [], widgets: [], wires: [],
  }
})
const activeValues = computed(() => {
  const id = activeSheetId.value
  return id ? (routingValues.value?.sheets?.[id] || {}) : {}
})
const activePeripherals = computed(() => {
  const id = activeSheetId.value
  return id ? (nodePeripherals.value[id] || []) : []
})

// ── Routes list (active sheet only) ──────────────────────────────────
const activeWires = computed(() => activeSheet.value?.wires || [])

function endpointLabel (ep) {
  if (!ep) return ''
  if (ep.kind === 'input')      return `in:${ep.parts[0]}`
  if (ep.kind === 'ws_input')   return `ws:${ep.parts[0]}`
  if (ep.kind === 'operator')   return `op:${ep.parts.join('/')}`
  if (ep.kind === 'output')     return `out:${ep.parts[0]}`
  if (ep.kind === 'widget')     return `w:${ep.parts.join('/')}`
  if (ep.kind === 'peripheral') return `p:${ep.parts.join('/')}`
  return (ep.parts || []).join('/')
}

async function removeWire (wireId) {
  if (!confirm('Remove this wire?')) return
  try { await ws.management('remove_routing_wire', { node_id: activeSheetId.value, wire_id: wireId }) }
  catch (e) { console.warn('remove_routing_wire failed:', e) }
}

// ── Initial load ─────────────────────────────────────────────────────
async function loadAll () {
  await catalog.ensureLoaded()
  try {
    const r = await ws.management('get_peripheral_catalog', {})
    operatorTypes.value = r?.operator_types || []
  } catch (_) {}
  try {
    const r = await ws.management('get_system_routing', {})
    routing.value = r || { version: 0, sheets: {} }
  } catch (e) { console.warn('routing load failed:', e) }
  try {
    const r = await ws.send('ros', 'list_topic_channels', {})
    topicCatalog.value = r?.topics || []
  } catch (_) {}
  await nodes.fetchAll().catch(() => {})
  // Pre-fetch peripherals for all adopted nodes — small, and the active
  // sheet may change quickly so we want them ready.
  for (const n of nodes.all) {
    if (nodePeripherals.value[n.node_id]) continue
    try {
      const r = await ws.management('get_node_peripherals', { node_id: n.node_id })
      nodePeripherals.value[n.node_id] = r?.peripherals || []
    } catch (_) {}
  }
  ensureActiveSheet()
}
onMounted(loadAll)

// Re-ensure active sheet if the list mutates.
watch(sheetEntries, () => ensureActiveSheet())

function selectSheet (id) {
  activeSheetId.value = id
  saveActiveSheet()
}

// ── Add-input / output / operator / widget modals ────────────────────
const inputModal = ref({ open: false, kind: 'ws_input', topic: '', field: '', label: '', error: '' })
function openAddInput () {
  if (!activeSheetId.value) return
  const first = topicCatalog.value[0]
  inputModal.value = {
    open: true, kind: 'ws_input',
    topic: first?.topic || '', field: first?.channels?.[0]?.field || '',
    label: '', error: '',
  }
}
const inputTopicChannels = computed(() =>
  topicCatalog.value.find(t => t.topic === inputModal.value.topic)?.channels || [],
)
async function saveAddInput () {
  const { kind, topic, field, label } = inputModal.value
  let derivedLabel = label.trim()
  if (!derivedLabel && topic) derivedLabel = field ? `${topic}.${field}` : topic
  inputModal.value.error = ''
  try {
    let r
    if (kind === 'ws_input') {
      r = await ws.management('add_routing_ws_input', { node_id: activeSheetId.value, label: derivedLabel })
    } else {
      if (!topic) { inputModal.value.error = 'Pick a ROS topic'; return }
      r = await ws.management('add_routing_input', {
        node_id: activeSheetId.value, topic, field, label: derivedLabel,
      })
    }
    if (r && r.success === false) { inputModal.value.error = r.message || 'Failed'; return }
    inputModal.value.open = false
  } catch (e) {
    inputModal.value.error = e.message || String(e)
  }
}

const outputModal = ref({ open: false, topic: '', field: '', label: '', error: '' })
function openAddOutput () {
  if (!activeSheetId.value) return
  const first = topicCatalog.value[0]
  outputModal.value = {
    open: true,
    topic: first?.topic || '', field: first?.channels?.[0]?.field || '',
    label: '', error: '',
  }
}
const outputTopicChannels = computed(() =>
  topicCatalog.value.find(t => t.topic === outputModal.value.topic)?.channels || [],
)
async function saveAddOutput () {
  const { topic, field, label } = outputModal.value
  outputModal.value.error = ''
  if (!topic) { outputModal.value.error = 'Pick a ROS topic'; return }
  try {
    const r = await ws.management('add_routing_output', {
      node_id: activeSheetId.value, topic, field, label: label.trim(),
    })
    if (r && r.success === false) { outputModal.value.error = r.message || 'Failed'; return }
    outputModal.value.open = false
  } catch (e) {
    outputModal.value.error = e.message || String(e)
  }
}

const operatorModal = ref({ open: false, op: '', label: '', error: '' })
function openAddOperator () {
  if (!activeSheetId.value) return
  operatorModal.value = {
    open: true, op: operatorTypes.value[0]?.id || '', label: '', error: '',
  }
}
const operatorDesc = computed(() =>
  operatorTypes.value.find(t => t.id === operatorModal.value.op)?.description || '',
)
async function saveAddOperator () {
  const { op, label } = operatorModal.value
  operatorModal.value.error = ''
  if (!op) { operatorModal.value.error = 'Pick an operator'; return }
  try {
    const r = await ws.management('add_routing_operator', {
      node_id: activeSheetId.value, op, label: label.trim(),
    })
    if (r && r.success === false) { operatorModal.value.error = r.message || 'Failed'; return }
    operatorModal.value.open = false
  } catch (e) {
    operatorModal.value.error = e.message || String(e)
  }
}

const widgetModal = ref({ open: false, type: '', label: '', error: '' })
function openAddWidget () {
  if (!activeSheetId.value) return
  const first = catalog.widgetTypes?.[0]?.id || ''
  widgetModal.value = { open: true, type: first, label: '', error: '' }
}
const widgetDesc = computed(() =>
  catalog.widgetTypes.find(t => t.id === widgetModal.value.type)?.description || '',
)
async function saveAddWidget () {
  const { type, label } = widgetModal.value
  widgetModal.value.error = ''
  if (!type) { widgetModal.value.error = 'Pick a widget type'; return }
  const labelOrFallback = label.trim() || (catalog.widgetType(type)?.label || type)
  try {
    const r = await ws.management('add_widget', {
      node_id: activeSheetId.value, type, label: labelOrFallback, position: [640, 20],
    })
    if (r && r.success === false) { widgetModal.value.error = r.message || 'Failed'; return }
    widgetModal.value.open = false
  } catch (e) {
    widgetModal.value.error = e.message || String(e)
  }
}

function autoLayout () { sheetRef.value?.autoLayout() }

// ── Debug log gate ───────────────────────────────────────────────────
// Surfaces every value snapshot in devtools when the global is on; off
// by default so the console isn't flooded. Toggle from the console:
//   window.__SAINT_DEBUG_ROUTING__ = true
watch(routingValues, (v) => {
  if (typeof window !== 'undefined' && window.__SAINT_DEBUG_ROUTING__) {
    const keys = Object.keys(v?.sheets || {})
    // eslint-disable-next-line no-console
    console.debug('[routing] routing_values', {
      active: activeSheetId.value, sheets: keys,
      sheet_data: v?.sheets?.[activeSheetId.value],
    })
  }
})

// Counters for sheet subtitle.
const sheetCounts = computed(() => {
  const s = activeSheet.value || {}
  return {
    inputs:    (s.inputs || []).length,
    ws_inputs: (s.ws_inputs || []).length,
    outputs:   (s.outputs || []).length,
    operators: (s.operators || []).length,
    widgets:   (s.widgets || []).length,
    wires:     (s.wires || []).length,
    peripherals: activePeripherals.value.length,
  }
})
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-4">
      <div>
        <h2 class="text-2xl font-bold text-white">Routing</h2>
        <p class="text-sm text-slate-400">One sheet per controller node, plus a dashboard sheet for cross-system widgets.</p>
      </div>
      <div class="flex items-center gap-2">
        <button class="btn-secondary text-sm" :disabled="!activeSheetId" @click="openAddInput">
          <span class="material-icons icon-sm">input</span> Input
        </button>
        <button class="btn-secondary text-sm" :disabled="!activeSheetId" @click="openAddOutput">
          <span class="material-icons icon-sm">output</span> Output
        </button>
        <button class="btn-secondary text-sm" :disabled="!activeSheetId" @click="openAddOperator">
          <span class="material-icons icon-sm">functions</span> Operator
        </button>
        <button class="btn-secondary text-sm" :disabled="!activeSheetId" @click="openAddWidget">
          <span class="material-icons icon-sm">widgets</span> Widget
        </button>
        <button class="btn-secondary text-sm" :disabled="!activeSheetId" title="Auto-arrange nodes" @click="autoLayout">
          <span class="material-icons icon-sm">grid_view</span> Auto-layout
        </button>
      </div>
    </div>

    <!-- Tab bar -->
    <div class="routing-tabs">
      <button
        v-for="e in sheetEntries"
        :key="e.id"
        :class="['routing-tab', activeSheetId === e.id ? 'active' : '', e.orphan ? 'orphan' : '']"
        :title="e.orphan ? 'Orphan sheet — owning node is no longer adopted' : ''"
        @click="selectSheet(e.id)"
      >
        <span class="material-icons icon-sm">
          {{ e.kind === 'dashboard' ? 'dashboard' : (e.orphan ? 'help_outline' : 'memory') }}
        </span>
        <span class="truncate">{{ e.label }}</span>
        <span v-if="sheets[e.id]?.wires?.length" class="routing-tab-count">
          {{ sheets[e.id].wires.length }}
        </span>
      </button>
    </div>

    <div class="card">
      <div class="flex items-center justify-between mb-3">
        <div class="min-w-0">
          <h3 class="text-sm font-semibold text-white truncate">{{ activeEntry?.label || 'Routing' }}</h3>
          <p class="text-xs text-slate-500 truncate">
            <template v-if="activeEntry?.kind === 'dashboard'">
              Cross-controller widgets · {{ sheetCounts.widgets }} widget{{ sheetCounts.widgets === 1 ? '' : 's' }} · {{ sheetCounts.wires }} wire{{ sheetCounts.wires === 1 ? '' : 's' }}
            </template>
            <template v-else-if="activeEntry">
              {{ sheetCounts.peripherals }} peripheral{{ sheetCounts.peripherals === 1 ? '' : 's' }} ·
              {{ sheetCounts.inputs }} in · {{ sheetCounts.ws_inputs }} ws-in ·
              {{ sheetCounts.outputs }} out · {{ sheetCounts.operators }} op ·
              {{ sheetCounts.widgets }} widget{{ sheetCounts.widgets === 1 ? '' : 's' }} ·
              {{ sheetCounts.wires }} wire{{ sheetCounts.wires === 1 ? '' : 's' }}
            </template>
            <template v-else>Adopt a controller node to start.</template>
          </p>
        </div>
        <p class="text-xs text-slate-500">
          Drag node headers. Drag pin → pin to wire (or click pin then pin). Click a wire to delete.
        </p>
      </div>

      <RoutingSheet
        v-if="activeSheetId && activeEntry"
        ref="sheetRef"
        :key="activeSheetId"
        :sheet-id="activeSheetId"
        :sheet="activeSheet"
        :node="activeEntry.node"
        :peripherals="activePeripherals"
        :topic-catalog="topicCatalog"
        :values="activeValues"
        :is-dashboard="activeEntry.kind === 'dashboard'"
      />
      <div v-else class="text-slate-400 italic text-sm py-12 text-center">
        No sheets yet. Adopt a controller node to start.
      </div>
    </div>

    <!-- Routes list for the active sheet only -->
    <div class="card mt-4">
      <div class="flex items-center justify-between mb-3">
        <h3 class="text-sm font-semibold text-white">Wires on this sheet ({{ activeWires.length }})</h3>
      </div>
      <ul v-if="activeWires.length" class="divide-y divide-slate-700/50 text-xs font-mono">
        <li v-for="w in activeWires" :key="w.id" class="flex items-center gap-2 py-1.5">
          <span class="text-slate-500 w-10">{{ w.id }}</span>
          <span class="text-amber-300">{{ endpointLabel(w.source) }}</span>
          <span class="material-icons icon-sm text-slate-500">arrow_forward</span>
          <span class="text-cyan-300">{{ endpointLabel(w.sink) }}</span>
          <span class="flex-1" />
          <button class="text-slate-500 hover:text-rose-400" @click="removeWire(w.id)">
            <span class="material-icons icon-sm">delete</span>
          </button>
        </li>
      </ul>
      <p v-else class="text-sm text-slate-400 italic">No wires on this sheet yet.</p>
    </div>

    <!-- Add Input modal -->
    <AppModal v-if="inputModal.open" title="Add input node" @close="inputModal.open = false">
      <div class="space-y-3">
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Kind</label>
          <select v-model="inputModal.kind" class="input-field w-full">
            <option value="ws_input">WebSocket Input (controller-driven)</option>
            <option value="topic">ROS Topic (subscribe to state)</option>
          </select>
        </div>
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">ROS topic</label>
          <select v-model="inputModal.topic" class="input-field w-full">
            <option v-for="t in topicCatalog" :key="t.topic" :value="t.topic">{{ t.topic }}</option>
            <option v-if="!topicCatalog.length" value="">(no ROS topics discovered)</option>
          </select>
        </div>
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Channel</label>
          <select v-model="inputModal.field" class="input-field w-full">
            <option v-for="c in inputTopicChannels" :key="c.field" :value="c.field">
              {{ c.label }} ({{ c.type || 'num' }})
            </option>
          </select>
        </div>
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Label (optional)</label>
          <input v-model="inputModal.label" type="text" class="input-field w-full" placeholder="Defaults to topic.channel" />
        </div>
        <p v-if="inputModal.error" class="text-sm text-red-300">{{ inputModal.error }}</p>
      </div>
      <template #actions>
        <button class="btn-secondary" @click="inputModal.open = false">Cancel</button>
        <button class="btn-primary" @click="saveAddInput">Add</button>
      </template>
    </AppModal>

    <!-- Add Output modal -->
    <AppModal v-if="outputModal.open" title="Add output" @close="outputModal.open = false">
      <div class="space-y-3">
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">ROS topic</label>
          <select v-model="outputModal.topic" class="input-field w-full">
            <option v-for="t in topicCatalog" :key="t.topic" :value="t.topic">{{ t.topic }}</option>
            <option v-if="!topicCatalog.length" value="">(no ROS topics discovered)</option>
          </select>
        </div>
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Channel</label>
          <select v-model="outputModal.field" class="input-field w-full">
            <option v-for="c in outputTopicChannels" :key="c.field" :value="c.field">
              {{ c.label }} ({{ c.type || 'num' }})
            </option>
          </select>
        </div>
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Label (optional)</label>
          <input v-model="outputModal.label" type="text" class="input-field w-full" placeholder="Defaults to topic.channel" />
        </div>
        <p v-if="outputModal.error" class="text-sm text-red-300">{{ outputModal.error }}</p>
      </div>
      <template #actions>
        <button class="btn-secondary" @click="outputModal.open = false">Cancel</button>
        <button class="btn-primary" @click="saveAddOutput">Add</button>
      </template>
    </AppModal>

    <!-- Add Operator modal -->
    <AppModal v-if="operatorModal.open" title="Add operator" @close="operatorModal.open = false">
      <div class="space-y-3">
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Operator</label>
          <select v-model="operatorModal.op" class="input-field w-full">
            <option v-for="t in operatorTypes" :key="t.id" :value="t.id">{{ t.label }}</option>
          </select>
          <p v-if="operatorDesc" class="text-xs text-slate-500 mt-1">{{ operatorDesc }}</p>
        </div>
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Label (optional)</label>
          <input v-model="operatorModal.label" type="text" class="input-field w-full" placeholder="Defaults to operator name" />
        </div>
        <p v-if="operatorModal.error" class="text-sm text-red-300">{{ operatorModal.error }}</p>
      </div>
      <template #actions>
        <button class="btn-secondary" @click="operatorModal.open = false">Cancel</button>
        <button class="btn-primary" @click="saveAddOperator">Add</button>
      </template>
    </AppModal>

    <!-- Add Widget modal -->
    <AppModal v-if="widgetModal.open" title="Add widget" @close="widgetModal.open = false">
      <div class="space-y-3">
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Widget type</label>
          <select v-model="widgetModal.type" class="input-field w-full">
            <option v-for="t in catalog.widgetTypes" :key="t.id" :value="t.id">{{ t.label }}</option>
          </select>
          <p v-if="widgetDesc" class="text-xs text-slate-500 mt-1">{{ widgetDesc }}</p>
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
.routing-tabs {
  display: flex;
  align-items: center;
  gap: 0.25rem;
  overflow-x: auto;
  border-bottom: 1px solid #334155;
  margin-bottom: 0.75rem;
  padding-bottom: 0.25rem;
}
.routing-tab {
  display: flex;
  align-items: center;
  gap: 0.4rem;
  padding: 0.45rem 0.75rem;
  font-size: 0.8rem;
  color: #cbd5e1;
  background: transparent;
  border: 1px solid transparent;
  border-bottom: none;
  border-radius: 0.4rem 0.4rem 0 0;
  white-space: nowrap;
  cursor: pointer;
  transition: background 120ms ease;
}
.routing-tab:hover {
  background: #1e293b;
}
.routing-tab.active {
  background: #1e293b;
  border-color: #334155;
  border-bottom: 1px solid #1e293b;
  color: #ffffff;
  margin-bottom: -1px;
}
.routing-tab.orphan {
  color: #fb923c;
}
.routing-tab-count {
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 0.65rem;
  background: #334155;
  border-radius: 999px;
  padding: 0 0.4rem;
  color: #e2e8f0;
}
</style>
