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
// Operator types come from the shared peripheralCatalog store now —
// single fetch per page load, shared with RoutingSheet.vue.
const operatorTypes = computed(() => catalog.operatorTypes || [])
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

// Every adopted node gets a sheet entry, plus any sheet on disk that
// isn't backed by an adopted node ("orphan"). Role is a body-part tag
// (`tracks`, `neck`, `head`, …, `host` for the host controller), not a
// controller/leaf distinction — legacy routing.js similarly listed every
// adopted node regardless of role. The dashboard sheet always shows up.
const sheetEntries = computed(() => {
  const entries = []
  const seen = new Set()
  for (const n of nodes.all) {
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

// ── Initial load ─────────────────────────────────────────────────────
async function loadAll () {
  await catalog.ensureLoaded()
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
// `labelEdited` mirrors vanilla: as long as the operator hasn't typed
// into the label field, picking a different topic/channel rewrites the
// label to "topic.channel". Once they edit by hand we stop clobbering.
const inputModal = ref({
  open: false, kind: 'ws_input', topic: '', field: '', joint: '',
  label: '', labelEdited: false, error: '',
  urdfJoints: [],
})
function inputDerivedLabel () {
  if (inputModal.value.kind === 'urdf_joint') {
    return inputModal.value.joint || ''
  }
  const t = inputModal.value.topic || ''
  const f = inputModal.value.field || ''
  return t ? (f ? `${t}.${f}` : t) : ''
}
function openAddInput () {
  if (!activeSheetId.value) return
  const first = topicCatalog.value[0]
  const topic = first?.topic || ''
  const field = first?.channels?.[0]?.field || ''
  inputModal.value = {
    open: true, kind: 'ws_input',
    topic, field, joint: '',
    label: topic ? (field ? `${topic}.${field}` : topic) : '',
    labelEdited: false,
    error: '',
    urdfJoints: [],
  }
  // Fire-and-forget: load the joint list so the URDF Model option has
  // something to pick from when the operator switches to it.
  fetch('/api/robot/joints')
    .then(r => r.ok ? r.json() : { joints: [] })
    .then(d => { inputModal.value.urdfJoints = d.joints || [] })
    .catch(() => { inputModal.value.urdfJoints = [] })
}
const inputTopicChannels = computed(() =>
  topicCatalog.value.find(t => t.topic === inputModal.value.topic)?.channels || [],
)
const inputIsWs = computed(() => inputModal.value.kind === 'ws_input')
const inputIsUrdf = computed(() => inputModal.value.kind === 'urdf_joint')
const inputTopicFieldLabel = computed(() =>
  inputIsWs.value ? 'Pick a topic to copy its name (optional)' : 'ROS topic',
)
const inputLabelPlaceholder = computed(() => {
  if (inputIsUrdf.value) return 'Defaults to joint name — or type a custom name'
  return inputIsWs.value
    ? 'Defaults to topic.channel — or type a custom name'
    : 'Defaults to topic.channel'
})
// Auto-derive label on topic/field/joint changes — but only until the
// user types something into the label box.
watch(
  () => [inputModal.value.topic, inputModal.value.field, inputModal.value.joint, inputModal.value.kind],
  () => {
    if (inputModal.value.open && !inputModal.value.labelEdited) {
      inputModal.value.label = inputDerivedLabel()
    }
  },
)
function onInputLabelInput (evt) {
  inputModal.value.label = evt.target.value
  inputModal.value.labelEdited = !!evt.target.value.trim()
}
async function saveAddInput () {
  const { kind, topic, field, joint, label } = inputModal.value
  let derivedLabel = label.trim()
  // Belt-and-suspenders: if the operator cleared the label after
  // picking a topic/channel/joint, fall back to the derived name.
  if (!derivedLabel) {
    if (kind === 'urdf_joint') derivedLabel = joint || ''
    else if (topic) derivedLabel = field ? `${topic}.${field}` : topic
  }
  inputModal.value.error = ''
  try {
    let r
    if (kind === 'ws_input') {
      r = await ws.management('add_routing_ws_input', { node_id: activeSheetId.value, label: derivedLabel })
    } else if (kind === 'urdf_joint') {
      if (!joint) { inputModal.value.error = 'Pick a URDF joint'; return }
      r = await ws.management('add_routing_input', {
        node_id: activeSheetId.value,
        kind: 'urdf_joint', joint, label: derivedLabel,
      })
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

const outputModal = ref({
  open: false, topic: '', field: '', label: '', error: '',
})
function openAddOutput () {
  if (!activeSheetId.value) return
  const first = topicCatalog.value[0]
  const topic = first?.topic || ''
  const field = first?.channels?.[0]?.field || ''
  outputModal.value = { open: true, topic, field, label: '', error: '' }
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
const operatorDesc = computed(() => {
  const t = operatorTypes.value.find(x => x.id === operatorModal.value.op)
  return t?.description || ''
})
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
// Matches vanilla: log every value snapshot unless explicitly opted
// out. Silence from the console with:
//   window.__SAINT_DEBUG_ROUTING__ = false
watch(routingValues, (v) => {
  if (typeof window !== 'undefined' && window.__SAINT_DEBUG_ROUTING__ !== false) {
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
  <section class="page routing-page">
    <div class="routing-shell">
      <!-- Left rail: one entry per controller node sheet -->
      <aside class="routing-sidebar">
        <div class="routing-sidebar-header">
          <span class="text-xs uppercase tracking-wide text-fg-muted">Sheets</span>
        </div>
        <div class="routing-sheet-list">
          <div
            v-for="e in sheetEntries"
            :key="e.id"
            :class="['routing-sheet-item', activeSheetId === e.id ? 'active' : '', e.orphan ? 'orphan' : '']"
            :title="e.orphan ? 'Orphan sheet — owning node is no longer adopted' : ''"
            @click="selectSheet(e.id)"
          >
            <span class="material-icons">{{ e.orphan ? 'help_outline' : 'memory' }}</span>
            <span class="truncate">{{ e.label }}</span>
            <span v-if="sheets[e.id]?.wires?.length" class="routing-sheet-count">
              {{ sheets[e.id].wires.length }}
            </span>
          </div>
          <div v-if="!sheetEntries.length" class="text-xs text-fg-faint p-2">
            No controller nodes adopted yet.
          </div>
        </div>
      </aside>

      <!-- Main area: per-sheet toolbar + canvas -->
      <div class="routing-main">
        <div class="routing-toolbar">
          <div class="min-w-0">
            <h2 class="text-lg font-semibold text-fg-strong truncate">{{ activeEntry?.label || 'Routing' }}</h2>
            <p class="text-xs text-fg-muted truncate">
              <template v-if="activeEntry">
                {{ sheetCounts.peripherals }} peripheral{{ sheetCounts.peripherals === 1 ? '' : 's' }} ·
                {{ sheetCounts.inputs }} in ·
                {{ sheetCounts.ws_inputs }} ws-in ·
                {{ sheetCounts.outputs }} out ·
                {{ sheetCounts.operators }} op ·
                {{ sheetCounts.widgets }} widget{{ sheetCounts.widgets === 1 ? '' : 's' }} ·
                {{ sheetCounts.wires }} wire{{ sheetCounts.wires === 1 ? '' : 's' }}
              </template>
              <template v-else>
                Pick a sheet on the left to wire its peripherals.
              </template>
            </p>
          </div>
          <div class="flex gap-2 items-center">
            <button class="btn-secondary" :disabled="!activeSheetId" @click="openAddInput">
              <span class="material-icons icon-sm">login</span>
              Input
            </button>
            <button class="btn-secondary" :disabled="!activeSheetId" @click="openAddOutput">
              <span class="material-icons icon-sm">logout</span>
              Output
            </button>
            <button class="btn-secondary" :disabled="!activeSheetId" @click="openAddOperator">
              <span class="material-icons icon-sm">functions</span>
              Operator
            </button>
            <button class="btn-secondary" :disabled="!activeSheetId" @click="openAddWidget">
              <span class="material-icons icon-sm">widgets</span>
              Widget
            </button>
            <button class="btn-secondary" :disabled="!activeSheetId" title="Auto-arrange nodes" @click="autoLayout">
              <span class="material-icons icon-sm">grid_view</span>
            </button>
          </div>
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
        <div v-else class="routing-canvas">
          <div class="routing-empty">
            <p class="text-fg-muted">No sheets yet. Adopt a controller node to start.</p>
          </div>
        </div>
      </div>
    </div>

    <!-- Add Input modal -->
    <AppModal v-if="inputModal.open" title="Add input node" @close="inputModal.open = false">
      <div class="space-y-3">
        <div>
          <label class="block text-sm font-medium text-fg mb-1">Kind</label>
          <select v-model="inputModal.kind" class="input-field w-full">
            <option value="ws_input">WebSocket Input (controller-driven)</option>
            <option value="topic">ROS Topic (subscribe to state)</option>
            <option value="urdf_joint">URDF Model (animation joint)</option>
          </select>
        </div>
        <template v-if="inputIsUrdf">
          <div>
            <label class="block text-sm font-medium text-fg mb-1">Joint</label>
            <select v-model="inputModal.joint" class="input-field w-full">
              <option v-for="j in inputModal.urdfJoints" :key="j.name" :value="j.name">
                {{ j.name }} <span class="text-fg-faint">({{ j.type }})</span>
              </option>
              <option v-if="!inputModal.urdfJoints.length" value="" disabled>
                (no URDF installed — upload one in Settings → Robot Model)
              </option>
            </select>
            <p class="text-xs text-fg-faint mt-1">
              When an animation with a value track named for this joint plays, its sampled value flows through this input.
            </p>
          </div>
        </template>
        <template v-else>
          <div>
            <label class="block text-sm font-medium text-fg mb-1">{{ inputTopicFieldLabel }}</label>
            <select v-model="inputModal.topic" class="input-field w-full">
              <option v-for="t in topicCatalog" :key="t.topic" :value="t.topic">{{ t.topic }}</option>
              <option v-if="!topicCatalog.length" value="">(no ROS topics discovered)</option>
            </select>
          </div>
          <div>
            <label class="block text-sm font-medium text-fg mb-1">Channel</label>
            <select v-model="inputModal.field" class="input-field w-full">
              <option v-for="c in inputTopicChannels" :key="c.field" :value="c.field">
                {{ c.label }} ({{ c.type || 'num' }})
              </option>
            </select>
          </div>
        </template>
        <div>
          <label class="block text-sm font-medium text-fg mb-1">Label (optional)</label>
          <input
            :value="inputModal.label"
            type="text"
            class="input-field w-full"
            :placeholder="inputLabelPlaceholder"
            @input="onInputLabelInput"
          />
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
          <label class="block text-sm font-medium text-fg mb-1">ROS topic</label>
          <select v-model="outputModal.topic" class="input-field w-full">
            <option v-for="t in topicCatalog" :key="t.topic" :value="t.topic">{{ t.topic }}</option>
            <option v-if="!topicCatalog.length" value="">(no ROS topics discovered)</option>
          </select>
        </div>
        <div>
          <label class="block text-sm font-medium text-fg mb-1">Channel</label>
          <select v-model="outputModal.field" class="input-field w-full">
            <option v-for="c in outputTopicChannels" :key="c.field" :value="c.field">
              {{ c.label }} ({{ c.type || 'num' }})
            </option>
          </select>
        </div>
        <div>
          <label class="block text-sm font-medium text-fg mb-1">Label (optional)</label>
          <input
            v-model="outputModal.label"
            type="text"
            class="input-field w-full"
            placeholder="Defaults to topic.channel"
          />
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
          <label class="block text-sm font-medium text-fg mb-1">Operator</label>
          <select v-model="operatorModal.op" class="input-field w-full">
            <option v-for="t in operatorTypes" :key="t.id" :value="t.id">{{ t.label }}</option>
          </select>
          <p v-if="operatorDesc" class="text-xs text-fg-faint mt-1">{{ operatorDesc }}</p>
        </div>
        <div>
          <label class="block text-sm font-medium text-fg mb-1">Label (optional)</label>
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
          <label class="block text-sm font-medium text-fg mb-1">Widget type</label>
          <select v-model="widgetModal.type" class="input-field w-full">
            <option v-for="t in catalog.widgetTypes" :key="t.id" :value="t.id">{{ t.label }}</option>
          </select>
          <p v-if="widgetDesc" class="text-xs text-fg-faint mt-1">{{ widgetDesc }}</p>
        </div>
        <div>
          <label class="block text-sm font-medium text-fg mb-1">Label</label>
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
  border-bottom: 1px solid var(--color-surface);
  margin-bottom: 0.75rem;
  padding-bottom: 0.25rem;
}
.routing-tab {
  display: flex;
  align-items: center;
  gap: 0.4rem;
  padding: 0.45rem 0.75rem;
  font-size: 0.8rem;
  color: var(--color-fg);
  background: transparent;
  border: 1px solid transparent;
  border-bottom: none;
  border-radius: 0.4rem 0.4rem 0 0;
  white-space: nowrap;
  cursor: pointer;
  transition: background 120ms ease;
}
.routing-tab:hover {
  background: var(--color-panel);
}
.routing-tab.active {
  background: var(--color-panel);
  border-color: var(--color-surface);
  border-bottom: 1px solid var(--color-panel);
  color: var(--color-fg-strong);
  margin-bottom: -1px;
  /* 2-px cyan accent strip along the top edge. The "merging tab"
     pattern (active tab's bottom border = its background = the
     content card behind) needs *something* to mark the active state
     when canvas and panel are close in lightness (light theme). The
     accent is the brand cyan that stays the same across themes, so
     active is always unambiguous. */
  box-shadow: inset 0 2px 0 0 var(--color-cyan-500);
}
.routing-tab.orphan {
  color: #fb923c;
}
.routing-tab-count {
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 0.65rem;
  background: var(--color-surface);
  border-radius: 999px;
  padding: 0 0.4rem;
  color: var(--color-fg);
}
</style>
