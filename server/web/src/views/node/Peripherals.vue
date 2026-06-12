<script setup>
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useChannelHistory } from '@/composables/useChannelHistory'
import AppModal from '@/components/AppModal.vue'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const history = useChannelHistory()
const peripherals = ref([])
const syncStatus  = ref('unknown')
const capabilities = ref(null)        // { pins, uart_pairs, reserved_pins }
const logErrors = ref({})              // peripheral_id -> inline error message
const logPending = ref({})             // peripheral_id -> true while in-flight

const modalOpen = ref(false)
const modalMode = ref('add')          // 'add' | 'edit'
const modalEditingId = ref(null)
const modalTypeId = ref('')
const modalLabel = ref('')
const modalPins = ref({})             // {gpio:N} | {uart_tx,uart_rx}
const modalParams = ref({})           // { paramId: value }
const modalError = ref('')

async function loadAll () {
  try {
    const p = await ws.management('get_node_peripherals', { node_id: props.nodeId })
    peripherals.value = p?.peripherals || []
    syncStatus.value  = p?.sync_status || 'unknown'
  } catch (e) { console.warn('get_node_peripherals failed:', e) }
  try {
    const c = await ws.management('get_node_capabilities', { node_id: props.nodeId })
    capabilities.value = c || null
  } catch (e) { console.warn('get_node_capabilities failed:', e) }
}

// Live sync-status feed. The server broadcasts on
// `sync_status/<node_id>` after _maybe_handle_sync_ack flips the
// node's peripheral_config.sync_status (success → "synced", or
// "error" if the firmware logged "Config apply failed" /
// "Flash save failed"). Without this subscription the Sync badge
// would only refresh on a full loadAll() — which never re-runs after
// the initial click, so the pill would stay on "Pending" forever
// even after a successful sync.
let syncTopic = null
function onStateFrame (msg) {
  if (typeof msg?.node !== 'string') return
  if (msg.node !== syncTopic) return
  const data = msg.data
  if (!data || typeof data !== 'object') return
  if (typeof data.sync_status === 'string') {
    syncStatus.value = data.sync_status
  }
}

async function attachSyncFeed (nodeId) {
  if (syncTopic) {
    try { await ws.unsubscribe([syncTopic]) } catch (_) {}
  }
  syncTopic = `sync_status/${nodeId}`
  try { await ws.subscribe([syncTopic], 5) } catch (_) {}
}

onMounted(async () => {
  catalog.ensureLoaded()
  await loadAll()
  ws.on('state', onStateFrame)
  await attachSyncFeed(props.nodeId)
})

watch(() => props.nodeId, async (id) => {
  await loadAll()
  await attachSyncFeed(id)
})

onUnmounted(async () => {
  ws.off('state', onStateFrame)
  if (syncTopic) {
    try { await ws.unsubscribe([syncTopic]) } catch (_) {}
    syncTopic = null
  }
  await detachScanFeed()
})

// When the modal closes, drop the scan subscription and clear the
// per-modal scan state so a re-open starts clean. Driven by `watch`
// rather than the close handler so it covers ESC, backdrop click,
// and the explicit Cancel button.
watch(modalOpen, async (open) => {
  if (!open) {
    scanResults.value = []
    scanError.value = ''
    scanRunning.value = false
    await detachScanFeed()
  }
})

const typesById = computed(() => {
  const out = {}
  for (const t of catalog.types) out[t.id] = t
  return out
})

// "GPIO number → peripheral that owns it" — covers both `pins` (the
// peripheral's pin_kind assignment: gpio, uart_tx/rx) AND any `params`
// whose catalog schema declares type === "gpio" (e.g. RoboClaw's
// estop_pin). Mirrors PeripheralInstance.claimed_gpios() on the server
// so the dashboard's pin/gpio dropdowns flag the same conflicts the
// upsert path will reject. A param value of 0 is the "no pin" sentinel
// and is skipped.
const claimedPins = computed(() => {
  const out = {}
  for (const p of peripherals.value) {
    if (modalMode.value === 'edit' && p.id === modalEditingId.value) continue
    for (const v of Object.values(p.pins || {})) {
      if (typeof v === 'number') out[v] = p
    }
    const t = typesById.value[p.type]
    if (t && t.params) {
      for (const param of t.params) {
        if (param.type !== 'gpio') continue
        const v = p.params ? p.params[param.id] : undefined
        if (typeof v === 'number' && v > 0) out[v] = p
      }
    }
  }
  return out
})

function openAdd () {
  modalMode.value = 'add'
  modalEditingId.value = null
  modalTypeId.value = catalog.types[0]?.id || ''
  modalLabel.value = ''
  modalPins.value = {}
  modalParams.value = {}
  modalError.value = ''
  applyDefaults()
  modalOpen.value = true
}
function openEdit (p) {
  modalMode.value = 'edit'
  modalEditingId.value = p.id
  modalTypeId.value = p.type
  modalLabel.value = p.label
  modalPins.value = { ...p.pins }
  modalParams.value = { ...p.params }
  modalError.value = ''
  modalOpen.value = true
}
function applyDefaults () {
  const type = typesById.value[modalTypeId.value]
  if (!type) return
  modalParams.value = {}
  for (const param of (type.params || [])) modalParams.value[param.id] = param.default
}

watch(modalTypeId, () => {
  if (modalMode.value === 'add') {
    modalPins.value = {}
    applyDefaults()
  }
})

// Coerce a dropdown-selected string value back to the param's base
// type (the <select>'s value is always a string; the on-the-wire
// type can be int/float/bool/string).
function coerceChoiceValue (param, raw) {
  if (param.type === 'int')   return parseInt(raw, 10)
  if (param.type === 'float') return parseFloat(raw)
  if (param.type === 'bool')  return raw === 'true'
  return raw
}

// `visible_when` predicate from the catalog: hide a param unless
// every named dependency in modalParams matches. Used by the BMS
// type to gate the MAC field behind transport=ble.
function paramVisible (param) {
  if (!param.visible_when) return true
  for (const [k, v] of Object.entries(param.visible_when)) {
    if (modalParams.value[k] !== v) return false
  }
  return true
}

// Pin selectors (UART pair or single GPIO) are normally shown for any
// non-builtin pin_kind. But peripherals that swap to a wireless
// transport (BMS with transport=ble) have no pins — hide the picker
// in that case so the operator doesn't get a pointless "no pair
// selected" error.
const pinsVisible = computed(() => {
  const t = modalType.value
  if (!t) return false
  if (t.pin_kind === 'builtin') return false
  // If the type declares a `transport` param and the operator has
  // picked a non-uart transport, pins don't apply.
  const transportParam = (t.params || []).find(p => p.id === 'transport')
  if (transportParam && modalParams.value.transport &&
      modalParams.value.transport !== 'uart') {
    return false
  }
  return true
})

// ── BLE scan (BMS discovery) ────────────────────────────────────
// Toggled by a button on the modal when the catalog type is the
// BMS *and* the operator has picked the BLE transport. Results
// arrive asynchronously on `ble_scan_results/<node_id>`; we
// subscribe lazily right before triggering the action so other
// nodes' scans don't leak into this modal.
const scanRunning = ref(false)
const scanResults = ref([])
const scanError = ref('')
let scanTopic = null
let scanRequestId = null

function onScanFrame (msg) {
  if (typeof msg?.node !== 'string') return
  if (msg.node !== scanTopic) return
  const data = msg.data
  if (!data || typeof data !== 'object') return
  // Late frames from a prior scan are ignored.
  if (scanRequestId && data.request_id && data.request_id !== scanRequestId) return
  scanRunning.value = false
  if (data.status === 'ok') {
    scanResults.value = Array.isArray(data.devices) ? data.devices : []
    if (!scanResults.value.length) {
      scanError.value = 'No JBD-protocol BMSes found. Make sure the BMS is powered on and within range.'
    } else {
      scanError.value = ''
    }
  } else {
    scanError.value = data.message || `Scan ${data.status || 'failed'}`
  }
}

async function attachScanFeed () {
  if (scanTopic) return
  scanTopic = `ble_scan_results/${props.nodeId}`
  ws.on('state', onScanFrame)
  try { await ws.subscribe([scanTopic], 5) } catch (_) {}
}

async function detachScanFeed () {
  ws.off('state', onScanFrame)
  if (scanTopic) {
    try { await ws.unsubscribe([scanTopic]) } catch (_) {}
    scanTopic = null
  }
  scanRequestId = null
}

async function scanForBms () {
  scanError.value = ''
  scanResults.value = []
  scanRunning.value = true
  await attachScanFeed()
  // Random request id so result frames from a previous scan don't
  // overwrite a newer one if the node publishes them back-to-back.
  scanRequestId = `scan-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`
  try {
    const r = await ws.management('ble_scan_node', {
      node_id: props.nodeId,
      duration_s: 8,
      filter_jbd: true,
      request_id: scanRequestId,
    })
    if (r?.status === 'error') {
      scanRunning.value = false
      scanError.value = r.message || 'Scan request failed'
    }
  } catch (e) {
    scanRunning.value = false
    scanError.value = e.message || String(e)
  }
}

function pickScanResult (device) {
  modalParams.value.mac = device.mac
  // Pre-fill the label too if the operator hasn't typed one.
  if (!modalLabel.value && device.name) modalLabel.value = device.name
}

const scanAvailable = computed(() =>
  modalType.value?.id === 'pathfinder_bms' &&
  modalParams.value.transport === 'ble'
)

async function saveModal () {
  modalError.value = ''
  const type = typesById.value[modalTypeId.value]
  if (!type) { modalError.value = 'Pick a type'; return }

  // Drop params hidden by `visible_when` from the saved payload so
  // stale values (e.g. a MAC entered before switching transport
  // back to UART) don't reach the firmware. Pins likewise get
  // cleared when the picker is hidden.
  const params = {}
  for (const p of (type.params || [])) {
    if (paramVisible(p)) params[p.id] = modalParams.value[p.id]
  }
  const pins = pinsVisible.value ? { ...modalPins.value } : {}

  // Server contract (websocket_handler.py:1018): expects
  //   { node_id, peripheral: { id?, type, label, pins, params } }
  // — the nested envelope, not a flat payload.
  const peripheral = {
    type: type.id,
    label: modalLabel.value || type.label,
    pins,
    params,
  }
  if (modalMode.value === 'edit') peripheral.id = modalEditingId.value
  try {
    const r = await ws.management('save_node_peripheral', {
      node_id: props.nodeId,
      peripheral,
    })
    if (r?.success === false) { modalError.value = r.message || 'Save failed'; return }
    modalOpen.value = false
    loadAll()
  } catch (e) {
    modalError.value = e.message || String(e)
  }
}

async function removeIt (id) {
  if (!confirm(`Remove peripheral "${id}"?`)) return
  try {
    await ws.management('remove_node_peripheral', { node_id: props.nodeId, peripheral_id: id })
    loadAll()
  } catch (e) {
    console.warn('remove_node_peripheral failed:', e)
  }
}

// Channels exposed by a peripheral instance, after applying per-type
// clipping rules. The Maestro family is one catalog entry with 24
// channels statically declared; instances pick how many to expose via
// `params.channel_count` (6 / 12 / 18 / 24). Anything else returns
// the full type channel list unchanged.
function visibleChannelsOf (p) {
  const t = typesById.value[p.type]
  const all = t?.channels || []
  if (p.type === 'maestro') {
    const n = Number(p.params?.channel_count) || 6
    return all.slice(0, n)
  }
  return all
}

function readableChannelsOf (p) {
  return visibleChannelsOf(p).filter(c => c.dir === 'in')
}

async function toggleLog (p) {
  if (logPending.value[p.id]) return
  const inputs = readableChannelsOf(p)
  if (!inputs.length) return
  const want = !p.log_enabled
  // Optimistic flip so the pill state matches the click immediately.
  p.log_enabled = want
  logPending.value[p.id] = true
  logErrors.value[p.id] = ''
  try {
    const r = await ws.management('set_peripheral_log_enabled', {
      node_id: props.nodeId,
      peripheral_id: p.id,
      enabled: want,
    })
    if (r && r.success === false) throw new Error(r.message || 'Failed to toggle logging')
    if (want) {
      // Backfill so the Live tab sparkline is ready next visit.
      for (const ch of inputs) {
        history.fetchHistory(props.nodeId, p.id, ch.id)
      }
    } else {
      history.clearHistory(props.nodeId, p.id)
    }
  } catch (e) {
    p.log_enabled = !want                    // revert
    logErrors.value[p.id] = e.message || String(e)
  } finally {
    logPending.value[p.id] = false
  }
}

async function sync () {
  try {
    await ws.management('sync_node_peripherals', { node_id: props.nodeId })
    syncStatus.value = 'pending'
    loadAll()
  } catch (e) {
    console.warn('sync_node_peripherals failed:', e)
  }
}

const syncBadge = computed(() => ({
  synced:       { label: 'Synced',       cls: 'bg-emerald-500/20 text-emerald-400' },
  pending:      { label: 'Pending',      cls: 'bg-amber-500/20 text-amber-300' },
  error:        { label: 'Error',        cls: 'bg-red-500/20 text-red-400' },
  unknown:      { label: 'Unknown',      cls: 'bg-surface text-fg-muted' },
  unconfigured: { label: 'Unconfigured', cls: 'bg-surface text-fg-muted' },
}[syncStatus.value] || { label: 'Unknown', cls: 'bg-surface text-fg-muted' }))

const modalType = computed(() => typesById.value[modalTypeId.value])
</script>

<template>
  <div class="space-y-4">
    <div class="flex items-center justify-between">
      <h3 class="text-lg font-semibold text-fg-strong">Peripherals</h3>
      <div class="flex items-center gap-2">
        <span :class="['px-2 py-1 text-xs font-medium rounded-full', syncBadge.cls]">{{ syncBadge.label }}</span>
        <button class="btn-secondary text-sm" @click="sync"><span class="material-icons icon-sm">sync</span>Sync</button>
        <button class="btn-primary" @click="openAdd"><span class="material-icons icon-sm">add</span>Add peripheral</button>
      </div>
    </div>

    <div v-if="!peripherals.length" class="card text-center py-10">
      <span class="material-icons icon-lg text-fg-faint">cable</span>
      <p class="text-fg-muted text-sm mt-3">No peripherals configured.</p>
      <button class="btn-primary mt-4" @click="openAdd"><span class="material-icons icon-sm">add</span>Add the first one</button>
    </div>

    <div v-else class="grid grid-cols-1 md:grid-cols-2 gap-4">
      <div v-for="p in peripherals" :key="p.id" class="card">
        <header class="flex items-start justify-between mb-3">
          <div>
            <h4 class="text-base font-semibold text-fg-strong flex items-center gap-2 flex-wrap">
              {{ p.label || p.id }}
              <span v-if="p.builtin" class="px-2 py-0.5 text-xs rounded-full bg-surface text-fg">Built-in</span>
            </h4>
            <p class="text-xs text-fg-faint mt-0.5">
              {{ typesById[p.type]?.label || p.type }} · <span class="font-mono">{{ p.id }}</span>
            </p>
          </div>
          <div class="flex items-center gap-1">
            <button
              :class="[
                'px-2 py-1 text-xs font-medium rounded-full border transition-colors',
                readableChannelsOf(p).length === 0
                  ? 'bg-panel text-fg-faint border-line cursor-not-allowed'
                  : p.log_enabled
                    ? 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30 hover:bg-emerald-500/30'
                    : 'bg-surface/40 text-fg border-line-strong hover:bg-surface'
              ]"
              :disabled="readableChannelsOf(p).length === 0 || !!logPending[p.id]"
              :title="readableChannelsOf(p).length === 0
                ? 'No input channels to log'
                : (p.log_enabled ? 'Disable 30s history + NDJSON log' : 'Enable 30s history + NDJSON log')"
              @click="toggleLog(p)"
            >
              Log {{ readableChannelsOf(p).length === 0 ? '—' : (p.log_enabled ? 'on' : 'off') }}
            </button>
            <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong" @click="openEdit(p)" title="Edit">
              <span class="material-icons icon-sm">edit</span>
            </button>
            <button v-if="!p.builtin" class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong" @click="removeIt(p.id)" title="Remove">
              <span class="material-icons icon-sm">delete</span>
            </button>
          </div>
        </header>
        <div v-if="logErrors[p.id]" class="mb-2 px-2 py-1 text-xs rounded bg-red-500/20 border border-red-500/40 text-red-300">
          {{ logErrors[p.id] }}
        </div>

        <div class="text-xs text-fg-muted space-y-1 font-mono">
          <div v-for="(v, k) in p.pins" :key="k">{{ k }}: GP{{ v }}</div>
          <div v-if="Object.keys(p.params || {}).length === 0 && Object.keys(p.pins || {}).length === 0" class="italic text-fg-faint">
            No pins / params
          </div>
        </div>

        <div v-if="visibleChannelsOf(p).length" class="mt-3 pt-3 border-t border-line/50">
          <div class="text-xs uppercase tracking-wide text-fg-faint mb-1">Channels</div>
          <div class="flex flex-wrap gap-1">
            <span
              v-for="ch in visibleChannelsOf(p)"
              :key="ch.id"
              :class="['px-2 py-0.5 text-xs rounded font-mono',
                       ch.dir === 'in'
                         ? 'bg-cyan-900/30 text-cyan-300'
                         : 'bg-amber-900/30 text-amber-300']"
              :title="`${ch.display} (${ch.dir})`"
            >
              {{ ch.id }} {{ ch.dir === 'in' ? '↑' : '↓' }}
            </span>
          </div>
        </div>
      </div>
    </div>

    <AppModal v-if="modalOpen" :title="modalMode === 'add' ? 'Add peripheral' : 'Edit peripheral'" @close="modalOpen = false">
      <div v-if="modalError" class="mb-3 p-2 rounded bg-red-500/20 border border-red-500/40 text-sm text-red-300">{{ modalError }}</div>

      <div class="space-y-4">
        <div>
          <label class="block text-sm font-medium text-fg mb-1">Type</label>
          <select v-model="modalTypeId" class="input-field w-full" :disabled="modalMode === 'edit'">
            <option v-for="t in catalog.types" :key="t.id" :value="t.id">{{ t.label }}</option>
          </select>
          <p class="text-xs text-fg-faint mt-1">{{ modalType?.description }}</p>
        </div>

        <div>
          <label class="block text-sm font-medium text-fg mb-1">Label</label>
          <input v-model="modalLabel" type="text" class="input-field w-full" :placeholder="modalType?.label" />
        </div>

        <div v-if="pinsVisible && modalType?.pin_kind === 'uart'">
          <label class="block text-sm font-medium text-fg mb-1">UART pins</label>
          <select
            class="input-field w-full"
            :value="modalPins.uart_tx !== undefined ? `${modalPins.uart_tx}:${modalPins.uart_rx}` : ''"
            @change="(e) => { const [tx, rx] = e.target.value.split(':').map(n => parseInt(n,10)); modalPins = { uart_tx: tx, uart_rx: rx } }"
          >
            <option value="" disabled>Pick a UART pin pair…</option>
            <option
              v-for="pair in (capabilities?.uart_pairs || [])"
              :key="`${pair.tx}:${pair.rx}`"
              :value="`${pair.tx}:${pair.rx}`"
              :disabled="!!(claimedPins[pair.tx] || claimedPins[pair.rx])"
            >
              UART{{ pair.uart }}: TX=GP{{ pair.tx }}, RX=GP{{ pair.rx }}
              {{ (claimedPins[pair.tx] || claimedPins[pair.rx]) ? ` — in use by ${(claimedPins[pair.tx] || claimedPins[pair.rx]).label}` : '' }}
            </option>
          </select>
        </div>

        <div v-else-if="pinsVisible && modalType && modalType.pin_kind !== 'builtin'">
          <label class="block text-sm font-medium text-fg mb-1">Pin</label>
          <select
            class="input-field w-full"
            :value="modalPins.gpio !== undefined ? String(modalPins.gpio) : ''"
            @change="(e) => { modalPins = { gpio: parseInt(e.target.value, 10) } }"
          >
            <option value="" disabled>Pick a pin…</option>
            <option
              v-for="pin in (capabilities?.pins || [])"
              :key="pin.gpio"
              :value="pin.gpio"
              :disabled="!!claimedPins[pin.gpio]"
            >
              GP{{ pin.gpio }} ({{ pin.name }}){{ claimedPins[pin.gpio] ? ` — in use by ${claimedPins[pin.gpio].label}` : '' }}
            </option>
          </select>
        </div>

        <div v-if="modalType?.params?.length" class="space-y-3 pt-2 border-t border-line">
          <template v-for="p in modalType.params" :key="p.id">
          <div v-if="paramVisible(p)">
            <label class="block text-xs text-fg-muted mb-1">{{ p.label }}</label>
            <!-- Constrained-choice params (catalog `choices: [...]`)
                 render as a dropdown regardless of base `type`. Each
                 choice is either a scalar (used as both value+label)
                 or `{ value, label }`. -->
            <select
              v-if="p.choices && p.choices.length"
              class="input-field w-full"
              :value="String(modalParams[p.id] ?? '')"
              @change="(e) => modalParams[p.id] = coerceChoiceValue(p, e.target.value)"
            >
              <option v-for="(c, i) in p.choices" :key="i"
                      :value="String(typeof c === 'object' ? c.value : c)">
                {{ typeof c === 'object' ? (c.label ?? c.value) : c }}
              </option>
            </select>
            <label v-else-if="p.type === 'bool'" class="flex items-center gap-2 text-sm text-fg">
              <input type="checkbox" v-model="modalParams[p.id]" class="rounded bg-surface border-line-strong" />
              {{ p.label }}
            </label>
            <!--
              gpio-typed params (e.g. RoboClaw estop_pin) render as a
              pin picker rather than a freeform int so the operator
              can only pick a real pin on this node. Pins already
              claimed by OTHER peripherals are disabled with a hint;
              the one currently held by the peripheral being edited
              stays enabled so it can be reselected. Value 0 is the
              "no pin / feature disabled" sentinel.
            -->
            <select
              v-else-if="p.type === 'gpio'"
              class="input-field w-full"
              :value="String(modalParams[p.id] ?? 0)"
              @change="(e) => modalParams[p.id] = parseInt(e.target.value, 10) || 0"
            >
              <option value="0">— None —</option>
              <option
                v-for="pin in (capabilities?.pins || [])"
                :key="pin.gpio"
                :value="pin.gpio"
                :disabled="!!claimedPins[pin.gpio]"
              >
                GP{{ pin.gpio }} ({{ pin.name }}){{ claimedPins[pin.gpio] ? ` (claimed by ${claimedPins[pin.gpio].label || claimedPins[pin.gpio].id})` : '' }}
              </option>
            </select>
            <input
              v-else-if="p.type === 'int' || p.type === 'float'"
              type="number"
              :step="p.type === 'int' ? '1' : '0.01'"
              :min="p.min" :max="p.max"
              :value="modalParams[p.id]"
              @input="(e) => modalParams[p.id] = (p.type === 'int' ? parseInt : parseFloat)(e.target.value) || 0"
              class="input-field w-full"
            />
            <input
              v-else
              type="text"
              v-model="modalParams[p.id]"
              class="input-field w-full"
            />
            <!-- Optional inline guidance from the catalog. Renders
                 just under the input so the operator sees context
                 about what the param actually does. Single-line if
                 short, wraps if longer. -->
            <p v-if="p.help" class="text-[11px] text-fg-faint mt-1 leading-snug">{{ p.help }}</p>
            <!-- BMS-specific: a "Scan for BMS" button sits under the
                 MAC field so the operator doesn't have to type the
                 address by hand. We attach to the result topic
                 lazily on first click. -->
            <div v-if="p.id === 'mac' && scanAvailable" class="mt-2 space-y-2">
              <button type="button"
                      class="btn-secondary text-sm"
                      :disabled="scanRunning"
                      @click="scanForBms">
                <span class="material-icons icon-sm">{{ scanRunning ? 'hourglass_top' : 'bluetooth_searching' }}</span>
                {{ scanRunning ? 'Scanning…' : 'Scan for BMS' }}
              </button>
              <div v-if="scanError"
                   class="px-2 py-1 text-xs rounded bg-amber-500/20 border border-amber-500/40 text-amber-300">
                {{ scanError }}
              </div>
              <div v-if="scanResults.length" class="rounded border border-line bg-surface/40 divide-y divide-line/60">
                <button v-for="d in scanResults" :key="d.mac"
                        type="button"
                        class="w-full flex items-center justify-between px-3 py-2 text-left text-xs hover:bg-surface transition-colors"
                        :class="modalParams.mac === d.mac ? 'bg-cyan-900/30' : ''"
                        @click="pickScanResult(d)">
                  <span class="flex flex-col">
                    <span class="text-fg-strong">{{ d.name || '(unnamed)' }}</span>
                    <span class="font-mono text-fg-faint">{{ d.mac }}</span>
                  </span>
                  <span class="text-fg-muted">{{ d.rssi }} dBm</span>
                </button>
              </div>
            </div>
          </div>
          </template>
        </div>
      </div>

      <template #actions>
        <button class="btn-secondary" @click="modalOpen = false">Cancel</button>
        <button class="btn-primary" @click="saveModal">{{ modalMode === 'add' ? 'Add' : 'Save' }}</button>
      </template>
    </AppModal>
  </div>
</template>
