<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import AppModal from '@/components/AppModal.vue'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const peripherals = ref([])
const syncStatus  = ref('unknown')
const capabilities = ref(null)        // { pins, uart_pairs, reserved_pins }

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

onMounted(() => { catalog.ensureLoaded(); loadAll() })
watch(() => props.nodeId, loadAll)

const typesById = computed(() => {
  const out = {}
  for (const t of catalog.types) out[t.id] = t
  return out
})

const claimedPins = computed(() => {
  const out = {}
  for (const p of peripherals.value) {
    if (modalMode.value === 'edit' && p.id === modalEditingId.value) continue
    for (const v of Object.values(p.pins || {})) out[v] = p
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

async function saveModal () {
  modalError.value = ''
  const type = typesById.value[modalTypeId.value]
  if (!type) { modalError.value = 'Pick a type'; return }

  const payload = {
    node_id: props.nodeId,
    type: type.id,
    label: modalLabel.value || type.label,
    pins: { ...modalPins.value },
    params: { ...modalParams.value },
  }
  if (modalMode.value === 'edit') payload.id = modalEditingId.value
  try {
    const r = await ws.management('save_node_peripheral', payload)
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
  unknown:      { label: 'Unknown',      cls: 'bg-slate-700 text-slate-400' },
  unconfigured: { label: 'Unconfigured', cls: 'bg-slate-700 text-slate-400' },
}[syncStatus.value] || { label: 'Unknown', cls: 'bg-slate-700 text-slate-400' }))

const modalType = computed(() => typesById.value[modalTypeId.value])
</script>

<template>
  <div class="space-y-4">
    <div class="flex items-center justify-between">
      <h3 class="text-lg font-semibold text-white">Peripherals</h3>
      <div class="flex items-center gap-2">
        <span :class="['px-2 py-1 text-xs font-medium rounded-full', syncBadge.cls]">{{ syncBadge.label }}</span>
        <button class="btn-secondary text-sm" @click="sync"><span class="material-icons icon-sm">sync</span>Sync</button>
        <button class="btn-primary" @click="openAdd"><span class="material-icons icon-sm">add</span>Add peripheral</button>
      </div>
    </div>

    <div v-if="!peripherals.length" class="card text-center py-10">
      <span class="material-icons icon-lg text-slate-600">cable</span>
      <p class="text-slate-400 text-sm mt-3">No peripherals configured.</p>
      <button class="btn-primary mt-4" @click="openAdd"><span class="material-icons icon-sm">add</span>Add the first one</button>
    </div>

    <div v-else class="grid grid-cols-1 md:grid-cols-2 gap-4">
      <div v-for="p in peripherals" :key="p.id" class="card">
        <header class="flex items-start justify-between mb-3">
          <div>
            <h4 class="text-base font-semibold text-white flex items-center gap-2 flex-wrap">
              {{ p.label || p.id }}
              <span v-if="p.builtin" class="px-2 py-0.5 text-xs rounded-full bg-slate-700 text-slate-300">Built-in</span>
            </h4>
            <p class="text-xs text-slate-500 mt-0.5">
              {{ typesById[p.type]?.label || p.type }} · <span class="font-mono">{{ p.id }}</span>
            </p>
          </div>
          <div class="flex items-center gap-1">
            <button class="btn-sm bg-slate-700 hover:bg-slate-600 text-slate-200" @click="openEdit(p)" title="Edit">
              <span class="material-icons icon-sm">edit</span>
            </button>
            <button v-if="!p.builtin" class="btn-sm bg-slate-700 hover:bg-red-600 text-slate-300 hover:text-white" @click="removeIt(p.id)" title="Remove">
              <span class="material-icons icon-sm">delete</span>
            </button>
          </div>
        </header>

        <div class="text-xs text-slate-400 space-y-1 font-mono">
          <div v-for="(v, k) in p.pins" :key="k">{{ k }}: GP{{ v }}</div>
          <div v-if="Object.keys(p.params || {}).length === 0 && Object.keys(p.pins || {}).length === 0" class="italic text-slate-500">
            No pins / params
          </div>
        </div>

        <div v-if="typesById[p.type]?.channels?.length" class="mt-3 pt-3 border-t border-slate-700/50">
          <div class="text-xs uppercase tracking-wide text-slate-500 mb-1">Channels</div>
          <div class="flex flex-wrap gap-1">
            <span
              v-for="ch in typesById[p.type].channels"
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
          <label class="block text-sm font-medium text-slate-300 mb-1">Type</label>
          <select v-model="modalTypeId" class="input-field w-full" :disabled="modalMode === 'edit'">
            <option v-for="t in catalog.types" :key="t.id" :value="t.id">{{ t.label }}</option>
          </select>
          <p class="text-xs text-slate-500 mt-1">{{ modalType?.description }}</p>
        </div>

        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">Label</label>
          <input v-model="modalLabel" type="text" class="input-field w-full" :placeholder="modalType?.label" />
        </div>

        <div v-if="modalType?.pin_kind === 'uart'">
          <label class="block text-sm font-medium text-slate-300 mb-1">UART pins</label>
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

        <div v-else-if="modalType && modalType.pin_kind !== 'builtin'">
          <label class="block text-sm font-medium text-slate-300 mb-1">Pin</label>
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

        <div v-if="modalType?.params?.length" class="space-y-3 pt-2 border-t border-slate-700">
          <div v-for="p in modalType.params" :key="p.id">
            <label class="block text-xs text-slate-400 mb-1">{{ p.label }}</label>
            <label v-if="p.type === 'bool'" class="flex items-center gap-2 text-sm text-slate-300">
              <input type="checkbox" v-model="modalParams[p.id]" class="rounded bg-slate-700 border-slate-600" />
              {{ p.label }}
            </label>
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
          </div>
        </div>
      </div>

      <template #actions>
        <button class="btn-secondary" @click="modalOpen = false">Cancel</button>
        <button class="btn-primary" @click="saveModal">{{ modalMode === 'add' ? 'Add' : 'Save' }}</button>
      </template>
    </AppModal>
  </div>
</template>
