<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useWsTopic } from '@/composables/useWsTopic'
import PeripheralCard from '@/components/PeripheralCard.vue'
import { specFor } from '@/components/channel/ChannelSpec'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const peripherals = ref([])
const syncStatus  = ref('unknown')
const pinState = useWsTopic(() => `pin_state/${props.nodeId}`)
// Live sync-status feed — same source the Peripherals tab uses.
// Without this the badge would freeze at whatever loadPeripherals
// last fetched and never reflect a successful sync until the next
// tab switch.
const syncFeed = useWsTopic(() => `sync_status/${props.nodeId}`, 5)
watch(syncFeed, (v) => {
  if (v && typeof v.sync_status === 'string') syncStatus.value = v.sync_status
})

const values = computed(() => {
  const out = {}
  const channels = pinState.value?.channels || []
  for (const ch of channels) {
    if (!ch.peripheral_id || !ch.channel_id) continue
    out[ch.peripheral_id] ??= {}
    out[ch.peripheral_id][ch.channel_id] = ch.value
  }
  return out
})

async function loadPeripherals () {
  try {
    const r = await ws.management('get_node_peripherals', { node_id: props.nodeId })
    peripherals.value = r?.peripherals || []
    syncStatus.value  = r?.sync_status || 'unknown'
  } catch (e) {
    console.warn('Failed to load peripherals:', e)
  }
}

onMounted(() => {
  catalog.ensureLoaded()
  loadPeripherals()
})
watch(() => props.nodeId, () => loadPeripherals())

const connectedMap = computed(() => props.node?.peripheral_connected || {})

async function onCommit (peripheralId, { channelId, value }) {
  try {
    await ws.control('set_channel_value', {
      node_id: props.nodeId, peripheral_id: peripheralId, channel_id: channelId, value,
    })
  } catch (e) {
    console.warn('set_channel_value failed:', e)
  }
}

async function resetAll () {
  for (const p of peripherals.value) {
    const type = catalog.byType(p.type)
    if (!type) continue
    for (const ch of (type.channels || [])) {
      if (ch.dir !== 'out') continue
      const spec = specFor(p.type, ch)
      if (spec.unsupported) continue
      await onCommit(p.id, { channelId: ch.id, value: spec.neutral ?? 0 })
    }
  }
}

const syncBadge = computed(() => ({
  synced:       { label: 'Synced',       cls: 'bg-emerald-500/20 text-emerald-400' },
  pending:      { label: 'Pending',      cls: 'bg-amber-500/20 text-amber-300' },
  error:        { label: 'Error',        cls: 'bg-red-500/20 text-red-400' },
  unknown:      { label: 'Unknown',      cls: 'bg-surface text-fg-muted' },
  unconfigured: { label: 'Unconfigured', cls: 'bg-surface text-fg-muted' },
}[syncStatus.value] || { label: 'Unknown', cls: 'bg-surface text-fg-muted' }))
</script>

<template>
  <div class="grid grid-cols-1 lg:grid-cols-3 gap-6">
    <div class="lg:col-span-2 space-y-4">
      <div class="flex items-center justify-between">
        <h3 class="text-lg font-semibold text-fg-strong">Peripheral Controls</h3>
        <span :class="['px-2 py-1 text-xs font-medium rounded-full', syncBadge.cls]">{{ syncBadge.label }}</span>
      </div>

      <div v-if="!peripherals.length" class="card text-center py-10">
        <span class="material-icons icon-lg text-fg-faint">tune</span>
        <p class="text-fg-muted text-sm mt-3">No peripherals configured on this node yet.</p>
      </div>

      <PeripheralCard
        v-for="p in peripherals"
        :key="p.id"
        :peripheral="p"
        :connected="connectedMap[p.id]"
        :channel-values="values[p.id] || {}"
        @commit="evt => onCommit(p.id, evt)"
      />
    </div>

    <div class="space-y-4">
      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-4">State Summary</h3>
        <div class="space-y-3">
          <div class="stat-item">
            <span class="stat-label">Node Status</span>
            <span class="stat-value text-sm">{{ node?.online ? 'Online' : 'Offline' }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Peripherals</span>
            <span class="stat-value text-sm">{{ peripherals.length }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Last Frame</span>
            <span class="stat-value text-sm">{{ pinState ? 'live' : '—' }}</span>
          </div>
        </div>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-4">Quick Actions</h3>
        <button class="btn-secondary w-full justify-center text-sm" @click="resetAll">
          <span class="material-icons icon-sm">center_focus_strong</span>
          Reset all outputs
        </button>
      </div>
    </div>
  </div>
</template>
