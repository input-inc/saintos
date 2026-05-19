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

// Latest value per (peripheral_id, channel_id).
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
      node_id: props.nodeId,
      peripheral_id: peripheralId,
      channel_id: channelId,
      value,
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
      const neutral = spec.neutral ?? 0
      await onCommit(p.id, { channelId: ch.id, value: neutral })
    }
  }
}

const syncBadge = computed(() => {
  const map = {
    synced:       { label: 'Synced',       cls: 'bg-emerald-900/40 text-emerald-300' },
    pending:      { label: 'Pending',      cls: 'bg-amber-900/40 text-amber-300' },
    error:        { label: 'Error',        cls: 'bg-rose-900/40 text-rose-300' },
    unknown:      { label: 'Unknown',      cls: 'bg-slate-800 text-slate-400' },
    unconfigured: { label: 'Unconfigured', cls: 'bg-slate-800 text-slate-400' },
  }
  return map[syncStatus.value] || map.unknown
})
</script>

<template>
  <div class="grid grid-cols-1 lg:grid-cols-3 gap-5">
    <section class="lg:col-span-2 space-y-4">
      <div class="flex items-center justify-between">
        <h3 class="text-lg font-semibold">Peripheral Controls</h3>
        <span :class="['px-2 py-0.5 rounded-full text-xs', syncBadge.cls]">
          {{ syncBadge.label }}
        </span>
      </div>

      <div v-if="!peripherals.length" class="card text-sm text-slate-400">
        No peripherals configured on this node yet.
      </div>

      <PeripheralCard
        v-for="p in peripherals"
        :key="p.id"
        :peripheral="p"
        :connected="connectedMap[p.id]"
        :channel-values="values[p.id] || {}"
        @commit="evt => onCommit(p.id, evt)"
      />
    </section>

    <aside class="space-y-4">
      <div class="card">
        <h3 class="text-sm font-semibold mb-3 text-slate-200">Summary</h3>
        <dl class="space-y-2 text-sm">
          <div class="flex justify-between"><dt class="stat-label">Node</dt><dd>{{ node?.online ? 'Online' : 'Offline' }}</dd></div>
          <div class="flex justify-between"><dt class="stat-label">Peripherals</dt><dd>{{ peripherals.length }}</dd></div>
          <div class="flex justify-between"><dt class="stat-label">Last frame</dt><dd>{{ pinState ? 'live' : '—' }}</dd></div>
        </dl>
      </div>
      <div class="card">
        <h3 class="text-sm font-semibold mb-3 text-slate-200">Quick actions</h3>
        <button class="btn w-full justify-center" @click="resetAll">Reset all outputs</button>
      </div>
    </aside>
  </div>
</template>
