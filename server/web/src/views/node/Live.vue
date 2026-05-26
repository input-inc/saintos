<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useWsTopic } from '@/composables/useWsTopic'
import { useChannelHistory } from '@/composables/useChannelHistory'
import Sparkline from '@/components/Sparkline.vue'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const peripherals = ref([])
const pinState = useWsTopic(() => `pin_state/${props.nodeId}`)
const history = useChannelHistory()

async function loadPeripherals () {
  try {
    const r = await ws.management('get_node_peripherals', { node_id: props.nodeId })
    peripherals.value = r?.peripherals || []
    // Backfill the 30s ring for every log-enabled peripheral's input
    // channels. Fired in parallel; sparklines populate as responses
    // land. Non-log-enabled peripherals get no backfill — they show
    // the "not logging" hint instead.
    for (const p of peripherals.value) {
      if (!p.log_enabled) continue
      const t = catalog.byType(p.type)
      for (const ch of (t?.channels || [])) {
        if (ch.dir !== 'in') continue
        history.fetchHistory(props.nodeId, p.id, ch.id)
      }
    }
  } catch (e) {
    console.warn('Failed to load peripherals:', e)
  }
}

onMounted(async () => {
  await catalog.ensureLoaded()
  await loadPeripherals()
})
watch(() => props.nodeId, async () => {
  await catalog.ensureLoaded()
  await loadPeripherals()
})

// Route live pin_state samples into the channel-history rings. The
// composable trims to the 30s window on every push, so this is safe
// to run at the topic's full broadcast rate.
watch(pinState, (data) => {
  if (!data) return
  const channels = Array.isArray(data.channels) ? data.channels : []
  // Only feed rings for peripherals the operator has enabled logging
  // on — matches the legacy behaviour where flipping Log off stops
  // the sparkline from growing.
  const loggedIds = new Set(peripherals.value.filter(p => p.log_enabled).map(p => p.id))
  const now = Date.now()
  for (const ch of channels) {
    if (!ch.peripheral_id || !ch.channel_id) continue
    if (!loggedIds.has(ch.peripheral_id)) continue
    if (typeof ch.value !== 'number') continue
    history.pushSample(`${props.nodeId}/${ch.peripheral_id}/${ch.channel_id}`, ch.value, now)
  }
})

const values = computed(() => {
  const out = {}
  for (const ch of (pinState.value?.channels || [])) {
    if (!ch.peripheral_id || !ch.channel_id) continue
    out[ch.peripheral_id] ??= {}
    out[ch.peripheral_id][ch.channel_id] = ch
  }
  return out
})

const stale = computed(() => pinState.value?.stale)
const lastFeedback = computed(() => pinState.value?.last_feedback)

function fmtValue (v) {
  if (v === null || v === undefined) return null
  if (typeof v !== 'number') return String(v)
  return Number.isInteger(v) ? v.toString() : v.toFixed(3)
}
function ageStr (ts) {
  if (!ts) return ''
  const age = Math.max(0, Date.now() / 1000 - ts)
  return `${age.toFixed(1)}s ago`
}
function sparkSamples (nodeId, peripheralId, channelId) {
  return history.getHistory(`${nodeId}/${peripheralId}/${channelId}`)
}
</script>

<template>
  <div>
    <div class="flex items-center justify-between mb-4">
      <h3 class="text-lg font-semibold text-white">Live readings</h3>
      <span v-if="lastFeedback" :class="['text-xs', stale ? 'text-amber-400' : 'text-emerald-400']">
        {{ stale ? 'Stale feed' : 'Live feed' }}
      </span>
      <span v-else class="text-xs text-slate-500">No feedback yet</span>
    </div>

    <div v-if="!peripherals.length" class="card text-center py-10">
      <span class="material-icons icon-lg text-slate-600">sensors</span>
      <p class="text-slate-400 text-sm mt-3">No peripherals on this node yet.</p>
    </div>

    <div v-else class="grid grid-cols-1 md:grid-cols-2 gap-4">
      <div v-for="p in peripherals" :key="p.id" class="card">
        <header class="flex items-center justify-between mb-3">
          <div>
            <h4 class="text-base font-semibold text-white flex items-center gap-2 flex-wrap">
              {{ p.label || p.id }}
              <span v-if="p.builtin" class="px-2 py-0.5 text-xs rounded-full bg-slate-700 text-slate-300">Built-in</span>
              <span
                v-if="p.log_enabled"
                class="px-2 py-0.5 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400 border border-emerald-500/30"
                title="Recording 30s in-RAM history"
              >Logging</span>
            </h4>
            <p class="text-xs text-slate-500">{{ catalog.byType(p.type)?.label || p.type }} · <span class="font-mono">{{ p.id }}</span></p>
          </div>
        </header>

        <div v-if="!(catalog.byType(p.type)?.channels?.length)" class="text-xs text-slate-500 italic">
          No declared channels.
        </div>
        <div v-else class="space-y-1">
          <div
            v-for="ch in catalog.byType(p.type).channels"
            :key="ch.id"
            class="flex items-center justify-between text-sm font-mono py-1 border-b border-slate-700/50 last:border-b-0"
          >
            <span class="text-slate-400">{{ ch.display || ch.id }}</span>
            <div class="flex items-center gap-3">
              <Sparkline
                v-if="ch.dir === 'in' && p.log_enabled"
                :samples="sparkSamples(nodeId, p.id, ch.id)"
              />
              <span
                v-else-if="ch.dir === 'in'"
                class="text-[10px] italic text-slate-600"
                title="Enable logging on this peripheral to see history"
              >not logging</span>
              <span
                v-if="values[p.id]?.[ch.id] && values[p.id][ch.id].value !== null && values[p.id][ch.id].value !== undefined"
                :class="ch.dir === 'in' ? 'text-cyan-300' : 'text-amber-300'"
              >{{ fmtValue(values[p.id][ch.id].value) }}</span>
              <span v-else class="text-slate-500">—</span>
              <span class="text-xs text-slate-600 w-20 text-right">{{ ageStr(values[p.id]?.[ch.id]?.last_updated) }}</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>
