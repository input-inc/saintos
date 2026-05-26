<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useWsTopic } from '@/composables/useWsTopic'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const peripherals = ref([])
const pinState = useWsTopic(() => `pin_state/${props.nodeId}`)

async function loadPeripherals () {
  try {
    const r = await ws.management('get_node_peripherals', { node_id: props.nodeId })
    peripherals.value = r?.peripherals || []
  } catch (e) {
    console.warn('Failed to load peripherals:', e)
  }
}

onMounted(() => { catalog.ensureLoaded(); loadPeripherals() })
watch(() => props.nodeId, loadPeripherals)

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
            <h4 class="text-base font-semibold text-white">{{ p.label || p.id }}</h4>
            <p class="text-xs text-slate-500">{{ catalog.byType(p.type)?.label || p.type }} · <span class="font-mono">{{ p.id }}</span></p>
          </div>
          <span v-if="p.builtin" class="px-2 py-0.5 text-xs rounded-full bg-slate-700 text-slate-300">Built-in</span>
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
