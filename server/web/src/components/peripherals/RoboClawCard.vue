<script setup>
import { computed } from 'vue'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useDisplayStore } from '@/stores/display'
import Sparkline from '@/components/Sparkline.vue'
import { decodeRoboclawFaults } from '@/composables/roboclawFaults'

// Live-Readings card for a RoboClaw motor controller. Same data pipeline
// as MaestroCard/BMSCard: the Live tab supplies a per-channel `values`
// map built off pin_state/<node_id>. We decode it into a status row
// (online dot + fault badges) plus the motor/encoder/voltage/current/
// temperature telemetry table with sparklines.
//
// `connected` and `error_flags` come from the firmware driver's
// state_emit_channels callback (peripheral-first, keyed by
// peripheral_id — no virtual GPIO); the numeric telemetry rows come
// through the runtime-value path the generic table also uses.

const props = defineProps({
  peripheral: { type: Object, required: true },
  channels:   { type: Object, default: () => ({}) },   // { channel_id: {value, last_updated} }
  sparkSamples: { type: Function, default: () => [] },  // (channelId) => samples
})

const catalog = usePeripheralCatalog()
const display = useDisplayStore()

function chValue (id) {
  const v = props.channels?.[id]?.value
  return (typeof v === 'number') ? v : null
}
function chAge (id) {
  const ts = props.channels?.[id]?.last_updated
  if (!ts) return null
  return Math.max(0, Date.now() / 1000 - ts)
}

const connected    = computed(() => chValue('connected') === 1)
const errorFlags   = computed(() => chValue('error_flags') | 0)
const faults       = computed(() => decodeRoboclawFaults(errorFlags.value))
const connectedAge = computed(() => chAge('connected'))
const statusStale  = computed(() => connectedAge.value != null && connectedAge.value > 3.0)

// Telemetry rows (motor/encoder/voltage/current/temp) — value + unit
// formatting mirrors the RoboClawMonitor dashboard widget so the two
// surfaces read identically.
const ROWS = [
  { id: 'motor',   label: 'Motor duty',  cls: 'text-violet-300' },
  { id: 'encoder', label: 'Encoder',     cls: 'text-fg-strong' },
  { id: 'voltage', label: 'Bus voltage', cls: 'text-amber-300' },
  { id: 'current', label: 'Current',     cls: 'text-cyan-300' },
  { id: 'temp',    label: 'Temperature', cls: 'text-rose-300' },
]
function rowText (id) {
  const v = chValue(id)
  if (v == null) return '—'
  if (id === 'motor')   { const p = v * 100; return `${p > 0 ? '+' : ''}${p.toFixed(0)}` }
  if (id === 'encoder') return Number.isInteger(v) ? v.toString() : v.toFixed(0)
  if (id === 'temp')    return display.formatTemperature(v).replace(/°[CF]$/, '')
  return v.toFixed(2)
}
function rowUnit (id) {
  if (id === 'motor')   return '%'
  if (id === 'encoder') return 'cnt'
  if (id === 'voltage') return 'V'
  if (id === 'current') return 'A'
  if (id === 'temp')    return display.temperatureUnit === 'fahrenheit' ? '°F' : '°C'
  return ''
}
function rowAge (id) {
  const a = chAge(id)
  return a == null ? '' : `${a.toFixed(1)}s ago`
}

const typeLabel = computed(() =>
  catalog.byType(props.peripheral.type)?.label || props.peripheral.type)
</script>

<template>
  <div class="card">
    <header class="flex items-center justify-between mb-3">
      <div class="min-w-0">
        <h4 class="text-base font-semibold text-fg-strong flex items-center gap-2 flex-wrap">
          {{ peripheral.label || peripheral.id }}
          <!-- Connected dot: green answering, amber stale (>3s), red
               offline — same idiom as MaestroCard / the Nodes view. -->
          <span class="inline-flex items-center gap-1.5 px-2 py-0.5 text-xs rounded-full border"
                :class="connected && !statusStale
                  ? 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30'
                  : (statusStale
                      ? 'bg-amber-500/20 text-amber-400 border-amber-500/30'
                      : 'bg-red-500/20 text-red-400 border-red-500/30')">
            <span class="w-1.5 h-1.5 rounded-full"
                  :class="connected && !statusStale
                    ? 'bg-emerald-400 animate-pulse-dot'
                    : (statusStale ? 'bg-amber-400' : 'bg-red-400')"></span>
            {{ connected && !statusStale ? 'Online' : (statusStale ? 'Stale' : 'Offline') }}
          </span>
          <!-- Fault chip in the header when any fault is active, so a
               problem is visible even before you scan the badge strip. -->
          <span v-if="connected && faults.length"
                class="inline-flex items-center gap-1 px-2 py-0.5 text-xs rounded-full bg-red-500/20 text-red-300 border border-red-500/30">
            <span class="material-icons text-[14px] leading-none">warning</span>
            {{ faults.length }} fault{{ faults.length === 1 ? '' : 's' }}
          </span>
        </h4>
        <p class="text-xs text-fg-faint">
          {{ typeLabel }} · <span class="font-mono">{{ peripheral.id }}</span>
        </p>
      </div>
    </header>

    <!-- Decoded fault badges. Error-severity faults are red, warnings
         amber. Renders nothing when healthy so the card stays quiet. -->
    <div v-if="connected && faults.length" class="mb-3 flex flex-wrap gap-1.5">
      <span v-for="f in faults" :key="f.label"
            class="px-2 py-0.5 text-xs rounded-full border"
            :class="f.severity === 'error'
              ? 'bg-red-500/20 text-red-300 border-red-500/30'
              : 'bg-amber-500/20 text-amber-300 border-amber-500/30'">
        {{ f.label }}
      </span>
    </div>

    <!-- Offline hint. -->
    <div v-if="!connected" class="mb-3 p-2 text-xs text-amber-300 bg-amber-500/10 border border-amber-500/30 rounded">
      Not answering on the packet-serial link. Check the cable, the
      controller's address/baud, and that it's powered.
    </div>

    <!-- Telemetry table. -->
    <div class="space-y-1">
      <div v-for="row in ROWS" :key="row.id"
           class="flex items-center justify-between text-sm font-mono py-1 border-b border-line/50 last:border-b-0">
        <span class="text-fg-muted">{{ row.label }}</span>
        <div class="flex items-center gap-3">
          <Sparkline :samples="sparkSamples(row.id)" />
          <span :class="chValue(row.id) != null ? row.cls : 'text-fg-faint'">
            {{ rowText(row.id) }}<span class="text-xs text-fg-faint ml-1">{{ chValue(row.id) != null ? rowUnit(row.id) : '' }}</span>
          </span>
          <span class="text-xs text-fg-faint w-16 text-right">{{ rowAge(row.id) }}</span>
        </div>
      </div>
    </div>
  </div>
</template>
