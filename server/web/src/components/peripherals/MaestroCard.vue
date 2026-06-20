<script setup>
import { computed } from 'vue'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'

// Live-Readings card for a Pololu Maestro peripheral. Same data
// pipeline as BMSCard: the Live tab supplies a per-channel `values`
// map built off pin_state/<node_id>; we just decode it into the
// status row (online dot + error badges + moving indicator) plus the
// per-channel servo target table.
//
// The status channels (connected, error_flags, moving) are emitted by
// firmware/shared/src/maestro_driver.c's state_emit_channels callback
// at the pin_state broadcast rate, sourced from a ~2 Hz internal
// poll cache (GET_ERRORS + GET_MOVING_STATE round-trip the link, so
// we throttle).

const props = defineProps({
  peripheral: { type: Object, required: true },
  channels:   { type: Object, default: () => ({}) },   // { channel_id: {value, last_updated} }
  sparkSamples: { type: Function, default: () => [] },  // (channelId) => samples (unused for Maestro)
})

const catalog = usePeripheralCatalog()

function chValue (id) {
  const c = props.channels?.[id]
  const v = c?.value
  return (typeof v === 'number') ? v : null
}

function chAge (id) {
  const ts = props.channels?.[id]?.last_updated
  if (!ts) return null
  return Math.max(0, Date.now() / 1000 - ts)
}

// Pololu Maestro Error register (User's Guide §6.4 — same bit layout
// for Micro and Mini Maestros). Bit indices 0..7 are documented; 8..15
// are reserved on current firmware but we render them as "Reserved
// bit N" rather than dropping them so a future Maestro fw revision
// that sets a new bit shows SOMETHING.
const ERROR_LABELS = [
  'Serial signal error',     // bit 0
  'Serial overrun',          // bit 1
  'Serial buffer full',      // bit 2
  'Serial CRC error',        // bit 3
  'Serial protocol error',   // bit 4
  'Serial timeout',          // bit 5
  'Script stack error',      // bit 6
  'Script call stack error', // bit 7
  'Script program counter',  // bit 8
]
function decodeErrors (bits) {
  const b = (bits | 0) & 0xFFFF
  const out = []
  for (let i = 0; i < 16; i++) {
    if (b & (1 << i)) {
      out.push(ERROR_LABELS[i] || `Reserved bit ${i}`)
    }
  }
  return out
}

const connected   = computed(() => chValue('connected') === 1)
const errorFlags  = computed(() => chValue('error_flags') | 0)
const errors      = computed(() => decodeErrors(errorFlags.value))
const moving      = computed(() => chValue('moving') === 1)
const connectedAge = computed(() => chAge('connected'))
const statusStale = computed(() => connectedAge.value != null && connectedAge.value > 3.0)

// Channel-count clip mirrors what Live.vue does for the generic table.
const channelCount = computed(() => Number(props.peripheral?.params?.channel_count) || 6)
const channelIds   = computed(() => {
  const ids = []
  for (let i = 0; i < channelCount.value; i++) ids.push(`ch${i}`)
  return ids
})

function fmtChannel (id) {
  const v = chValue(id)
  if (v == null) return '—'
  return Number.isInteger(v) ? v.toString() : v.toFixed(2)
}
function fmtAge (id) {
  const a = chAge(id)
  if (a == null) return ''
  return `${a.toFixed(1)}s ago`
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
          <!-- Connected dot — green when answering polls, amber when
               stale (>3s no broadcast), red when explicitly
               disconnected. Same idiom as the per-node online dot in
               the Nodes view so the visual language stays consistent. -->
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
          <!-- Moving indicator only renders when actually moving so
               the steady-state card is clean. -->
          <span v-if="connected && moving"
                class="inline-flex items-center gap-1.5 px-2 py-0.5 text-xs rounded-full bg-cyan-500/20 text-cyan-300 border border-cyan-500/30">
            <span class="material-icons text-[14px] leading-none">arrow_outward</span>
            Moving
          </span>
        </h4>
        <p class="text-xs text-fg-faint">
          {{ typeLabel }} · <span class="font-mono">{{ peripheral.id }}</span>
        </p>
      </div>
    </header>

    <!-- Error flag badges. Renders nothing when 0 — keeps the card
         visually quiet under normal operation. The Pololu convention
         is that reading GET_ERRORS clears the device-side latches,
         and the firmware-side poller clears on connect — so this
         only shows persistent / repeated faults the operator should
         actually care about. -->
    <div v-if="connected && errors.length" class="mb-3 flex flex-wrap gap-1.5">
      <span v-for="e in errors" :key="e"
            class="px-2 py-0.5 text-xs rounded-full bg-red-500/20 text-red-300 border border-red-500/30">
        {{ e }}
      </span>
    </div>

    <!-- Disconnected state: explain what to check. The kiosk operator
         is the audience here, not the dev — keep this terse but
         actionable. -->
    <div v-if="!connected" class="mb-3 p-2 text-xs text-amber-300 bg-amber-500/10 border border-amber-500/30 rounded">
      Not answering on
      <span class="font-mono">{{ peripheral?.params?.transport || 'transport' }}</span>.
      Check the cable, the Maestro's Serial Mode in MCC, and that
      it's powered.
    </div>

    <!-- Per-channel servo targets. The Maestro has output-only servo
         channels — we show the last value the routing layer sent
         (target pulse / angle) with its age. There's no
         actual-position polling today; if/when we add GET_POSITION
         per channel to the firmware poll cache, this is where the
         "actual" column lands. -->
    <div class="space-y-1">
      <div v-for="id in channelIds" :key="id"
           class="flex items-center justify-between text-sm font-mono py-1 border-b border-line/50 last:border-b-0">
        <span class="text-fg-muted">{{ id }}</span>
        <div class="flex items-center gap-3">
          <span :class="chValue(id) != null ? 'text-amber-300' : 'text-fg-faint'">
            {{ fmtChannel(id) }}
          </span>
          <span class="text-xs text-fg-faint w-20 text-right">{{ fmtAge(id) }}</span>
        </div>
      </div>
    </div>
  </div>
</template>
