<script setup>
import { computed } from 'vue'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'

// Live-Readings card for a Kangaroo x2 motion channel. Same data
// pipeline as MaestroCard/RoboClawCard: the Live tab supplies a
// per-channel `values` map, here fed by the virtual-GPIO translation in
// state_manager._FIRMWARE_CHANNEL_MAP (the Kangaroo driver hasn't
// migrated to state_emit_channels yet).
//
// The Kangaroo has no `connected` channel of its own, so liveness is
// inferred from whether position readings are still arriving — see
// `stale` below.

const props = defineProps({
  peripheral: { type: Object, required: true },
  channels:   { type: Object, default: () => ({}) },
  sparkSamples: { type: Function, default: () => [] },
})

const catalog = usePeripheralCatalog()

function chValue (id) {
  const v = props.channels?.[id]?.value
  return (typeof v === 'number') ? v : null
}
function chAge (id) {
  const ts = props.channels?.[id]?.last_updated
  if (!ts) return null
  return Math.max(0, Date.now() / 1000 - ts)
}

// Get-reply error codes, Packet Serial Reference p.12. NOT the LED blink
// codes from the main manual — different numbering entirely.
const ERROR_LABEL = {
  1: 'Not started',
  2: 'Needs homing',
  3: 'Control error',
  4: 'Wrong mode — DIPs disagree with the tune',
  5: 'Unknown parameter',
  6: 'Serial timeout / TX disconnected',
}

// kangaroo_tune_state_t in firmware/shared/include/kangaroo_driver.h.
const TUNE_LABEL = {
  1: 'Entering tune', 2: 'Tune: ready to jog',
  3: 'Tuning — stand clear', 4: 'Tune complete — power cycle',
  5: 'Tune failed',
}

const position   = computed(() => chValue('current_position'))
const speed      = computed(() => chValue('current_speed'))
const moving     = computed(() => chValue('moving') === 1)
const errorCode  = computed(() => chValue('error_status') ?? 0)
const errorText  = computed(() =>
  errorCode.value ? (ERROR_LABEL[errorCode.value] || `Error ${errorCode.value}`) : '')
const tuneState  = computed(() => chValue('tune_state') ?? 0)
const tuneText   = computed(() => TUNE_LABEL[tuneState.value] || '')
const taughtMin  = computed(() => chValue('taught_min'))
const taughtMax  = computed(() => chValue('taught_max'))

// No connected channel on this peripheral — position age is the only
// liveness signal available. Say "Stale" rather than "Offline" because
// that is genuinely all we know.
const posAge = computed(() => chAge('current_position'))
const stale  = computed(() => posAge.value == null || posAge.value > 3.0)

const isLinear = computed(() => props.peripheral?.params?.motion_mode === 'linear')
const typeLabel = computed(() =>
  catalog.byType(props.peripheral.type)?.label || props.peripheral.type)

const ROWS = [
  { id: 'current_position', label: 'Position' },
  { id: 'current_speed',    label: 'Speed' },
  { id: 'target_position',  label: 'Target position' },
  { id: 'target_speed',     label: 'Target speed' },
]
function fmt (id) {
  const v = chValue(id)
  if (v == null) return '—'
  return Number.isInteger(v) ? v.toString() : v.toFixed(2)
}
function fmtAge (id) {
  const a = chAge(id)
  return a == null ? '' : `${a.toFixed(1)}s ago`
}
</script>

<template>
  <div class="card">
    <header class="flex items-center justify-between mb-3">
      <div class="min-w-0">
        <h4 class="text-base font-semibold text-fg-strong flex items-center gap-2 flex-wrap">
          {{ peripheral.label || peripheral.id }}
          <span class="inline-flex items-center gap-1.5 px-2 py-0.5 text-xs rounded-full border"
                :class="!stale
                  ? 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30'
                  : 'bg-amber-500/20 text-amber-400 border-amber-500/30'">
            <span class="w-1.5 h-1.5 rounded-full"
                  :class="!stale ? 'bg-emerald-400 animate-pulse-dot' : 'bg-amber-400'"></span>
            {{ !stale ? 'Online' : 'Stale' }}
          </span>
          <span v-if="moving"
                class="inline-flex items-center gap-1.5 px-2 py-0.5 text-xs rounded-full bg-cyan-500/20 text-cyan-300 border border-cyan-500/30">
            <span class="material-icons text-[14px] leading-none">arrow_outward</span>
            Moving
          </span>
          <!-- A tune in progress is the single most important thing on
               this card when it's happening — the axis is under the
               Kangaroo's control, not ours. -->
          <span v-if="tuneText"
                class="inline-flex items-center gap-1.5 px-2 py-0.5 text-xs rounded-full border"
                :class="tuneState === 3
                  ? 'bg-amber-500/20 text-amber-300 border-amber-500/30'
                  : tuneState === 5
                    ? 'bg-red-500/20 text-red-300 border-red-500/30'
                    : 'bg-violet-500/20 text-violet-300 border-violet-500/30'">
            <span class="material-icons text-[14px] leading-none">tune</span>
            {{ tuneText }}
          </span>
        </h4>
        <p class="text-xs text-fg-faint">
          {{ typeLabel }} · <span class="font-mono">{{ peripheral.id }}</span>
          · addr {{ peripheral?.params?.address ?? '?' }}
          ch {{ peripheral?.params?.channel ?? '?' }}
        </p>
      </div>
    </header>

    <div v-if="errorText" class="mb-3 flex flex-wrap gap-1.5">
      <span class="px-2 py-0.5 text-xs rounded-full bg-red-500/20 text-red-300 border border-red-500/30">
        {{ errorText }}
      </span>
    </div>

    <div v-if="stale" class="mb-3 p-2 text-xs text-amber-300 bg-amber-500/10 border border-amber-500/30 rounded">
      No recent position reading. Check the serial wiring (controller TX → S1,
      RX → S2), that the SyRen is in packetized serial mode at the matching
      address, and that the baud rate matches DEScribe — the Kangaroo has no
      autobaud.
    </div>

    <div class="space-y-1">
      <div v-for="r in ROWS" :key="r.id"
           class="flex items-center justify-between text-sm font-mono py-1 border-b border-line/50 last:border-b-0">
        <span class="text-fg-muted">{{ r.label }}</span>
        <div class="flex items-center gap-3">
          <span :class="chValue(r.id) != null ? 'text-amber-300' : 'text-fg-faint'">
            {{ fmt(r.id) }}
          </span>
          <span class="text-xs text-fg-faint w-20 text-right">{{ fmtAge(r.id) }}</span>
        </div>
      </div>
    </div>

    <!-- Taught travel only means something on a linear channel that has
         actually been tuned; showing "— … —" on a rotational one would
         just be noise. -->
    <div v-if="isLinear && (taughtMin != null || taughtMax != null)"
         class="mt-2 pt-2 border-t border-line flex items-center justify-between text-sm font-mono">
      <span class="text-xs text-fg-muted">Taught travel</span>
      <span class="text-emerald-300">{{ taughtMin ?? '—' }} … {{ taughtMax ?? '—' }}</span>
    </div>
  </div>
</template>
