<script setup>
import { computed } from 'vue'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'

// Live-Readings card for a Pimoroni Servo 2040. Same data pipeline as
// MaestroCard/RoboClawCard: the Live tab supplies a per-channel `values`
// map built off pin_state/<node_id>. We decode the status channels
// (connected, current_a, error_flags) into an online dot + current
// readout + fault badges, show the 6 onboard-LED colors as swatches, and
// fall back to a compact per-servo target table.
//
// Status channels are emitted by the firmware driver's
// state_emit_channels (firmware/shared/src/pimoroni_servo2040_driver.c)
// off its ~5 Hz I2C poll of the board's WHOAMI/CURRENT/STATUS registers.

const props = defineProps({
  peripheral: { type: Object, required: true },
  channels:   { type: Object, default: () => ({}) },   // { channel_id: {value, last_updated} }
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

// STATUS bitmask — keep in sync with PIMORONI_SERVO2040_FLAG_* in
// firmware/shared/include/pimoroni_servo2040_protocol.h.
const FLAG_LABELS = [
  'Over-current',   // 0x01
  'Failsafe (no heartbeat)', // 0x02
  // 0x04 HOMED is informational, not a fault — not surfaced as a badge.
]
function decodeFlags (bits) {
  const b = (bits | 0) & 0xFF
  const out = []
  if (b & 0x01) out.push(FLAG_LABELS[0])
  if (b & 0x02) out.push(FLAG_LABELS[1])
  return out
}

const connected    = computed(() => chValue('connected') === 1)
const currentA     = computed(() => chValue('current_a'))
const flags        = computed(() => chValue('error_flags') | 0)
const faults       = computed(() => decodeFlags(flags.value))
const connectedAge = computed(() => chAge('connected'))
const statusStale  = computed(() => connectedAge.value != null && connectedAge.value > 3.0)

const NUM_SERVOS = 18
const NUM_LEDS = 6
const servoIds = computed(() => Array.from({ length: NUM_SERVOS }, (_, i) => `ch${i}`))

// LED swatches: the color channels are output-only, so the value we
// show is the last color the operator set (packed uint24 the picker
// emitted). Render each as a small swatch.
function ledHex (i) {
  const v = chValue(`led${i}`)
  if (v == null) return null
  const n = (v | 0) & 0xFFFFFF
  return '#' + n.toString(16).padStart(6, '0')
}

function fmtServo (id) {
  const v = chValue(id)
  if (v == null) return '—'
  return Number(v).toFixed(2)
}
function fmtAge (id) {
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
        </h4>
        <p class="text-xs text-fg-faint">
          {{ typeLabel }} · <span class="font-mono">{{ peripheral.id }}</span>
        </p>
      </div>
      <!-- Aggregate servo current — the board's headline telemetry. -->
      <div v-if="connected" class="text-right">
        <div class="text-lg font-mono font-semibold text-cyan-300">
          {{ currentA != null ? currentA.toFixed(2) : '—' }}<span class="text-xs text-fg-faint ml-0.5">A</span>
        </div>
        <div class="text-[10px] uppercase tracking-wide text-fg-faint">servo current</div>
      </div>
    </header>

    <!-- Fault badges (over-current / failsafe). Quiet when healthy. -->
    <div v-if="connected && faults.length" class="mb-3 flex flex-wrap gap-1.5">
      <span v-for="f in faults" :key="f"
            class="px-2 py-0.5 text-xs rounded-full bg-red-500/20 text-red-300 border border-red-500/30">
        {{ f }}
      </span>
    </div>

    <div v-if="!connected" class="mb-3 p-2 text-xs text-amber-300 bg-amber-500/10 border border-amber-500/30 rounded">
      Not answering on I2C. Check the Qwiic/STEMMA-QT cable, the board's
      power, and that it's flashed with the SAINT.OS Servo 2040 image.
    </div>

    <!-- Onboard RGB LED swatches. -->
    <div class="mb-3">
      <div class="text-[10px] uppercase tracking-wide text-fg-faint mb-1">Onboard LEDs</div>
      <div class="flex items-center gap-1.5">
        <span v-for="i in NUM_LEDS" :key="i"
              class="w-5 h-5 rounded border border-line"
              :style="{ backgroundColor: ledHex(i - 1) || 'transparent' }"
              :title="`led${i - 1}: ${ledHex(i - 1) || 'unset'}`"></span>
      </div>
    </div>

    <!-- Per-servo target table (output-only; shows last routed −1..+1). -->
    <div class="space-y-1">
      <div v-for="id in servoIds" :key="id"
           class="flex items-center justify-between text-sm font-mono py-1 border-b border-line/50 last:border-b-0">
        <span class="text-fg-muted">{{ id }}</span>
        <div class="flex items-center gap-3">
          <span :class="chValue(id) != null ? 'text-amber-300' : 'text-fg-faint'">{{ fmtServo(id) }}</span>
          <span class="text-xs text-fg-faint w-20 text-right">{{ fmtAge(id) }}</span>
        </div>
      </div>
    </div>
  </div>
</template>
