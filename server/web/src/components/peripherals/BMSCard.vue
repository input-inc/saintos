<script setup>
import { computed } from 'vue'
import { useDisplayStore } from '@/stores/display'

// Compact BMS card for the Live tab. Renders a pathfinder_bms
// peripheral's pack-level metrics, per-cell voltages with imbalance
// highlight, NTC temps, FET state, and any asserted protection bits.
//
// All data comes from the per-channel `values` map the Live tab
// builds out of the pin_state/<node_id> WS topic — no extra
// subscriptions or routing. Cell visibility is driven by the BMS-
// reported `cell_count` channel: cells inside that count are always
// shown (even at 0 V — a real 0 V reading means a dropped sensor or
// a fully-failed cell, exactly what the operator wants to see); only
// cells beyond `cell_count` are hidden as unused slots.

const props = defineProps({
  peripheral: { type: Object, required: true },
  channels:   { type: Object, default: () => ({}) },   // { channel_id: { value, last_updated } }
  sparkSamples: { type: Function, default: () => [] },  // (channelId) => samples
})

const display = useDisplayStore()

function ch (id) {
  const c = props.channels?.[id]
  if (!c) return null
  const v = c.value
  if (v === null || v === undefined || typeof v !== 'number') return null
  return v
}

// JBD protection bit -> human label. Mirrors the server-side decoder.
const PROTECTION_LABELS = [
  'cell overvoltage',
  'cell undervoltage',
  'pack overvoltage',
  'pack undervoltage',
  'charge overtemp',
  'charge undertemp',
  'discharge overtemp',
  'discharge undertemp',
  'charge overcurrent',
  'discharge overcurrent',
  'short circuit',
  'front-end IC error',
  'MOS software lock',
]
function decodeProtection (bits) {
  const b = bits | 0
  const out = []
  for (let i = 0; i < 16; i++) {
    if (b & (1 << i)) out.push(PROTECTION_LABELS[i] || `fault bit ${i}`)
  }
  return out
}

const packVoltage = computed(() => ch('pack_voltage'))
const current     = computed(() => ch('current'))
const soc         = computed(() => ch('soc'))
const remainCap   = computed(() => ch('remain_cap'))
const cycles      = computed(() => ch('cycles'))
const protBits    = computed(() => ch('protection'))
const fetStatus   = computed(() => ch('fet_status'))
const temp1       = computed(() => ch('temp_1'))
const temp2       = computed(() => ch('temp_2'))
// `cell_count` is the BMS-reported series count. Until the first
// basic-info poll lands, fall back to detecting non-zero cells so
// we don't render a blank Cells block during the first second.
const cellCount   = computed(() => {
  const reported = ch('cell_count')
  if (reported && reported > 0) return Math.round(reported)
  let highest = 0
  for (let i = 1; i <= 16; i++) {
    const v = ch(`cell_${String(i).padStart(2, '0')}`)
    if (v !== null && v !== undefined && v > 0.5) highest = i
  }
  return highest
})

// Cells inside the reported series count are always shown (a real
// 0 V reading is operator-actionable, not noise). Cells beyond the
// count are hidden as unused slots.
const cells = computed(() => {
  const n = cellCount.value
  const out = []
  for (let i = 1; i <= n; i++) {
    const v = ch(`cell_${String(i).padStart(2, '0')}`)
    out.push({ idx: i, v: v ?? 0 })
  }
  return out
})

const cellMin = computed(() => cells.value.reduce((m, c) => c.v < m ? c.v : m, Infinity))
const cellMax = computed(() => cells.value.reduce((m, c) => c.v > m ? c.v : m, -Infinity))
const cellAvg = computed(() => {
  if (!cells.value.length) return 0
  const sum = cells.value.reduce((s, c) => s + c.v, 0)
  return sum / cells.value.length
})
const cellSpread = computed(() => {
  if (!cells.value.length) return 0
  return cellMax.value - cellMin.value
})

// Imbalance threshold for highlighting — > 50 mV is the rule of
// thumb in lithium-pack BMS docs; tweakable as we learn more.
const IMBALANCE_THRESHOLD_V = 0.050

// Cell-voltage bar: scale within [min, max] of the current pack so
// imbalance is visually obvious even on packs that operate at very
// different cell voltages (LFP at 3.2-3.4 vs Li-ion at 3.6-4.2).
// We reserve the [10%, 90%] band so the lowest cell's bar still
// renders a visible sliver in its color (otherwise a `v === min`
// cell maps to 0% width and the bar disappears — the operator can
// see the readout but not which cell is the outlier).
function cellBarPct (v) {
  const lo = cellMin.value
  const hi = cellMax.value
  if (!isFinite(lo) || !isFinite(hi) || hi <= lo) return 50
  const ratio = (v - lo) / (hi - lo)
  return Math.round(10 + ratio * 80)
}
function cellColor (v) {
  if (cellSpread.value < IMBALANCE_THRESHOLD_V) return 'bg-cyan-500'
  if (v === cellMax.value) return 'bg-amber-400'
  if (v === cellMin.value) return 'bg-rose-500'
  return 'bg-cyan-500'
}

const faults = computed(() => decodeProtection(protBits.value || 0))
const chargeOn = computed(() => ((fetStatus.value | 0) & 0x01) === 0x01)
const dischargeOn = computed(() => ((fetStatus.value | 0) & 0x02) === 0x02)

function fmt (v, places = 2, unit = '') {
  if (v === null || v === undefined || !isFinite(v)) return '—'
  return `${v.toFixed(places)}${unit ? ' ' + unit : ''}`
}

// NTC readings flow through the user's display preference so a
// server-wide °F setting carries through the BMS card. The store's
// formatTemperature also handles the —/missing case.
function fmtTemp (celsius) {
  return display.formatTemperature(celsius)
}

// SOC bar — gradient color across red/amber/green so a 12% SOC
// pack is visually distinct from a 90% SOC pack.
function socColor (s) {
  if (s === null || s === undefined) return 'bg-fg-faint'
  if (s < 20) return 'bg-rose-500'
  if (s < 50) return 'bg-amber-400'
  return 'bg-emerald-500'
}
</script>

<template>
  <div class="card space-y-3">
    <header class="flex items-center justify-between">
      <div>
        <h4 class="text-base font-semibold text-fg-strong">{{ peripheral.label || peripheral.id }}</h4>
        <p class="text-xs text-fg-faint">Pathfinder BMS · <span class="font-mono">{{ peripheral.id }}</span></p>
      </div>
      <div class="flex items-center gap-1 text-xs">
        <span :class="['px-2 py-0.5 rounded-full', chargeOn ? 'bg-emerald-500/20 text-emerald-300' : 'bg-surface text-fg-faint']">CHG {{ chargeOn ? 'ON' : 'OFF' }}</span>
        <span :class="['px-2 py-0.5 rounded-full', dischargeOn ? 'bg-emerald-500/20 text-emerald-300' : 'bg-surface text-fg-faint']">DSG {{ dischargeOn ? 'ON' : 'OFF' }}</span>
      </div>
    </header>

    <!-- SOC bar + pack metrics row -->
    <div class="space-y-1">
      <div class="flex justify-between text-xs text-fg-muted">
        <span>State of charge</span>
        <span class="font-mono text-fg-strong">{{ fmt(soc, 0, '%') }}</span>
      </div>
      <div class="h-2 w-full rounded-full bg-surface overflow-hidden">
        <div :class="[socColor(soc), 'h-2 transition-all']"
             :style="{ width: `${Math.max(0, Math.min(100, soc ?? 0))}%` }"/>
      </div>
    </div>

    <div class="grid grid-cols-3 gap-2 text-sm">
      <div class="bg-surface/40 rounded p-2">
        <div class="text-[10px] uppercase tracking-wide text-fg-faint">Voltage</div>
        <div class="font-mono text-cyan-300 text-lg">{{ fmt(packVoltage, 2, 'V') }}</div>
      </div>
      <div class="bg-surface/40 rounded p-2">
        <div class="text-[10px] uppercase tracking-wide text-fg-faint">Current</div>
        <div :class="['font-mono text-lg', (current ?? 0) >= 0 ? 'text-emerald-300' : 'text-amber-300']">{{ fmt(current, 2, 'A') }}</div>
      </div>
      <div class="bg-surface/40 rounded p-2">
        <div class="text-[10px] uppercase tracking-wide text-fg-faint">Remaining</div>
        <div class="font-mono text-fg-strong text-lg">{{ fmt(remainCap, 1, 'Ah') }}</div>
      </div>
    </div>

    <!-- Temps + cycle count row -->
    <div class="grid grid-cols-3 gap-2 text-xs font-mono">
      <div class="flex justify-between bg-surface/30 rounded px-2 py-1">
        <span class="text-fg-faint">Temp 1</span><span class="text-rose-300">{{ fmtTemp(temp1) }}</span>
      </div>
      <div class="flex justify-between bg-surface/30 rounded px-2 py-1">
        <span class="text-fg-faint">Temp 2</span><span class="text-rose-300">{{ fmtTemp(temp2) }}</span>
      </div>
      <div class="flex justify-between bg-surface/30 rounded px-2 py-1">
        <span class="text-fg-faint">Cycles</span><span class="text-fg-strong">{{ fmt(cycles, 0) }}</span>
      </div>
    </div>

    <!-- Cell voltage bars: only show when we have at least 1 -->
    <div v-if="cells.length" class="pt-2 border-t border-line/50 space-y-2">
      <div class="flex items-center justify-between text-xs">
        <span class="text-fg-muted">Cells ({{ cells.length }}S)</span>
        <span class="font-mono text-fg-faint">
          min {{ fmt(cellMin, 3, 'V') }} ·
          max {{ fmt(cellMax, 3, 'V') }} ·
          <span :class="cellSpread > IMBALANCE_THRESHOLD_V ? 'text-amber-300' : 'text-fg-faint'">
            Δ {{ fmt(cellSpread * 1000, 0, 'mV') }}
          </span>
        </span>
      </div>
      <div class="space-y-1">
        <div v-for="c in cells" :key="c.idx" class="flex items-center gap-2 text-xs font-mono">
          <span class="w-8 text-fg-faint shrink-0">#{{ c.idx }}</span>
          <div class="flex-1 h-1.5 rounded-full bg-surface overflow-hidden">
            <div :class="[cellColor(c.v), 'h-1.5 transition-all']"
                 :style="{ width: `${cellBarPct(c.v)}%` }"/>
          </div>
          <span class="w-16 text-right text-cyan-300">{{ fmt(c.v, 3, 'V') }}</span>
        </div>
      </div>
    </div>

    <!-- Faults: only show when something is asserted -->
    <div v-if="faults.length"
         class="rounded border border-red-500/40 bg-red-500/10 px-2 py-1.5 text-xs">
      <div class="flex items-center gap-1 text-red-300 font-medium mb-0.5">
        <span class="material-icons icon-xs">warning</span>
        BMS faults asserted (0x{{ ((protBits | 0) >>> 0).toString(16).padStart(4, '0') }})
      </div>
      <ul class="list-disc list-inside text-red-200 ml-1">
        <li v-for="f in faults" :key="f">{{ f }}</li>
      </ul>
    </div>
  </div>
</template>
