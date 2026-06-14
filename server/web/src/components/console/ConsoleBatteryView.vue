<script setup>
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import ConsoleScreen from './ConsoleScreen.vue'

// LED-styled battery summary. Unlike the dashboard BMSMonitor widget
// (which is route-driven so an operator can wire arbitrary channels
// into it), Console views bind directly to a node + peripheral —
// they're fixed-purpose panel meters, not configurable surfaces. The
// channel IDs (pack_voltage / current / soc / protection / fet_status
// / temp_1 / temp_2) match the JBD driver's `state_cb` emissions in
// server/saint_server/host_peripherals/jbd_bms.py.

const props = defineProps({
  nodeId: { type: String, required: true },
  peripheralId: { type: String, required: true },
})

const ws = useWsStore()

const topic = computed(() => `pin_state/${props.nodeId}`)
let subscribed = false

async function sub () {
  try { await ws.subscribe([topic.value], 5); subscribed = true } catch (_) {}
}
async function unsub () {
  if (!subscribed) return
  try { await ws.unsubscribe([topic.value]) } catch (_) {}
  subscribed = false
}

// Local mirror of the latest pin_state frame for our nodeId. Driven
// by the 'state' WS event the store fires on every topic broadcast.
// We can't rely on a computed that reads `ws.topics.get(topic)`
// reactively — the path from `reactive(new Map())` through Pinia's
// setup-store proxy doesn't always re-trigger downstream computeds
// on .set() (Pinia tracks state-as-object, not Map internals across
// every code path). A ref bumped explicitly is bulletproof and
// matches the pattern Fas100Monitor / RoboClawMonitor use.
const latestFrame = ref(null)

function snapshotFromStore () {
  const f = ws.topics.get(topic.value)
  latestFrame.value = (f && Array.isArray(f.channels)) ? f : null
}
function onStateFrame (msg) {
  if (!msg || msg.node !== topic.value) return
  const d = msg.data
  latestFrame.value = (d && Array.isArray(d.channels)) ? d : null
}

let resubscribe = null
onMounted(async () => {
  await sub()
  snapshotFromStore()              // initial paint if a frame already cached
  ws.on('state', onStateFrame)
  resubscribe = () => { sub() }
  ws.on('ready', resubscribe)
})
onUnmounted(async () => {
  ws.off('state', onStateFrame)
  if (resubscribe) ws.off('ready', resubscribe)
  await unsub()
})
watch(topic, async (next, prev) => {
  if (prev) {
    try { await ws.unsubscribe([prev]) } catch (_) {}
    subscribed = false
  }
  latestFrame.value = null
  await sub()
  snapshotFromStore()
})

function chan (id) {
  const frame = latestFrame.value
  if (!frame || !Array.isArray(frame.channels)) return null
  for (const c of frame.channels) {
    if (c.peripheral_id === props.peripheralId && c.channel_id === id && typeof c.value === 'number') {
      return c.value
    }
  }
  return null
}

const soc      = computed(() => chan('soc'))
const voltage  = computed(() => chan('pack_voltage'))
const current  = computed(() => chan('current'))
const tempC    = computed(() => {
  // Show the hotter of the two NTCs — that's what an operator cares
  // about for thermal headroom; the cooler one is rarely the limiting
  // sensor.
  const a = chan('temp_1')
  const b = chan('temp_2')
  if (a == null && b == null) return null
  if (a == null) return b
  if (b == null) return a
  return Math.max(a, b)
})
const protBits  = computed(() => chan('protection'))
const fetStatus = computed(() => chan('fet_status'))

// Per-cell stats. Same channel-naming convention as the JBD driver
// and the dashboard BMSCard: cell_count is the series count, and
// each series cell publishes as cell_NN (zero-padded). Cells inside
// the reported count are always shown — a real 0 V cell is operator-
// actionable, not noise — only cells beyond the count are hidden.
const cellCount = computed(() => {
  const reported = chan('cell_count')
  if (reported && reported > 0) return Math.round(reported)
  // Fallback during the first poll cycle: detect highest non-zero
  // cell index so the strip renders something rather than nothing.
  let highest = 0
  for (let i = 1; i <= 16; i++) {
    const v = chan(`cell_${String(i).padStart(2, '0')}`)
    if (v !== null && v !== undefined && v > 0.5) highest = i
  }
  return highest
})
const cells = computed(() => {
  const n = cellCount.value
  const out = []
  for (let i = 1; i <= n; i++) {
    const v = chan(`cell_${String(i).padStart(2, '0')}`)
    out.push({ idx: i, v: v ?? 0 })
  }
  return out
})
const cellMin    = computed(() => cells.value.reduce((m, c) => c.v < m ? c.v : m,  Infinity))
const cellMax    = computed(() => cells.value.reduce((m, c) => c.v > m ? c.v : m, -Infinity))
const cellSpread = computed(() => {
  if (!cells.value.length || !isFinite(cellMin.value) || !isFinite(cellMax.value)) return 0
  return cellMax.value - cellMin.value
})
// 50 mV imbalance trips the color-coded outlier highlight — matches
// the threshold used in the dashboard BMSCard / BMSMonitor.
const IMBALANCE_THRESHOLD_V = 0.050

// Bar height %: scale within [min, max] of the current pack so a tight
// pack (all cells within a few mV) still shows differences. Reserves
// the [15%, 100%] band so the min cell's bar still renders a visible
// stub in its color.
function cellBarHeightPct (v) {
  if (!cells.value.length) return 0
  const lo = cellMin.value
  const hi = cellMax.value
  if (!isFinite(lo) || !isFinite(hi) || hi <= lo) return 60
  return Math.round(15 + ((v - lo) / (hi - lo)) * 85)
}
function cellFillClass (v) {
  if (cellSpread.value < IMBALANCE_THRESHOLD_V) return 'led-fill-green'
  if (v === cellMax.value) return 'led-fill-amber'   // highest — possibly overcharged
  if (v === cellMin.value) return 'led-fill'         // lowest red — the outlier the operator cares about
  return 'led-fill-green'
}

const chargeOn    = computed(() => ((fetStatus.value | 0) & 0x01) === 0x01)
const dischargeOn = computed(() => ((fetStatus.value | 0) & 0x02) === 0x02)

// Mirror the protection-bit table from BMSMonitor.vue / BMSCard.vue —
// keep these three in sync with jbd_bms.py.
const PROTECTION_LABELS = [
  'CELL OVERVOLTAGE',
  'CELL UNDERVOLTAGE',
  'PACK OVERVOLTAGE',
  'PACK UNDERVOLTAGE',
  'CHARGE OVERTEMP',
  'CHARGE UNDERTEMP',
  'DISCHARGE OVERTEMP',
  'DISCHARGE UNDERTEMP',
  'CHARGE OVERCURRENT',
  'DISCHARGE OVERCURRENT',
  'SHORT CIRCUIT',
  'IC ERROR',
  'MOS LOCK',
]
const faults = computed(() => {
  const b = (protBits.value | 0)
  const out = []
  for (let i = 0; i < 16; i++) {
    if (b & (1 << i)) out.push(PROTECTION_LABELS[i] || `FAULT ${i}`)
  }
  return out
})

// Single-glance status drives header subtitle color.
const status = computed(() => {
  if (faults.value.length) return { label: 'FAULT',    klass: 'led-text' }
  if (!chargeOn.value || !dischargeOn.value) return { label: 'ISOLATED', klass: 'led-text-amber' }
  if (soc.value == null) return { label: 'NO DATA',   klass: 'led-text-dim' }
  return { label: 'ONLINE', klass: 'led-text-green' }
})

// SOC color — visually responds to remaining capacity. Plain
// (no-glow) variants here; the headline number is big enough to
// dominate the screen on size alone and the glow makes the digits
// blurry from a few feet away. Segments below still glow because
// the bar IS a row of physical-looking LEDs.
const socClass = computed(() => {
  const s = soc.value
  if (s == null) return 'led-text-dim'
  if (s < 20)    return 'led-text-plain'
  if (s < 50)    return 'led-text-amber-plain'
  return 'led-text-green-plain'
})
const segColor = computed(() => {
  const s = soc.value
  if (s == null) return { fill: 'led-fill-dim',   outline: 'led-outline-dim'  }
  if (s < 20)    return { fill: 'led-fill',       outline: 'led-outline'      }
  if (s < 50)    return { fill: 'led-fill-amber', outline: 'led-outline-amber' }
  return { fill: 'led-fill-green', outline: 'led-outline-green' }
})

const SEGMENTS = 20
const litSegments = computed(() => {
  const s = soc.value
  if (s == null) return 0
  return Math.round((Math.max(0, Math.min(100, s)) / 100) * SEGMENTS)
})

function fmt (v, places = 1, unit = '') {
  if (v == null || !isFinite(v)) return '—'
  return `${v.toFixed(places)}${unit}`
}
</script>

<template>
  <ConsoleScreen :title="`BATTERY · ${peripheralId}`" :subtitle="status.label">
    <!-- Detail-view sizing matches the overview's 1440×2560 kiosk
         target: large headline, generous tile padding, tall cell
         strip. flex-1 on the cell-strip section lets it absorb the
         remaining vertical space so the screen fills without a
         scrollbar regardless of pack series count. -->
    <div class="flex flex-col gap-10 h-full">
      <!-- Headline SOC. Centered, oversized — readable across the
           room on size alone, no glow needed. -->
      <div class="flex flex-col items-center pt-6">
        <div class="text-4xl led-text-plain mb-8 tracking-[0.2em]">STATE OF CHARGE</div>
        <div :class="['text-[18rem] font-bold tabular-nums leading-none', socClass]">
          {{ fmt(soc, 0) }}<span class="text-8xl ml-4">%</span>
        </div>

        <!-- Segmented LED-bar — flex-1 segments so the bar always
             spans the available width regardless of how many
             segments the pack defines. -->
        <div class="mt-12 flex gap-2 h-20 w-full px-6">
          <div
            v-for="i in SEGMENTS"
            :key="i"
            class="flex-1 h-full min-w-0"
            :class="i <= litSegments ? segColor.fill : 'led-outline-dim'"
          />
        </div>
      </div>

      <!-- Secondary numerics: voltage, current, temp — three big tiles. -->
      <div class="grid grid-cols-3 gap-6">
        <div class="led-outline-dim p-6">
          <div class="text-2xl led-text-plain mb-3 tracking-[0.18em]">PACK V</div>
          <div class="led-text-plain text-6xl tabular-nums">{{ fmt(voltage, 2, 'V') }}</div>
        </div>
        <div class="led-outline-dim p-6">
          <div class="text-2xl led-text-plain mb-3 tracking-[0.18em]">CURRENT</div>
          <div
            :class="['text-6xl tabular-nums', (current ?? 0) >= 0 ? 'led-text-green-plain' : 'led-text-amber-plain']"
          >{{ fmt(current, 2, 'A') }}</div>
        </div>
        <div class="led-outline-dim p-6">
          <div class="text-2xl led-text-plain mb-3 tracking-[0.18em]">TEMP</div>
          <div :class="['text-6xl tabular-nums', (tempC ?? 0) >= 50 ? 'led-text-amber-plain' : 'led-text-plain']">
            {{ fmt(tempC, 1, '°') }}
          </div>
        </div>
      </div>

      <!-- FET status row — two large chips, lit when the FET is on. -->
      <div class="grid grid-cols-2 gap-6 text-3xl tracking-[0.2em] text-center">
        <div :class="['py-6', chargeOn ? 'led-fill-green' : 'led-outline-dim']"
             :style="chargeOn ? 'color: var(--led-black)' : 'color: var(--led-red-dim)'">
          CHG {{ chargeOn ? 'ON' : 'OFF' }}
        </div>
        <div :class="['py-6', dischargeOn ? 'led-fill-green' : 'led-outline-dim']"
             :style="dischargeOn ? 'color: var(--led-black)' : 'color: var(--led-red-dim)'">
          DSG {{ dischargeOn ? 'ON' : 'OFF' }}
        </div>
      </div>

      <!-- Cell strip — vertical LED bars, one per series cell. Same
           min/max highlight rules as the overview's mini-strip. The
           strip flex-1's so it absorbs the rest of the screen
           height. -->
      <div v-if="cells.length" class="flex flex-col gap-4 flex-1 min-h-0">
        <div class="flex items-baseline justify-between text-2xl tracking-[0.16em]">
          <span class="led-text-plain">CELLS · {{ cells.length }}S</span>
          <span class="led-text-plain tabular-nums">
            MIN <span class="led-text-plain">{{ fmt(cellMin, 3) }}V</span>
            · MAX <span class="led-text-plain">{{ fmt(cellMax, 3) }}V</span>
            · Δ <span :class="cellSpread > IMBALANCE_THRESHOLD_V ? 'led-text-amber-plain' : 'led-text-plain'">
              {{ fmt(cellSpread * 1000, 0) }}mV
            </span>
          </span>
        </div>
        <div class="flex items-end gap-3 flex-1 min-h-0">
          <div v-for="c in cells" :key="c.idx"
               class="flex-1 flex flex-col items-stretch justify-end h-full min-w-0">
            <div class="led-outline-dim relative h-full flex flex-col-reverse">
              <div :class="cellFillClass(c.v)"
                   :style="{ height: cellBarHeightPct(c.v) + '%' }"
                   class="transition-[height] duration-150"/>
            </div>
            <div class="text-2xl text-center led-text-plain mt-2 tabular-nums">
              {{ fmt(c.v, 2) }}
            </div>
          </div>
        </div>
      </div>

      <!-- Fault list — only rendered when something is asserted; large
           and glowing red so it dominates the screen when present. -->
      <div v-if="faults.length" class="led-outline p-6">
        <div class="led-text text-3xl mb-3 tracking-[0.2em]">► BMS FAULTS</div>
        <ul class="space-y-2 text-2xl led-text">
          <li v-for="f in faults" :key="f">▌ {{ f }}</li>
        </ul>
      </div>
    </div>
  </ConsoleScreen>
</template>
