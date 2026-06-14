<script setup>
import { computed, onMounted, onUnmounted, reactive, watch } from 'vue'
import { useRouter } from 'vue-router'
import { useWsStore } from '@/stores/ws'
import ConsoleScreen from './ConsoleScreen.vue'

// Top-level battery dashboard for the Console kiosk. Aggregates N
// independently-adopted BMS peripherals into one screen so an
// operator can read the system's overall charge state at a glance:
//
//   * Combined headline = MIN SOC across packs (the weakest pack is
//     what limits the system). AVG and MAX shown as smaller stats so
//     the operator can also see the spread.
//   * One tile per pack, each showing its own SOC bar + a compact
//     cell-imbalance strip + a fault badge. Tap a tile to drill into
//     ConsoleBatteryView for that pack.
//   * Combined fault list at the bottom — each entry prefixed with
//     the pack label so the operator knows where to look.
//
// Data flow mirrors ConsoleBatteryView: subscribe to each pack's
// pin_state/<nodeId> topic and ingest channels via the 'state' event.
// One subscription per distinct nodeId (multiple packs on the same
// node share one frame stream).

const props = defineProps({
  // [{ nodeId, peripheralId, label }, ...]. Required at least one.
  packs: { type: Array, required: true },
})

const ws = useWsStore()
const router = useRouter()

const packKey = (p) => `${p.nodeId}/${p.peripheralId}`

// Latest pin_state frame per nodeId. Multiple packs on the same node
// will all read from the same frame; we just filter channels by
// peripheral_id when extracting values.
const frames = reactive({})       // nodeId → frame
const activeNodes = computed(() => [...new Set(props.packs.map(p => p.nodeId))])

function snapshotAll () {
  for (const n of activeNodes.value) {
    const f = ws.topics.get(`pin_state/${n}`)
    if (f && Array.isArray(f.channels)) frames[n] = f
  }
}
function onStateFrame (msg) {
  if (!msg?.node?.startsWith('pin_state/')) return
  const nodeId = msg.node.slice('pin_state/'.length)
  if (!activeNodes.value.includes(nodeId)) return
  const d = msg.data
  if (d && Array.isArray(d.channels)) frames[nodeId] = d
}

let subscribed = new Set()
async function sub () {
  const want = new Set(activeNodes.value.map(n => `pin_state/${n}`))
  const toAdd = [...want].filter(t => !subscribed.has(t))
  const toDrop = [...subscribed].filter(t => !want.has(t))
  if (toAdd.length) {
    try { await ws.subscribe(toAdd, 5) } catch (_) {}
    for (const t of toAdd) subscribed.add(t)
  }
  if (toDrop.length) {
    try { await ws.unsubscribe(toDrop) } catch (_) {}
    for (const t of toDrop) subscribed.delete(t)
  }
}

let resubscribe = null
onMounted(async () => {
  await sub()
  snapshotAll()
  ws.on('state', onStateFrame)
  resubscribe = () => { sub() }
  ws.on('ready', resubscribe)
})
onUnmounted(async () => {
  ws.off('state', onStateFrame)
  if (resubscribe) ws.off('ready', resubscribe)
  if (subscribed.size) {
    try { await ws.unsubscribe([...subscribed]) } catch (_) {}
    subscribed.clear()
  }
})
watch(activeNodes, () => { sub() })

function chanFor (pack, channelId) {
  const f = frames[pack.nodeId]
  if (!f || !Array.isArray(f.channels)) return null
  for (const c of f.channels) {
    if (c.peripheral_id === pack.peripheralId && c.channel_id === channelId && typeof c.value === 'number') {
      return c.value
    }
  }
  return null
}
function cellsFor (pack) {
  const reported = chanFor(pack, 'cell_count')
  const n = (reported && reported > 0) ? Math.round(reported) : 0
  const out = []
  for (let i = 1; i <= n; i++) {
    const v = chanFor(pack, `cell_${String(i).padStart(2, '0')}`)
    out.push(v ?? 0)
  }
  return out
}

// Per-pack derived state.
const PROTECTION_LABELS = [
  'CELL OVERVOLTAGE', 'CELL UNDERVOLTAGE', 'PACK OVERVOLTAGE', 'PACK UNDERVOLTAGE',
  'CHARGE OVERTEMP', 'CHARGE UNDERTEMP', 'DISCHARGE OVERTEMP', 'DISCHARGE UNDERTEMP',
  'CHARGE OVERCURRENT', 'DISCHARGE OVERCURRENT', 'SHORT CIRCUIT', 'IC ERROR', 'MOS LOCK',
]
const IMBALANCE_THRESHOLD_V = 0.050

const packStates = computed(() => props.packs.map(p => {
  const cells = cellsFor(p)
  const fets = chanFor(p, 'fet_status') | 0
  const protBits = chanFor(p, 'protection') | 0
  const faults = []
  for (let i = 0; i < 16; i++) {
    if (protBits & (1 << i)) faults.push(PROTECTION_LABELS[i] || `FAULT ${i}`)
  }
  const min = cells.length ? Math.min(...cells) : null
  const max = cells.length ? Math.max(...cells) : null
  const spread = (min != null && max != null) ? max - min : 0
  return {
    pack: p,
    key: packKey(p),
    soc: chanFor(p, 'soc'),
    voltage: chanFor(p, 'pack_voltage'),
    current: chanFor(p, 'current'),
    cells, cellMin: min, cellMax: max, cellSpread: spread,
    chargeOn: (fets & 0x01) === 0x01,
    dischargeOn: (fets & 0x02) === 0x02,
    faults,
  }
}))

// Aggregate stats across packs that have SOC data.
const withSoc = computed(() => packStates.value.filter(s => typeof s.soc === 'number'))
const minSoc  = computed(() => withSoc.value.length ? Math.min(...withSoc.value.map(s => s.soc)) : null)
const maxSoc  = computed(() => withSoc.value.length ? Math.max(...withSoc.value.map(s => s.soc)) : null)
const avgSoc  = computed(() => {
  if (!withSoc.value.length) return null
  return withSoc.value.reduce((a, s) => a + s.soc, 0) / withSoc.value.length
})

const allFaults = computed(() => {
  const out = []
  for (const s of packStates.value) {
    for (const f of s.faults) out.push({ pack: s.pack.label || s.pack.peripheralId, fault: f })
  }
  return out
})

const headlineStatus = computed(() => {
  if (allFaults.value.length) return { label: 'FAULT',    klass: 'led-text' }
  // Any pack isolated?
  if (packStates.value.some(s => !s.chargeOn || !s.dischargeOn)) {
    return { label: 'ISOLATED', klass: 'led-text-amber' }
  }
  if (!withSoc.value.length) return { label: 'NO DATA', klass: 'led-text-dim' }
  return { label: 'ONLINE', klass: 'led-text-green' }
})

// Color helpers — match the per-pack-view conventions so visual
// language stays consistent between overview and detail.
function socClassFor (soc) {
  if (soc == null) return 'led-text-dim'
  if (soc < 20)    return 'led-text-plain'
  if (soc < 50)    return 'led-text-amber-plain'
  return 'led-text-green-plain'
}
function socFillFor (soc) {
  if (soc == null) return 'led-fill-dim'
  if (soc < 20)    return 'led-fill'
  if (soc < 50)    return 'led-fill-amber'
  return 'led-fill-green'
}
function cellFillFor (state, v) {
  if (!state.cellMin || !state.cellMax || state.cellSpread < IMBALANCE_THRESHOLD_V) return 'led-fill-green'
  if (v === state.cellMax) return 'led-fill-amber'
  if (v === state.cellMin) return 'led-fill'
  return 'led-fill-green'
}
function cellHeightPct (state, v) {
  if (!state.cells.length) return 0
  const lo = state.cellMin, hi = state.cellMax
  if (lo === hi) return 60
  return Math.round(15 + ((v - lo) / (hi - lo)) * 85)
}

// 20-segment LED bar — same constant as the per-pack detail view so
// the visual language stays consistent. flex-1 lets each segment grow
// to fit whatever width the tile gets, so a 2-pack overview gets fat
// readable segments while a 4-pack overview gets tight ones.
const SEGMENTS = 20
function litSegmentsFor (soc) {
  if (soc == null) return 0
  return Math.round((Math.max(0, Math.min(100, soc)) / 100) * SEGMENTS)
}

function fmt (v, places = 0, unit = '') {
  if (v == null || !isFinite(v)) return '—'
  return `${v.toFixed(places)}${unit}`
}

function openPack (pack) {
  router.push({
    name: 'console-battery',
    params: { nodeId: pack.nodeId, peripheralId: pack.peripheralId },
  })
}
</script>

<template>
  <ConsoleScreen :title="`BATTERIES · ${packs.length} PACK${packs.length === 1 ? '' : 'S'}`"
                 :subtitle="headlineStatus.label">

    <!-- Sized for the 1440×2560 portrait kiosk. The outer column uses
         flex-1 on the pack-card grid so 2 cards expand vertically to
         fill whatever height is left after the headline + (optional)
         fault list. Text sizes are tuned for read-across-the-room
         viewing on a 1440-wide panel; the preview in Histoire is
         transform-scaled to the same ratio so what you see there
         matches what lands on the kiosk. -->
    <div class="flex flex-col gap-12 h-full">
      <!-- Headline aggregate. The big number is AVG SOC — what an
           operator naturally reads as "system fuel level". But the
           COLOR follows MIN SOC so a single dragging pack lights the
           whole headline red even when the average looks fine; the
           color disagreement is the signal. The segmented bar shows
           MIN's fill + color to make the weakest-pack story explicit
           visually, with the actual MIN value spelled out below. -->
      <div class="flex flex-col items-center pt-6">
        <div class="text-4xl led-text-plain mb-8 tracking-[0.2em]">STATE OF CHARGE</div>
        <div :class="['text-[18rem] font-bold tabular-nums leading-none', socClassFor(minSoc)]">
          {{ fmt(avgSoc, 0) }}<span class="text-8xl ml-4">%</span>
        </div>
        <div class="mt-12 flex gap-2 h-20 w-full px-6">
          <div v-for="i in SEGMENTS" :key="i"
               class="flex-1 h-full min-w-0"
               :class="i <= litSegmentsFor(minSoc) ? socFillFor(minSoc) : 'led-outline-dim'" />
        </div>
        <div class="mt-8 flex gap-20 text-3xl led-text-plain tracking-[0.2em]">
          <span>MIN {{ fmt(minSoc, 0, '%') }}</span>
          <span>MAX {{ fmt(maxSoc, 0, '%') }}</span>
        </div>
      </div>

      <!-- Per-pack cards — landscape orientation on a 1440×2560
           portrait kiosk: pack info on the left, SOC % on the far
           right, individual cells rendered as battery-shaped icons
           spread horizontally along the bottom edge. Two-pack rigs
           stack vertically (full width each); 3-4 packs fall back
           to a 2-column grid. Each card is tappable → drills into
           ConsoleBatteryView. -->
      <!-- Pack cards. flex-1 on the grid means with 2 packs each card
           expands to half the remaining vertical space, so on a
           1440×2560 portrait kiosk a 2-pack rig fills the screen
           without scrolling. Each card uses its own internal flex
           so the cell row (bottom) gets the leftover height after
           the top row, making the battery-icon strip generously
           tall on a 2-pack rig and compressing gracefully when 3-4
           packs share the space. -->
      <div class="grid gap-6 flex-1 min-h-0"
           :class="packs.length <= 2 ? 'grid-cols-1' : 'grid-cols-2'">
        <button
          v-for="s in packStates" :key="s.key"
          class="led-outline-dim p-8 text-left cursor-pointer transition-colors hover:bg-[var(--led-red-dim)]/30 flex flex-col gap-6 min-h-0"
          :class="s.faults.length ? 'led-outline' : ''"
          @click="openPack(s.pack)"
        >
          <!-- Top row — label + FETs on the left, SOC % on the far
               right. Per-pack SOC color follows that pack's own
               SOC (independent of the headline's weakest-pack color
               so the operator can read both at once). -->
          <div class="flex items-start justify-between gap-8">
            <div class="flex flex-col gap-4 min-w-0">
              <span class="text-5xl tracking-[0.18em] led-text-plain truncate">
                {{ s.pack.label || s.pack.peripheralId }}
              </span>
              <div class="flex items-center gap-8 text-3xl tracking-[0.18em]">
                <span :class="s.chargeOn    ? 'led-text-green-plain' : 'led-text-dim'">CHG</span>
                <span :class="s.dischargeOn ? 'led-text-green-plain' : 'led-text-dim'">DSG</span>
              </div>
              <span v-if="s.faults.length" class="led-text text-3xl tracking-[0.18em]">
                ⚠ {{ s.faults.length }} FAULT{{ s.faults.length === 1 ? '' : 'S' }}
              </span>
            </div>
            <div :class="['text-[12rem] font-bold tabular-nums leading-none shrink-0', socClassFor(s.soc)]">
              {{ fmt(s.soc, 0) }}<span class="text-6xl ml-3">%</span>
            </div>
          </div>

          <!-- Bottom — individual cells as battery icons. The strip
               takes the leftover height (flex-1) so on a tall card
               (2-pack layout) the batteries are generously large;
               on a 4-pack rig they shrink. Each cell: narrow
               terminal nub above an outlined body that fills
               upward proportional to where this cell sits within
               [min, max] of the pack. Lowest red, highest amber,
               rest green when imbalanced; all green when under
               the 50 mV threshold. -->
          <div v-if="s.cells.length" class="flex flex-col gap-4 flex-1 min-h-0">
            <div class="flex items-center justify-between text-2xl tracking-[0.16em] led-text-plain">
              <span>CELLS · {{ s.cells.length }}S</span>
              <span v-if="s.cellSpread > 0" class="tabular-nums"
                    :class="s.cellSpread > IMBALANCE_THRESHOLD_V ? 'led-text-amber-plain' : 'led-text-plain'">
                Δ {{ fmt(s.cellSpread * 1000, 0) }}mV
              </span>
            </div>
            <div class="flex items-end gap-3 flex-1 min-h-0">
              <div v-for="(v, i) in s.cells" :key="i"
                   class="flex-1 h-full flex flex-col items-center min-w-0">
                <!-- Battery terminal nub — narrower than the body so
                     the icon reads as a battery silhouette. Tall
                     enough (16 px ≈ 0.75 mm at 533 PPI) to read as
                     a deliberate terminal on the kiosk's small
                     physical screen. -->
                <div class="w-1/3 h-4 bg-[var(--led-red-dim)]" />
                <!-- Battery body — outlined; fills upward from the
                     bottom according to this cell's relative
                     voltage. -->
                <div class="w-full flex-1 mt-1 led-outline-dim flex flex-col-reverse">
                  <div :class="cellFillFor(s, v)"
                       :style="{ height: cellHeightPct(s, v) + '%' }" />
                </div>
              </div>
            </div>
          </div>
        </button>
      </div>

      <!-- Combined fault list. Only rendered when something is
           asserted. Each line names the pack so the operator knows
           which one to drill into. -->
      <div v-if="allFaults.length" class="led-outline p-5 text-2xl">
        <div class="led-text mb-3 tracking-[0.2em] text-3xl">► BMS FAULTS</div>
        <ul class="space-y-1 led-text">
          <li v-for="(f, i) in allFaults" :key="i">
            ▌ <span class="tracking-[0.16em]">{{ f.pack }}:</span> {{ f.fault }}
          </li>
        </ul>
      </div>
    </div>

  </ConsoleScreen>
</template>
