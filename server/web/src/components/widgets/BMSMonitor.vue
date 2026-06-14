<script setup>
import { computed, onMounted, onUnmounted, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useDisplayStore } from '@/stores/display'

// Dashboard-card sized BMS summary. Surfaces SOC + pack metrics +
// FET state + protection faults, but deliberately omits the per-cell
// voltage bars rendered by the Live tab's BMSCard. The intent is at-a-
// glance pack health on a multi-pack dashboard, not the deep dive a
// technician needs when investigating imbalance.
//
// Data flow mirrors Fas100Monitor.vue: each declared input maps to a
// route, the widget subscribes to that route's source `pin_state/<node>`
// topic, and pulls the latest channel value out of the broadcast.

const props = defineProps({
  widget: { type: Object, required: true },
  routes: { type: Array,  default: () => [] },
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const display = useDisplayStore()

const ROW_IDS = ['soc', 'voltage', 'current', 'temp', 'protection', 'fet_status']

const widgetType = computed(() => catalog.widgetType(props.widget.type))
const inputs = computed(() => {
  const declared = widgetType.value?.inputs || []
  const byId = new Map(declared.map(i => [i.id, i]))
  return ROW_IDS.map(id => byId.get(id) || { id, display: id })
})

function sourceFor (inputId) {
  for (const r of props.routes) {
    if (r.sink?.kind !== 'widget') continue
    if (r.sink.parts?.[0] !== props.widget.id) continue
    if (r.sink.parts?.[1] !== inputId) continue
    return r.source || null
  }
  return null
}

function upstreamKeyParts (inputId) {
  const src = sourceFor(inputId)
  if (!src || src.kind !== 'peripheral') return null
  const [nodeId, peripheralId, channelId] = src.parts || []
  if (!nodeId || !peripheralId || !channelId) return null
  return { nodeId, peripheralId, channelId }
}

const subscribedTopics = computed(() => {
  const out = new Set()
  for (const inp of inputs.value) {
    const k = upstreamKeyParts(inp.id)
    if (k) out.add(`pin_state/${k.nodeId}`)
  }
  return [...out]
})

const activeSubs = new Set()

async function syncSubs () {
  const desired = new Set(subscribedTopics.value)
  const toAdd = [...desired].filter(t => !activeSubs.has(t))
  const toRemove = [...activeSubs].filter(t => !desired.has(t))
  if (toAdd.length) {
    try { await ws.subscribe(toAdd, 10) } catch (_) {}
    for (const t of toAdd) activeSubs.add(t)
  }
  if (toRemove.length) {
    try { await ws.unsubscribe(toRemove) } catch (_) {}
    for (const t of toRemove) activeSubs.delete(t)
  }
}

const latest = computed(() => ws.topics)
function lastSampleFor (inputId) {
  const k = upstreamKeyParts(inputId)
  if (!k) return null
  const data = latest.value.get(`pin_state/${k.nodeId}`)
  if (!data || !Array.isArray(data.channels)) return null
  for (const ch of data.channels) {
    if (ch.peripheral_id === k.peripheralId && ch.channel_id === k.channelId) {
      return ch
    }
  }
  return null
}
function valueOf (inputId) {
  const last = lastSampleFor(inputId)
  if (!last || typeof last.value !== 'number') return null
  return last.value
}

let resubscribeFn = null
onMounted(async () => {
  await catalog.ensureLoaded()
  await syncSubs()
  resubscribeFn = () => {
    const topics = [...activeSubs]
    if (topics.length) ws.subscribe(topics, 10).catch(() => {})
  }
  ws.on('ready', resubscribeFn)
})
onUnmounted(() => {
  if (resubscribeFn) ws.off('ready', resubscribeFn)
  if (activeSubs.size) {
    ws.unsubscribe([...activeSubs]).catch(() => {})
    activeSubs.clear()
  }
})
watch(subscribedTopics, () => { syncSubs() })

// JBD protection bit → human label. Mirrors BMSCard's table and the
// server-side decoder in jbd_bms.py — keep these three in sync.
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

const soc       = computed(() => valueOf('soc'))
const voltage   = computed(() => valueOf('voltage'))
const current   = computed(() => valueOf('current'))
const tempC     = computed(() => valueOf('temp'))
const protBits  = computed(() => valueOf('protection'))
const fetStatus = computed(() => valueOf('fet_status'))

const faults    = computed(() => decodeProtection(protBits.value || 0))
const chargeOn    = computed(() => ((fetStatus.value | 0) & 0x01) === 0x01)
const dischargeOn = computed(() => ((fetStatus.value | 0) & 0x02) === 0x02)

function fmt (v, places = 2, unit = '') {
  if (v === null || v === undefined || !isFinite(v)) return '—'
  return `${v.toFixed(places)}${unit ? ' ' + unit : ''}`
}
function fmtTemp (c) {
  if (c === null || c === undefined || !isFinite(c)) return '—'
  return display.formatTemperature(c)
}

// SOC threshold colors mirror BMSCard: <20 red, <50 amber, else green.
const socColor = computed(() => {
  const s = soc.value
  if (s === null || s === undefined) return 'bg-fg-faint'
  if (s < 20) return 'bg-rose-500'
  if (s < 50) return 'bg-amber-400'
  return 'bg-emerald-500'
})
const socPct = computed(() => Math.max(0, Math.min(100, soc.value ?? 0)))

// Header status chip: red if faults asserted, amber if either FET is
// off (pack isolated), else green. Lets an operator scan a dashboard
// full of these and spot the angry one without reading the panel.
const status = computed(() => {
  if (faults.value.length) return { label: 'FAULT', cls: 'bg-rose-500/20 text-rose-300' }
  if (!chargeOn.value || !dischargeOn.value) return { label: 'ISOLATED', cls: 'bg-amber-500/20 text-amber-300' }
  if (soc.value === null) return { label: 'NO DATA', cls: 'bg-surface text-fg-faint' }
  return { label: 'OK', cls: 'bg-emerald-500/20 text-emerald-300' }
})
</script>

<template>
  <div class="card" :data-widget-id="widget.id">
    <div class="flex items-center justify-between mb-3">
      <div class="flex items-center gap-2">
        <span class="material-icons text-emerald-400 icon-md">battery_charging_full</span>
        <h4 class="text-base font-semibold text-fg-strong">{{ widget.label || widget.id }}</h4>
      </div>
      <span :class="['px-2 py-0.5 text-xs font-medium rounded-full', status.cls]">
        {{ status.label }}
      </span>
    </div>
    <div class="h-0.5 bg-emerald-500 rounded-full mb-3"></div>

    <!-- SOC bar -->
    <div class="space-y-1 mb-3">
      <div class="flex justify-between text-xs text-fg-muted">
        <span>State of charge</span>
        <span class="font-mono text-fg-strong">{{ fmt(soc, 0, '%') }}</span>
      </div>
      <div class="h-2 w-full rounded-full bg-surface overflow-hidden">
        <div :class="[socColor, 'h-2 transition-all']"
             :style="{ width: `${socPct}%` }"/>
      </div>
    </div>

    <!-- Pack metrics: voltage / current / temp -->
    <div class="grid grid-cols-3 gap-2 text-sm mb-3">
      <div class="bg-surface/40 rounded p-2">
        <div class="text-[10px] uppercase tracking-wide text-fg-faint">Voltage</div>
        <div class="font-mono text-cyan-300 text-lg">{{ fmt(voltage, 2, 'V') }}</div>
      </div>
      <div class="bg-surface/40 rounded p-2">
        <div class="text-[10px] uppercase tracking-wide text-fg-faint">Current</div>
        <div :class="['font-mono text-lg', (current ?? 0) >= 0 ? 'text-emerald-300' : 'text-amber-300']">
          {{ fmt(current, 2, 'A') }}
        </div>
      </div>
      <div class="bg-surface/40 rounded p-2">
        <div class="text-[10px] uppercase tracking-wide text-fg-faint">Temp</div>
        <div class="font-mono text-rose-300 text-lg">{{ fmtTemp(tempC) }}</div>
      </div>
    </div>

    <!-- FET state chips -->
    <div class="flex items-center gap-2 text-xs mb-2">
      <span :class="['px-2 py-0.5 rounded-full', chargeOn ? 'bg-emerald-500/20 text-emerald-300' : 'bg-surface text-fg-faint']">
        CHG {{ chargeOn ? 'ON' : 'OFF' }}
      </span>
      <span :class="['px-2 py-0.5 rounded-full', dischargeOn ? 'bg-emerald-500/20 text-emerald-300' : 'bg-surface text-fg-faint']">
        DSG {{ dischargeOn ? 'ON' : 'OFF' }}
      </span>
    </div>

    <!-- Fault panel: only renders when something is asserted. -->
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
