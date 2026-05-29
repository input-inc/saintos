<script setup>
import { computed, onMounted, onUnmounted, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useDisplayStore } from '@/stores/display'
import { useChannelHistory } from '@/composables/useChannelHistory'
import Sparkline from '@/components/Sparkline.vue'

// Mockup-faithful Vue port of the legacy `_renderRoboClawCard` widget.
// Five rows (motor / encoder / voltage / current / temperature). Each
// row pairs an inline 30s sparkline with a value readout and a range
// bar. Motor's bar is bidirectional (centered on zero); encoder has
// no bar (counts are unbounded); voltage/current/temp use unipolar
// 0..max fills.

const props = defineProps({
  widget: { type: Object, required: true },   // { id, type, label, ... }
  routes: { type: Array,  default: () => [] }, // system_routing.routes
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const display = useDisplayStore()
const history = useChannelHistory()

// Per-input row metadata. Matches `_renderRoboClawCard` in the legacy
// widgets.js. `bar` is one of:
//   'bidirectional' — motor duty (-1..+1), violet fill anchored center
//   'unipolar'      — single-sided 0..max fill
//   'none'          — no bar (e.g. encoder counts)
const ROW_META = {
  motor:   { color: 'bg-violet-500', text: 'text-violet-300', bar: 'bidirectional' },
  encoder: { color: 'bg-slate-500',  text: 'text-fg-strong',  bar: 'none' },
  voltage: { color: 'bg-amber-500',  text: 'text-amber-300',  bar: 'unipolar', max: 30 },
  current: { color: 'bg-cyan-500',   text: 'text-cyan-300',   bar: 'unipolar', max: 60 },
  temp:    { color: 'bg-rose-500',   text: 'text-rose-300',   bar: 'unipolar', max: 80 },
}
// Row order matches the widget catalog declaration so input.display
// labels come from the server-defined catalog.
const ROW_IDS = ['motor', 'encoder', 'voltage', 'current', 'temp']

const widgetType = computed(() => catalog.widgetType(props.widget.type))
const inputs = computed(() => {
  // Honour the catalog order when present; fall back to ROW_IDS so
  // the card still lays out if the catalog hasn't loaded yet.
  const declared = widgetType.value?.inputs || []
  const byId = new Map(declared.map(i => [i.id, i]))
  return ROW_IDS.map(id => byId.get(id) || { id, display: id })
})

// route source endpoint for one input, or null if unrouted.
function sourceFor (inputId) {
  for (const r of props.routes) {
    if (r.sink?.kind !== 'widget') continue
    if (r.sink.parts?.[0] !== props.widget.id) continue
    if (r.sink.parts?.[1] !== inputId) continue
    return r.source || null
  }
  return null
}

// Resolve to {node_id, peripheral_id, channel_id} only for direct
// peripheral sources. Routes through an operator/signal don't expose
// a single upstream channel here; show "--" until/unless the server
// evaluator surfaces the evaluated value separately.
function upstreamKeyParts (inputId) {
  const src = sourceFor(inputId)
  if (!src || src.kind !== 'peripheral') return null
  const [nodeId, peripheralId, channelId] = src.parts || []
  if (!nodeId || !peripheralId || !channelId) return null
  return { nodeId, peripheralId, channelId }
}

// Set of distinct pin_state topics this widget needs subscribed.
const subscribedTopics = computed(() => {
  const out = new Set()
  for (const inp of inputs.value) {
    const k = upstreamKeyParts(inp.id)
    if (k) out.add(`pin_state/${k.nodeId}`)
  }
  return [...out]
})

// Manual subscribe/unsubscribe — the set of upstream nodes is dynamic
// (changes when the operator re-routes a widget input). `useWsTopic`
// is single-topic, so we maintain our own subscription set here and
// listen to the `state` event directly to feed channel history.
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

// Latest scalar value per input, populated from pin_state broadcasts.
// We mirror to a Map so the template can reactively re-render on each
// frame without depending on internal state of the history ring.
const latest = computed(() => ws.topics) // reactive Map reference
// Per-input lookup of the most recent {value} from the right node.
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

// Feed live samples into the shared channel-history ring so the
// inline sparklines update at the broadcast cadence. Mirrors what
// node/Live.vue does on its `pinState` watcher.
function ingestLive () {
  const now = Date.now()
  for (const inp of inputs.value) {
    const k = upstreamKeyParts(inp.id)
    if (!k) continue
    const data = ws.topics.get(`pin_state/${k.nodeId}`)
    if (!data || !Array.isArray(data.channels)) continue
    for (const ch of data.channels) {
      if (ch.peripheral_id !== k.peripheralId) continue
      if (ch.channel_id !== k.channelId) continue
      if (typeof ch.value !== 'number') continue
      history.pushSample(`${k.nodeId}/${k.peripheralId}/${k.channelId}`, ch.value, now)
    }
  }
}

// Listen for raw state broadcasts; the ws store already updates
// topics.set(...) before emitting 'state', so we can read fresh data
// here and feed the history ring.
let stateHandler = null

onMounted(async () => {
  await catalog.ensureLoaded()
  await syncSubs()
  // Backfill 30s history for each routed input so sparklines aren't
  // empty on first paint.
  for (const inp of inputs.value) {
    const k = upstreamKeyParts(inp.id)
    if (k) history.fetchHistory(k.nodeId, k.peripheralId, k.channelId)
  }
  stateHandler = (msg) => {
    if (!msg || typeof msg.node !== 'string') return
    if (!msg.node.startsWith('pin_state/')) return
    if (!activeSubs.has(msg.node)) return
    ingestLive()
  }
  ws.on('state', stateHandler)
  // Re-subscribe after a WS reconnect so the widget keeps streaming.
  ws.on('ready', resubscribe)
})

function resubscribe () {
  const topics = [...activeSubs]
  if (!topics.length) return
  ws.subscribe(topics, 10).catch(() => {})
}

onUnmounted(() => {
  if (stateHandler) ws.off('state', stateHandler)
  if (activeSubs.size) {
    ws.unsubscribe([...activeSubs]).catch(() => {})
    activeSubs.clear()
  }
})

// Reactively keep subscriptions in sync when the operator re-routes
// or removes a widget input mid-session.
watch(subscribedTopics, () => { syncSubs() })

// ── Per-row presentation helpers ────────────────────────────────────

function samplesFor (inputId) {
  const k = upstreamKeyParts(inputId)
  if (!k) return []
  return history.getHistory(`${k.nodeId}/${k.peripheralId}/${k.channelId}`)
}

function isRouted (inputId) {
  return upstreamKeyParts(inputId) !== null
}

function valueText (inputId) {
  const last = lastSampleFor(inputId)
  if (!last || typeof last.value !== 'number') return '--'
  if (inputId === 'motor') {
    // duty fraction -1..+1 rendered as a signed integer percentage.
    const pct = last.value * 100
    const sign = pct > 0 ? '+' : ''
    return `${sign}${pct.toFixed(0)}`
  }
  if (inputId === 'encoder') {
    return Number.isInteger(last.value) ? last.value.toString() : last.value.toFixed(0)
  }
  if (inputId === 'temp') {
    // Display in operator's preferred unit; formatTemperature returns
    // the value WITH its unit suffix, so we drop the trailing unit
    // here and render it separately in the template (consistent with
    // the other rows that show value + unit as separate spans).
    const s = display.formatTemperature(last.value)
    return s.replace(/°[CF]$/, '')
  }
  // voltage / current — 2-decimal numeric.
  return last.value.toFixed(2)
}

function unitText (inputId) {
  if (inputId === 'motor') return '%'
  if (inputId === 'encoder') return 'cnt'
  if (inputId === 'voltage') return 'V'
  if (inputId === 'current') return 'A'
  if (inputId === 'temp') return display.temperatureUnit === 'fahrenheit' ? '°F' : '°C'
  return ''
}

// Bar geometry per row. Returns inline style or null for the 'none'
// case (encoder).
function barStyle (inputId) {
  const meta = ROW_META[inputId]
  if (!meta || meta.bar === 'none') return null
  const last = lastSampleFor(inputId)
  const val = (last && typeof last.value === 'number') ? last.value : null
  if (meta.bar === 'bidirectional') {
    // Motor duty: clamp to [-1, +1] in case the firmware glitches and
    // produces an out-of-range reading.
    const duty = val === null ? 0 : Math.max(-1, Math.min(1, val))
    const half = Math.abs(duty) * 50
    return duty >= 0
      ? { left: '50%', width: `${half.toFixed(1)}%` }
      : { left: `${(50 - half).toFixed(1)}%`, width: `${half.toFixed(1)}%` }
  }
  // Unipolar. Temperature ALWAYS scales on raw Celsius so the visual
  // means the same thing regardless of the operator's display unit.
  const max = meta.max || 1
  const pct = val === null ? 0 : Math.max(0, Math.min(100, (val / max) * 100))
  return { width: `${pct.toFixed(1)}%` }
}

function sourceLabel (inputId) {
  const src = sourceFor(inputId)
  if (!src) return 'unrouted'
  if (src.kind === 'peripheral') return src.parts.join('/')
  if (src.kind === 'signal') return src.parts.join('/')
  return `${src.kind}:${(src.parts || []).join('/')}`
}
</script>

<template>
  <div class="card" :data-widget-id="widget.id">
    <div class="flex items-center justify-between mb-3">
      <div class="flex items-center gap-2">
        <span class="material-icons text-violet-400 icon-md">precision_manufacturing</span>
        <h4 class="text-base font-semibold text-fg-strong">{{ widget.label || widget.id }}</h4>
      </div>
      <span class="px-2 py-0.5 text-xs font-medium rounded-full bg-violet-900/40 text-violet-200">
        RoboClaw
      </span>
    </div>
    <div class="h-0.5 bg-violet-500 rounded-full mb-3"></div>

    <div>
      <div
        v-for="inp in inputs"
        :key="inp.id"
        class="border-t border-line/60 pt-2 mt-2 first:border-t-0 first:pt-0 first:mt-0"
      >
        <div class="flex items-center justify-between">
          <span class="stat-label">{{ inp.display || inp.id }}</span>
          <span class="inline-flex items-center gap-2">
            <Sparkline :samples="samplesFor(inp.id)" :width="140" :height="24" />
            <span class="stat-value" :class="ROW_META[inp.id]?.text">{{ valueText(inp.id) }}</span>
            <span class="text-xs text-fg-faint">{{ unitText(inp.id) }}</span>
          </span>
        </div>
        <div class="flex items-center mt-1">
          <div class="text-[0.65rem] text-fg-faint truncate flex-1" :title="sourceLabel(inp.id)">
            <template v-if="isRouted(inp.id)">← {{ sourceLabel(inp.id) }}</template>
            <template v-else>unrouted</template>
          </div>

          <!-- Bidirectional motor bar: track with a 1px center divider
               and a violet fill that grows L/R from 50% as duty signs. -->
          <div
            v-if="ROW_META[inp.id]?.bar === 'bidirectional'"
            class="relative flex-1 h-1 bg-surface rounded-full overflow-hidden ml-2"
          >
            <div class="absolute inset-y-0 left-1/2 w-px bg-slate-500"></div>
            <div
              class="absolute inset-y-0 transition-all"
              :class="ROW_META[inp.id].color"
              :style="barStyle(inp.id) || { left: '50%', width: '0%' }"
            ></div>
          </div>

          <!-- Unipolar bar: voltage / current / temperature. -->
          <div
            v-else-if="ROW_META[inp.id]?.bar === 'unipolar'"
            class="flex-1 h-1 bg-surface rounded-full overflow-hidden ml-2"
          >
            <div
              class="h-full transition-all"
              :class="ROW_META[inp.id].color"
              :style="barStyle(inp.id) || { width: '0%' }"
            ></div>
          </div>
          <!-- Encoder: no bar, just preserves alignment via the flex-1
               source-label above. -->
        </div>
      </div>
    </div>
  </div>
</template>
