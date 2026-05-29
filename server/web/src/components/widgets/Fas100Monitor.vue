<script setup>
import { computed, onMounted, onUnmounted, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useDisplayStore } from '@/stores/display'
import { useChannelHistory } from '@/composables/useChannelHistory'
import Sparkline from '@/components/Sparkline.vue'

// Mockup-faithful Vue port of the legacy `_renderFas100Card` widget.
// Four rows (current / voltage / temp1 / temp2). Each row pairs an
// inline 30s sparkline with a value readout and a unit. Current and
// voltage carry a unipolar 0..max range bar; the two temp rows omit
// the bar to keep the card visually anchored around battery health.

const props = defineProps({
  widget: { type: Object, required: true },
  routes: { type: Array,  default: () => [] },
})

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const display = useDisplayStore()
const history = useChannelHistory()

const ROW_META = {
  current: { color: 'bg-cyan-500',  text: 'text-cyan-300',  bar: 'unipolar', max: 100 },
  voltage: { color: 'bg-amber-500', text: 'text-amber-300', bar: 'unipolar', max: 30  },
  temp1:   { color: 'bg-rose-500',  text: 'text-rose-300',  bar: 'none',     max: 80  },
  temp2:   { color: 'bg-rose-500',  text: 'text-rose-300',  bar: 'none',     max: 80  },
}
const ROW_IDS = ['current', 'voltage', 'temp1', 'temp2']

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

let stateHandler = null

onMounted(async () => {
  await catalog.ensureLoaded()
  await syncSubs()
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

watch(subscribedTopics, () => { syncSubs() })

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
  if (inputId === 'temp1' || inputId === 'temp2') {
    const s = display.formatTemperature(last.value)
    return s.replace(/°[CF]$/, '')
  }
  return last.value.toFixed(2)
}

function unitText (inputId) {
  if (inputId === 'current') return 'A'
  if (inputId === 'voltage') return 'V'
  if (inputId === 'temp1' || inputId === 'temp2') {
    return display.temperatureUnit === 'fahrenheit' ? '°F' : '°C'
  }
  return ''
}

// Bar fill % for unipolar rows. Temperature rows have bar:'none' so
// this never runs for them; current/voltage scale against the meta max.
function barStyle (inputId) {
  const meta = ROW_META[inputId]
  if (!meta || meta.bar === 'none') return null
  const last = lastSampleFor(inputId)
  const val = (last && typeof last.value === 'number') ? last.value : null
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
        <span class="material-icons text-amber-400 icon-md">bolt</span>
        <h4 class="text-base font-semibold text-fg-strong">{{ widget.label || widget.id }}</h4>
      </div>
      <span class="px-2 py-0.5 text-xs font-medium rounded-full bg-amber-900/40 text-amber-200">
        FAS100
      </span>
    </div>
    <div class="h-0.5 bg-amber-500 rounded-full mb-3"></div>

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
          <div
            v-if="ROW_META[inp.id]?.bar === 'unipolar'"
            class="flex-1 h-1 bg-surface rounded-full overflow-hidden ml-2"
          >
            <div
              class="h-full transition-all"
              :class="ROW_META[inp.id].color"
              :style="barStyle(inp.id) || { width: '0%' }"
            ></div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>
