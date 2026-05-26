<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useNodesStore } from '@/stores/nodes'
import { useDisplayStore } from '@/stores/display'
import { useActivityStore } from '@/stores/activity'
import { useWsStore } from '@/stores/ws'
import { useWsTopic } from '@/composables/useWsTopic'
import Widgets from '@/views/Widgets.vue'
import WifiChannelModal from '@/components/WifiChannelModal.vue'
import WifiSwitchingOverlay from '@/components/WifiSwitchingOverlay.vue'

const nodes = useNodesStore()
const display = useDisplayStore()
const activity = useActivityStore()
const ws = useWsStore()
const systemStatus = useWsTopic(() => 'system_status')
// Live WiFi metrics (signal/retry/noise/bitrate) ride the same
// pin_state broadcast the host_controller streams to widgets. We just
// pluck the system_monitor.wifi_* channels out of the latest frame.
const hostPinState = useWsTopic(() => 'pin_state/host_controller', 1)

// WiFi card — static config from wifi_get_config (SSID + band/channel).
const wifiCfg = ref(null)
const wifiUpdatedAt = ref(null)
const wifiModalOpen = ref(false)
const wifiSwitching = ref(null)  // { detail } when overlay should show

async function loadWifiConfig () {
  try {
    const cfg = await ws.management('wifi_get_config', {})
    if (cfg && cfg.ok) wifiCfg.value = cfg
  } catch (_) {
    // Non-fatal — missing iw / non-Pi host just leaves the card blank.
  }
}

onMounted(() => {
  nodes.fetchAll().catch(() => {})
  loadWifiConfig()
})

const online  = computed(() => nodes.all.filter(n => n.online).length)
const offline = computed(() => nodes.all.length - online.value)

const cpu = computed(() => systemStatus.value?.cpu_usage ?? null)
const mem = computed(() => systemStatus.value?.memory_usage ?? null)
const temp = computed(() => systemStatus.value?.cpu_temp ?? null)
const uptime = computed(() => systemStatus.value?.uptime ?? null)

// Walk host_controller's channel list once per frame, extract the
// system_monitor.wifi_* readings. Missing channels (e.g. host without
// iw installed) leave previous values in place — don't flicker.
const wifiMetrics = ref({ signal: null, retry: null, noise: null, bitrate: null })
watch(hostPinState, (data) => {
  if (!data || !Array.isArray(data.channels)) return
  let changed = false
  const next = { ...wifiMetrics.value }
  for (const ch of data.channels) {
    if (ch.peripheral_id !== 'system_monitor') continue
    if (typeof ch.value !== 'number') continue
    if      (ch.channel_id === 'wifi_signal')     { next.signal = ch.value;  changed = true }
    else if (ch.channel_id === 'wifi_retry_pct')  { next.retry = ch.value;   changed = true }
    else if (ch.channel_id === 'wifi_noise')      { next.noise = ch.value;   changed = true }
    else if (ch.channel_id === 'wifi_bitrate')    { next.bitrate = ch.value; changed = true }
  }
  if (changed) {
    wifiMetrics.value = next
    wifiUpdatedAt.value = Date.now()
  }
})

const wifiSsid = computed(() => wifiCfg.value?.ssid || '--')
const wifiBandCh = computed(() => {
  const cfg = wifiCfg.value
  if (!cfg) return '--'
  const band = cfg.band === 'a' ? '5 GHz' : (cfg.band === 'bg' ? '2.4 GHz' : (cfg.band || '?'))
  return cfg.channel ? `${band} · ch ${cfg.channel}` : `${band} · auto`
})
const wifiSignalText  = computed(() => wifiMetrics.value.signal  != null ? `${Math.round(wifiMetrics.value.signal)} dBm` : '-- dBm')
const wifiRetryText   = computed(() => wifiMetrics.value.retry   != null ? `${wifiMetrics.value.retry.toFixed(1)}%`       : '--%')
const wifiNoiseText   = computed(() => wifiMetrics.value.noise   != null ? `${Math.round(wifiMetrics.value.noise)} dBm`   : '-- dBm')
const wifiBitrateText = computed(() => wifiMetrics.value.bitrate != null ? `${wifiMetrics.value.bitrate.toFixed(1)} Mbps` : '-- Mbps')
const wifiUpdatedText = computed(() => wifiUpdatedAt.value ? new Date(wifiUpdatedAt.value).toLocaleTimeString([], { hour12: false }) : '--')

function onSwitching (info) {
  wifiSwitching.value = info || { detail: '' }
}
function onSwitchingDone () {
  wifiSwitching.value = null
  // Refresh static fields once we're back — band/channel may have changed.
  loadWifiConfig()
}

function fmtUptime (sec) {
  if (sec == null) return '--'
  const d = Math.floor(sec / 86400)
  const h = Math.floor((sec % 86400) / 3600)
  const m = Math.floor((sec % 3600) / 60)
  if (d) return `${d}d ${h}h`
  if (h) return `${h}h ${m}m`
  return `${m}m`
}

function fmtTime (ts) {
  if (!ts) return ''
  const t = new Date(ts * 1000)
  return t.toLocaleTimeString([], { hour12: false })
}
function classFor (level) {
  return ({ error: 'error', warn: 'warn', info: 'info', debug: 'debug' }[(level || 'info').toLowerCase()]) || 'info'
}
</script>

<template>
  <section class="space-y-6">
    <div class="flex items-center justify-between">
      <h2 class="text-2xl font-bold text-white">Dashboard</h2>
      <button class="btn-secondary" @click="nodes.scan()">
        <span class="material-icons icon-sm">search</span>
        Scan Nodes
      </button>
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-3 gap-6">
      <div class="card">
        <div class="flex items-center justify-between mb-4">
          <h3 class="text-lg font-semibold text-white">System status</h3>
          <span class="px-2 py-1 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400">Online</span>
        </div>
        <div class="grid grid-cols-2 gap-4">
          <div class="stat-item">
            <span class="stat-label">Uptime</span>
            <span class="stat-value">{{ fmtUptime(uptime) }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">CPU temp</span>
            <span class="stat-value">{{ display.formatTemperature(temp) }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">CPU usage</span>
            <div class="flex items-center gap-2">
              <div class="flex-1 h-2 bg-slate-700 rounded-full overflow-hidden">
                <div class="h-full bg-cyan-500 transition-all duration-300" :style="{ width: `${cpu ?? 0}%` }" />
              </div>
              <span class="stat-value text-sm w-12 text-right">{{ cpu != null ? `${cpu.toFixed(0)}%` : '--' }}</span>
            </div>
          </div>
          <div class="stat-item">
            <span class="stat-label">Memory</span>
            <div class="flex items-center gap-2">
              <div class="flex-1 h-2 bg-slate-700 rounded-full overflow-hidden">
                <div class="h-full bg-violet-500 transition-all duration-300" :style="{ width: `${mem ?? 0}%` }" />
              </div>
              <span class="stat-value text-sm w-12 text-right">{{ mem != null ? `${mem.toFixed(0)}%` : '--' }}</span>
            </div>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="flex items-center justify-between mb-4">
          <h3 class="text-lg font-semibold text-white">Nodes</h3>
          <RouterLink to="/nodes" class="text-xs text-cyan-400 hover:text-cyan-300 underline">manage</RouterLink>
        </div>
        <div class="grid grid-cols-3 gap-4">
          <div class="stat-item">
            <span class="stat-label">Adopted</span>
            <span class="stat-value text-2xl">{{ nodes.all.length }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Online</span>
            <span class="stat-value text-2xl text-emerald-400">{{ online }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Offline</span>
            <span class="stat-value text-2xl text-slate-400">{{ offline }}</span>
          </div>
        </div>
        <div v-if="nodes.unadopted.length" class="mt-4 pt-4 border-t border-slate-700/50 flex items-center justify-between">
          <span class="text-sm text-amber-300 flex items-center gap-2">
            <span class="material-icons icon-sm">new_releases</span>
            {{ nodes.unadopted.length }} unadopted node{{ nodes.unadopted.length === 1 ? '' : 's' }}
          </span>
          <RouterLink to="/nodes" class="text-xs text-amber-300 hover:text-amber-200 underline">adopt now →</RouterLink>
        </div>
      </div>

      <!-- WiFi Status — server-level card (not a routable widget),
           treated like System Status because it reports on the AP the
           dashboard itself depends on. Static-ish bits (SSID, band,
           channel) come from wifi_get_config; live metrics come from
           pin_state/host_controller's system_monitor.wifi_* channels. -->
      <div class="card">
        <div class="flex items-center justify-between mb-4">
          <h3 class="text-lg font-semibold text-white">WiFi Status</h3>
          <RouterLink to="/settings" class="text-sm text-cyan-400 hover:text-cyan-300 transition-colors">Manage →</RouterLink>
        </div>
        <div class="grid grid-cols-2 gap-4">
          <div class="stat-item col-span-2">
            <span class="stat-label">SSID</span>
            <span class="stat-value font-mono">{{ wifiSsid }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Band / Channel</span>
            <span class="stat-value">{{ wifiBandCh }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Signal</span>
            <span class="stat-value">{{ wifiSignalText }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">TX retry</span>
            <span class="stat-value">{{ wifiRetryText }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Noise floor</span>
            <span class="stat-value">{{ wifiNoiseText }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">TX bitrate</span>
            <span class="stat-value">{{ wifiBitrateText }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Last updated</span>
            <span class="stat-value text-sm">{{ wifiUpdatedText }}</span>
          </div>
        </div>
        <div class="mt-4 pt-4 border-t border-slate-700 flex items-center justify-end">
          <button class="btn-secondary text-sm flex items-center gap-2" @click="wifiModalOpen = true">
            <span class="material-icons icon-sm">wifi_find</span>
            Find better channel
          </button>
        </div>
      </div>
    </div>

    <div class="card">
      <h3 class="text-lg font-semibold text-white mb-3">Recent activity</h3>
      <div v-if="!activity.entries.length" class="text-sm text-slate-400 italic">No activity yet.</div>
      <div v-else class="font-mono text-xs space-y-1 max-h-64 overflow-y-auto">
        <div
          v-for="(e, i) in activity.entries.slice().reverse().slice(0, 100)"
          :key="i"
          :class="['log-entry', classFor(e.level)]"
        >
          <span class="text-slate-500 mr-2">{{ fmtTime(e.time) }}</span>
          {{ e.text }}
        </div>
      </div>
    </div>

    <div>
      <div class="flex items-center justify-between mb-4">
        <h3 class="text-lg font-semibold text-slate-400">Widgets</h3>
        <RouterLink to="/routes" class="btn-secondary text-sm">
          <span class="material-icons icon-sm">add</span>
          Configure on Routes page
        </RouterLink>
      </div>
      <Widgets embedded />
    </div>

    <WifiChannelModal
      v-if="wifiModalOpen"
      @close="wifiModalOpen = false"
      @switching="onSwitching"
    />
    <WifiSwitchingOverlay
      v-if="wifiSwitching"
      :detail="wifiSwitching.detail"
      @close="onSwitchingDone"
    />
  </section>
</template>
