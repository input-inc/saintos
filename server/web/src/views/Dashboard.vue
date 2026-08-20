<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useNodesStore } from '@/stores/nodes'
import { useDisplayStore } from '@/stores/display'
import { useWsStore } from '@/stores/ws'
import { useWsTopic } from '@/composables/useWsTopic'
import Widgets from '@/views/Widgets.vue'
import WifiChannelModal from '@/components/WifiChannelModal.vue'
import WifiSwitchingOverlay from '@/components/WifiSwitchingOverlay.vue'

const nodes = useNodesStore()
const display = useDisplayStore()
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

const cpu = computed(() => systemStatus.value?.cpu_usage ?? null)
const mem = computed(() => systemStatus.value?.memory_usage ?? null)
// Server publishes Celsius as `cpu_temp_c`; color logic stays in Celsius
// regardless of the operator's display-unit preference.
const tempC = computed(() => {
  const v = systemStatus.value?.cpu_temp_c
  return typeof v === 'number' ? v : null
})
const uptime = computed(() => systemStatus.value?.uptime_seconds ?? null)
const serverName = computed(() => systemStatus.value?.server_name || '--')

// CPU temp color thresholds match vanilla (warns at 65 °C, critical at
// 80 °C where the Pi enters soft-throttle).
const tempClass = computed(() => {
  const t = tempC.value
  if (t == null) return 'stat-value text-fg-faint'
  if (t >= 80) return 'stat-value text-red-400'
  if (t >= 65) return 'stat-value text-amber-400'
  return 'stat-value text-emerald-400'
})
const tempText = computed(() => tempC.value == null ? '--' : display.formatTemperature(tempC.value))

// Throttle dict shape from server: { raw, status: 'ok'|'warning'|'critical',
// summary, flags[], descriptions[] } — or null on non-Pi hosts.
const throttle = computed(() => systemStatus.value?.throttle ?? null)
const throttleText = computed(() => {
  const t = throttle.value
  if (!t) return 'n/a'
  if (t.status === 'ok') return 'OK'
  if (t.status === 'warning') return 'Past events'
  return 'Active'
})
const throttleClass = computed(() => {
  const t = throttle.value
  if (!t) return 'stat-value text-sm text-fg-faint'
  if (t.status === 'ok') return 'stat-value text-sm text-emerald-400'
  if (t.status === 'warning') return 'stat-value text-sm text-amber-400'
  return 'stat-value text-sm text-red-400'
})
const throttleTitle = computed(() => {
  const t = throttle.value
  if (!t) return 'vcgencmd not available on this host'
  if (t.status === 'ok') return `${t.summary} (${t.raw})`
  return `${t.summary} (${t.raw}): ${(t.descriptions || []).join(', ')}`
})

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

// ── WiFi visual treatment ───────────────────────────────────────────
// Seven label/value rows of dBm and percentages is a lot of reading for
// what's usually one question: is the link good enough right now? These
// derive the same numbers into things that can be seen at a glance. The
// raw figures stay reachable — in the strip below the bars, and in
// tooltips — because when the answer IS "no", the raw numbers are what
// you diagnose with.

// Signal → 0-4 bars, on the usual dBm breakpoints for 802.11.
const wifiBars = computed(() => {
  const s = wifiMetrics.value.signal
  if (s == null) return 0
  if (s >= -55) return 4
  if (s >= -65) return 3
  if (s >= -72) return 2
  if (s >= -80) return 1
  return 0
})

// Signal-to-noise ratio. The honest single measure of link quality —
// -70 dBm over a -95 dBm floor is a fine link, while the same -70 over a
// -75 floor is unusable, and no signal number alone distinguishes them.
// Null unless BOTH readings are present rather than assuming a floor.
const wifiSnr = computed(() => {
  const { signal, noise } = wifiMetrics.value
  if (signal == null || noise == null) return null
  return signal - noise
})

const wifiQuality = computed(() => {
  const snr = wifiSnr.value
  // With no noise floor reported (host without `iw`, or a driver that
  // doesn't expose it), fall back to signal alone and say so.
  if (snr == null) {
    const s = wifiMetrics.value.signal
    if (s == null) return { label: 'No data', cls: 'text-fg-faint', dot: 'bg-slate-500' }
    if (s >= -65) return { label: 'Good signal', cls: 'text-emerald-400', dot: 'bg-emerald-400' }
    if (s >= -75) return { label: 'Fair signal', cls: 'text-amber-400', dot: 'bg-amber-400' }
    return { label: 'Weak signal', cls: 'text-rose-400', dot: 'bg-rose-400' }
  }
  if (snr >= 40) return { label: 'Excellent', cls: 'text-emerald-400', dot: 'bg-emerald-400' }
  if (snr >= 25) return { label: 'Good',      cls: 'text-emerald-400', dot: 'bg-emerald-400' }
  if (snr >= 15) return { label: 'Fair',      cls: 'text-amber-400',   dot: 'bg-amber-400' }
  return { label: 'Poor', cls: 'text-rose-400', dot: 'bg-rose-400' }
})

const wifiSnrText = computed(() =>
  wifiSnr.value != null ? `${Math.round(wifiSnr.value)} dB SNR` : 'SNR unavailable')

// TX retry is the metric that actually predicts control-latency trouble,
// so it gets a meter rather than a number in a list. Scale caps at 30% —
// past that the link is unusable and the exact figure stops mattering.
const wifiRetryPct = computed(() => {
  const r = wifiMetrics.value.retry
  return r == null ? null : Math.max(0, Math.min(100, (r / 30) * 100))
})
const wifiRetryClass = computed(() => {
  const r = wifiMetrics.value.retry
  if (r == null) return 'bg-slate-500'
  if (r < 5) return 'bg-emerald-500'
  if (r < 15) return 'bg-amber-500'
  return 'bg-rose-500'
})

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
</script>

<template>
  <section class="space-y-6">
    <div class="flex items-center justify-between">
      <h2 class="text-2xl font-bold text-fg-strong">Dashboard</h2>
      <button class="btn-secondary" @click="nodes.scan()">
        <span class="material-icons icon-sm">search</span>
        Scan Nodes
      </button>
    </div>

    <!-- System + WiFi in one card. They were separate boxes reporting on
         the same machine, and the WiFi half was seven rows of dBm and
         percentages — a lot of reading for "is the link OK?". Split
         internally: host on the left, network on the right, stacking on
         narrow screens. -->
    <div class="card">
      <div class="flex items-center justify-between mb-4">
        <h3 class="text-lg font-semibold text-fg-strong">System</h3>
        <div class="flex items-center gap-3">
          <span class="px-2 py-1 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400">Online</span>
          <RouterLink to="/settings" class="text-sm text-cyan-400 hover:text-cyan-300 transition-colors">Manage →</RouterLink>
        </div>
      </div>

      <div class="grid grid-cols-1 lg:grid-cols-2 gap-6 lg:gap-8">
        <!-- ── Host ──────────────────────────────────────────────── -->
        <div class="grid grid-cols-2 gap-4 content-start">
          <div class="stat-item">
            <span class="stat-label">Uptime</span>
            <span class="stat-value">{{ fmtUptime(uptime) }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Server Name</span>
            <span class="stat-value">{{ serverName }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">CPU Usage</span>
            <div class="flex items-center gap-2">
              <div class="flex-1 h-2 bg-surface rounded-full overflow-hidden">
                <div class="h-full bg-cyan-500 transition-all duration-300" :style="{ width: `${cpu ?? 0}%` }" />
              </div>
              <span class="stat-value text-sm w-12 text-right">{{ cpu != null ? `${cpu.toFixed(0)}%` : '--' }}</span>
            </div>
          </div>
          <div class="stat-item">
            <span class="stat-label">Memory Usage</span>
            <div class="flex items-center gap-2">
              <div class="flex-1 h-2 bg-surface rounded-full overflow-hidden">
                <div class="h-full bg-violet-500 transition-all duration-300" :style="{ width: `${mem ?? 0}%` }" />
              </div>
              <span class="stat-value text-sm w-12 text-right">{{ mem != null ? `${mem.toFixed(0)}%` : '--' }}</span>
            </div>
          </div>
          <div class="stat-item">
            <span class="stat-label">CPU Temp</span>
            <span :class="tempClass">{{ tempText }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Throttle</span>
            <span :class="throttleClass" :title="throttleTitle">{{ throttleText }}</span>
          </div>
        </div>

        <!-- ── Network ───────────────────────────────────────────── -->
        <div class="lg:border-l lg:border-line lg:pl-8 border-t border-line pt-6 lg:border-t-0 lg:pt-0">
          <!-- Signal bars + quality verdict. The bars answer "is it OK?";
               the dBm/SNR line underneath is what you diagnose with. -->
          <div class="flex items-center gap-3 mb-4">
            <div class="flex items-end gap-[3px] h-6" :title="wifiSignalText" aria-hidden="true">
              <div
                v-for="b in 4"
                :key="b"
                class="w-1.5 rounded-sm transition-colors"
                :class="b <= wifiBars ? wifiQuality.dot : 'bg-surface'"
                :style="{ height: `${b * 25}%` }"
              />
            </div>
            <div class="min-w-0">
              <div class="flex items-center gap-2">
                <span class="text-lg font-semibold leading-none" :class="wifiQuality.cls">
                  {{ wifiQuality.label }}
                </span>
              </div>
              <div class="text-xs text-fg-faint font-mono mt-1 truncate"
                   :title="`Signal ${wifiSignalText} · noise ${wifiNoiseText}`">
                {{ wifiSignalText }} · {{ wifiSnrText }}
              </div>
            </div>
            <span class="ml-auto text-[10px] text-fg-faint font-mono whitespace-nowrap"
                  :title="`Metrics last updated ${wifiUpdatedText}`">
              {{ wifiUpdatedText }}
            </span>
          </div>

          <!-- SSID + band/channel: identity, not health, so it's a quiet
               single line rather than two labelled rows. -->
          <div class="flex items-baseline gap-2 mb-4 text-sm min-w-0">
            <span class="material-icons icon-sm text-fg-faint">wifi</span>
            <span class="font-mono text-fg-strong truncate">{{ wifiSsid }}</span>
            <span class="text-xs text-fg-muted whitespace-nowrap">{{ wifiBandCh }}</span>
          </div>

          <!-- TX retry gets a meter because it's the metric that actually
               predicts control-latency trouble; bitrate is just a number. -->
          <div class="space-y-3">
            <div>
              <div class="flex items-baseline justify-between mb-1">
                <span class="stat-label">TX retry</span>
                <span class="text-sm font-semibold text-fg-strong tabular-nums">{{ wifiRetryText }}</span>
              </div>
              <div class="h-2 bg-surface rounded-full overflow-hidden"
                   title="Retransmission rate — the first thing to check when control feels laggy. Scale caps at 30%.">
                <div class="h-full transition-all duration-300"
                     :class="wifiRetryClass"
                     :style="{ width: `${wifiRetryPct ?? 0}%` }" />
              </div>
            </div>
            <div class="flex items-baseline justify-between">
              <span class="stat-label">TX bitrate</span>
              <span class="text-sm font-semibold text-fg-strong tabular-nums">{{ wifiBitrateText }}</span>
            </div>
          </div>

          <div class="mt-4 pt-4 border-t border-line flex items-center justify-end">
            <button class="btn-secondary text-sm flex items-center gap-2" @click="wifiModalOpen = true">
              <span class="material-icons icon-sm">wifi_find</span>
              Find better channel
            </button>
          </div>
        </div>
      </div>
    </div>

    <div>
      <div class="flex items-center justify-between mb-4">
        <h3 class="text-lg font-semibold text-fg-muted">Widgets</h3>
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
