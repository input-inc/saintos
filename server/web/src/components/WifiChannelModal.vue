<script setup>
//
// Shared WiFi channel-survey modal — opened from the dashboard's WiFi
// Status card AND from the Wireless tab on Settings. Runs the
// `wifi_survey` management action (10 s scan on the server) and lets
// the operator pick a less-busy channel.
//
// Behaviour matches legacy `app.js` openWifiChannelModal /
// _renderWifiChannelRows / _renderWifiChannelSummary / applyWifiChannel.
//
import { computed, onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import AppModal from './AppModal.vue'

const emit = defineEmits(['close', 'switching'])
const ws = useWsStore()

const loading = ref(true)
const errorMsg = ref('')
const channels = ref([])
const currentChannel = ref(null)
const selected = ref(null)  // { band, channel, is_current }
const applying = ref(false)

onMounted(runSurvey)

async function runSurvey () {
  loading.value = true
  errorMsg.value = ''
  channels.value = []
  selected.value = null
  try {
    const data = (await ws.management('wifi_survey', {})) || {}
    if (!data.ok) {
      errorMsg.value = data.error || 'Survey failed (no detail).'
    } else {
      channels.value = data.channels || []
      currentChannel.value = data.current_channel ?? null
    }
  } catch (err) {
    console.error('wifi_survey failed:', err)
    errorMsg.value = `Survey failed: ${err.message || err}`
  } finally {
    loading.value = false
  }
}

// "Best" pick: lowest ap_count among non-current rows, preferring
// non-DFS when tied. DFS channels work but require a 60 s CAC silence
// on activation, so a non-DFS pick at the same busyness tier is
// friendlier.
const best = computed(() => {
  const alternatives = channels.value.filter(c => !c.is_current)
  if (!alternatives.length) return null
  const minAp = alternatives[0].ap_count
  const tied = alternatives.filter(c => c.ap_count === minAp)
  return tied.find(c => !c.is_dfs) || tied[0]
})

function bandLabel (band) {
  if (band === '5' || band === 'a') return '5 GHz'
  if (band === '2' || band === 'bg' || band === '2.4') return '2.4 GHz'
  return band || '?'
}

// AP-count steps tuned to consumer-WiFi reality:
// 0 clear / 1-2 light / 3-5 busy / 6+ avoid.
function apClass (n) {
  if (n === 0) return 'text-emerald-400 font-semibold'
  if (n <= 2) return 'text-fg'
  if (n <= 5) return 'text-amber-300'
  return 'text-red-300 font-semibold'
}

function rowClass (c) {
  if (best.value && c === best.value) return 'bg-emerald-500/10 hover:bg-emerald-500/20'
  if (c.is_current)                   return 'bg-cyan-500/5 hover:bg-cyan-500/10'
  return 'hover:bg-surface/30'
}

function selectRow (c) {
  selected.value = {
    band: c.band === '5' ? 'a' : 'bg',
    channel: c.channel,
    is_current: !!c.is_current,
  }
}

const canApply = computed(() =>
  !!selected.value && !selected.value.is_current && !applying.value
)

async function apply () {
  if (!canApply.value) return
  const sel = selected.value
  const label = bandLabel(sel.band)
  if (!confirm(
    `Switch the AP to ${label} channel ${sel.channel}?\n\n` +
    `Every connected client (including this dashboard) will drop and ` +
    `reconnect over ~5–10 s.`
  )) {
    return
  }
  applying.value = true
  try {
    await ws.management('wifi_set_channel', { band: sel.band, channel: sel.channel })
    emit('switching', { detail: `Switching to ${label} channel ${sel.channel}.` })
    emit('close')
  } catch (err) {
    console.error('wifi_set_channel failed:', err)
    alert(`Channel switch failed: ${err.message || err}`)
  } finally {
    applying.value = false
  }
}

const DFS_TOOLTIP =
  'DFS — Dynamic Frequency Selection. 5 GHz channels overlapping ' +
  'weather/military radar bands. On activation: 60 s silent listen ' +
  'before transmit (CAC). After activation: if the radio detects a ' +
  'radar pulse it must vacate the channel within seconds. Usually ' +
  'fine for ground robots; avoid near airports / weather radar ' +
  'installations.'
</script>

<template>
  <AppModal title="WiFi Channel Survey" width="max-w-3xl" @close="emit('close')">
    <!-- Loading state — survey runs ~10 s on the server. -->
    <div v-if="loading" class="text-sm text-fg-muted py-8 text-center">
      <span class="material-icons animate-spin align-middle">refresh</span>
      Scanning…
    </div>

    <!-- Error state. -->
    <div v-else-if="errorMsg" class="text-sm text-red-300 p-3 rounded bg-red-900/40 border border-red-700">
      {{ errorMsg }}
    </div>

    <!-- Results. -->
    <div v-else class="flex-1 min-h-0 flex flex-col">
      <!-- Recommendation banner — names the best pick + DFS caveat if any. -->
      <div
        v-if="best"
        class="text-sm text-emerald-200 bg-emerald-500/10 border border-emerald-500/30 rounded px-3 py-2 mb-3"
      >
        <div class="flex items-center gap-2">
          <span class="material-icons icon-sm text-emerald-300">recommend</span>
          <span>
            Best pick: <strong>{{ bandLabel(best.band) }} channel {{ best.channel }}</strong>
            ({{ best.ap_count === 0 ? 'no nearby APs' : `${best.ap_count} nearby AP${best.ap_count === 1 ? '' : 's'}` }}).
          </span>
        </div>
        <div v-if="best.is_dfs" class="text-xs text-amber-200/90 mt-1 flex gap-2">
          <span class="material-icons icon-sm align-middle">info</span>
          <span>
            This is a DFS channel. Activation requires a 60-second silent
            listen for radar before the AP comes back up, and the radio may
            move off this channel later if it detects a radar pulse.
            Generally fine for indoor/yard operation; avoid if you're near
            an airport or weather radar installation.
          </span>
        </div>
      </div>

      <!-- Legend. -->
      <div class="text-xs text-fg-muted mb-2 flex flex-wrap gap-x-4 gap-y-1">
        <span><span class="inline-block w-2 h-2 rounded-full bg-emerald-400 mr-1" />0 APs — clear</span>
        <span><span class="inline-block w-2 h-2 rounded-full bg-slate-400 mr-1" />1-2 — light</span>
        <span><span class="inline-block w-2 h-2 rounded-full bg-amber-400 mr-1" />3-5 — busy</span>
        <span><span class="inline-block w-2 h-2 rounded-full bg-red-400 mr-1" />6+ — avoid</span>
        <span class="ml-auto">DFS = radar-detection silence on activation</span>
      </div>

      <div v-if="!channels.length" class="text-sm text-fg-faint italic p-4 text-center border border-line rounded">
        No channels available.
      </div>
      <div v-else class="overflow-y-auto border border-line rounded max-h-[55vh]">
        <table class="w-full text-sm">
          <thead class="bg-panel sticky top-0">
            <tr class="text-left text-xs uppercase text-fg-muted">
              <th class="p-2">Band</th>
              <th class="p-2">Channel</th>
              <th class="p-2">Freq</th>
              <th class="p-2">Nearby APs</th>
              <th class="p-2">Strongest</th>
              <th class="p-2"></th>
            </tr>
          </thead>
          <tbody>
            <tr
              v-for="(c, i) in channels"
              :key="i"
              :class="[
                'border-t border-line cursor-pointer',
                rowClass(c),
                selected && selected.channel === c.channel
                  && ((selected.band === 'a') === (c.band === '5'))
                  ? 'ring-2 ring-cyan-400' : '',
              ]"
              @click="selectRow(c)"
            >
              <td class="p-2 text-fg">{{ bandLabel(c.band) }}</td>
              <td class="p-2 font-mono">{{ c.channel }}</td>
              <td class="p-2 text-fg-faint text-xs">{{ c.freq_mhz }} MHz</td>
              <td :class="['p-2', apClass(c.ap_count)]">{{ c.ap_count }}</td>
              <td class="p-2 text-fg-muted">
                {{ c.strongest_signal_dbm != null ? `${Math.round(c.strongest_signal_dbm)} dBm` : '—' }}
              </td>
              <td class="p-2">
                <div class="flex gap-1 flex-wrap">
                  <span
                    v-if="c.is_current"
                    class="px-1.5 py-0.5 text-xs rounded bg-cyan-500/20 text-cyan-300"
                    title="The AP is currently on this channel."
                  >Current</span>
                  <span
                    v-if="best && c === best"
                    class="px-1.5 py-0.5 text-xs rounded bg-emerald-500/20 text-emerald-300"
                    title="Fewest nearby APs among non-current channels. Non-DFS preferred when available."
                  >Best</span>
                  <span
                    v-if="c.is_dfs"
                    class="px-1.5 py-0.5 text-xs rounded bg-amber-500/20 text-amber-300 cursor-help"
                    :title="DFS_TOOLTIP"
                  >DFS</span>
                </div>
              </td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>

    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button class="btn-primary" :disabled="!canApply" @click="apply">
        <span class="material-icons icon-sm">power_settings_new</span>
        Apply &amp; restart AP
      </button>
    </template>
  </AppModal>
</template>
