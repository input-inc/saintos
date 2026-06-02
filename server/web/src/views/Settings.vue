<script setup>
import { computed, onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useSettingsStore } from '@/stores/settings'
import { useDisplayStore } from '@/stores/display'
import { useWsTopic } from '@/composables/useWsTopic'
import BoardEditorModal from '@/components/BoardEditorModal.vue'
import WifiChannelModal from '@/components/WifiChannelModal.vue'
import WifiSwitchingOverlay from '@/components/WifiSwitchingOverlay.vue'
import RobotModelTab from '@/components/settings/RobotModelTab.vue'
import Updates from '@/views/Updates.vue'

const ws = useWsStore()
const settings = useSettingsStore()
const display = useDisplayStore()

// Group themes by mode for the Appearance dropdown.
const darkThemes  = computed(() => display.themes.filter(t => t.mode === 'dark'))
const lightThemes = computed(() => display.themes.filter(t => t.mode === 'light'))

const tab = ref('server')

// Server-side data
const clients = ref([])
// `get_firmware_builds` returns a dict keyed by firmware type:
//   { simulation, hardware, raspberrypi, controller, (teensy41?) }
// Each entry has at least { available, version, build_date, ... } —
// the exact extras depend on the type (rp2040 has git_hash/version_full,
// raspberrypi/controller have filename/checksum, teensy41 has bin_size/crc32).
const firmwareBuilds = ref({})
const firmwareLastChecked = ref(null)  // ms epoch — drives the "checked 5s ago" label

// Boards
const boards = ref([])
const boardModal = ref({ open: false, boardId: null })

// Wireless tab
const wifi = ref({ ssid: '', password: '', band: null, channel: null, loaded: false })
const wifiShowPassword = ref(false)
const wifiCredsStatus = ref({ text: '', tone: 'slate' })  // tone: slate|emerald|red
const wifiModalOpen = ref(false)
const wifiSwitching = ref(null)

async function loadWifi () {
  try {
    const cfg = await ws.management('wifi_get_config', {})
    if (!cfg || !cfg.ok) return
    wifi.value = {
      ssid: cfg.ssid || '',
      password: cfg.password || '',
      band: cfg.band || null,
      channel: cfg.channel ?? null,
      loaded: true,
    }
  } catch (e) {
    // Non-fatal — host may not have iw / wifi_admin available.
  }
}

const wifiBandLabel = computed(() => {
  const b = wifi.value.band
  if (b === 'a')  return '5 GHz'
  if (b === 'bg') return '2.4 GHz'
  return b || '?'
})
const wifiCurrentChannelText = computed(() => {
  if (!wifi.value.loaded) return '--'
  return wifi.value.channel
    ? `Currently ${wifiBandLabel.value}, channel ${wifi.value.channel}`
    : `Currently ${wifiBandLabel.value}, channel auto`
})

async function saveWifiCredentials () {
  const ssid = wifi.value.ssid || ''
  const password = wifi.value.password || ''
  if (!confirm(
    `Save new credentials and restart the AP?\n\n` +
    `New SSID: ${ssid}\n\n` +
    `Every connected client will drop and reconnect (~5–10 s). ` +
    `If you change the SSID, your laptop/phone will need to reconnect ` +
    `to the new network name manually.`
  )) return
  try {
    const r = await ws.management('wifi_set_credentials', { ssid, password })
    if (r && r.switching) {
      wifiSwitching.value = {
        detail: `New SSID: ${ssid}. If the dashboard doesn't reconnect within 30 s, refresh after rejoining the WiFi network manually.`,
      }
    } else {
      wifiCredsStatus.value = { text: 'Saved.', tone: 'emerald' }
    }
  } catch (err) {
    console.error('wifi_set_credentials failed:', err)
    wifiCredsStatus.value = { text: `Save failed: ${err.message || err}`, tone: 'red' }
  }
}

function onWifiSwitching (info) { wifiSwitching.value = info || { detail: '' } }
function onWifiSwitchingDone () {
  wifiSwitching.value = null
  loadWifi()
}

// LiveLink live status (already broadcast on the 'livelink' topic).
const livelink = useWsTopic(() => 'livelink')

// Connection settings live in localStorage (per-browser, like the legacy login).
const conn = ref({ host: window.location.host, password: '' })
const connMessage = ref('')

onMounted(async () => {
  try {
    const s = JSON.parse(localStorage.getItem('saint_ws_settings') || '{}')
    if (s.host) conn.value.host = s.host
    if (s.password) conn.value.password = s.password
  } catch (_) {}
  settings.load()
  reloadClients()
  reloadBuilds()
  reloadBoards()
  loadWifi()
})

async function reloadClients () {
  try { const r = await ws.management('list_clients'); clients.value = r?.clients || [] }
  catch (_) {}
}
async function reloadBuilds () {
  try {
    // Response is the dict from state_manager.get_all_firmware_builds() —
    // keyed by firmware type, each value is the per-type info dict (or
    // {available: false, ...} when nothing is staged).
    const r = await ws.management('get_firmware_builds', {})
    firmwareBuilds.value = r || {}
    firmwareLastChecked.value = Date.now()
  } catch (e) {
    console.warn('get_firmware_builds failed:', e)
    firmwareBuilds.value = {}
    firmwareLastChecked.value = Date.now()
  }
}
async function reloadBoards () {
  try { const r = await ws.management('list_boards', {}); boards.value = r?.boards || [] }
  catch (_) {}
}

function saveConn () {
  try {
    localStorage.setItem('saint_ws_settings', JSON.stringify(conn.value))
    connMessage.value = 'Saved. Reconnect to apply.'
  } catch (e) { connMessage.value = `Save failed: ${e.message}` }
}
function reconnect () {
  ws.disconnect()
  setTimeout(() => ws.connect(), 200)
  connMessage.value = 'Reconnecting…'
}

async function disconnectClient (id) {
  try { await ws.management('disconnect_client', { client_id: id }); reloadClients() }
  catch (e) { console.warn('disconnect_client failed:', e) }
}

async function removeBoard (boardId) {
  if (!confirm(`Remove board ${boardId}?`)) return
  try {
    await ws.management('delete_board', { board_id: boardId })
    reloadBoards()
  } catch (e) {
    console.warn('delete_board failed:', e)
  }
}

function fmtDuration (sec) {
  if (sec == null) return '--'
  if (sec < 60)    return `${Math.floor(sec)}s`
  if (sec < 3600)  return `${Math.floor(sec / 60)}m ${Math.floor(sec % 60)}s`
  if (sec < 86400) return `${Math.floor(sec / 3600)}h ${Math.floor((sec % 3600) / 60)}m`
  return `${Math.floor(sec / 86400)}d ${Math.floor((sec % 86400) / 3600)}h`
}

// ── Firmware-tab formatters ─────────────────────────────────────────
// File size in binary units. Backend returns `bin_size` for RP2040/Teensy
// and (sometimes) raw `size` for raspberrypi/controller from info.json packages
// array — but `get_firmware_info_for_type` doesn't lift `size` into the
// top-level dict today, so for raspberrypi/controller this returns "—".
function fmtBytes (bytes) {
  if (bytes == null) return '—'
  const KiB = 1024, MiB = KiB * 1024
  if (bytes < KiB) return `${bytes} B`
  if (bytes < MiB) return `${(bytes / KiB).toFixed(1)} KB`
  return `${(bytes / MiB).toFixed(2)} MB`
}

// Build date arrives in two shapes:
//   - RP2040/Teensy: "YYYY-MM-DD HH:MM:SS" (local-time string)
//   - raspberrypi/controller: ISO "2026-05-26T02:58:32Z"
// Parse both, fall back to the raw string if parsing fails.
function parseBuildDate (s) {
  if (!s) return null
  const direct = Date.parse(s)
  if (!Number.isNaN(direct)) return new Date(direct)
  // Coerce "YYYY-MM-DD HH:MM:SS" → "YYYY-MM-DDTHH:MM:SS" for older Safari.
  const alt = Date.parse(String(s).replace(' ', 'T'))
  if (!Number.isNaN(alt)) return new Date(alt)
  return null
}
function fmtBuildDate (s) {
  if (!s) return '—'
  const d = parseBuildDate(s)
  if (!d) return s
  const ageMs = Date.now() - d.getTime()
  if (ageMs < 0) return d.toLocaleString()  // future-dated, just show absolute
  const sec = Math.floor(ageMs / 1000)
  if (sec < 60)        return `${sec} sec ago`
  if (sec < 3600)      return `${Math.floor(sec / 60)} min ago`
  if (sec < 86400)     return `${Math.floor(sec / 3600)} hr ago`
  if (sec < 86400 * 7) return `${Math.floor(sec / 86400)} days ago`
  return d.toLocaleDateString() + ' ' + d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
}

// Short hash for display. Accepts sha256 (raspberrypi/controller) or git hash (rp2040).
function fmtShortHash (h) {
  if (!h) return null
  const s = String(h)
  return s.length > 12 ? s.slice(0, 12) + '…' : s
}

function fmtChecked (ms) {
  if (!ms) return ''
  const sec = Math.floor((Date.now() - ms) / 1000)
  if (sec < 5)    return 'just now'
  if (sec < 60)   return `${sec} sec ago`
  if (sec < 3600) return `${Math.floor(sec / 60)} min ago`
  return new Date(ms).toLocaleTimeString()
}

// Static descriptors for each firmware type. Cards render in this order;
// teensy41 only appears when the backend actually returned an entry for it.
const FIRMWARE_TYPES = [
  { key: 'simulation', label: 'RP2040 Simulation Build',  detail: 'For Renode-simulated RP2040 nodes',          icon: 'computer',         iconClass: 'text-cyan-400'   },
  { key: 'hardware',   label: 'RP2040 Hardware Build',    detail: 'For physical Feather RP2040 + Ethernet nodes', icon: 'developer_board',  iconClass: 'text-violet-400' },
  { key: 'teensy41',   label: 'Teensy 4.1 Hardware Build', detail: 'For physical Teensy 4.1 nodes',             icon: 'developer_board',  iconClass: 'text-amber-400'  },
  { key: 'raspberrypi',       label: 'Raspberry Pi 5 Production Build', detail: 'For Raspberry Pi 5 nodes with GPIO control', icon: 'memory',     iconClass: 'text-rose-400'   },
  { key: 'controller', label: 'Steam Deck Controller AppImage', detail: 'Tauri operator app — self-contained AppImage', icon: 'sports_esports', iconClass: 'text-cyan-400' },
]

const firmwareCards = computed(() => {
  const builds = firmwareBuilds.value || {}
  return FIRMWARE_TYPES
    // Hide Teensy entirely when the backend didn't include it (current
    // get_all_firmware_builds() doesn't, but we surface it if it shows up).
    .filter(t => t.key !== 'teensy41' || t.key in builds)
    .map(t => {
      const info = builds[t.key]
      const present = info != null
      const available = !!(info && info.available)
      const version = info?.version_full || info?.version || null
      const buildDate = info?.build_date || null
      const size = info?.bin_size ?? info?.size ?? null
      const hash = info?.git_hash || info?.checksum || info?.bin_crc32 || null
      // Status badge: "Available" when staged; "Missing" when entry
      // exists but available=false; "Unknown" when the backend didn't
      // return the type at all (only possible if FIRMWARE_TYPES grew
      // beyond get_all_firmware_builds()'s keys).
      let status, statusClass
      if (!present) {
        status = 'Unknown'
        statusClass = 'bg-surface text-fg-muted'
      } else if (available) {
        status = 'Available'
        statusClass = 'bg-emerald-500/20 text-emerald-400'
      } else {
        status = 'Missing'
        statusClass = 'bg-surface text-fg-muted'
      }
      return {
        ...t,
        version,
        buildDate,
        size,
        hash,
        filename: info?.filename || null,
        status,
        statusClass,
      }
    })
})

const receiver = computed(() => livelink.value?.receiver || {})
const router   = computed(() => livelink.value?.router   || {})

const tabs = [
  { id: 'server',    label: 'Server',    icon: 'dns' },
  { id: 'interface', label: 'Interface', icon: 'display_settings' },
  { id: 'wireless',  label: 'Wireless',  icon: 'wifi' },
  { id: 'updates',   label: 'Updates',   icon: 'system_update' },
  { id: 'livelink',  label: 'LiveLink',  icon: 'face' },
  { id: 'firmware',  label: 'Firmware',  icon: 'memory' },
  { id: 'boards',    label: 'Boards',    icon: 'developer_board' },
  { id: 'robot',     label: 'Robot Model', icon: 'view_in_ar' },
]
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-fg-strong">Settings</h2>
      <div class="flex items-center gap-3">
        <span v-if="settings.dirty" class="text-xs text-amber-300">Unsaved changes</span>
        <span v-else-if="settings.saving" class="text-xs text-fg-faint">Saving…</span>
        <button class="btn-secondary" :disabled="!settings.dirty" @click="settings.reset()">Reset</button>
        <button class="btn-primary" :disabled="!settings.dirty || settings.saving" @click="settings.save()">
          <span class="material-icons icon-sm">save</span>
          Save changes
        </button>
      </div>
    </div>

    <div v-if="settings.error" class="mb-4 p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">{{ settings.error }}</div>

    <div class="flex gap-1 border-b border-line/50 mb-6 overflow-x-auto">
      <button
        v-for="t in tabs"
        :key="t.id"
        :class="['node-tab', tab === t.id ? 'router-link-active' : '']"
        @click="tab = t.id"
      >
        <span class="material-icons icon-sm align-middle">{{ t.icon }}</span>
        {{ t.label }}
      </button>
    </div>

    <!-- ─── Server ──────────────────────────────────────────────────────── -->
    <div v-show="tab === 'server'" class="space-y-6">
      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-4 flex items-center gap-2">
          <span class="material-icons text-cyan-400">cable</span>
          This browser's connection
        </h3>
        <div class="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label class="block text-sm font-medium text-fg mb-1">Server host</label>
            <input v-model="conn.host" type="text" class="input-field w-full" placeholder="opensaint.local" />
          </div>
          <div>
            <label class="block text-sm font-medium text-fg mb-1">Password</label>
            <input v-model="conn.password" type="password" class="input-field w-full" placeholder="Leave empty if none" />
          </div>
        </div>
        <div class="flex items-center gap-2 mt-3">
          <button class="btn-primary" @click="saveConn">Save</button>
          <button class="btn-secondary" @click="reconnect">Reconnect</button>
          <span v-if="connMessage" class="text-xs text-fg-muted ml-2">{{ connMessage }}</span>
        </div>
      </div>

      <div v-if="settings.current" class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-4 flex items-center gap-2">
          <span class="material-icons text-cyan-400">badge</span>
          Identity
        </h3>
        <label class="block text-sm font-medium text-fg mb-1">Server name</label>
        <input v-model="settings.current.server_name" type="text" class="input-field w-full max-w-md" placeholder="SAINT-01" />
        <p class="text-xs text-fg-faint mt-1">Display name for this server instance.</p>
      </div>

      <div v-if="settings.current" class="grid grid-cols-1 md:grid-cols-3 gap-4">
        <div class="card">
          <h3 class="text-base font-semibold text-fg-strong mb-3 flex items-center gap-2">
            <span class="material-icons text-violet-400 text-lg">lock</span>
            WebSocket
          </h3>
          <label class="block text-sm font-medium text-fg mb-1">Password</label>
          <input v-model="settings.current.websocket.password" type="password" class="input-field w-full" placeholder="Empty = disabled" />
          <p class="text-xs text-fg-faint mt-1 mb-3">Clients must authenticate with this.</p>
          <label class="block text-sm font-medium text-fg mb-1">Auth timeout (sec)</label>
          <input v-model.number="settings.current.websocket.auth_timeout" type="number" min="1" max="60" class="input-field w-24" />
        </div>
        <div class="card">
          <h3 class="text-base font-semibold text-fg-strong mb-3 flex items-center gap-2">
            <span class="material-icons text-emerald-400 text-lg">language</span>
            Network
          </h3>
          <label class="block text-sm font-medium text-fg mb-1">Web port</label>
          <input v-model.number="settings.current.network.web_port" type="number" min="1" max="65535" class="input-field w-24" />
          <p class="text-xs text-fg-faint mt-1 mb-3">Requires restart.</p>
          <label class="block text-sm font-medium text-fg mb-1">WebSocket port</label>
          <input v-model.number="settings.current.network.ws_port" type="number" min="1" max="65535" class="input-field w-24" placeholder="same" />
        </div>
        <div class="card">
          <h3 class="text-base font-semibold text-fg-strong mb-3 flex items-center gap-2">
            <span class="material-icons text-orange-400 text-lg">sync_alt</span>
            ROS Bridge
          </h3>
          <label class="block text-sm font-medium text-fg mb-1">Command throttle (ms)</label>
          <input v-model.number="settings.current.ros_bridge.command_throttle_ms" type="number" min="10" max="1000" step="10" class="input-field w-24" />
          <p class="text-xs text-fg-faint mt-1">Min interval between control commands.</p>
        </div>
      </div>

      <div class="card">
        <div class="flex items-center justify-between mb-3">
          <h3 class="text-lg font-semibold text-fg-strong flex items-center gap-2">
            <span class="material-icons text-cyan-400">people</span>
            Connected clients
          </h3>
          <button class="btn-secondary text-sm" @click="reloadClients"><span class="material-icons icon-sm">refresh</span>Refresh</button>
        </div>
        <div v-if="!clients.length" class="text-sm text-fg-muted italic">No connected clients.</div>
        <ul v-else class="divide-y divide-line/50 text-sm">
          <li v-for="c in clients" :key="c.id" class="flex items-center gap-3 py-2 font-mono">
            <span :class="['w-2 h-2 rounded-full', c.authenticated ? 'bg-emerald-500' : 'bg-amber-500']" />
            <span>{{ c.id }}</span>
            <span class="text-fg-faint">{{ c.ip || '' }}</span>
            <span class="flex-1" />
            <span class="text-fg-faint">{{ fmtDuration(c.connected_for) }}</span>
            <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong" @click="disconnectClient(c.id)">
              <span class="material-icons icon-sm">power_settings_new</span>
            </button>
          </li>
        </ul>
      </div>
    </div>

    <!-- ─── Interface ───────────────────────────────────────────────────── -->
    <div v-show="tab === 'interface'" class="space-y-6">
      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-4 flex items-center gap-2">
          <span class="material-icons text-amber-400">thermostat</span>
          Temperature display
        </h3>
        <div class="flex items-center gap-6">
          <label class="flex items-center gap-2 cursor-pointer">
            <input v-model="display.temperatureUnit" type="radio" value="celsius" class="w-4 h-4 text-cyan-600 bg-surface border-line-strong focus:ring-cyan-500" />
            <span class="text-sm text-fg">Celsius (°C)</span>
          </label>
          <label class="flex items-center gap-2 cursor-pointer">
            <input v-model="display.temperatureUnit" type="radio" value="fahrenheit" class="w-4 h-4 text-cyan-600 bg-surface border-line-strong focus:ring-cyan-500" />
            <span class="text-sm text-fg">Fahrenheit (°F)</span>
          </label>
        </div>
        <p class="text-xs text-fg-faint mt-1">How temperatures are displayed throughout the interface.</p>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-4 flex items-center gap-2">
          <span class="material-icons text-purple-400">palette</span>
          Appearance
        </h3>
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Theme</span>
          <select v-model="display.theme" class="input-field w-full max-w-xs">
            <optgroup label="Dark">
              <option v-for="t in darkThemes" :key="t.value" :value="t.value">{{ t.label }}</option>
            </optgroup>
            <optgroup label="Light">
              <option v-for="t in lightThemes" :key="t.value" :value="t.value">{{ t.label }}</option>
            </optgroup>
          </select>
        </label>
        <p class="text-xs text-fg-faint mt-2">Saved per browser; takes effect immediately.</p>
      </div>
    </div>

    <!-- ─── Wireless ────────────────────────────────────────────────────── -->
    <!-- Operator-managed AP credentials + channel picker. Backed by
         wifi_admin on the server (NetworkManager). Every save here
         triggers an AP restart and a ~5-10 s WS outage; the overlay
         covers the gap and ws.js auto-reconnects. -->
    <div v-show="tab === 'wireless'" class="space-y-6">
      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-4 flex items-center gap-2">
          <span class="material-icons text-cyan-400">wifi</span>
          Wireless AP
        </h3>
        <div class="text-xs text-fg-muted mb-4 p-3 rounded bg-amber-900/30 border border-amber-700/50">
          <span class="material-icons text-amber-400 text-sm align-middle">warning</span>
          Saving credentials or switching the channel restarts the AP.
          Every connected client (including this dashboard) will drop and
          reconnect — expect a ~5-10 s blackout. Do it while the robot is idle.
        </div>
        <div class="grid grid-cols-1 md:grid-cols-2 gap-4 mb-4">
          <div>
            <label class="block text-sm font-medium text-fg mb-1">SSID</label>
            <input v-model="wifi.ssid" type="text" maxlength="32" placeholder="OpenSAINT" class="input-field w-full" />
            <p class="text-xs text-fg-faint mt-1">1-32 printable ASCII characters.</p>
          </div>
          <div>
            <label class="block text-sm font-medium text-fg mb-1">Password</label>
            <div class="flex gap-2">
              <input
                v-model="wifi.password"
                :type="wifiShowPassword ? 'text' : 'password'"
                minlength="8"
                maxlength="63"
                placeholder="••••••••"
                class="input-field w-full"
              />
              <button
                type="button"
                class="btn-secondary text-xs"
                title="Show / hide password"
                @click="wifiShowPassword = !wifiShowPassword"
              >
                <span class="material-icons icon-sm">{{ wifiShowPassword ? 'visibility_off' : 'visibility' }}</span>
              </button>
            </div>
            <p class="text-xs text-fg-faint mt-1">8-63 characters (WPA2-PSK).</p>
          </div>
        </div>
        <div class="flex items-center gap-3 mb-6">
          <button class="btn-primary" @click="saveWifiCredentials">
            <span class="material-icons icon-sm">save</span>
            Save credentials &amp; restart AP
          </button>
          <span
            v-if="wifiCredsStatus.text"
            :class="{
              'text-sm': true,
              'text-fg-faint':  wifiCredsStatus.tone === 'slate',
              'text-emerald-400': wifiCredsStatus.tone === 'emerald',
              'text-red-300':    wifiCredsStatus.tone === 'red',
            }"
          >{{ wifiCredsStatus.text }}</span>
        </div>

        <div class="pt-4 border-t border-line">
          <div class="flex items-center justify-between mb-2">
            <h4 class="text-sm font-semibold text-fg">Channel</h4>
            <span class="text-xs text-fg-faint">{{ wifiCurrentChannelText }}</span>
          </div>
          <p class="text-xs text-fg-faint mb-3">
            Scan nearby APs to see which channels are busy, then pick a quieter one.
            Brief beacon interruption during scan; full disconnect on apply.
          </p>
          <button class="btn-secondary" @click="wifiModalOpen = true">
            <span class="material-icons icon-sm">radar</span>
            Find Better Channel…
          </button>
        </div>
      </div>
    </div>

    <!-- ─── Updates ─────────────────────────────────────────────────────── -->
    <div v-show="tab === 'updates'">
      <Updates embedded />
    </div>

    <!-- ─── LiveLink ────────────────────────────────────────────────────── -->
    <div v-show="tab === 'livelink'" class="space-y-6">
      <div v-if="settings.current?.livelink" class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-4 flex items-center gap-2">
          <span class="material-icons text-pink-400">face</span>
          LiveLink receiver
        </h3>
        <label class="flex items-center gap-3 mb-4">
          <input v-model="settings.current.livelink.enabled" type="checkbox" class="rounded bg-surface border-line-strong" />
          <span class="text-sm text-fg">Enable LiveLink receiver</span>
        </label>
        <div>
          <label class="block text-sm font-medium text-fg mb-1">UDP port</label>
          <input v-model.number="settings.current.livelink.port" type="number" min="1024" max="65535" class="input-field w-32" />
          <p class="text-xs text-fg-faint mt-1">Requires restart.</p>
        </div>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-3 flex items-center gap-2">
          <span class="material-icons text-pink-400">monitoring</span>
          Status
        </h3>
        <div class="space-y-2 text-sm">
          <div class="flex items-center justify-between py-2 border-b border-line/50">
            <span class="text-fg-muted">Connection</span>
            <span>{{ receiver.connected ? 'Connected' : 'Idle' }}</span>
          </div>
          <div class="flex items-center justify-between py-2 border-b border-line/50">
            <span class="text-fg-muted">Active source</span>
            <span class="font-mono">{{ receiver.source_address || '—' }}</span>
          </div>
          <div class="flex items-center justify-between py-2">
            <span class="text-fg-muted">Packets/sec</span>
            <span>{{ receiver.fps != null ? receiver.fps.toFixed(1) : '—' }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- ─── Firmware ────────────────────────────────────────────────────── -->
    <!-- Firmware builds staged on the server. The list mirrors the legacy
         dashboard Firmware tab: RP2040 sim+hw, Teensy 4.1 (when present),
         Pi 5, and the Steam Deck controller AppImage (added in a2e788a). -->
    <div v-show="tab === 'firmware'" class="space-y-6">
      <div class="card">
        <div class="flex items-center justify-between mb-4">
          <h3 class="text-lg font-semibold text-fg-strong flex items-center gap-2">
            <span class="material-icons text-cyan-400">memory</span>
            Firmware builds on this server
          </h3>
          <div class="flex items-center gap-3">
            <span v-if="firmwareLastChecked" class="text-xs text-fg-faint">
              Checked {{ fmtChecked(firmwareLastChecked) }}
            </span>
            <button class="btn-secondary text-sm" @click="reloadBuilds">
              <span class="material-icons icon-sm">refresh</span>
              Refresh
            </button>
          </div>
        </div>
        <p class="text-sm text-fg-muted mb-4">
          OTA targets staged under <code class="text-cyan-300 text-xs">server/resources/firmware/</code>.
          Adopted nodes (and the controller app) fetch from here.
        </p>

        <div v-if="!firmwareCards.length" class="text-sm text-fg-muted italic">
          No firmware bundled.
        </div>
        <div v-else class="space-y-3">
          <div
            v-for="card in firmwareCards"
            :key="card.key"
            class="bg-panel/50 rounded-lg p-4 border border-line"
          >
            <div class="flex items-start justify-between gap-3">
              <div class="flex items-center gap-2 min-w-0">
                <span class="material-icons" :class="card.iconClass">{{ card.icon }}</span>
                <div class="min-w-0">
                  <h4 class="font-medium text-fg-strong truncate">{{ card.label }}</h4>
                  <p class="text-xs text-fg-faint truncate">{{ card.detail }}</p>
                </div>
              </div>
              <span :class="['px-2 py-1 text-xs font-medium rounded-full whitespace-nowrap', card.statusClass]">
                {{ card.status }}
              </span>
            </div>
            <div class="mt-3 ml-8 grid grid-cols-1 sm:grid-cols-2 gap-x-4 gap-y-2 text-sm">
              <div>
                <div class="stat-label">Version</div>
                <div class="font-mono text-fg-strong">{{ card.version || '—' }}</div>
              </div>
              <div>
                <div class="stat-label">Built</div>
                <div class="font-mono text-xs text-fg-strong" :title="card.buildDate || ''">
                  {{ fmtBuildDate(card.buildDate) }}
                </div>
              </div>
              <div v-if="card.filename" class="sm:col-span-2">
                <div class="stat-label">Package</div>
                <div class="font-mono text-xs text-fg-strong break-all">{{ card.filename }}</div>
              </div>
              <div>
                <div class="stat-label">Size</div>
                <div class="font-mono text-xs text-fg-strong">{{ fmtBytes(card.size) }}</div>
              </div>
              <div>
                <div class="stat-label">Checksum</div>
                <div
                  class="font-mono text-xs text-fg-strong"
                  :title="card.hash || ''"
                >{{ fmtShortHash(card.hash) || '—' }}</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ─── Boards ──────────────────────────────────────────────────────── -->
    <div v-show="tab === 'boards'" class="space-y-4">
      <div class="card">
        <div class="flex items-center justify-between mb-3">
          <h3 class="text-lg font-semibold text-fg-strong flex items-center gap-2">
            <span class="material-icons text-cyan-400">developer_board</span>
            Board configurations
          </h3>
          <button class="btn-primary text-sm" @click="boardModal = { open: true, boardId: null }">
            <span class="material-icons icon-sm">add</span>
            New board
          </button>
        </div>
        <p class="text-sm text-fg-muted mb-4">
          Pin layouts and built-in peripherals per board. Built-in entries ship with SAINT.OS and are read-only; operator-authored entries are stored as YAML under <code class="text-cyan-300 text-xs">saint_os/config/boards/&lt;chip&gt;/</code>.
        </p>
        <ul v-if="boards.length" class="divide-y divide-line/50 text-sm">
          <li v-for="b in boards" :key="b.board_id" class="flex items-center gap-3 py-2">
            <span class="material-icons icon-sm" :class="b.builtin ? 'text-fg-faint' : 'text-cyan-400'">developer_board</span>
            <span class="font-mono">{{ b.board_id }}</span>
            <span class="text-xs text-fg-faint">{{ b.display_name }}</span>
            <span v-if="b.builtin" class="px-2 py-0.5 text-xs rounded-full bg-surface text-fg">Built-in</span>
            <span class="flex-1" />
            <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong" @click="boardModal = { open: true, boardId: b.board_id }">
              <span class="material-icons icon-sm">{{ b.builtin ? 'visibility' : 'edit' }}</span>
            </button>
            <button v-if="!b.builtin" class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong" @click="removeBoard(b.board_id)">
              <span class="material-icons icon-sm">delete</span>
            </button>
          </li>
        </ul>
        <p v-else class="text-sm text-fg-muted italic">No boards registered.</p>
      </div>
    </div>

    <!-- ─── Robot Model ─────────────────────────────────────────────────── -->
    <div v-show="tab === 'robot'">
      <RobotModelTab />
    </div>

    <BoardEditorModal
      v-if="boardModal.open"
      :board-id="boardModal.boardId"
      @close="boardModal = { open: false, boardId: null }"
      @saved="reloadBoards()"
    />

    <WifiChannelModal
      v-if="wifiModalOpen"
      @close="wifiModalOpen = false"
      @switching="onWifiSwitching"
    />
    <WifiSwitchingOverlay
      v-if="wifiSwitching"
      :detail="wifiSwitching.detail"
      @close="onWifiSwitchingDone"
    />
  </section>
</template>
