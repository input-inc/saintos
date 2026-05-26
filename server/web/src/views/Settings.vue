<script setup>
import { computed, onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useSettingsStore } from '@/stores/settings'
import { useDisplayStore } from '@/stores/display'
import { useWsTopic } from '@/composables/useWsTopic'
import BoardEditorModal from '@/components/BoardEditorModal.vue'
import Updates from '@/views/Updates.vue'

const ws = useWsStore()
const settings = useSettingsStore()
const display = useDisplayStore()

const tab = ref('server')

// Server-side data
const clients = ref([])
const firmwareBuilds = ref([])

// Boards
const boards = ref([])
const boardModal = ref({ open: false, boardId: null })

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
})

async function reloadClients () {
  try { const r = await ws.management('list_clients'); clients.value = r?.clients || [] }
  catch (_) {}
}
async function reloadBuilds () {
  try { const r = await ws.management('get_firmware_builds', {}); firmwareBuilds.value = r?.builds || [] }
  catch (_) {}
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

const receiver = computed(() => livelink.value?.receiver || {})
const router   = computed(() => livelink.value?.router   || {})

const tabs = [
  { id: 'server',    label: 'Server',    icon: 'dns' },
  { id: 'interface', label: 'Interface', icon: 'display_settings' },
  { id: 'updates',   label: 'Updates',   icon: 'system_update' },
  { id: 'livelink',  label: 'LiveLink',  icon: 'face' },
  { id: 'firmware',  label: 'Firmware',  icon: 'memory' },
  { id: 'boards',    label: 'Boards',    icon: 'developer_board' },
]
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-white">Settings</h2>
      <div class="flex items-center gap-3">
        <span v-if="settings.dirty" class="text-xs text-amber-300">Unsaved changes</span>
        <span v-else-if="settings.saving" class="text-xs text-slate-500">Saving…</span>
        <button class="btn-secondary" :disabled="!settings.dirty" @click="settings.reset()">Reset</button>
        <button class="btn-primary" :disabled="!settings.dirty || settings.saving" @click="settings.save()">
          <span class="material-icons icon-sm">save</span>
          Save changes
        </button>
      </div>
    </div>

    <div v-if="settings.error" class="mb-4 p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">{{ settings.error }}</div>

    <div class="flex gap-1 border-b border-slate-700/50 mb-6 overflow-x-auto">
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
        <h3 class="text-lg font-semibold text-white mb-4 flex items-center gap-2">
          <span class="material-icons text-cyan-400">cable</span>
          This browser's connection
        </h3>
        <div class="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-1">Server host</label>
            <input v-model="conn.host" type="text" class="input-field w-full" placeholder="opensaint.local" />
          </div>
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-1">Password</label>
            <input v-model="conn.password" type="password" class="input-field w-full" placeholder="Leave empty if none" />
          </div>
        </div>
        <div class="flex items-center gap-2 mt-3">
          <button class="btn-primary" @click="saveConn">Save</button>
          <button class="btn-secondary" @click="reconnect">Reconnect</button>
          <span v-if="connMessage" class="text-xs text-slate-400 ml-2">{{ connMessage }}</span>
        </div>
      </div>

      <div v-if="settings.current" class="card">
        <h3 class="text-lg font-semibold text-white mb-4 flex items-center gap-2">
          <span class="material-icons text-cyan-400">badge</span>
          Identity
        </h3>
        <label class="block text-sm font-medium text-slate-300 mb-1">Server name</label>
        <input v-model="settings.current.server_name" type="text" class="input-field w-full max-w-md" placeholder="SAINT-01" />
        <p class="text-xs text-slate-500 mt-1">Display name for this server instance.</p>
      </div>

      <div v-if="settings.current" class="grid grid-cols-1 md:grid-cols-3 gap-4">
        <div class="card">
          <h3 class="text-base font-semibold text-white mb-3 flex items-center gap-2">
            <span class="material-icons text-violet-400 text-lg">lock</span>
            WebSocket
          </h3>
          <label class="block text-sm font-medium text-slate-300 mb-1">Password</label>
          <input v-model="settings.current.websocket.password" type="password" class="input-field w-full" placeholder="Empty = disabled" />
          <p class="text-xs text-slate-500 mt-1 mb-3">Clients must authenticate with this.</p>
          <label class="block text-sm font-medium text-slate-300 mb-1">Auth timeout (sec)</label>
          <input v-model.number="settings.current.websocket.auth_timeout" type="number" min="1" max="60" class="input-field w-24" />
        </div>
        <div class="card">
          <h3 class="text-base font-semibold text-white mb-3 flex items-center gap-2">
            <span class="material-icons text-emerald-400 text-lg">language</span>
            Network
          </h3>
          <label class="block text-sm font-medium text-slate-300 mb-1">Web port</label>
          <input v-model.number="settings.current.network.web_port" type="number" min="1" max="65535" class="input-field w-24" />
          <p class="text-xs text-slate-500 mt-1 mb-3">Requires restart.</p>
          <label class="block text-sm font-medium text-slate-300 mb-1">WebSocket port</label>
          <input v-model.number="settings.current.network.ws_port" type="number" min="1" max="65535" class="input-field w-24" placeholder="same" />
        </div>
        <div class="card">
          <h3 class="text-base font-semibold text-white mb-3 flex items-center gap-2">
            <span class="material-icons text-orange-400 text-lg">sync_alt</span>
            ROS Bridge
          </h3>
          <label class="block text-sm font-medium text-slate-300 mb-1">Command throttle (ms)</label>
          <input v-model.number="settings.current.ros_bridge.command_throttle_ms" type="number" min="10" max="1000" step="10" class="input-field w-24" />
          <p class="text-xs text-slate-500 mt-1">Min interval between control commands.</p>
        </div>
      </div>

      <div class="card">
        <div class="flex items-center justify-between mb-3">
          <h3 class="text-lg font-semibold text-white flex items-center gap-2">
            <span class="material-icons text-cyan-400">people</span>
            Connected clients
          </h3>
          <button class="btn-secondary text-sm" @click="reloadClients"><span class="material-icons icon-sm">refresh</span>Refresh</button>
        </div>
        <div v-if="!clients.length" class="text-sm text-slate-400 italic">No connected clients.</div>
        <ul v-else class="divide-y divide-slate-700/50 text-sm">
          <li v-for="c in clients" :key="c.id" class="flex items-center gap-3 py-2 font-mono">
            <span :class="['w-2 h-2 rounded-full', c.authenticated ? 'bg-emerald-500' : 'bg-amber-500']" />
            <span>{{ c.id }}</span>
            <span class="text-slate-500">{{ c.ip || '' }}</span>
            <span class="flex-1" />
            <span class="text-slate-500">{{ fmtDuration(c.connected_for) }}</span>
            <button class="btn-sm bg-slate-700 hover:bg-red-600 text-slate-300 hover:text-white" @click="disconnectClient(c.id)">
              <span class="material-icons icon-sm">power_settings_new</span>
            </button>
          </li>
        </ul>
      </div>
    </div>

    <!-- ─── Interface ───────────────────────────────────────────────────── -->
    <div v-show="tab === 'interface'" class="space-y-6">
      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-4 flex items-center gap-2">
          <span class="material-icons text-amber-400">thermostat</span>
          Temperature display
        </h3>
        <div class="flex items-center gap-6">
          <label class="flex items-center gap-2 cursor-pointer">
            <input v-model="display.temperatureUnit" type="radio" value="celsius" class="w-4 h-4 text-cyan-600 bg-slate-700 border-slate-600 focus:ring-cyan-500" />
            <span class="text-sm text-slate-300">Celsius (°C)</span>
          </label>
          <label class="flex items-center gap-2 cursor-pointer">
            <input v-model="display.temperatureUnit" type="radio" value="fahrenheit" class="w-4 h-4 text-cyan-600 bg-slate-700 border-slate-600 focus:ring-cyan-500" />
            <span class="text-sm text-slate-300">Fahrenheit (°F)</span>
          </label>
        </div>
        <p class="text-xs text-slate-500 mt-1">How temperatures are displayed throughout the interface.</p>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-4 flex items-center gap-2">
          <span class="material-icons text-purple-400">palette</span>
          Appearance
        </h3>
        <div class="flex items-center gap-6">
          <label class="flex items-center gap-2"><input type="radio" name="theme" value="dark" checked class="w-4 h-4" /><span class="text-sm text-slate-300">Dark</span></label>
          <label class="flex items-center gap-2 opacity-50"><input type="radio" name="theme" value="light" disabled class="w-4 h-4" /><span class="text-sm text-slate-300">Light (coming soon)</span></label>
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
        <h3 class="text-lg font-semibold text-white mb-4 flex items-center gap-2">
          <span class="material-icons text-pink-400">face</span>
          LiveLink receiver
        </h3>
        <label class="flex items-center gap-3 mb-4">
          <input v-model="settings.current.livelink.enabled" type="checkbox" class="rounded bg-slate-700 border-slate-600" />
          <span class="text-sm text-slate-300">Enable LiveLink receiver</span>
        </label>
        <div>
          <label class="block text-sm font-medium text-slate-300 mb-1">UDP port</label>
          <input v-model.number="settings.current.livelink.port" type="number" min="1024" max="65535" class="input-field w-32" />
          <p class="text-xs text-slate-500 mt-1">Requires restart.</p>
        </div>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <span class="material-icons text-pink-400">monitoring</span>
          Status
        </h3>
        <div class="space-y-2 text-sm">
          <div class="flex items-center justify-between py-2 border-b border-slate-700/50">
            <span class="text-slate-400">Connection</span>
            <span>{{ receiver.connected ? 'Connected' : 'Idle' }}</span>
          </div>
          <div class="flex items-center justify-between py-2 border-b border-slate-700/50">
            <span class="text-slate-400">Active source</span>
            <span class="font-mono">{{ receiver.source_address || '—' }}</span>
          </div>
          <div class="flex items-center justify-between py-2">
            <span class="text-slate-400">Packets/sec</span>
            <span>{{ receiver.fps != null ? receiver.fps.toFixed(1) : '—' }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- ─── Firmware ────────────────────────────────────────────────────── -->
    <div v-show="tab === 'firmware'" class="space-y-6">
      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-4 flex items-center gap-2">
          <span class="material-icons text-cyan-400">memory</span>
          Firmware builds on this server
        </h3>
        <div v-if="!firmwareBuilds.length" class="text-sm text-slate-400 italic">No firmware bundled.</div>
        <ul v-else class="divide-y divide-slate-700/50 text-sm">
          <li v-for="b in firmwareBuilds" :key="b.type" class="py-3 grid grid-cols-3 gap-3">
            <div>
              <div class="font-semibold text-white">{{ b.type }}</div>
              <div class="text-xs text-slate-500">{{ b.chip_family || '' }}</div>
            </div>
            <div>
              <div class="stat-label">Version</div>
              <div class="font-mono">{{ b.version || '—' }}</div>
            </div>
            <div>
              <div class="stat-label">Built</div>
              <div class="font-mono text-xs">{{ b.build_date || '—' }}</div>
            </div>
          </li>
        </ul>
      </div>
    </div>

    <!-- ─── Boards ──────────────────────────────────────────────────────── -->
    <div v-show="tab === 'boards'" class="space-y-4">
      <div class="card">
        <div class="flex items-center justify-between mb-3">
          <h3 class="text-lg font-semibold text-white flex items-center gap-2">
            <span class="material-icons text-cyan-400">developer_board</span>
            Board configurations
          </h3>
          <button class="btn-primary text-sm" @click="boardModal = { open: true, boardId: null }">
            <span class="material-icons icon-sm">add</span>
            New board
          </button>
        </div>
        <p class="text-sm text-slate-400 mb-4">
          Pin layouts and built-in peripherals per board. Built-in entries ship with SAINT.OS and are read-only; operator-authored entries are stored as YAML under <code class="text-cyan-300 text-xs">saint_os/config/boards/&lt;chip&gt;/</code>.
        </p>
        <ul v-if="boards.length" class="divide-y divide-slate-700/50 text-sm">
          <li v-for="b in boards" :key="b.board_id" class="flex items-center gap-3 py-2">
            <span class="material-icons icon-sm" :class="b.builtin ? 'text-slate-500' : 'text-cyan-400'">developer_board</span>
            <span class="font-mono">{{ b.board_id }}</span>
            <span class="text-xs text-slate-500">{{ b.display_name }}</span>
            <span v-if="b.builtin" class="px-2 py-0.5 text-xs rounded-full bg-slate-700 text-slate-300">Built-in</span>
            <span class="flex-1" />
            <button class="btn-sm bg-slate-700 hover:bg-slate-600 text-slate-200" @click="boardModal = { open: true, boardId: b.board_id }">
              <span class="material-icons icon-sm">{{ b.builtin ? 'visibility' : 'edit' }}</span>
            </button>
            <button v-if="!b.builtin" class="btn-sm bg-slate-700 hover:bg-red-600 text-slate-300 hover:text-white" @click="removeBoard(b.board_id)">
              <span class="material-icons icon-sm">delete</span>
            </button>
          </li>
        </ul>
        <p v-else class="text-sm text-slate-400 italic">No boards registered.</p>
      </div>
    </div>

    <BoardEditorModal
      v-if="boardModal.open"
      :board-id="boardModal.boardId"
      @close="boardModal = { open: false, boardId: null }"
      @saved="reloadBoards()"
    />
  </section>
</template>
