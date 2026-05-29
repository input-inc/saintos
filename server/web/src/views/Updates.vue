<script setup>
import { computed, onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useWsTopic } from '@/composables/useWsTopic'
import AppModal from '@/components/AppModal.vue'

defineProps({ embedded: { type: Boolean, default: false } })

const ws = useWsStore()
const live = useWsTopic(() => 'update')
const fetched = ref(null)
const installing = ref(false)
const installModalOpen = ref(false)
const error = ref('')

const state = computed(() => live.value || fetched.value || {})

async function fetchStatus () {
  try {
    const r = await ws.management('update.get_status', {})
    fetched.value = r?.data || r || null
  } catch (e) {
    error.value = e.message || String(e)
  }
}
async function checkNow () {
  try { fetched.value = (await ws.management('update.check_now', {}))?.data || null }
  catch (e) { error.value = e.message }
}
async function scanUsb () {
  try { fetched.value = (await ws.management('update.scan_usb', {}))?.data || null }
  catch (e) { error.value = e.message }
}
async function stageUsb (version) {
  try { await ws.management('update.stage_usb', { version }) }
  catch (e) { error.value = e.message }
}
async function startDownload () {
  try { await ws.management('update.download', {}) }
  catch (e) { error.value = e.message }
}
async function confirmInstall () {
  installModalOpen.value = false
  installing.value = true
  try {
    await ws.management('update.install', {})
  } catch (e) {
    error.value = e.message
    installing.value = false
  }
}

onMounted(fetchStatus)

const currentVersion = computed(() => state.value.current_version || '—')
const latestVersion  = computed(() => state.value.latest_version || null)
const updateAvailable = computed(() => state.value.update_available || false)
const downloading     = computed(() => state.value.phase === 'downloading')
const downloaded      = computed(() => state.value.phase === 'downloaded')
const progress        = computed(() => state.value.progress ?? null)
const usbStaged       = computed(() => state.value.usb_staged_version || null)
const usbAvailable    = computed(() => state.value.usb_available || [])
const releaseNotes    = computed(() => state.value.release_notes || '')
</script>

<template>
  <section>
    <div v-if="!embedded" class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-fg-strong">Software updates</h2>
      <div class="flex items-center gap-2">
        <button class="btn-secondary text-sm" @click="checkNow"><span class="material-icons icon-sm">refresh</span>Check now</button>
        <button class="btn-secondary text-sm" @click="scanUsb"><span class="material-icons icon-sm">usb</span>Scan USB</button>
      </div>
    </div>
    <div v-else class="flex items-center justify-end gap-2 mb-3">
      <button class="btn-secondary text-sm" @click="checkNow"><span class="material-icons icon-sm">refresh</span>Check now</button>
      <button class="btn-secondary text-sm" @click="scanUsb"><span class="material-icons icon-sm">usb</span>Scan USB</button>
    </div>

    <div v-if="error" class="mb-4 p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">{{ error }}</div>

    <div class="card">
      <div class="grid grid-cols-1 md:grid-cols-2 gap-6">
        <div class="space-y-3">
          <div class="stat-item"><span class="stat-label">Current version</span><span class="stat-value">{{ currentVersion }}</span></div>
          <div class="stat-item"><span class="stat-label">Latest available</span><span class="stat-value">{{ latestVersion || 'unknown' }}</span></div>
          <div v-if="usbStaged" class="stat-item">
            <span class="stat-label">USB staged</span>
            <span class="stat-value text-amber-300">{{ usbStaged }}</span>
          </div>
        </div>

        <div class="flex flex-col items-stretch justify-center gap-3">
          <div v-if="downloading" class="text-sm text-fg space-y-2">
            <div>Downloading {{ latestVersion }}…</div>
            <div class="w-full h-2 bg-surface rounded-full overflow-hidden">
              <div class="h-full bg-cyan-500" :style="{ width: `${Math.round((progress || 0) * 100)}%` }" />
            </div>
          </div>
          <button v-else-if="updateAvailable && !downloaded" class="btn-primary justify-center" @click="startDownload">
            <span class="material-icons icon-sm">download</span>
            Download {{ latestVersion }}
          </button>
          <button v-else-if="downloaded" class="btn-primary justify-center" @click="installModalOpen = true">
            <span class="material-icons icon-sm">system_update_alt</span>
            Install
          </button>
          <p v-else class="text-sm text-fg-muted text-center">No updates available.</p>
        </div>
      </div>
    </div>

    <div v-if="usbAvailable.length" class="card mt-4">
      <h3 class="text-lg font-semibold text-fg-strong mb-3 flex items-center gap-2">
        <span class="material-icons text-amber-400">usb</span>
        Install from USB
      </h3>
      <ul class="divide-y divide-line/50 text-sm">
        <li v-for="u in usbAvailable" :key="u.version" class="flex items-center gap-3 py-2">
          <span class="font-mono">{{ u.version }}</span>
          <span class="text-fg-faint text-xs">{{ u.path || '' }}</span>
          <span class="flex-1" />
          <button class="btn-secondary text-sm" @click="stageUsb(u.version)">Stage</button>
        </li>
      </ul>
    </div>

    <AppModal v-if="installModalOpen" title="Confirm install" @close="installModalOpen = false">
      <p class="text-sm text-fg mb-3">Installing will restart the server. Active connections will reconnect automatically.</p>
      <pre v-if="releaseNotes" class="text-xs font-mono text-fg bg-canvas rounded p-3 overflow-x-auto max-h-64">{{ releaseNotes }}</pre>
      <template #actions>
        <button class="btn-secondary" @click="installModalOpen = false">Cancel</button>
        <button class="btn-danger" @click="confirmInstall">Install now</button>
      </template>
    </AppModal>

    <div v-if="installing" class="fixed inset-0 z-50 flex items-center justify-center bg-canvas/80 backdrop-blur-sm">
      <div class="text-center">
        <span class="material-icons text-cyan-400 animate-spin" style="font-size: 64px;">refresh</span>
        <p class="mt-4 text-fg-strong">Installing update… the server will restart.</p>
      </div>
    </div>
  </section>
</template>
