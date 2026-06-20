<script setup>
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'
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

// Browser upload of a dist tarball.
const uploading = ref(false)
const uploadProgress = ref(0)        // 0..1
const dragOver = ref(false)
const fileInput = ref(null)

// Live install-log console. The install detaches and restarts the
// saint-os service, which makes ws.js auto-reload this page (see the
// 'connected' reload in stores/ws.js). So the install marker is parked
// in sessionStorage and the log resumed on mount after the reload.
const INSTALL_KEY = 'saint_install_active'
const installLog = ref('')
const logOffset = ref(0)
const logPresent = ref(false)
const installTarget = ref(null)
const installDone = ref(false)
let logTimer = null
let stableTicks = 0

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
  // Defensive: the Install button is disabled when the preflight fails,
  // but never kick off an install the host can't apply.
  if (!canSelfUpdate.value) { error.value = selfUpdateBlocker.value; return }
  installTarget.value = state.value.github_release?.version || latestVersion.value || null
  installLog.value = ''
  logOffset.value = 0
  logPresent.value = false
  installDone.value = false
  // Park the marker so the log resumes after ws.js reloads the page on
  // the post-install reconnect.
  try {
    sessionStorage.setItem(INSTALL_KEY, JSON.stringify({ target: installTarget.value }))
  } catch { /* private mode — live log still works pre-reload */ }
  installing.value = true
  startLogPolling()
  try {
    // Resolves to the post-launch state. If the privileged apply wrapper
    // never started (missing wrapper, sudo needs a password, /var/log
    // perms), it comes back status=error — surface it instead of leaving
    // the operator on "Waiting for install log…".
    const st = await ws.management('update.install', {})
    if (st && st.status === 'error') {
      failInstall(st.last_error || 'Install failed to start')
      return
    }
  } catch (e) {
    error.value = e.message
    // Keep the overlay up: install.sh is detached and may still be
    // running even though the WS call's response was lost to the restart.
  }
}

// Surface an install failure and tear down the live-log overlay so the
// operator isn't stuck watching a log that will never appear.
function failInstall (msg) {
  error.value = msg
  installing.value = false
  if (logTimer) { clearTimeout(logTimer); logTimer = null }
  try { sessionStorage.removeItem(INSTALL_KEY) } catch { /* ignore */ }
}

// ── Browser upload ─────────────────────────────────────────────────
function pickFile () { fileInput.value?.click() }

function onFilePicked (e) {
  const f = e.target.files?.[0]
  if (f) uploadFile(f)
  e.target.value = ''   // allow re-selecting the same file
}

function onDrop (e) {
  dragOver.value = false
  const f = e.dataTransfer?.files?.[0]
  if (f) uploadFile(f)
}

function uploadFile (file) {
  error.value = ''
  uploading.value = true
  uploadProgress.value = 0
  const form = new FormData()
  form.append('file', file)
  const xhr = new XMLHttpRequest()
  xhr.open('POST', '/api/update/upload')
  xhr.upload.onprogress = (ev) => {
    if (ev.lengthComputable) uploadProgress.value = ev.loaded / ev.total
  }
  xhr.onload = () => {
    uploading.value = false
    if (xhr.status >= 200 && xhr.status < 300) {
      // Server broadcasts the 'downloaded' update state over the WS, which
      // flips the Install button. Refresh as a fallback in case we missed it.
      fetchStatus()
    } else {
      let msg = `Upload failed (HTTP ${xhr.status})`
      try { msg = JSON.parse(xhr.responseText).error || msg } catch { /* non-JSON */ }
      error.value = msg
    }
  }
  xhr.onerror = () => { uploading.value = false; error.value = 'Upload failed (network error)' }
  xhr.send(form)
}

// ── Live install log ───────────────────────────────────────────────
function startLogPolling () {
  if (logTimer) return
  stableTicks = 0
  pollLog()
}

async function pollLog () {
  logTimer = null
  let grew = false
  try {
    const r = await fetch(`/api/update/log?offset=${logOffset.value}`, { cache: 'no-store' })
    if (r.ok) {
      const j = await r.json()
      logPresent.value = j.present
      if (j.data) { installLog.value += j.data; grew = true }
      if (typeof j.offset === 'number') logOffset.value = j.offset
    }
  } catch { /* server mid-restart — keep retrying */ }

  // Refresh status quietly so we can tell when the new build is live.
  // `ws.management` resolves to the inner data, so use the same
  // `r?.data || r` unwrap fetchStatus does — the old `?.data` alone was
  // always undefined here, leaving `fetched` stale (so status changes,
  // including errors, were never seen mid-install).
  try {
    const r = await ws.management('update.get_status', {})
    fetched.value = r?.data || r || fetched.value
  } catch { /* WS down during restart */ }

  // If the install bailed (status flipped to error), stop waiting and
  // show why — otherwise a failed launch loops forever on
  // "Waiting for install log…".
  if (state.value.status === 'error') {
    failInstall(state.value.last_error || 'Install failed')
    return
  }

  // Done when output has stopped growing and the server is no longer
  // reporting 'installing' (true after the restart brings up the new
  // build). status stays 'installing' on the original server, so a pause
  // mid-install won't falsely complete.
  stableTicks = grew ? 0 : stableTicks + 1
  const settled = stableTicks >= 4 && state.value.status !== 'installing' && logPresent.value
  if (settled) {
    installDone.value = true
    installing.value = false
    try { sessionStorage.removeItem(INSTALL_KEY) } catch { /* ignore */ }
    return
  }
  logTimer = setTimeout(pollLog, 1000)
}

function dismissInstall () {
  installing.value = false
  installDone.value = false
  if (logTimer) { clearTimeout(logTimer); logTimer = null }
  try { sessionStorage.removeItem(INSTALL_KEY) } catch { /* ignore */ }
}

onMounted(() => {
  fetchStatus()
  // Resume the log after the post-install page reload, if one was in flight.
  let marker = null
  try { marker = JSON.parse(sessionStorage.getItem(INSTALL_KEY) || 'null') } catch { /* ignore */ }
  if (marker) {
    installTarget.value = marker.target
    installing.value = true
    startLogPolling()
  }
})

onBeforeUnmount(() => { if (logTimer) clearTimeout(logTimer) })

// UpdateState field names (server/saint_server/update_manager.py UpdateState):
//   installed_version, status, github_release {version, notes, ...},
//   usb_releases [], download_received, download_total, staged_tarball.
// status enum: unknown|checking|up_to_date|available|no_network|
//              downloading|downloaded|installing|error.
const currentVersion  = computed(() => state.value.installed_version || '—')
const latestVersion   = computed(() => state.value.github_release?.version || null)
const updateAvailable = computed(() => state.value.status === 'available')
const downloading     = computed(() => state.value.status === 'downloading')
const downloaded      = computed(() => state.value.status === 'downloaded')
const progress        = computed(() => {
  const total = state.value.download_total
  const recv  = state.value.download_received
  if (!total || total <= 0) return null
  return Math.max(0, Math.min(1, recv / total))
})
const usbStaged       = computed(() => {
  // staged_tarball is a filesystem path; surface its basename for the
  // operator. The server doesn't carry a separate "staged version"
  // field, so this is the best signal we have.
  const p = state.value.staged_tarball
  if (!p) return null
  return String(p).split('/').pop()
})
const usbAvailable    = computed(() => state.value.usb_releases || [])
const releaseNotes    = computed(() => state.value.github_release?.notes || '')
// Server preflight: false only when it explicitly checked and found this
// host can't apply updates (missing apply wrapper / no passwordless
// sudo). null or true → allow (older servers don't report the field).
const canSelfUpdate     = computed(() => state.value.can_self_update !== false)
const selfUpdateBlocker = computed(() => state.value.self_update_blocker || 'This server is not configured to apply updates.')
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

    <!-- Preflight: warn (and disable Install below) when the host can't
         apply updates, so the operator isn't sent into a guaranteed
         failure. -->
    <div v-if="!canSelfUpdate" class="mb-4 p-3 bg-amber-500/15 border border-amber-500/40 rounded-lg text-sm text-amber-200 flex items-start gap-2">
      <span class="material-icons icon-sm shrink-0">warning</span>
      <span><span class="font-medium">This server can't apply updates.</span> {{ selfUpdateBlocker }}</span>
    </div>

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
          <button v-else-if="downloaded" class="btn-primary justify-center"
                  :disabled="!canSelfUpdate"
                  :title="canSelfUpdate ? '' : selfUpdateBlocker"
                  @click="installModalOpen = true">
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

    <div class="card mt-4">
      <h3 class="text-lg font-semibold text-fg-strong mb-3 flex items-center gap-2">
        <span class="material-icons text-cyan-400">upload_file</span>
        Upload update file
      </h3>
      <div
        class="border-2 border-dashed rounded-lg p-6 text-center transition-colors cursor-pointer"
        :class="dragOver ? 'border-cyan-400 bg-cyan-500/10' : 'border-line/60 hover:border-line'"
        @click="pickFile"
        @dragover.prevent="dragOver = true"
        @dragleave.prevent="dragOver = false"
        @drop.prevent="onDrop"
      >
        <input ref="fileInput" type="file" accept=".tar.zst,.tar.gz" class="hidden" @change="onFilePicked" />
        <template v-if="uploading">
          <div class="text-sm text-fg mb-2">Uploading… {{ Math.round(uploadProgress * 100) }}%</div>
          <div class="w-full h-2 bg-surface rounded-full overflow-hidden">
            <div class="h-full bg-cyan-500 transition-all" :style="{ width: `${Math.round(uploadProgress * 100)}%` }" />
          </div>
        </template>
        <template v-else>
          <span class="material-icons text-fg-muted" style="font-size: 40px;">cloud_upload</span>
          <p class="mt-2 text-sm text-fg">Drag a <span class="font-mono">.tar.zst</span> dist file here, or click to choose</p>
          <p class="mt-1 text-xs text-fg-faint">Once uploaded it appears above as ready to install.</p>
        </template>
      </div>
    </div>

    <AppModal v-if="installModalOpen" title="Confirm install" @close="installModalOpen = false">
      <p class="text-sm text-fg mb-3">Installing will restart the server. Active connections will reconnect automatically.</p>
      <pre v-if="releaseNotes" class="text-xs font-mono text-fg bg-canvas rounded p-3 overflow-x-auto max-h-64">{{ releaseNotes }}</pre>
      <template #actions>
        <button class="btn-secondary" @click="installModalOpen = false">Cancel</button>
        <button class="btn-danger" @click="confirmInstall">Install now</button>
      </template>
    </AppModal>

    <div v-if="installing || installDone" class="fixed inset-0 z-50 flex items-center justify-center bg-canvas/90 backdrop-blur-sm p-4">
      <div class="w-full max-w-3xl flex flex-col" style="max-height: 90vh;">
        <div class="flex items-center gap-3 mb-3">
          <span v-if="!installDone" class="material-icons text-cyan-400 animate-spin">refresh</span>
          <span v-else class="material-icons text-green-400">check_circle</span>
          <p class="text-fg-strong">
            <template v-if="installDone">Update complete — now running {{ currentVersion }}.</template>
            <template v-else>Installing{{ installTarget ? ` ${installTarget}` : '' }}… the server will restart.</template>
          </p>
        </div>
        <pre class="flex-1 overflow-auto text-xs font-mono text-fg bg-black/60 rounded-lg p-3 whitespace-pre-wrap">{{ installLog || (logPresent ? '' : 'Waiting for install log…') }}</pre>
        <div class="mt-3 flex justify-end">
          <button v-if="installDone" class="btn-primary" @click="dismissInstall">Close</button>
          <button v-else class="btn-secondary" @click="dismissInstall">Hide</button>
        </div>
      </div>
    </div>
  </section>
</template>
