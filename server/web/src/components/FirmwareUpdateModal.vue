<script setup>
import { computed, onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useFirmwareUpdatesStore } from '@/stores/firmwareUpdates'
import AppModal from './AppModal.vue'

const props = defineProps({
  nodeId:         { type: String, required: true },
  currentVersion: { type: String, default: '' },
  node:           { type: Object, default: null },
})
const emit = defineEmits(['close', 'updated'])

const ws = useWsStore()
const firmwareUpdates = useFirmwareUpdatesStore()

const builds = ref({ simulation: null, hardware: null })
const loading = ref(true)
const sending = ref(false)
const sendingType = ref(null)        // 'simulation' | 'hardware' while in-flight
const force = ref(false)
const error = ref('')
const success = ref('')

// Filter which build buttons to show by chip family. RP2040 / Teensy can run
// either sim (Renode) or hardware firmware. Pi 5 has no sim image — it runs
// the server-side ROS node directly. Unknown chips: show both and let the
// operator pick (the server still validates).
const chip = computed(() => (props.node?.chip_family || '').toLowerCase())
const hwModel = computed(() => (props.node?.hardware_model || '').toLowerCase())
const isRpi = computed(() =>
  chip.value.includes('rpi') || chip.value.includes('pi5') ||
  chip.value.includes('raspberry') ||
  hwModel.value.includes('rpi') || hwModel.value.includes('raspberry')
)
const isTeensy = computed(() => chip.value.includes('teensy'))
const showSim = computed(() => !isRpi.value)
const showHw  = computed(() => true)

function buildLabel (b) {
  if (!b || !b.available) return 'Not available on server'
  const ver = b.version_full || b.version || '—'
  const built = b.build_date ? ` · ${b.build_date}` : ''
  return `${ver}${built}`
}

async function load () {
  loading.value = true
  error.value = ''
  try {
    const r = await ws.management('get_firmware_builds', {})
    // Route the "hardware" slot to the chip-family-specific build.
    // The server's get_all_firmware_builds returns every chip's info
    // in one payload (simulation, hardware [RP2040 .uf2/.bin],
    // teensy41 [.hex/.bin], raspberrypi [.zip]) — without this
    // routing, a Pi node was offered the RP2040 hardware build
    // (e.g. "1.2.0-1781755077") as an "update" because it's what
    // .hardware always pointed at. The chip-family check here mirrors
    // what websocket_handler.force_firmware_update does on the server
    // side so the modal and the actual OTA agree on which build is
    // being installed.
    let hardware = r?.hardware || null
    if (isRpi.value) {
      hardware = r?.raspberrypi || null
    } else if (isTeensy.value) {
      hardware = r?.teensy41 || null
    }
    builds.value = {
      simulation: r?.simulation || null,
      hardware,
    }
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    loading.value = false
  }
}
onMounted(load)

async function send (buildType) {
  if (sending.value) return
  const b = builds.value[buildType]
  if (!b || !b.available) return

  sending.value = true
  sendingType.value = buildType
  error.value = ''
  success.value = ''
  // Start tracking BEFORE we issue the command so we don't miss the
  // first progress frame on fast nodes. The store owns the per-node
  // progress state and survives this modal closing — the operator
  // sees updates in the node card / overview status card.
  firmwareUpdates.start(props.nodeId)
  try {
    const r = await ws.management('force_firmware_update', {
      node_id:    props.nodeId,
      build_type: buildType,
      force:      force.value,
    })
    success.value = r?.message || `Firmware update initiated (${buildType})`
    emit('updated')
    // Brief confirmation, then auto-close. Progress continues to render
    // in the node card / overview status card via the store.
    setTimeout(() => emit('close'), 1200)
  } catch (e) {
    error.value = e.message || String(e)
    firmwareUpdates.cancel(props.nodeId)
  } finally {
    sending.value = false
    sendingType.value = null
  }
}
</script>

<template>
  <AppModal title="Force Firmware Update" width="max-w-md" @close="emit('close')">
    <div v-if="error" class="mb-3 p-2 rounded bg-red-500/20 border border-red-500/40 text-sm text-red-300">{{ error }}</div>
    <div v-if="success" class="mb-3 p-2 rounded bg-emerald-500/20 border border-emerald-500/40 text-sm text-emerald-300">{{ success }}</div>

    <p class="text-sm text-fg-muted mb-4">
      Select which firmware build to upload to
      <code class="text-cyan-300 font-mono">{{ nodeId }}</code>.
      Installed version:
      <span class="font-mono text-fg">{{ currentVersion || '—' }}</span>.
    </p>

    <div v-if="sending" class="mb-4">
      <div class="flex items-center justify-between text-xs text-fg-muted mb-1">
        <span>Sending {{ sendingType }} update…</span>
      </div>
      <div class="w-full h-1.5 bg-surface rounded-full overflow-hidden">
        <div class="h-full bg-cyan-500 animate-pulse" style="width: 100%"></div>
      </div>
    </div>

    <div class="space-y-3 mb-4">
      <button
        v-if="showSim"
        type="button"
        class="w-full p-4 rounded-lg border border-line hover:border-cyan-500 hover:bg-cyan-500/10 transition-all text-left disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:border-line disabled:hover:bg-transparent"
        :disabled="sending || !builds.simulation?.available"
        @click="send('simulation')"
      >
        <div class="flex items-center gap-3">
          <span class="material-icons text-cyan-400">computer</span>
          <div class="flex-1">
            <div class="font-medium text-fg-strong">Simulation Build</div>
            <div class="text-xs text-fg-muted">
              <span v-if="loading">Checking…</span>
              <span v-else>{{ buildLabel(builds.simulation) }}</span>
            </div>
          </div>
        </div>
      </button>

      <button
        v-if="showHw"
        type="button"
        class="w-full p-4 rounded-lg border border-line hover:border-violet-500 hover:bg-violet-500/10 transition-all text-left disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:border-line disabled:hover:bg-transparent"
        :disabled="sending || !builds.hardware?.available"
        @click="send('hardware')"
      >
        <div class="flex items-center gap-3">
          <span class="material-icons text-violet-400">memory</span>
          <div class="flex-1">
            <div class="font-medium text-fg-strong">Hardware Build</div>
            <div class="text-xs text-fg-muted">
              <span v-if="loading">Checking…</span>
              <span v-else>{{ buildLabel(builds.hardware) }}</span>
            </div>
          </div>
        </div>
      </button>
    </div>

    <label class="flex items-center gap-2 text-sm text-fg mt-2">
      <input
        v-model="force"
        type="checkbox"
        class="rounded bg-surface border-line-strong"
        :disabled="sending"
      />
      Install even if same version
    </label>

    <template #actions>
      <button class="btn-secondary" :disabled="sending" @click="emit('close')">Cancel</button>
    </template>
  </AppModal>
</template>
