<script setup>
import { computed, onUnmounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import AppModal from '@/components/AppModal.vue'
import LinearTravelControl from '@/components/peripherals/LinearTravelControl.vue'

// Kangaroo x2 teach-tune (Mode 1) workflow.
//
// This is NOT the Maestro servo dial. There you type a pulse width and
// the servo follows; here the travel limits are read-only on the wire
// (Get 8/9) and come from WHERE YOU JOG the actuator during the teach.
// So the flow is jog-and-capture, then read back to confirm.
//
// Everything safety-critical lives in the firmware, not here — the
// keep-alive that stops the Kangaroo aborting, the dead-man that decays
// jog power when this UI stops talking, the open-loop power cap, and
// abort-on-estop. This component's job is to be a well-behaved caller:
// hold the jog repeat faster than the dead-man window, and never
// pretend to know more about the tune's state than the firmware told us.
//
// See docs/KANGAROO_BRINGUP.md for the protocol and the hardware setup.

const props = defineProps({
  nodeId:     { type: String, required: true },
  peripheral: { type: Object, required: true },
  // Live channel values for this peripheral, same {channel_id: {value}}
  // map the Live tab builds. tune_state / current_position / taught_*
  // are read straight off it.
  channels:   { type: Object, default: () => ({}) },
})
const emit = defineEmits(['close'])

const ws = useWsStore()
const error = ref('')
const busy  = ref(false)

// ── Firmware tune states (kangaroo_tune_state_t) ────────────────────
// Kept in lock-step with the enum in
// firmware/shared/include/kangaroo_driver.h.
const TUNE = {
  IDLE: 0, ENTERING: 1, JOG: 2, GOING: 3, DONE: 4, FAILED: 5,
}
const STATE_LABEL = {
  0: 'Idle', 1: 'Entering tune mode', 2: 'Ready to jog',
  3: 'Tuning — stand clear', 4: 'Complete', 5: 'Failed',
}

// Kangaroo Get-reply error codes (Reference Manual p.12). These are NOT
// the LED blink codes from the main manual — different numbering.
const ERROR_LABEL = {
  1: 'Channel not started',
  2: 'Channel needs homing',
  3: 'Control error — command Start to clear',
  4: 'Wrong mode (DIP switches disagree with the tune)',
  5: 'Unknown parameter',
  6: 'Serial timeout or TX disconnected',
}

// Accepts either shape the app produces: the Live tab's
// {channel_id: {value, last_updated}} records, or the Peripherals tab's
// flatter {channel_id: number} map.
function chValue (id) {
  const c = props.channels?.[id]
  const v = (c !== null && typeof c === 'object') ? c.value : c
  return (typeof v === 'number') ? v : null
}

const tuneState = computed(() => chValue('tune_state') ?? TUNE.IDLE)
const stateLabel = computed(() => STATE_LABEL[tuneState.value] || 'Unknown')
const isJogStage = computed(() => tuneState.value === TUNE.JOG)
const isRunning  = computed(() => tuneState.value === TUNE.GOING)
const isTuning   = computed(() =>
  tuneState.value === TUNE.ENTERING || isJogStage.value || isRunning.value)

const position   = computed(() => chValue('current_position'))
const taughtMin  = computed(() => chValue('taught_min'))
const taughtMax  = computed(() => chValue('taught_max'))
const errorCode  = computed(() => chValue('error_status') ?? 0)
const errorText  = computed(() =>
  errorCode.value ? (ERROR_LABEL[errorCode.value] || `Error ${errorCode.value}`) : '')

const jogPct = computed(() => Number(props.peripheral?.params?.jog_power_pct) || 10)

// ── Captured endpoints ──────────────────────────────────────────────
// Purely local bookkeeping. The Kangaroo does not record these — it
// watches the pot for itself during the teach. We track them so the
// operator can see they actually visited both ends before hitting Go,
// which is the single most common way a teach tune fails (error 2,
// "system range").
const marks = ref({ retract: null, extend: null, center: null })
const allMarked = computed(() =>
  marks.value.retract != null && marks.value.extend != null && marks.value.center != null)

function markHere (which) {
  if (position.value == null) {
    error.value = 'No position reading yet — is the channel responding?'
    return
  }
  marks.value[which] = position.value
  error.value = ''
}

// ── Command / channel transport ─────────────────────────────────────

async function sendCommand (command, args = {}) {
  error.value = ''
  busy.value = true
  try {
    const r = await ws.control('peripheral_command', {
      node_id: props.nodeId,
      peripheral_id: props.peripheral.id,
      command,
      args,
    })
    if (r?.status === 'error') error.value = r.message || `${command} failed`
    return r?.status !== 'error'
  } catch (e) {
    error.value = e?.message || String(e)
    return false
  } finally {
    busy.value = false
  }
}

// Jog rides the CHANNEL, not the command path. /control is best-effort
// depth 1 (newest-wins); /command is reliable depth 8, where a stalled
// link would queue stale non-zero jogs ahead of our release-to-zero —
// and because each arrival refreshes the firmware dead-man, that burst
// would defeat the dead-man rather than trip it.
function sendJog (fraction) {
  ws.control('set_channel_value', {
    node_id: props.nodeId,
    peripheral_id: props.peripheral.id,
    channel_id: 'jog',
    value: fraction,
  }).catch(() => {})
}

// ── Press-and-hold jog ──────────────────────────────────────────────
// The firmware zeroes jog power if it hasn't heard a fresh value within
// KANGAROO_JOG_DEADMAN_MS (250 ms), so repeat comfortably inside that.
// Releasing sends an explicit 0 as well — the dead-man is the backstop
// for a crash or a dropped link, not the normal stop path.
const JOG_REPEAT_MS = 100
let jogTimer = null
const jogging = ref(0)

function startJog (direction) {
  if (!isJogStage.value) return
  stopJog()
  jogging.value = direction
  sendJog(direction)
  jogTimer = setInterval(() => sendJog(direction), JOG_REPEAT_MS)
}

function stopJog () {
  if (jogTimer) { clearInterval(jogTimer); jogTimer = null }
  if (jogging.value !== 0) sendJog(0)
  jogging.value = 0
}

// Any exit path must stop the actuator. Losing the component without
// this would leave the dead-man as the only thing stopping it — which
// works, but relies on a 250 ms timeout instead of an explicit stop.
onUnmounted(stopJog)
watch(isJogStage, (ok) => { if (!ok) stopJog() })

async function onAbort () {
  stopJog()
  await sendCommand('tune_abort')
}

async function onEnter () {
  marks.value = { retract: null, extend: null, center: null }
  await sendCommand('tune_enter')
}

async function onGo () {
  stopJog()
  await sendCommand('tune_go')
}

function close () {
  stopJog()
  emit('close')
}
</script>

<template>
  <AppModal title="Teach tune — linear actuator" @close="close">
    <div v-if="error" class="mb-3 p-2 rounded bg-red-500/20 border border-red-500/40 text-sm text-red-300">
      {{ error }}
    </div>

    <div class="space-y-4">
      <!-- Status strip: firmware state is the source of truth here, not
           anything this component thinks it knows. -->
      <div class="flex items-center justify-between gap-3 p-2 rounded border border-line bg-surface">
        <div class="flex items-center gap-2 min-w-0">
          <span class="w-2 h-2 rounded-full shrink-0"
                :class="isRunning ? 'bg-amber-400 animate-pulse-dot'
                      : tuneState === TUNE.DONE ? 'bg-emerald-400'
                      : tuneState === TUNE.FAILED ? 'bg-red-400'
                      : isTuning ? 'bg-cyan-400' : 'bg-slate-500'" />
          <span class="text-sm text-fg-strong truncate">{{ stateLabel }}</span>
        </div>
        <span class="text-xs font-mono text-fg-muted shrink-0">
          pos {{ position ?? '—' }}
        </span>
      </div>

      <div v-if="errorText"
           class="p-2 text-xs rounded bg-red-500/10 border border-red-500/30 text-red-300">
        Kangaroo reports: {{ errorText }}
      </div>

      <!-- Travel visualisation. Read-only by design: these endpoints
           cannot be typed, only jogged to. -->
      <LinearTravelControl
        :position="position"
        :retract="marks.retract"
        :extend="marks.extend"
        :center="marks.center"
        :taught-min="taughtMin"
        :taught-max="taughtMax"
      />

      <!-- ── Step 1: enter tune mode ─────────────────────────────── -->
      <div v-if="!isTuning" class="space-y-3">
        <p class="text-xs text-fg-muted leading-snug">
          This runs a <strong>Mode 1 Teach tune</strong>. The actuator will be
          driven <strong>open loop</strong> while you jog it — no feedback, no
          travel limits, and it will drive into the hard stops if you let it.
          Jog power is capped at <strong>{{ jogPct }}%</strong>
          (<em>Jog power</em> in this peripheral's settings).
        </p>
        <p class="text-xs text-fg-faint leading-snug">
          Direction is unknown until the first tune — tap a jog button briefly
          and see which way it goes before holding it.
        </p>
        <button class="btn-primary w-full" :disabled="busy" @click="onEnter">
          Enter tune mode
        </button>
      </div>

      <!-- ── Step 2: jog + capture ───────────────────────────────── -->
      <div v-else-if="isJogStage" class="space-y-3">
        <div class="flex items-center gap-2">
          <button
            class="btn-secondary flex-1 flex items-center justify-center gap-1 select-none"
            :class="jogging < 0 ? 'ring-2 ring-cyan-500' : ''"
            @pointerdown.prevent="startJog(-1)"
            @pointerup="stopJog" @pointerleave="stopJog" @pointercancel="stopJog"
          >
            <span class="material-icons icon-sm">chevron_left</span> Retract
          </button>
          <button
            class="btn-secondary flex-1 flex items-center justify-center gap-1 select-none"
            :class="jogging > 0 ? 'ring-2 ring-cyan-500' : ''"
            @pointerdown.prevent="startJog(1)"
            @pointerup="stopJog" @pointerleave="stopJog" @pointercancel="stopJog"
          >
            Extend <span class="material-icons icon-sm">chevron_right</span>
          </button>
        </div>
        <p class="text-[11px] text-fg-faint text-center">
          Hold to move — releasing stops immediately.
        </p>

        <div class="grid grid-cols-3 gap-2">
          <button class="btn-secondary text-xs py-1" @click="markHere('retract')">
            Mark retract
            <span class="block font-mono text-[10px] text-fg-faint">{{ marks.retract ?? '—' }}</span>
          </button>
          <button class="btn-secondary text-xs py-1" @click="markHere('extend')">
            Mark extend
            <span class="block font-mono text-[10px] text-fg-faint">{{ marks.extend ?? '—' }}</span>
          </button>
          <button class="btn-secondary text-xs py-1" @click="markHere('center')">
            Mark centre
            <span class="block font-mono text-[10px] text-fg-faint">{{ marks.center ?? '—' }}</span>
          </button>
        </div>

        <p class="text-[11px] text-fg-faint leading-snug">
          Jog to each end of travel and then back to roughly the centre, leaving
          it there. The marks are a checklist for you — the Kangaroo watches the
          potentiometer itself. Teaching a range that stops a few mm short of any
          internal end-stops avoids a control error later.
        </p>

        <button
          class="btn-primary w-full"
          :disabled="busy || !allMarked"
          :title="allMarked ? '' : 'Mark both ends and the centre first'"
          @click="onGo"
        >
          Start tune — stand clear
        </button>
      </div>

      <!-- ── Step 3: running ─────────────────────────────────────── -->
      <div v-else-if="isRunning" class="space-y-2">
        <div class="p-2 rounded bg-amber-500/10 border border-amber-500/30 text-xs text-amber-200 leading-snug">
          <strong>Stand clear.</strong> The Kangaroo is driving the axis through
          its own tune cycle at varying speeds and powers. On a slow linear
          actuator this takes minutes.
        </div>
      </div>

      <!-- ── Step 4: done / failed ───────────────────────────────── -->
      <div v-else-if="tuneState === TUNE.DONE" class="space-y-3">
        <div class="p-2 rounded bg-emerald-500/10 border border-emerald-500/30 text-xs text-emerald-200 leading-snug">
          Tune complete. <strong>Power cycle the Kangaroo now</strong> — the new
          tune is not active until you do. Then read the taught travel back.
        </div>
        <div class="flex items-center justify-between text-sm font-mono">
          <span class="text-fg-muted text-xs">Taught travel</span>
          <span>{{ taughtMin ?? '—' }} … {{ taughtMax ?? '—' }}</span>
        </div>
        <button class="btn-secondary w-full" :disabled="busy"
                @click="sendCommand('tune_read_extents')">
          Read taught travel
        </button>
        <p class="text-[11px] text-fg-faint leading-snug">
          First power-up after a tune is the runaway window — keep a hand on the
          power. If it drives away instead of holding, the pot is reversed: cut
          power, hold the Kangaroo's tune button while powering back up, swap the
          5V and B wires on the pot, and retune.
        </p>
      </div>

      <div v-else-if="tuneState === TUNE.FAILED" class="space-y-3">
        <div class="p-2 rounded bg-red-500/10 border border-red-500/30 text-xs text-red-200 leading-snug">
          Tune failed or was aborted. Check the node log for the reason — every
          transition and refusal is logged there. A serial timeout (error 6)
          clears with a Start, which the driver sends automatically.
        </div>
        <button class="btn-primary w-full" :disabled="busy" @click="onEnter">
          Start over
        </button>
      </div>
    </div>

    <template #actions>
      <!-- Abort stays reachable at every stage, including while the
           Kangaroo is running its own cycle. -->
      <button v-if="isTuning" class="btn-danger" @click="onAbort">
        <span class="material-icons icon-sm">stop</span> Abort
      </button>
      <button class="btn-secondary" @click="close">Close</button>
    </template>
  </AppModal>
</template>
