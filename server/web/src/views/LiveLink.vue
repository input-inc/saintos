<script setup>
import { computed, ref } from 'vue'
import { useRouter } from 'vue-router'
import { useWsTopic } from '@/composables/useWsTopic'

// Mirrors vanilla js/livelink.js — connection stats + head pose +
// eye tracking + blend-shape grid + pause toggle.

const router = useRouter()
const status = useWsTopic(() => 'livelink')
const blendShapes = useWsTopic(() => 'livelink/blend_shapes')
const paused = ref(false)

const receiver = computed(() => status.value?.receiver || {})
const shapes   = computed(() => paused.value ? null : (blendShapes.value || {}))

// Connection pill (matches vanilla's three-state status header).
const connState = computed(() => {
  if (receiver.value.connected) return { label: 'Connected',    dot: 'bg-emerald-500',         text: 'text-emerald-400' }
  if (receiver.value.running)   return { label: 'Listening',    dot: 'bg-amber-500 animate-pulse', text: 'text-amber-400' }
  return                              { label: 'Disconnected', dot: 'bg-slate-500',           text: 'text-slate-400' }
})

const lastUpdateText = computed(() => {
  const t = receiver.value.last_packet_time
  if (!t) return '--'
  const elapsed = Date.now() / 1000 - t
  if (elapsed < 1)   return 'Just now'
  if (elapsed < 60)  return `${Math.floor(elapsed)}s ago`
  return `${Math.floor(elapsed / 60)}m ago`
})

// Head pose — vanilla converts radians to degrees for the rotation
// and uses |pitch| to scale the head shape vertically (looking up/down).
const yaw   = computed(() => shapes.value?.HeadYaw   || 0)
const pitch = computed(() => shapes.value?.HeadPitch || 0)
const roll  = computed(() => shapes.value?.HeadRoll  || 0)
const headStyle = computed(() => ({
  transform: `rotate(${yaw.value * 57.3}deg) scaleY(${1 - Math.abs(pitch.value) * 0.3})`,
}))

// Eye tracking — pupil x/y offsets in pixels, with blink → scaleY.
function eyeStyle (h, v, blink) {
  return {
    left: `calc(50% + ${(h || 0) * 30}px)`,
    top:  `calc(50% + ${-(v || 0) * 30}px)`,
    transform: `translate(-50%, -50%) scaleY(${1 - (blink || 0) * 0.9})`,
  }
}
const leftEyeStyle = computed(() => eyeStyle(
  shapes.value?.eye_look_horizontal_left,
  shapes.value?.eye_look_vertical_left,
  shapes.value?.EyeBlinkLeft,
))
const rightEyeStyle = computed(() => eyeStyle(
  shapes.value?.eye_look_horizontal_right,
  shapes.value?.eye_look_vertical_right,
  shapes.value?.EyeBlinkRight,
))
const leftBlinkPct  = computed(() => Math.round((shapes.value?.EyeBlinkLeft  || 0) * 100))
const rightBlinkPct = computed(() => Math.round((shapes.value?.EyeBlinkRight || 0) * 100))

// Blend shapes grid — keys & formatting from vanilla renderBlendShapes().
const KEY_SHAPES = [
  'HeadYaw', 'HeadPitch', 'HeadRoll',
  'EyeBlinkLeft', 'EyeBlinkRight',
  'EyeLookUpLeft', 'EyeLookDownLeft', 'EyeLookInLeft', 'EyeLookOutLeft',
  'EyeLookUpRight', 'EyeLookDownRight', 'EyeLookInRight', 'EyeLookOutRight',
  'EyeSquintLeft', 'EyeSquintRight', 'EyeWideLeft', 'EyeWideRight',
  'JawOpen', 'JawLeft', 'JawRight', 'JawForward',
  'MouthSmileLeft', 'MouthSmileRight', 'MouthFrownLeft', 'MouthFrownRight',
  'MouthPucker', 'MouthFunnel',
  'BrowDownLeft', 'BrowDownRight', 'BrowInnerUp', 'BrowOuterUpLeft', 'BrowOuterUpRight',
  'CheekPuff', 'CheekSquintLeft', 'CheekSquintRight',
  'NoseSneerLeft', 'NoseSneerRight',
  'TongueOut',
]

function formatName (name) {
  return name
    .replace(/^Eye/, '')
    .replace(/^Mouth/, '')
    .replace(/^Brow/, '')
    .replace(/^Jaw/, '')
    .replace(/^Cheek/, '')
    .replace(/^Nose/, '')
    .replace(/Left$/, ' L')
    .replace(/Right$/, ' R')
    .replace(/([A-Z])/g, ' $1')
    .trim()
}

function shapePercent (name, value) {
  if (name.includes('Head')) {
    return Math.min(100, Math.max(0, (value + 1) * 50))
  }
  return Math.min(100, Math.max(0, value * 100))
}

function valueColor (name, value) {
  const abs = Math.abs(value)
  if (name.includes('Head')) {
    if (abs > 0.5) return 'text-rose-400'
    if (abs > 0.2) return 'text-slate-300'
    return 'text-slate-500'
  }
  if (value > 0.7) return 'text-cyan-400'
  if (value > 0.3) return 'text-slate-300'
  return 'text-slate-500'
}

function barColor (name) {
  if (name.startsWith('Eye'))   return 'bg-cyan-500'
  if (name.startsWith('Mouth')) return 'bg-rose-500'
  if (name.startsWith('Brow'))  return 'bg-violet-500'
  if (name.startsWith('Jaw'))   return 'bg-amber-500'
  if (name.startsWith('Cheek')) return 'bg-pink-500'
  if (name.startsWith('Nose'))  return 'bg-orange-500'
  if (name.startsWith('Head'))  return 'bg-emerald-500'
  return 'bg-slate-500'
}

const shapeRows = computed(() => KEY_SHAPES.map(name => {
  const value = shapes.value?.[name] || 0
  return {
    name,
    label: formatName(name),
    value,
    percent: shapePercent(name, value),
    valueCls: valueColor(name, value),
    barCls: barColor(name),
  }
}))

function togglePause () { paused.value = !paused.value }
function back () { router.push('/inputs') }
</script>

<template>
  <section>
    <!-- Header with back button + connection pill -->
    <div class="flex items-center gap-4 mb-6">
      <button class="btn-secondary" @click="back">
        <span class="material-icons icon-sm">arrow_back</span>
        Back
      </button>
      <div class="flex-1">
        <h2 class="text-2xl font-bold text-white">LiveLink Face</h2>
      </div>
      <div class="flex items-center gap-2 px-3 py-1.5 rounded-full bg-slate-800 border border-slate-700">
        <span :class="['w-2 h-2 rounded-full', connState.dot]" />
        <span :class="['text-sm', connState.text]">{{ connState.label }}</span>
      </div>
    </div>

    <!-- Row 1: Stats / Head Pose / Eye Tracking -->
    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
      <!-- Connection Stats -->
      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-4">Connection Stats</h3>
        <div class="space-y-3">
          <div class="stat-item">
            <span class="stat-label">Receiver Port</span>
            <span class="stat-value text-sm font-mono">{{ receiver.port ?? 11111 }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Total Packets</span>
            <span class="stat-value text-sm">{{ (receiver.packet_count ?? 0).toLocaleString() }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Errors</span>
            <span class="stat-value text-sm">{{ receiver.error_count ?? 0 }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Last Update</span>
            <span class="stat-value text-sm">{{ lastUpdateText }}</span>
          </div>
        </div>
      </div>

      <!-- Head Pose -->
      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-4">Head Pose</h3>
        <div class="flex flex-col items-center">
          <div class="relative w-32 h-32 mb-4">
            <div class="absolute inset-0 flex items-center justify-center">
              <div class="absolute w-full h-full rounded-full border-2 border-slate-600"></div>
              <div
                class="relative w-20 h-24 bg-slate-700 rounded-full border-2 border-cyan-500/50 transition-transform duration-75"
                :style="headStyle"
              >
                <div class="absolute top-0 left-1/2 -translate-x-1/2 -translate-y-1 w-3 h-3 bg-cyan-500 rounded-full"></div>
                <div class="absolute top-6 left-3 w-2 h-3 bg-slate-500 rounded-full"></div>
                <div class="absolute top-6 right-3 w-2 h-3 bg-slate-500 rounded-full"></div>
              </div>
            </div>
          </div>
          <div class="w-full space-y-2">
            <div class="flex items-center justify-between text-sm">
              <span class="text-slate-400">Yaw (L/R)</span>
              <span class="font-mono text-cyan-400">{{ yaw.toFixed(2) }}</span>
            </div>
            <div class="flex items-center justify-between text-sm">
              <span class="text-slate-400">Pitch (U/D)</span>
              <span class="font-mono text-violet-400">{{ pitch.toFixed(2) }}</span>
            </div>
            <div class="flex items-center justify-between text-sm">
              <span class="text-slate-400">Roll (Tilt)</span>
              <span class="font-mono text-amber-400">{{ roll.toFixed(2) }}</span>
            </div>
          </div>
        </div>
      </div>

      <!-- Eye Tracking -->
      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-4">Eye Tracking</h3>
        <div class="flex items-center justify-around">
          <div class="text-center">
            <p class="text-sm text-slate-400 mb-2">Left Eye</p>
            <div class="relative w-24 h-24 bg-slate-900 rounded-full border-2 border-slate-600">
              <div class="absolute w-8 h-8 bg-cyan-500 rounded-full transition-all duration-75" :style="leftEyeStyle"></div>
              <div class="absolute w-3 h-3 bg-white rounded-full transform -translate-x-1/2 -translate-y-1/2 pointer-events-none" style="left: 55%; top: 40%;"></div>
            </div>
            <p class="text-xs text-slate-400 mt-2">Blink: {{ leftBlinkPct }}%</p>
          </div>
          <div class="text-center">
            <p class="text-sm text-slate-400 mb-2">Right Eye</p>
            <div class="relative w-24 h-24 bg-slate-900 rounded-full border-2 border-slate-600">
              <div class="absolute w-8 h-8 bg-cyan-500 rounded-full transition-all duration-75" :style="rightEyeStyle"></div>
              <div class="absolute w-3 h-3 bg-white rounded-full transform -translate-x-1/2 -translate-y-1/2 pointer-events-none" style="left: 55%; top: 40%;"></div>
            </div>
            <p class="text-xs text-slate-400 mt-2">Blink: {{ rightBlinkPct }}%</p>
          </div>
        </div>
      </div>
    </div>

    <!-- Row 2: Blend Shapes Preview -->
    <div class="card mt-6">
      <div class="flex items-center justify-between mb-4">
        <h3 class="text-lg font-semibold text-white">Blend Shapes Preview</h3>
        <button class="btn-secondary text-sm" @click="togglePause">
          <span class="material-icons icon-sm">{{ paused ? 'play_circle' : 'pause_circle' }}</span>
          {{ paused ? 'Resume' : 'Pause' }}
        </button>
      </div>
      <div class="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-6 xl:grid-cols-8 gap-2 max-h-[400px] overflow-y-auto">
        <div
          v-for="row in shapeRows"
          :key="row.name"
          class="p-2 bg-slate-900/50 rounded-lg border border-slate-700/50"
        >
          <div class="flex items-center justify-between mb-1">
            <span class="text-xs text-slate-400 truncate" :title="row.name">{{ row.label }}</span>
            <span :class="['text-xs font-mono', row.valueCls]">{{ row.value.toFixed(2) }}</span>
          </div>
          <div class="h-1.5 bg-slate-700 rounded-full overflow-hidden">
            <div :class="['h-full', row.barCls]" :style="{ width: `${row.percent}%`, transition: 'width 0.1s ease-out' }"></div>
          </div>
        </div>
      </div>
    </div>
  </section>
</template>
