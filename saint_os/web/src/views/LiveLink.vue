<script setup>
import { computed, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useWsTopic } from '@/composables/useWsTopic'

const ws = useWsStore()
const status = useWsTopic(() => 'livelink')
const blendShapes = useWsTopic(() => 'livelink/blend_shapes')
const paused = ref(false)

const receiver = computed(() => status.value?.receiver || {})
const router   = computed(() => status.value?.router   || {})
const shapes   = computed(() => paused.value ? null : (blendShapes.value || {}))

const shapeEntries = computed(() => Object.entries(shapes.value || {})
  .filter(([, v]) => typeof v === 'number')
  .sort(([a], [b]) => a.localeCompare(b)))

async function createDefault () {
  try { await ws.send('livelink', 'create_default_route', { node_id: null }) } catch (e) { console.warn(e) }
}
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-4">
      <h2 class="text-2xl font-bold text-white">LiveLink</h2>
      <div class="flex items-center gap-2">
        <button class="btn-secondary text-sm" @click="paused = !paused">
          <span class="material-icons icon-sm">{{ paused ? 'play_arrow' : 'pause' }}</span>
          {{ paused ? 'Resume' : 'Pause' }}
        </button>
        <button class="btn-secondary text-sm" @click="createDefault">
          <span class="material-icons icon-sm">add</span>
          Default route
        </button>
      </div>
    </div>

    <div class="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
      <div class="card">
        <span class="stat-label">Receiver</span>
        <div class="stat-value mt-1">{{ receiver.connected ? 'Connected' : 'Idle' }}</div>
        <p class="text-xs text-slate-500 mt-1 font-mono">{{ receiver.source_address || '—' }}</p>
      </div>
      <div class="card">
        <span class="stat-label">Frames</span>
        <div class="stat-value mt-1">{{ receiver.frames_received ?? 0 }}</div>
        <p class="text-xs text-slate-500 mt-1">{{ receiver.fps ? `${receiver.fps.toFixed(1)} fps` : '' }}</p>
      </div>
      <div class="card">
        <span class="stat-label">Routes</span>
        <div class="stat-value mt-1">{{ router.routes_count ?? 0 }}</div>
        <p class="text-xs text-slate-500 mt-1">{{ router.enabled_count ?? 0 }} enabled</p>
      </div>
    </div>

    <div class="card">
      <h3 class="text-lg font-semibold text-white mb-3">Blend shapes</h3>
      <div v-if="!shapeEntries.length" class="text-sm text-slate-400 italic">
        No data yet — open a LiveLink session and disable pause to see frames.
      </div>
      <div v-else class="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-x-6 gap-y-2 font-mono text-xs">
        <div v-for="[name, value] in shapeEntries" :key="name" class="flex items-center justify-between border-b border-slate-700/40 py-1">
          <span class="text-slate-400 truncate" :title="name">{{ name }}</span>
          <div class="flex items-center gap-2 w-28">
            <div class="flex-1 h-1.5 bg-slate-700 rounded-full overflow-hidden">
              <div class="h-full bg-cyan-500" :style="{ width: `${Math.min(100, value * 100)}%` }" />
            </div>
            <span class="text-cyan-300 w-10 text-right">{{ value.toFixed(2) }}</span>
          </div>
        </div>
      </div>
    </div>
  </section>
</template>
