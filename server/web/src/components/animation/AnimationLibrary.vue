<script setup>
import { onMounted, ref } from 'vue'
import { useAnimationsStore } from '@/stores/animations'

const emit = defineEmits(['edit', 'import-maestro'])

const animations = useAnimationsStore()
const fmtNumber = (n) => Number(n).toFixed(2)

let pollHandle = null

function startPolling () {
  if (pollHandle) return
  pollHandle = setInterval(() => animations.refreshPlayers(), 500)
}
function stopPolling () {
  if (pollHandle) clearInterval(pollHandle)
  pollHandle = null
}

async function play (id) {
  await animations.start(id)
  startPolling()
}
async function stop (id) {
  await animations.stop(id)
  // poll a tick more to catch the "stopped" state
  setTimeout(() => animations.refreshPlayers(), 200)
}

function newAnimation () {
  animations.startNew()
  emit('edit')
}

async function edit (id) {
  await animations.load(id)
  emit('edit')
}

onMounted(async () => {
  await animations.reload()
  await animations.refreshPlayers()
  startPolling()
})
import { onBeforeUnmount } from 'vue'
onBeforeUnmount(stopPolling)
</script>

<template>
  <div class="space-y-4">
    <div class="flex items-center justify-between">
      <div>
        <h3 class="text-lg font-semibold text-fg-strong">Animation Library</h3>
        <p class="text-sm text-fg-muted mt-1">
          Each animation owns value tracks (continuous channel data) and trigger
          tracks (one-shot commands at specific times). Wire individual tracks
          into routing sheets to drive peripherals or publish ROS topics.
        </p>
      </div>
      <div class="flex gap-2">
        <button class="btn-secondary" @click="emit('import-maestro')">
          <span class="material-icons icon-sm">file_upload</span>
          Import Maestro
        </button>
        <button class="btn-primary" @click="newAnimation">
          <span class="material-icons icon-sm">add</span>
          New animation
        </button>
      </div>
    </div>

    <div v-if="animations.error" class="p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">
      {{ animations.error }}
    </div>

    <ul v-if="animations.list.length" class="divide-y divide-line/50 rounded-lg border border-line/50">
      <li v-for="a in animations.list" :key="a.id" class="flex items-center gap-3 p-3">
        <div class="flex-1 min-w-0">
          <div class="text-sm font-medium text-fg-strong truncate">{{ a.name || a.id }}</div>
          <div class="text-xs text-fg-muted">
            {{ fmtNumber(a.duration) }}s · {{ a.fps }} fps ·
            {{ a.value_tracks }} value · {{ a.trigger_tracks }} trigger
            <span v-if="a.loop"> · loop</span>
          </div>
        </div>
        <button v-if="!animations.isPlaying(a.id)"
                class="btn-sm bg-emerald-500/80 hover:bg-emerald-500 text-fg-strong"
                @click="play(a.id)">
          <span class="material-icons icon-sm">play_arrow</span>
        </button>
        <button v-else class="btn-sm bg-red-500/80 hover:bg-red-500 text-fg-strong"
                @click="stop(a.id)">
          <span class="material-icons icon-sm">stop</span>
        </button>
        <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong" @click="edit(a.id)">
          <span class="material-icons icon-sm">edit</span>
        </button>
        <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong"
                @click="animations.remove(a.id)">
          <span class="material-icons icon-sm">delete</span>
        </button>
      </li>
    </ul>
    <p v-else-if="!animations.loading" class="text-sm text-fg-muted italic">
      No animations saved yet. Start one from scratch or import a Maestro `.xml`.
    </p>
  </div>
</template>
