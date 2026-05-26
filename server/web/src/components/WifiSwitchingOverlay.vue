<script setup>
//
// Full-screen "AP restarting…" overlay shown after a
// wifi_set_credentials or wifi_set_channel call. Hides itself the next
// time the websocket store emits 'ready' (i.e. reconnects after the AP
// comes back), or after a 60 s safety timeout.
//
import { onMounted, onUnmounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'

defineProps({
  detail: { type: String, default: '' },
})
const emit = defineEmits(['close'])

const ws = useWsStore()
const visible = ref(true)

function onReady () { close() }
function close () {
  if (!visible.value) return
  visible.value = false
  emit('close')
}

let timer = null
onMounted(() => {
  ws.on('ready', onReady)
  // Safety net — never strand the overlay if reconnect never fires.
  timer = setTimeout(close, 60_000)
})
onUnmounted(() => {
  ws.off('ready', onReady)
  if (timer) clearTimeout(timer)
})
</script>

<template>
  <div v-if="visible" class="fixed inset-0 z-50 bg-black/80 flex items-center justify-center">
    <div class="text-center text-white">
      <div class="material-icons text-6xl text-cyan-400 mb-4 animate-spin">refresh</div>
      <h3 class="text-xl font-semibold mb-2">Restarting WiFi AP…</h3>
      <p class="text-slate-400 mb-2">The dashboard will reconnect when the AP returns.</p>
      <p class="text-xs text-slate-500">
        {{ detail || 'If you changed the SSID or password, you may need to reconnect your laptop/phone to the new network manually.' }}
      </p>
    </div>
  </div>
</template>
