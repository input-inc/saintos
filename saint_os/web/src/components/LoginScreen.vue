<script setup>
import { onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'

const ws = useWsStore()
const host = ref(window.location.host)
const password = ref('')
const error = ref('')
const connecting = ref(false)

onMounted(() => {
  try {
    const s = JSON.parse(localStorage.getItem('saint_ws_settings') || '{}')
    if (s.host) host.value = s.host
    if (s.password) password.value = s.password
  } catch (_) {}
})

async function submit () {
  error.value = ''
  connecting.value = true
  try {
    localStorage.setItem('saint_ws_settings', JSON.stringify({ host: host.value, password: password.value }))
    if (!ws.connected) {
      ws.connect()
      // Wait briefly for the connection to come up.
      await new Promise((resolve) => {
        const tick = () => { if (ws.connected) resolve(); else setTimeout(tick, 100) }
        setTimeout(tick, 100)
      })
    }
    if (ws.authRequired) {
      await ws.authenticate(password.value)
    }
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    connecting.value = false
  }
}
</script>

<template>
  <div class="min-h-screen flex items-center justify-center p-4">
    <div class="w-full max-w-md">
      <div class="text-center mb-8">
        <div class="w-20 h-20 rounded-2xl bg-gradient-to-br from-cyan-500 to-blue-600 flex items-center justify-center mx-auto mb-4">
          <span class="material-icons text-white text-4xl">computer</span>
        </div>
        <h1 class="text-3xl font-bold text-white">SAINT.OS</h1>
        <p class="text-slate-400 mt-1">Administration Console</p>
      </div>

      <div class="card">
        <h2 class="text-lg font-semibold text-white mb-4">Connect to server</h2>

        <div v-if="error" class="mb-4 p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-red-300 text-sm">{{ error }}</div>

        <form class="space-y-4" @submit.prevent="submit">
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-1">Server address</label>
            <input v-model="host" type="text" class="input-field w-full" placeholder="opensaint.local" />
            <p class="text-xs text-slate-500 mt-1">Leave empty to use this page's host.</p>
          </div>
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-1">Password</label>
            <input v-model="password" type="password" class="input-field w-full" placeholder="Enter password" />
            <p class="text-xs text-slate-500 mt-1">Leave empty if the server has no password.</p>
          </div>
          <button
            type="submit"
            :disabled="connecting"
            class="w-full py-2.5 bg-gradient-to-r from-cyan-500 to-blue-600 text-white font-medium rounded-lg hover:from-cyan-600 hover:to-blue-700 transition-all"
          >
            {{ connecting ? 'Connecting…' : 'Connect' }}
          </button>
        </form>
      </div>
    </div>
  </div>
</template>
