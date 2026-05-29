<script setup>
import { computed } from 'vue'
import { useWsStore } from '@/stores/ws'

const ws = useWsStore()

const status = computed(() => {
  if (!ws.connected)                              return { dot: 'bg-amber-500 animate-pulse-dot', text: 'Connecting…' }
  if (ws.authRequired && !ws.authenticated)       return { dot: 'bg-amber-500', text: 'Auth required' }
  return { dot: 'bg-emerald-500', text: ws.serverName || 'Connected' }
})

function disconnect () { ws.disconnect() }
function estop () {
  ws.command('system', 'estop', { target: 'all' }).catch(() => {})
}
</script>

<template>
  <header class="sticky top-0 z-50 bg-canvas/80 backdrop-blur-lg border-b border-line/50">
    <div class="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
      <div class="flex items-center justify-between h-16">
        <!-- Logo -->
        <div class="flex items-center gap-3">
          <div class="w-10 h-10 rounded-xl bg-gradient-to-br from-cyan-500 to-blue-600 flex items-center justify-center">
            <span class="material-icons text-fg-strong icon-md">computer</span>
          </div>
          <div>
            <h1 class="text-xl font-bold text-fg-strong leading-tight">SAINT.OS</h1>
            <p class="text-xs text-fg-muted">Administration Console</p>
          </div>
        </div>

        <!-- Connection pill -->
        <div class="flex items-center gap-2 px-3 py-1.5 rounded-full bg-panel border border-line">
          <span :class="['status-dot w-2 h-2 rounded-full', status.dot]" />
          <span class="text-sm text-fg">{{ status.text }}</span>
          <button class="ml-1 p-0.5 rounded hover:bg-surface transition-colors" title="Disconnect" @click="disconnect">
            <span class="material-icons text-fg-muted text-base">logout</span>
          </button>
        </div>

        <!-- Global E-Stop -->
        <button class="btn-danger" @click="estop">
          <span class="material-icons icon-sm">warning</span>
          E-Stop
        </button>
      </div>
    </div>
    <!-- Accent line -->
    <div class="h-px bg-gradient-to-r from-transparent via-cyan-500 to-transparent"></div>
  </header>
</template>
