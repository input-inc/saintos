<script setup>
import { computed } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useNavDrawer } from '@/composables/useNav'

const ws = useWsStore()
const { toggleDrawer } = useNavDrawer()

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
      <div class="flex items-center justify-between gap-2 h-16">
        <!-- Left: hamburger (< lg) + logo -->
        <div class="flex items-center gap-2 sm:gap-3 min-w-0">
          <button
            class="lg:hidden -ml-1 p-2 rounded-lg text-fg-muted hover:text-fg-strong hover:bg-surface transition-colors"
            title="Menu"
            aria-label="Open navigation menu"
            @click="toggleDrawer"
          >
            <span class="material-icons">menu</span>
          </button>
          <div class="w-9 h-9 sm:w-10 sm:h-10 rounded-xl bg-gradient-to-br from-cyan-500 to-blue-600 flex items-center justify-center shrink-0">
            <span class="material-icons text-fg-strong icon-md">computer</span>
          </div>
          <div class="min-w-0">
            <h1 class="text-lg sm:text-xl font-bold text-fg-strong leading-tight truncate">SAINT.OS</h1>
            <p class="hidden sm:block text-xs text-fg-muted">Administration Console</p>
          </div>
        </div>

        <!-- Right: connection pill + E-Stop -->
        <div class="flex items-center gap-2 sm:gap-3 shrink-0">
          <!-- Connection pill — collapses to just the status dot + logout on
               narrow screens; the label reappears at sm. -->
          <div class="flex items-center gap-2 px-2 sm:px-3 py-1.5 rounded-full bg-panel border border-line">
            <span :class="['status-dot w-2 h-2 rounded-full shrink-0', status.dot]" />
            <span class="hidden sm:inline text-sm text-fg truncate max-w-[10rem]">{{ status.text }}</span>
            <button class="sm:ml-1 p-0.5 rounded hover:bg-surface transition-colors" title="Disconnect" @click="disconnect">
              <span class="material-icons text-fg-muted text-base">logout</span>
            </button>
          </div>

          <!-- Global E-Stop — always visible; label hides below sm so the
               button stays a compact tap target and never wraps. -->
          <button class="btn-danger whitespace-nowrap" title="Emergency stop — all nodes" @click="estop">
            <span class="material-icons icon-sm">warning</span>
            <span class="hidden sm:inline">E-Stop</span>
          </button>
        </div>
      </div>
    </div>
    <!-- Accent line -->
    <div class="h-px bg-gradient-to-r from-transparent via-cyan-500 to-transparent"></div>
  </header>
</template>
