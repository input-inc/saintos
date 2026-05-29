<script setup>
import { computed } from 'vue'
import { useWsStore } from '@/stores/ws'

const ws = useWsStore()

// Read the version from Vite's define replacement, fall back to a constant.
const version = computed(() => __APP_VERSION__ || '0.5.0')
const versionHash = computed(() => ws.serverName ? ws.serverName : '')
</script>

<template>
  <footer class="border-t border-line-subtle bg-canvas/50">
    <div class="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-4">
      <div class="flex items-center justify-between text-sm text-fg-faint">
        <span>
          SAINT.OS v{{ version }}
          <span v-if="versionHash" class="ml-2 font-mono text-xs text-fg-faint">{{ versionHash }}</span>
        </span>
        <span>System for Articulated Intelligence and Navigation Tasks</span>
      </div>
    </div>
  </footer>
</template>
