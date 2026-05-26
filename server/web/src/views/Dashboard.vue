<script setup>
import { computed, onMounted } from 'vue'
import { useNodesStore } from '@/stores/nodes'
import { useDisplayStore } from '@/stores/display'
import { useActivityStore } from '@/stores/activity'
import { useWsTopic } from '@/composables/useWsTopic'
import Widgets from '@/views/Widgets.vue'

const nodes = useNodesStore()
const display = useDisplayStore()
const activity = useActivityStore()
const systemStatus = useWsTopic(() => 'system_status')

onMounted(() => nodes.fetchAll().catch(() => {}))

const online  = computed(() => nodes.all.filter(n => n.online).length)
const offline = computed(() => nodes.all.length - online.value)

const cpu = computed(() => systemStatus.value?.cpu_usage ?? null)
const mem = computed(() => systemStatus.value?.memory_usage ?? null)
const temp = computed(() => systemStatus.value?.cpu_temp ?? null)
const uptime = computed(() => systemStatus.value?.uptime ?? null)

function fmtUptime (sec) {
  if (sec == null) return '--'
  const d = Math.floor(sec / 86400)
  const h = Math.floor((sec % 86400) / 3600)
  const m = Math.floor((sec % 3600) / 60)
  if (d) return `${d}d ${h}h`
  if (h) return `${h}h ${m}m`
  return `${m}m`
}

function fmtTime (ts) {
  if (!ts) return ''
  const t = new Date(ts * 1000)
  return t.toLocaleTimeString([], { hour12: false })
}
function classFor (level) {
  return ({ error: 'error', warn: 'warn', info: 'info', debug: 'debug' }[(level || 'info').toLowerCase()]) || 'info'
}
</script>

<template>
  <section class="space-y-6">
    <div class="flex items-center justify-between">
      <h2 class="text-2xl font-bold text-white">Dashboard</h2>
      <button class="btn-secondary" @click="nodes.scan()">
        <span class="material-icons icon-sm">search</span>
        Scan Nodes
      </button>
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-2 gap-6">
      <div class="card">
        <div class="flex items-center justify-between mb-4">
          <h3 class="text-lg font-semibold text-white">System status</h3>
          <span class="px-2 py-1 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400">Online</span>
        </div>
        <div class="grid grid-cols-2 gap-4">
          <div class="stat-item">
            <span class="stat-label">Uptime</span>
            <span class="stat-value">{{ fmtUptime(uptime) }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">CPU temp</span>
            <span class="stat-value">{{ display.formatTemperature(temp) }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">CPU usage</span>
            <div class="flex items-center gap-2">
              <div class="flex-1 h-2 bg-slate-700 rounded-full overflow-hidden">
                <div class="h-full bg-cyan-500 transition-all duration-300" :style="{ width: `${cpu ?? 0}%` }" />
              </div>
              <span class="stat-value text-sm w-12 text-right">{{ cpu != null ? `${cpu.toFixed(0)}%` : '--' }}</span>
            </div>
          </div>
          <div class="stat-item">
            <span class="stat-label">Memory</span>
            <div class="flex items-center gap-2">
              <div class="flex-1 h-2 bg-slate-700 rounded-full overflow-hidden">
                <div class="h-full bg-violet-500 transition-all duration-300" :style="{ width: `${mem ?? 0}%` }" />
              </div>
              <span class="stat-value text-sm w-12 text-right">{{ mem != null ? `${mem.toFixed(0)}%` : '--' }}</span>
            </div>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="flex items-center justify-between mb-4">
          <h3 class="text-lg font-semibold text-white">Nodes</h3>
          <RouterLink to="/nodes" class="text-xs text-cyan-400 hover:text-cyan-300 underline">manage</RouterLink>
        </div>
        <div class="grid grid-cols-3 gap-4">
          <div class="stat-item">
            <span class="stat-label">Adopted</span>
            <span class="stat-value text-2xl">{{ nodes.all.length }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Online</span>
            <span class="stat-value text-2xl text-emerald-400">{{ online }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Offline</span>
            <span class="stat-value text-2xl text-slate-400">{{ offline }}</span>
          </div>
        </div>
        <div v-if="nodes.unadopted.length" class="mt-4 pt-4 border-t border-slate-700/50 flex items-center justify-between">
          <span class="text-sm text-amber-300 flex items-center gap-2">
            <span class="material-icons icon-sm">new_releases</span>
            {{ nodes.unadopted.length }} unadopted node{{ nodes.unadopted.length === 1 ? '' : 's' }}
          </span>
          <RouterLink to="/nodes" class="text-xs text-amber-300 hover:text-amber-200 underline">adopt now →</RouterLink>
        </div>
      </div>
    </div>

    <div class="card">
      <h3 class="text-lg font-semibold text-white mb-3">Recent activity</h3>
      <div v-if="!activity.entries.length" class="text-sm text-slate-400 italic">No activity yet.</div>
      <div v-else class="font-mono text-xs space-y-1 max-h-64 overflow-y-auto">
        <div
          v-for="(e, i) in activity.entries.slice().reverse().slice(0, 100)"
          :key="i"
          :class="['log-entry', classFor(e.level)]"
        >
          <span class="text-slate-500 mr-2">{{ fmtTime(e.time) }}</span>
          {{ e.text }}
        </div>
      </div>
    </div>

    <div>
      <div class="flex items-center justify-between mb-4">
        <h3 class="text-lg font-semibold text-slate-400">Widgets</h3>
        <RouterLink to="/routes" class="btn-secondary text-sm">
          <span class="material-icons icon-sm">add</span>
          Configure on Routes page
        </RouterLink>
      </div>
      <Widgets embedded />
    </div>
  </section>
</template>
