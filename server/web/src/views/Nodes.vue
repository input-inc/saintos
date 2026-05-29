<script setup>
import { computed, onMounted, ref } from 'vue'
import { useNodesStore } from '@/stores/nodes'
import { useWsStore } from '@/stores/ws'
import AdoptModal from '@/components/AdoptModal.vue'

const ws = useWsStore()
const nodes = useNodesStore()
const adopting = ref(null)            // node object being adopted

onMounted(() => nodes.fetchAll().catch(() => {}))

async function remove (id) {
  if (!confirm(`Remove node ${id}?\n\nIf the node is still running, it will reappear when it announces itself.`)) return
  try { await ws.management('remove_node', { node_id: id }); nodes.fetchAll() }
  catch (e) { console.warn('remove_node failed:', e) }
}

// Bulk firmware update — issue update_firmware to every adopted node that
// has the `update_available` flag from the server's announcement.
async function updateAll () {
  const targets = nodes.all.filter(n => n.update_available)
  if (!targets.length) return
  if (!confirm(`Update firmware on ${targets.length} node(s)?`)) return
  for (const n of targets) {
    try { await ws.management('update_firmware', { node_id: n.node_id }) }
    catch (e) { console.warn(`update_firmware ${n.node_id} failed:`, e) }
  }
  await nodes.fetchAll()
}

const updatable = computed(() => nodes.all.filter(n => n.update_available).length)
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-fg-strong">Nodes</h2>
      <div class="flex items-center gap-2">
        <span class="text-xs text-fg-muted">{{ nodes.all.length }} adopted · {{ nodes.unadopted.length }} unadopted</span>
        <button v-if="updatable" class="btn-primary text-sm" title="Update firmware on all eligible nodes" @click="updateAll">
          <span class="material-icons icon-sm">system_update</span>
          Update all ({{ updatable }})
        </button>
        <button class="btn-secondary" @click="nodes.scan()">
          <span class="material-icons icon-sm">search</span>
          Scan
        </button>
      </div>
    </div>

    <div v-if="nodes.unadopted.length" class="mb-6">
      <h3 class="text-base font-semibold text-amber-300 mb-3 flex items-center gap-2">
        <span class="material-icons text-amber-400">new_releases</span>
        Unadopted ({{ nodes.unadopted.length }})
      </h3>
      <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        <div v-for="n in nodes.unadopted" :key="n.node_id" class="node-card border-amber-700/40 flex flex-col gap-3">
          <div class="flex items-center justify-between">
            <span class="font-semibold text-fg-strong">{{ n.node_id }}</span>
            <span class="px-2 py-0.5 text-xs rounded-full bg-amber-500/20 text-amber-300">new</span>
          </div>
          <div class="text-xs text-fg-muted font-mono space-y-0.5">
            <div>{{ n.hardware_model || '—' }}</div>
            <div>{{ n.ip_address || '—' }}</div>
            <div>{{ n.chip_family || 'unknown chip' }} · FW {{ n.firmware_version || '—' }}</div>
          </div>
          <div class="flex gap-2 mt-1">
            <button class="btn-primary text-sm flex-1 justify-center" @click="adopting = n">
              <span class="material-icons icon-sm">person_add</span> Adopt
            </button>
            <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong" @click="remove(n.node_id)">
              <span class="material-icons icon-sm">delete</span>
            </button>
          </div>
        </div>
      </div>
    </div>

    <h3 class="text-base font-semibold text-fg-strong mb-3 flex items-center gap-2">
      <span class="material-icons text-emerald-400">dns</span>
      Adopted ({{ nodes.all.length }})
    </h3>

    <div v-if="nodes.loading && !nodes.all.length" class="text-sm text-fg-muted">Loading…</div>

    <div v-else-if="!nodes.all.length" class="card text-center py-10">
      <span class="material-icons icon-lg text-fg-faint">dns</span>
      <h3 class="text-lg font-semibold text-fg mt-4">No nodes adopted yet</h3>
      <p class="text-fg-faint text-sm mt-1">Scan to discover available nodes.</p>
    </div>

    <div v-else class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
      <RouterLink
        v-for="n in nodes.all"
        :key="n.node_id"
        :to="{ name: 'node-overview', params: { id: n.node_id } }"
        class="node-card flex flex-col gap-3"
      >
        <div class="flex items-start justify-between">
          <div class="flex items-center gap-2">
            <span :class="['w-2.5 h-2.5 rounded-full', n.online ? 'bg-emerald-500 animate-pulse-dot' : 'bg-slate-500']" />
            <span class="font-semibold text-fg-strong">{{ n.display_name || n.node_id }}</span>
          </div>
          <span class="px-2 py-0.5 text-xs rounded-full bg-surface text-fg">
            {{ n.role || 'unassigned' }}
          </span>
        </div>
        <div class="text-xs text-fg-muted space-y-0.5 font-mono">
          <div>{{ n.hardware_model || '—' }}</div>
          <div>{{ n.ip_address || '—' }}</div>
          <div>FW {{ n.firmware_version || '—' }}</div>
        </div>
      </RouterLink>
    </div>

    <AdoptModal
      v-if="adopting"
      :node-id="adopting.node_id"
      :announced-chip="adopting.chip_family || ''"
      @close="adopting = null"
      @adopted="() => { adopting = null; nodes.fetchAll() }"
    />
  </section>
</template>
