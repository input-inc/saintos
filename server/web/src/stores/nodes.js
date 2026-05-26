import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { useWsStore } from './ws'

// Adopted + unadopted node registry. Backed by `list_adopted` /
// `list_unadopted` management queries plus the `adopted_nodes` and
// `unadopted_nodes` topic broadcasts. Components read via `byId(id)`
// to get a reactive view that updates as broadcasts arrive.
export const useNodesStore = defineStore('nodes', () => {
  const ws = useWsStore()
  const all = ref([])
  const unadopted = ref([])
  const loaded = ref(false)
  const loading = ref(false)

  async function fetchAll () {
    loading.value = true
    try {
      const [a, u] = await Promise.all([
        ws.management('list_adopted'),
        ws.management('list_unadopted'),
      ])
      all.value = a?.nodes || []
      unadopted.value = u?.nodes || []
      loaded.value = true
    } finally {
      loading.value = false
    }
  }

  async function scan () {
    try { await ws.management('scan_nodes', {}) } catch (_) {}
    await fetchAll().catch(() => {})
  }

  ws.on('ready', () => fetchAll().catch(() => {}))
  ws.on('state', (msg) => {
    if (!msg?.node) return
    if (msg.node === 'adopted_nodes' && Array.isArray(msg.data?.nodes)) {
      all.value = msg.data.nodes
    }
    if (msg.node === 'unadopted_nodes' && Array.isArray(msg.data?.nodes)) {
      unadopted.value = msg.data.nodes
    }
  })

  const byId = (id) => computed(() => all.value.find(n => n.node_id === id) || null)

  return { all, unadopted, loaded, loading, fetchAll, scan, byId }
})
