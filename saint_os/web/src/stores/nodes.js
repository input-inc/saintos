import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { useWsStore } from './ws'

// Adopted-nodes registry. Backed by the server's `list_adopted` management
// query plus the `adopted_nodes` topic broadcast. Components reading
// `byId(id)` get a reactive view that updates as broadcasts arrive.
export const useNodesStore = defineStore('nodes', () => {
  const ws = useWsStore()
  const all = ref([])
  const loaded = ref(false)
  const loading = ref(false)

  async function fetchAll () {
    loading.value = true
    try {
      const r = await ws.management('list_adopted')
      all.value = r?.nodes || []
      loaded.value = true
    } finally {
      loading.value = false
    }
  }

  // Keep the list fresh: re-fetch on connection, react to broadcasts.
  ws.on('ready', () => fetchAll().catch(() => {}))
  ws.on('state', (msg) => {
    if (msg?.node === 'adopted_nodes' && Array.isArray(msg.data?.nodes)) {
      all.value = msg.data.nodes
    }
  })

  const byId = (id) => computed(() => all.value.find(n => n.node_id === id) || null)

  return { all, loaded, loading, fetchAll, byId }
})
