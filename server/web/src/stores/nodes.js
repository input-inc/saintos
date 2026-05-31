import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { useWsStore } from './ws'

// Adopted + unadopted node registry. Backed by `list_adopted` /
// `list_unadopted` management queries for initial hydration, then
// live-updated by subscribing to the `nodes` topic broadcast (the
// server publishes the full list on a 1 Hz timer plus on any node
// event — adoption, removal, re-announce after firmware reboot, etc.).
// Components read via `byId(id)` to get a reactive view that updates
// as broadcasts arrive.
export const useNodesStore = defineStore('nodes', () => {
  const ws = useWsStore()
  const all = ref([])
  const unadopted = ref([])
  const loaded = ref(false)
  const loading = ref(false)
  let subscribed = false

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

  // Subscribe to the `nodes` broadcast so the store auto-refreshes
  // when a node's state changes (firmware version bump after OTA, role
  // change, adoption, going offline, etc.) without the UI having to
  // poll. Without this, the firmware version shown in node Overview
  // is whatever fetchAll saw — it doesn't update when the node reboots
  // with the new firmware unless the user navigates away and back.
  async function ensureSubscribed () {
    if (subscribed) return
    try {
      await ws.subscribe(['nodes'])
      subscribed = true
    } catch (e) {
      console.warn('nodes topic subscribe failed:', e)
    }
  }

  ws.on('ready', () => {
    subscribed = false
    ensureSubscribed().catch(() => {})
    fetchAll().catch(() => {})
  })
  ws.on('state', (msg) => {
    // Server publishes `{type:'state', node:'nodes', data:{adopted:[...],
    // unadopted:[...]}}` from websocket_handler._broadcast_loop. Match
    // both the topic name and the payload keys; the legacy shape
    // (`adopted_nodes` / `unadopted_nodes` topics with `{nodes:[...]}`)
    // never matched what the server actually emits.
    if (msg?.node !== 'nodes' || !msg.data) return
    if (Array.isArray(msg.data.adopted))   all.value = msg.data.adopted
    if (Array.isArray(msg.data.unadopted)) unadopted.value = msg.data.unadopted
  })

  // Auto-subscribe right now too, in case the WS was already ready
  // before the store got instantiated.
  ensureSubscribed().catch(() => {})

  const byId = (id) => computed(() => all.value.find(n => n.node_id === id) || null)

  return { all, unadopted, loaded, loading, fetchAll, scan, byId }
})
