import { defineStore } from 'pinia'
import { ref } from 'vue'
import { useWsStore } from './ws'

// Peripheral type catalog — fetched once from the server, then cached
// reactively. Components read `byType.value[id]` to get a type spec
// (channels, params, label).
export const usePeripheralCatalog = defineStore('peripheralCatalog', () => {
  const ws = useWsStore()
  const types = ref([])
  const loaded = ref(false)
  let inflight = null

  async function ensureLoaded () {
    if (loaded.value) return
    if (inflight) return inflight
    inflight = ws.management('get_peripheral_catalog', {}).then(r => {
      types.value = r?.peripheral_types || []
      loaded.value = true
      inflight = null
    }).catch(e => {
      console.warn('Peripheral catalog fetch failed:', e)
      inflight = null
    })
    return inflight
  }

  function byType (id) {
    return types.value.find(t => t.id === id) || null
  }

  return { types, loaded, ensureLoaded, byType }
})
