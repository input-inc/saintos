import { defineStore } from 'pinia'
import { ref } from 'vue'
import { useWsStore } from './ws'

// Peripheral + widget type catalog — fetched once from the server,
// then cached reactively. The server returns both lists in the same
// `get_peripheral_catalog` response, mirroring legacy behavior.
export const usePeripheralCatalog = defineStore('peripheralCatalog', () => {
  const ws = useWsStore()
  const types = ref([])
  const widgetTypes = ref([])
  const loaded = ref(false)
  let inflight = null

  async function ensureLoaded () {
    if (loaded.value) return
    if (inflight) return inflight
    inflight = ws.management('get_peripheral_catalog', {}).then(r => {
      types.value = r?.peripheral_types || []
      widgetTypes.value = r?.widget_types || []
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
  function widgetType (id) {
    return widgetTypes.value.find(t => t.id === id) || null
  }

  return { types, widgetTypes, loaded, ensureLoaded, byType, widgetType }
})
