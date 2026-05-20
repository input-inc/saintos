import { defineStore } from 'pinia'
import { computed, ref } from 'vue'
import { useWsStore } from './ws'

// Server-side settings (server name, ports, ROS throttle, LiveLink, …).
// Loaded once from `get_settings` and persisted back via `set_settings`.
// Dirty tracking is via deep JSON compare against the last fetched copy.
export const useSettingsStore = defineStore('settings', () => {
  const ws = useWsStore()
  const original = ref(null)
  const current = ref(null)
  const loading = ref(false)
  const saving = ref(false)
  const error = ref('')

  async function load () {
    loading.value = true
    error.value = ''
    try {
      const r = await ws.management('get_settings', {})
      const s = r?.settings ?? r ?? {}
      original.value = JSON.parse(JSON.stringify(s))
      current.value  = JSON.parse(JSON.stringify(s))
    } catch (e) {
      error.value = e.message || String(e)
    } finally {
      loading.value = false
    }
  }

  const dirty = computed(() =>
    JSON.stringify(current.value) !== JSON.stringify(original.value)
  )

  async function save () {
    if (!current.value || !dirty.value) return
    saving.value = true
    error.value = ''
    try {
      await ws.management('set_settings', { settings: current.value })
      original.value = JSON.parse(JSON.stringify(current.value))
    } catch (e) {
      error.value = e.message || String(e)
    } finally {
      saving.value = false
    }
  }

  function reset () {
    if (!original.value) return
    current.value = JSON.parse(JSON.stringify(original.value))
  }

  return { original, current, loading, saving, error, dirty, load, save, reset }
})
