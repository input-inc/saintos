import { defineStore } from 'pinia'
import { ref, watch } from 'vue'

// Operator display preferences. Persisted to localStorage so they
// survive page reloads (independent of the server's settings — these
// are per-browser, like dark/light theme).
const STORAGE_KEY = 'saint_display_prefs'

export const useDisplayStore = defineStore('display', () => {
  const initial = (() => {
    try { return JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}') }
    catch (_) { return {} }
  })()

  const temperatureUnit = ref(initial.temperature_unit || 'celsius')

  watch(temperatureUnit, () => {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({ temperature_unit: temperatureUnit.value }))
    } catch (_) {}
  })

  function formatTemperature (celsius) {
    if (celsius === null || celsius === undefined || Number.isNaN(celsius)) return '--'
    if (temperatureUnit.value === 'fahrenheit') {
      return `${(celsius * 9 / 5 + 32).toFixed(1)}°F`
    }
    return `${celsius.toFixed(1)}°C`
  }

  return { temperatureUnit, formatTemperature }
})
