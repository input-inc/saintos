import { defineStore } from 'pinia'
import { ref, watch } from 'vue'

// Operator display preferences. Persisted to localStorage so they
// survive page reloads (independent of the server's settings — these
// are per-browser, like dark/light theme).
const STORAGE_KEY = 'saint_display_prefs'

// Themes ride on a `data-theme` attribute on <html>. Each theme is
// a block of CSS variable overrides in style.css — adding a new
// theme is one block of values, no component changes.
//
// The list pairs the data-theme value with a human label for the
// dropdown in Settings → Appearance. The first entry is the default.
// Borrowed palettes from daisyUI v4; we map their base/200/300/content
// onto our SAINT.OS canvas/panel/surface/fg tokens.
const THEMES = [
  { value: 'dark',      label: 'Dark',       mode: 'dark'  },
  { value: 'light',     label: 'Light',      mode: 'light' },
  { value: 'night',     label: 'Night',      mode: 'dark'  },
  { value: 'business',  label: 'Business',   mode: 'dark'  },
  { value: 'dracula',   label: 'Dracula',    mode: 'dark'  },
  { value: 'synthwave', label: 'Synthwave',  mode: 'dark'  },
  { value: 'forest',    label: 'Forest',     mode: 'dark'  },
  { value: 'coffee',    label: 'Coffee',     mode: 'dark'  },
  { value: 'corporate', label: 'Corporate',  mode: 'light' },
  { value: 'cupcake',   label: 'Cupcake',    mode: 'light' },
  { value: 'nord',      label: 'Nord',       mode: 'light' },
  { value: 'autumn',    label: 'Autumn',     mode: 'light' },
]
const THEME_VALUES = THEMES.map(t => t.value)

function applyTheme (theme) {
  const root = typeof document !== 'undefined' ? document.documentElement : null
  if (root) root.setAttribute('data-theme', THEME_VALUES.includes(theme) ? theme : 'dark')
}

export const useDisplayStore = defineStore('display', () => {
  const initial = (() => {
    try { return JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}') }
    catch (_) { return {} }
  })()

  const temperatureUnit = ref(initial.temperature_unit || 'celsius')
  const theme           = ref(THEME_VALUES.includes(initial.theme) ? initial.theme : 'dark')

  // Eager apply so the html attribute is set before the first paint.
  applyTheme(theme.value)

  function persist () {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({
        temperature_unit: temperatureUnit.value,
        theme:            theme.value,
      }))
    } catch (_) {}
  }
  watch(temperatureUnit, persist)
  watch(theme, (t) => { applyTheme(t); persist() })

  function formatTemperature (celsius) {
    if (celsius === null || celsius === undefined || Number.isNaN(celsius)) return '--'
    if (temperatureUnit.value === 'fahrenheit') {
      return `${(celsius * 9 / 5 + 32).toFixed(1)}°F`
    }
    return `${celsius.toFixed(1)}°C`
  }

  return { temperatureUnit, theme, formatTemperature, themes: THEMES }
})
