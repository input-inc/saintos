import { createApp } from 'vue'
import { createPinia } from 'pinia'

import App from './App.vue'
import { router } from './router'
import { useWsStore } from './stores/ws'
import { useDisplayStore } from './stores/display'

import './style.css'

const app = createApp(App)
app.use(createPinia())
app.use(router)

// Initialize display preferences before mount so the saved theme is
// applied to <html data-theme="…"> before the first paint — avoids
// the dark-flash on load when the operator picked light mode.
useDisplayStore()

app.mount('#app')

// Open the WebSocket connection as soon as Pinia is wired up — every
// page assumes the live stream is available. The store handles
// reconnect, so this is fire-and-forget.
useWsStore().connect()
