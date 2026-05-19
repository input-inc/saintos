import { createApp } from 'vue'
import { createPinia } from 'pinia'

import App from './App.vue'
import { router } from './router'
import { useWsStore } from './stores/ws'

import './style.css'

const app = createApp(App)
app.use(createPinia())
app.use(router)
app.mount('#app')

// Open the WebSocket connection as soon as Pinia is wired up — every
// page assumes the live stream is available. The store handles
// reconnect, so this is fire-and-forget.
useWsStore().connect()
