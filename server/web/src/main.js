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
const _ws = useWsStore()
_ws.connect()

// Console-kiosk auto-auth: if the URL carries ?kiosk_token=<tok> (the
// console_display peripheral injects this into the kiosk URL it gives
// Chromium), present it during the auth handshake so the Pi browser
// doesn't sit at LoginScreen waiting for human input. The server only
// accepts the token if it matches WebSocketConfig.kiosk_token — see
// _handle_auth_login in websocket_handler.py.
const kioskToken = new URLSearchParams(window.location.search).get('kiosk_token')
if (kioskToken) {
  _ws.on('auth_required', () => {
    _ws.authenticate({ kiosk_token: kioskToken }).catch(() => {})
  })
}
