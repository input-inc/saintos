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
//
// Scan BOTH the outer ?query (window.location.search) AND inside the
// hash fragment (window.location.hash), because Vue Router uses
// hash-mode routing — depending on which side of the '#' the URL
// builder put the token, it lives in one place or the other. Older
// builders put it inside the hash; current ones put it before. Tolerate
// both so a Pi running an old console_display.py still auto-auths.
function extractKioskToken () {
  // 1. Outer query string: http://server/?kiosk_token=...#/route
  const outer = new URLSearchParams(window.location.search).get('kiosk_token')
  if (outer) return outer
  // 2. In-hash query: http://server/#/route?kiosk_token=...
  const hash = window.location.hash || ''
  const qIdx = hash.indexOf('?')
  if (qIdx >= 0) {
    const inner = new URLSearchParams(hash.slice(qIdx + 1)).get('kiosk_token')
    if (inner) return inner
  }
  return null
}
const kioskToken = extractKioskToken()
if (kioskToken) {
  _ws.on('auth_required', () => {
    _ws.authenticate({ kiosk_token: kioskToken }).catch(() => {})
  })
}
