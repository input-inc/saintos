<script setup>
import './console.css'

// Mobile-app-style chrome for every Console view:
//   - Title bar (fixed height, red underline — NO glow on the title
//     text; the underline carries the visual emphasis and the title
//     itself stays crisp so long strings stay readable).
//   - Content slot (fills the rest, scrolls if needed)
//   - Bottom button bar — always exactly 2 buttons, equal width.
//     Defaults are "Activate" and "System"; override via the
//     `buttons` prop. Passing fewer than 2 backfills with the
//     default labels; more than 2 is silently truncated. Callers
//     that need different chrome should use the `#buttons` slot.
//
// Server-disconnect handling: when the WS drops (server restart,
// network blip), the chrome flips the subtitle to a pulsing red
// "OFFLINE" pill and dims the content slot. The view itself keeps
// rendering the last-known data — better to show stale numbers
// dimmed than to blank the panel — so when the WS reconnects we just
// undim and the live values resume. The kiosk's whole job is to be
// visually informative across server hiccups; redirecting / blanking
// would defeat that.
//
// The shell is `position: relative` so a parent can constrain it (a
// Histoire variant, a tab inside the operator UI) without needing a
// full-viewport fixed layout. The kiosk launcher will mount the
// router-view directly into <body> at full size; this works there too.

import { computed } from 'vue'
import { useWsStore } from '@/stores/ws'

const props = defineProps({
  title: { type: String, default: '' },
  subtitle: { type: String, default: '' },
  buttons: {
    type: Array,
    default: () => [
      { label: 'Activate', onClick: null },
      { label: 'System',   onClick: null },
    ],
  },
  // Force pure black bg without the dot-matrix overlay — useful for
  // dense screens where the grid pattern would noise up tiny text.
  flatBackground: { type: Boolean, default: false },
})

const ws = useWsStore()

// Offline = either the WS isn't open OR the server requires auth and
// we haven't completed the kiosk_token handshake yet. Both states
// mean fresh data isn't reaching us. We treat the brief window
// between reconnect and auth-ok (sub-second on a healthy LAN) as
// "offline" too — that's accurate; the pin_state broadcasts the
// kiosk reads from need an authenticated client.
const offline = computed(() =>
  !ws.connected || (ws.authRequired && !ws.authenticated)
)

// Bottom bar always has exactly two slots. Pad with the defaults if
// the caller supplied fewer, truncate if more.
const DEFAULT_BUTTONS = [
  { label: 'Activate', onClick: null },
  { label: 'System',   onClick: null },
]
const twoButtons = computed(() => {
  const supplied = (props.buttons || []).slice(0, 2)
  while (supplied.length < 2) supplied.push(DEFAULT_BUTTONS[supplied.length])
  return supplied
})
</script>

<template>
  <div
    class="console-root flex flex-col h-full min-h-[480px] relative"
    :style="flatBackground ? 'background-image: none' : null"
  >
    <!-- Title bar — fixed height, red underline carries the visual
         emphasis. Title text itself is plain red (no glow) so longer
         strings stay crisp; the subtitle sits to the right in dim
         red as a status badge.

         When the WS is disconnected, the subtitle is replaced with a
         pulsing "OFFLINE" pill so the operator sees at a glance that
         the data isn't live. The view itself stays visible (just
         dimmed via the content slot wrapper) so the last-known
         numbers remain readable across the disconnect. -->
    <header class="flex items-baseline justify-between px-12 pt-10 pb-6 border-b-[3px] border-[var(--led-red)] shrink-0"
            style="box-shadow: 0 3px 0 0 var(--led-red-glow);">
      <h1 class="text-6xl tracking-[0.18em] font-semibold"
          style="color: var(--led-red);">
        {{ title }}
      </h1>
      <span v-if="offline"
            class="console-offline-pill text-3xl tracking-[0.18em] font-semibold">
        OFFLINE
      </span>
      <span v-else-if="subtitle" class="led-text-plain text-3xl tracking-[0.18em]">{{ subtitle }}</span>
    </header>

    <!-- Content area — fills available height; scrolls internally if
         the view overflows. Dimmed when offline so the stale data
         reads as "this is no longer live" without disappearing
         entirely. CSS-only fade so no layout reflow happens at the
         transition boundary. -->
    <main class="flex-1 min-h-0 overflow-auto p-12 console-content"
          :class="offline ? 'console-content--offline' : ''">
      <slot />
    </main>

    <!-- Bottom button bar — always exactly two equal-width buttons.
         Slot wins over prop if both are provided. -->
    <footer class="border-t-[3px] border-[var(--led-red)] shrink-0"
            style="box-shadow: 0 -3px 0 0 var(--led-red-glow);">
      <slot name="buttons">
        <div class="grid grid-cols-2">
          <button
            v-for="(b, i) in twoButtons"
            :key="i"
            class="led-button text-4xl py-10 tracking-[0.18em]"
            :style="i === 0 ? 'border-right-width: 0' : ''"
            @click="b.onClick && b.onClick($event)"
          >
            {{ b.label }}
          </button>
        </div>
      </slot>
    </footer>
  </div>
</template>
