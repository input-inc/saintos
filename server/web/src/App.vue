<script setup>
import { computed, watch } from 'vue'
import { useRoute } from 'vue-router'
import AppHeader from '@/components/AppHeader.vue'
import NavBar from '@/components/NavBar.vue'
import LoginScreen from '@/components/LoginScreen.vue'
import { useWsStore } from '@/stores/ws'

const ws = useWsStore()
const needLogin = computed(() => ws.authRequired && !ws.authenticated)

// useRoute() is inject()-backed, so it MUST be resolved once here in
// setup — never re-called lazily inside a computed getter. When such a
// computed re-evaluates outside the active component-setup context
// (route change, reconnect-driven re-render) inject() returns undefined
// and `.meta` throws "undefined is not an object", which crashes the
// whole app render and leaves the dashboard blank (no nodes /
// peripherals / routing). Resolve the route object once; every
// dependent computed reads off it.
const route = useRoute()

// Console kiosk routes opt out of the operator-UI chrome (AppHeader,
// NavBar, max-w container) via meta.chromeless. They render their own
// title bar / button bar via ConsoleScreen and take the full viewport.
const chromeless = computed(() => !!route.meta?.chromeless)

// Full-bleed pages (Routes, Logs) opt out of <main>'s max-w-7xl + padding
// via body-level classes that style.css keys off — matches vanilla's
// `body.routing-fullwidth` / `body.logs-fullwidth` mechanism.
watch(
  () => route.name,
  (name) => {
    document.body.classList.toggle('routing-fullwidth',    name === 'routes')
    document.body.classList.toggle('logs-fullwidth',       name === 'logs')
    // Both the management landing AND the editor share the
    // fullwidth + viewport-clamped shell.
    document.body.classList.toggle('animations-fullwidth',
      name === 'animations' || name === 'animation-editor')
  },
  { immediate: true },
)
</script>

<template>
  <LoginScreen v-if="needLogin" />
  <div v-else-if="chromeless" class="h-screen w-screen overflow-hidden">
    <RouterView v-slot="{ Component, route }">
      <component :is="Component" :key="route.fullPath" />
    </RouterView>
  </div>
  <div v-else class="flex flex-col h-screen overflow-hidden">
    <AppHeader />
    <NavBar />
    <main class="flex-1 min-h-0 max-w-7xl w-full mx-auto px-4 sm:px-6 lg:px-8 py-6 overflow-y-auto">
      <RouterView v-slot="{ Component, route }">
        <component :is="Component" :key="route.fullPath" class="page-fade" />
      </RouterView>
    </main>
  </div>
</template>
