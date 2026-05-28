<script setup>
import { computed, watch } from 'vue'
import { useRoute } from 'vue-router'
import AppHeader from '@/components/AppHeader.vue'
import AppFooter from '@/components/AppFooter.vue'
import NavBar from '@/components/NavBar.vue'
import LoginScreen from '@/components/LoginScreen.vue'
import { useWsStore } from '@/stores/ws'

const ws = useWsStore()
const needLogin = computed(() => ws.authRequired && !ws.authenticated)

// Full-bleed pages (Routes, Logs) opt out of <main>'s max-w-7xl + padding
// via body-level classes that style.css keys off — matches vanilla's
// `body.routing-fullwidth` / `body.logs-fullwidth` mechanism.
const route = useRoute()
watch(
  () => route.name,
  (name) => {
    document.body.classList.toggle('routing-fullwidth', name === 'routes')
    document.body.classList.toggle('logs-fullwidth',    name === 'logs')
  },
  { immediate: true },
)
</script>

<template>
  <LoginScreen v-if="needLogin" />
  <div v-else class="flex flex-col min-h-screen">
    <AppHeader />
    <NavBar />
    <main class="flex-1 max-w-7xl w-full mx-auto px-4 sm:px-6 lg:px-8 py-6">
      <RouterView v-slot="{ Component, route }">
        <component :is="Component" :key="route.fullPath" class="page-fade" />
      </RouterView>
    </main>
    <AppFooter />
  </div>
</template>
