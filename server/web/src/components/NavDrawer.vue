<script setup>
// Mobile / tablet-portrait nav — a slide-out drawer opened from the
// AppHeader hamburger (< lg). Vertical link list with generous tap
// targets. Closes on backdrop tap, link tap, route change, or Escape.
import { watch, onMounted, onUnmounted } from 'vue'
import { useRoute } from 'vue-router'
import { NAV_LINKS, useNavDrawer } from '@/composables/useNav'

const { drawerOpen, closeDrawer } = useNavDrawer()
const route = useRoute()

// Close whenever the route changes (e.g. a link tap navigates).
watch(() => route.fullPath, () => closeDrawer())

function onKey (e) { if (e.key === 'Escape') closeDrawer() }
onMounted(() => document.addEventListener('keydown', onKey))
onUnmounted(() => document.removeEventListener('keydown', onKey))
</script>

<template>
  <Teleport to="body">
    <Transition name="drawer-fade">
      <div
        v-if="drawerOpen"
        class="lg:hidden fixed inset-0 z-[60] bg-black/50 backdrop-blur-sm"
        @click="closeDrawer"
      />
    </Transition>
    <Transition name="drawer-slide">
      <aside
        v-if="drawerOpen"
        class="lg:hidden fixed inset-y-0 left-0 z-[70] w-72 max-w-[80vw]
               bg-panel border-r border-line flex flex-col shadow-2xl"
      >
        <div class="flex items-center justify-between h-16 px-4 border-b border-line/50 shrink-0">
          <div class="flex items-center gap-2">
            <div class="w-8 h-8 rounded-lg bg-gradient-to-br from-cyan-500 to-blue-600 flex items-center justify-center">
              <span class="material-icons text-fg-strong icon-sm">computer</span>
            </div>
            <span class="text-lg font-bold text-fg-strong">SAINT.OS</span>
          </div>
          <button
            class="p-2 -mr-2 rounded-lg text-fg-muted hover:text-fg-strong hover:bg-surface transition-colors"
            title="Close menu"
            @click="closeDrawer"
          >
            <span class="material-icons">close</span>
          </button>
        </div>
        <nav class="flex-1 overflow-y-auto p-3 flex flex-col gap-1">
          <RouterLink
            v-for="link in NAV_LINKS"
            :key="link.to"
            :to="link.to"
            class="nav-link !py-3 !text-base"
          >
            <span class="material-icons">{{ link.icon }}</span>
            {{ link.label }}
          </RouterLink>
        </nav>
      </aside>
    </Transition>
  </Teleport>
</template>

<style scoped>
.drawer-fade-enter-active,
.drawer-fade-leave-active { transition: opacity 0.2s ease; }
.drawer-fade-enter-from,
.drawer-fade-leave-to { opacity: 0; }

.drawer-slide-enter-active,
.drawer-slide-leave-active { transition: transform 0.25s ease; }
.drawer-slide-enter-from,
.drawer-slide-leave-to { transform: translateX(-100%); }
</style>
