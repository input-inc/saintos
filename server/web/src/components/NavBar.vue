<script setup>
// Desktop inline nav — the horizontal link bar shown at lg+ where all
// nine destinations fit on one row. Below lg the links live in
// NavDrawer.vue instead (opened from AppHeader's hamburger).
import { NAV_LINKS } from '@/composables/useNav'
</script>

<template>
  <nav class="hidden lg:block bg-panel/50 border-b border-line/50">
    <div class="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
      <div class="flex items-center gap-1 py-2">
        <RouterLink
          v-for="link in NAV_LINKS"
          :key="link.to"
          :to="link.to"
          class="nav-link"
        >
          <span class="material-icons icon-sm">{{ link.icon }}</span>
          {{ link.label }}
        </RouterLink>
      </div>
    </div>
  </nav>
</template>
