<script setup>
const links = [
  { to: '/dashboard',  icon: 'dashboard',    label: 'Dashboard' },
  { to: '/nodes',      icon: 'dns',          label: 'Nodes' },
  { to: '/routes',     icon: 'bolt',         label: 'Routes' },
  { to: '/control',    icon: 'tune',         label: 'Control' },
  { to: '/inputs',     icon: 'sensors',      label: 'Inputs' },
  { to: '/animations', icon: 'animation',    label: 'Animations & Poses' },
  { to: '/settings',   icon: 'settings',     label: 'Settings' },
  { to: '/logs',       icon: 'description',  label: 'Logs' },
  { to: '/terminal',   icon: 'terminal',     label: 'Terminal' },
]
</script>

<template>
  <nav class="bg-panel/50 border-b border-line/50">
    <div class="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
      <div class="flex items-center gap-1 overflow-x-auto py-2">
        <RouterLink
          v-for="link in links"
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
