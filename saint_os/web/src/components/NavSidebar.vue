<script setup>
const links = [
  { to: '/dashboard', label: 'Dashboard' },
  { to: '/nodes',     label: 'Nodes' },
  { to: '/routes',    label: 'Routes' },
  { to: '/widgets',   label: 'Widgets' },
  { to: '/inputs',    label: 'Inputs' },
  { to: '/livelink',  label: 'LiveLink' },
  { to: '/control',   label: 'Control' },
  { to: '/moods',     label: 'Moods' },
  { to: '/logs',      label: 'Logs' },
  { to: '/terminal',  label: 'Terminal' },
  { to: '/updates',   label: 'Updates' },
  { to: '/settings',  label: 'Settings' },
]
</script>

<template>
  <aside class="w-56 border-r border-slate-800 bg-slate-900/60 flex flex-col">
    <div class="px-4 py-4 border-b border-slate-800">
      <span class="text-xs uppercase tracking-widest text-slate-500">Operator</span>
    </div>
    <nav class="flex-1 px-2 py-3 space-y-0.5">
      <RouterLink
        v-for="link in links"
        :key="link.to"
        :to="link.to"
        active-class="bg-slate-800 text-white"
        class="block px-3 py-1.5 rounded-md text-sm text-slate-300 hover:bg-slate-800/60 hover:text-white"
      >
        {{ link.label }}
      </RouterLink>
    </nav>
  </aside>
</template>
