<script setup>
import NavSidebar from '@/components/NavSidebar.vue'
import ConnectionBadge from '@/components/ConnectionBadge.vue'
</script>

<template>
  <div class="flex min-h-screen">
    <NavSidebar />
    <main class="flex-1 min-w-0 flex flex-col">
      <header class="flex items-center justify-between px-6 py-3 border-b border-slate-800 bg-slate-900/40">
        <h1 class="text-base font-semibold tracking-tight">SAINT.OS</h1>
        <ConnectionBadge />
      </header>
      <section class="flex-1 px-6 py-5 overflow-auto">
        <RouterView />
      </section>
    </main>
  </div>
</template>
