<script setup>
import { computed, onMounted } from 'vue'
import { useNodesStore } from '@/stores/nodes'
import { useDisplayStore } from '@/stores/display'

const props = defineProps({ id: { type: String, required: true } })
const nodes = useNodesStore()
const display = useDisplayStore()
const node = nodes.byId(props.id)

onMounted(() => nodes.fetchAll().catch(() => {}))

// Control's actions moved onto Overview — a whole tab for six buttons
// wasn't paying for itself, and the destructive ones were a tab away
// from the identity they act on. The route is kept as a redirect so
// existing bookmarks and links don't 404; see router.js.
const tabs = [
  { name: 'node-overview',    label: 'Overview' },
  { name: 'node-peripherals', label: 'Peripherals' },
  { name: 'node-live',        label: 'Live readings' },
  { name: 'node-state',       label: 'State' },
  { name: 'node-logs',        label: 'Logs' },
]

const title = computed(() => node.value?.display_name || props.id)
const role  = computed(() => node.value?.role || 'unassigned')
const online = computed(() => node.value?.online !== false)

function formatLastSeen (ts) {
  if (!ts) return '—'
  try {
    const secs = Math.max(0, Date.now() / 1000 - ts)
    if (secs < 60) return `${Math.round(secs)}s ago`
    if (secs < 3600) return `${Math.round(secs / 60)}m ago`
    if (secs < 86400) return `${Math.round(secs / 3600)}h ago`
    return new Date(ts * 1000).toLocaleDateString()
  } catch (_) { return '—' }
}

// Header status readouts. Deliberately the few things worth knowing
// regardless of which tab you're on — anything more belongs in
// Overview's detail cards rather than following you around.
const stats = computed(() => [
  { label: 'State', value: node.value?.state || 'Unknown' },
  { label: 'CPU', value: display.formatTemperature(node.value?.cpu_temp) },
  { label: 'Last seen', value: formatLastSeen(node.value?.last_seen) },
])
</script>

<template>
  <section>
    <!-- Breadcrumb replaces the old back button: it says where you are,
         not just where you can go, and the trail is the way back.
         Structure follows Tailwind's canonical chevron-breadcrumb
         pattern (nav > ol > li, aria-current on the leaf) — no component
         library is installed, so this is the markup convention rather
         than an import. Colors use the project's design tokens instead
         of the pattern's hardcoded grays, and the separator uses
         material-icons to match the rest of the app. -->
    <nav class="flex mb-3" aria-label="Breadcrumb">
      <ol role="list" class="flex items-center gap-2 min-w-0">
        <li class="flex items-center">
          <RouterLink
            :to="{ name: 'nodes' }"
            class="text-sm font-medium text-fg-muted hover:text-fg-strong transition-colors"
          >
            All Nodes
          </RouterLink>
        </li>
        <li class="flex items-center min-w-0">
          <span class="material-icons icon-sm text-fg-faint" aria-hidden="true">chevron_right</span>
          <span class="ml-2 text-sm font-medium text-fg-strong truncate" aria-current="page">
            {{ title }}
          </span>
        </li>
      </ol>
    </nav>

    <!-- Title left, live status right. Wraps to its own line on narrow
         screens rather than crushing the title. -->
    <div class="flex flex-wrap items-center justify-between gap-x-6 gap-y-3 mb-4">
      <div class="flex items-center gap-3 min-w-0">
        <span :class="['w-3 h-3 rounded-full flex-shrink-0',
                       online ? 'bg-emerald-500 animate-pulse-dot' : 'bg-slate-500']" />
        <div class="min-w-0">
          <h2 class="text-2xl font-bold text-fg-strong leading-tight truncate">{{ title }}</h2>
          <p class="text-xs text-fg-muted font-mono truncate">{{ role }} · {{ id }}</p>
        </div>
      </div>

      <div class="flex items-center gap-5 sm:gap-6">
        <div v-for="s in stats" :key="s.label" class="text-right">
          <div class="text-[10px] uppercase tracking-wide text-fg-faint leading-none mb-1">
            {{ s.label }}
          </div>
          <div class="text-sm font-semibold text-fg-strong tabular-nums leading-none">
            {{ s.value }}
          </div>
        </div>
      </div>
    </div>

    <div class="flex gap-1 border-b border-line/50 mb-6 overflow-x-auto">
      <RouterLink
        v-for="t in tabs"
        :key="t.name"
        :to="{ name: t.name, params: { id } }"
        class="node-tab"
      >
        {{ t.label }}
      </RouterLink>
    </div>

    <RouterView :node-id="id" :node="node" />
  </section>
</template>
