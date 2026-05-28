<script setup>
import { computed, onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useWsTopic } from '@/composables/useWsTopic'
import RoboClawMonitor from '@/components/widgets/RoboClawMonitor.vue'

defineProps({ embedded: { type: Boolean, default: false } })

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const routing = ref({ sheets: {} })
const widgetCatalog = ref([])

const systemRouting = useWsTopic(() => 'system_routing')

async function load () {
  try {
    const r = await ws.management('get_system_routing', {})
    routing.value = r || { sheets: {} }
  } catch (e) {
    console.warn('widgets load failed:', e)
  }
}
onMounted(() => { catalog.ensureLoaded().then(() => widgetCatalog.value = catalog.widgetTypes); load() })

// Keep in sync with server broadcasts.
const live = computed(() => systemRouting.value || routing.value)

// Per-sheet routing model: flatten widgets and wires across all sheets so
// the Widgets page lists every widget regardless of which controller-node
// sheet (or _dashboard sheet) owns it.
const widgets = computed(() => {
  const all = []
  for (const s of Object.values(live.value.sheets || {})) {
    for (const w of (s.widgets || [])) all.push(w)
  }
  return all
})
const routes = computed(() => {
  const all = []
  for (const s of Object.values(live.value.sheets || {})) {
    for (const w of (s.wires || [])) all.push(w)
  }
  return all
})

function widgetType (id) { return widgetCatalog.value.find(t => t.id === id) }
function routesIntoWidget (widgetId, inputId) {
  return routes.value.filter(r =>
    r.sink?.kind === 'widget' && r.sink.parts?.[0] === widgetId && r.sink.parts?.[1] === inputId
  )
}
function sourceLabel (route) {
  const s = route.source || {}
  if (s.kind === 'peripheral') return `${s.parts[0]}/${s.parts[1]}/${s.parts[2]}`
  if (s.kind === 'signal') return s.parts.join('/')
  return s.kind + ':' + (s.parts || []).join('/')
}
</script>

<template>
  <section>
    <div v-if="!embedded" class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-white">Widgets</h2>
      <RouterLink to="/routes" class="btn-secondary">
        <span class="material-icons icon-sm">bolt</span>
        Manage routes
      </RouterLink>
    </div>

    <div v-if="!widgets.length" class="card text-center py-10">
      <span class="material-icons icon-lg text-slate-600">dashboard</span>
      <p class="text-slate-400 text-sm mt-3">No widgets configured yet.</p>
      <p class="text-slate-500 text-xs mt-1">Add widgets and connect them to peripherals from the Routes page.</p>
    </div>

    <div v-else class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
      <template v-for="w in widgets" :key="w.id">
        <!-- Type-specific renderers come first so they can opt out of
             the generic card layout. The dispatcher is a simple switch
             on widget.type — small enough to keep here, will grow into
             a Map<typeId, Component> if we sprout more bespoke types. -->
        <RoboClawMonitor
          v-if="w.type === 'roboclaw_monitor'"
          :widget="w"
          :routes="routes"
        />
        <div v-else class="card">
          <header class="flex items-center justify-between mb-3">
            <div>
              <h4 class="text-base font-semibold text-white">{{ w.label || w.id }}</h4>
              <p class="text-xs text-slate-500">{{ widgetType(w.type)?.label || w.type }}</p>
            </div>
            <span class="material-icons text-slate-500">{{ widgetType(w.type)?.icon || 'widgets' }}</span>
          </header>

          <div v-if="!(widgetType(w.type)?.inputs?.length)" class="text-xs text-slate-500 italic">
            No declared inputs.
          </div>
          <div v-else class="space-y-1">
            <div
              v-for="inp in widgetType(w.type).inputs"
              :key="inp.id"
              class="flex items-center justify-between text-sm border-b border-slate-700/40 py-1 font-mono"
            >
              <span class="text-slate-400">{{ inp.display || inp.id }}</span>
              <span v-if="routesIntoWidget(w.id, inp.id).length" class="text-cyan-300 text-xs">
                ← {{ sourceLabel(routesIntoWidget(w.id, inp.id)[0]) }}
              </span>
              <span v-else class="text-slate-500 text-xs">unconnected</span>
            </div>
          </div>
        </div>
      </template>
    </div>
  </section>
</template>
