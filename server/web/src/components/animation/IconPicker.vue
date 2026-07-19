<script setup>
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'

// Compact icon picker. Shows the currently-selected material-icon as a
// clickable swatch; clicking opens a dropdown with a search box at the
// TOP, a paginated grid of curated icons in the MIDDLE, and pagination
// controls at the BOTTOM. The search doubles as a free-text "use any
// material-icon name" input for power users (Enter picks the first match,
// or the raw typed name when nothing matches).

const props = defineProps({
  modelValue: { type: String, default: '' },     // current icon name
  fallback:   { type: String, default: 'animation' },
})
const emit = defineEmits(['update:modelValue'])

// Curated browseable set — big enough that paging beats typing. Grouped
// loosely by theme (playback, people/gesture, emotion, robot/tech, power,
// symbols, nature, audio, vision/nav, time, comms). All are names from
// the material-icons set (https://fonts.google.com/icons); anything not
// here can still be typed into the search box.
const CURATED = [
  // Playback / animation
  'animation', 'motion_photos_on', 'play_circle', 'play_arrow', 'pause_circle',
  'replay', 'stop_circle', 'fast_forward', 'fast_rewind', 'skip_next',
  'skip_previous', 'loop', 'repeat', 'repeat_one', 'shuffle',
  'slow_motion_video', 'movie', 'videocam', 'theaters', 'smart_display',
  // People / gesture
  'accessibility', 'accessibility_new', 'self_improvement', 'directions_walk',
  'directions_run', 'sports_kabaddi', 'sports_martial_arts', 'sports_gymnastics',
  'waving_hand', 'pan_tool', 'back_hand', 'front_hand', 'thumb_up',
  'thumb_down', 'handshake', 'sign_language', 'emoji_people', 'groups',
  'person', 'elderly',
  // Emotion / mood
  'mood', 'mood_bad', 'sentiment_satisfied', 'sentiment_neutral',
  'sentiment_dissatisfied', 'sentiment_very_satisfied', 'sentiment_very_dissatisfied',
  'emoji_emotions', 'celebration', 'cake', 'favorite', 'favorite_border',
  'heart_broken', 'psychology', 'sick', 'healing',
  // Robot / tech
  'precision_manufacturing', 'smart_toy', 'adb', 'memory', 'developer_board',
  'sensors', 'cable', 'settings', 'build', 'construction',
  'engineering', 'tune', 'science', 'biotech', 'hub',
  'router', 'cell_tower', 'terminal', 'code', 'dns',
  // Power / signals
  'bolt', 'flash_on', 'power', 'battery_full', 'electrical_services',
  'wifi', 'bluetooth', 'settings_remote', 'cast', 'sensors_off',
  // Symbols / misc
  'flag', 'star', 'star_border', 'military_tech', 'verified',
  'workspace_premium', 'emoji_events', 'diamond', 'key', 'lock',
  'lock_open', 'bug_report', 'extension', 'rocket_launch', 'auto_awesome',
  'whatshot', 'local_fire_department', 'priority_high', 'help', 'info',
  // Nature / animals
  'pets', 'cruelty_free', 'nights_stay', 'wb_sunny', 'dark_mode',
  'light_mode', 'ac_unit', 'water_drop', 'eco', 'park', 'egg',
  // Audio
  'music_note', 'queue_music', 'library_music', 'record_voice_over', 'mic',
  'mic_off', 'volume_up', 'volume_off', 'volume_down', 'hearing',
  'graphic_eq', 'equalizer', 'campaign', 'speaker', 'surround_sound',
  // Vision / navigation
  'visibility', 'visibility_off', 'remove_red_eye', 'camera', 'photo_camera',
  'explore', 'navigation', 'my_location', 'place', 'map',
  'near_me', '360', 'view_in_ar', 'threed_rotation',
  // Time / status
  'schedule', 'timer', 'alarm', 'hourglass_empty', 'event',
  'update', 'sync', 'autorenew', 'cached', 'pending',
  // Comms
  'chat', 'forum', 'sms', 'message', 'notifications', 'notifications_active',
]

const PAGE_SIZE = 35   // 7 columns × 5 rows

const open = ref(false)
const query = ref('')
const page = ref(0)

const filtered = computed(() => {
  const q = query.value.trim().toLowerCase()
  if (!q) return CURATED
  return CURATED.filter(n => n.includes(q))
})
const pageCount = computed(() =>
  Math.max(1, Math.ceil(filtered.value.length / PAGE_SIZE)))
const pageIcons = computed(() =>
  filtered.value.slice(page.value * PAGE_SIZE, page.value * PAGE_SIZE + PAGE_SIZE))
// Show the "use exactly what I typed" affordance when the query looks
// like an icon name that isn't already an exact curated entry.
const customName = computed(() => query.value.trim())
const showUseCustom = computed(() =>
  customName.value.length > 0 && !CURATED.includes(customName.value))

// Reset to the first page whenever the filter changes so results are
// visible immediately.
watch(query, () => { page.value = 0 })
// Keep the page in range if the filtered set shrinks.
watch(pageCount, (n) => { if (page.value > n - 1) page.value = n - 1 })

function pick (name) {
  emit('update:modelValue', name)
  open.value = false
  query.value = ''
  page.value = 0
}
// Enter in the search box: pick the first visible match, else use the
// raw typed name (custom icon).
function onSearchEnter () {
  if (filtered.value.length) pick(filtered.value[0])
  else if (customName.value) pick(customName.value)
}
function prevPage () { if (page.value > 0) page.value-- }
function nextPage () { if (page.value < pageCount.value - 1) page.value++ }

function toggle () {
  open.value = !open.value
  if (open.value) { query.value = ''; page.value = 0 }
}
function onDocClick (e) {
  if (!open.value) return
  if (!e.target.closest('.icon-picker')) open.value = false
}
onMounted(() => document.addEventListener('click', onDocClick))
onBeforeUnmount(() => document.removeEventListener('click', onDocClick))
</script>

<template>
  <div class="icon-picker relative">
    <button class="icon-swatch" type="button"
            :title="`Change icon — current: ${modelValue || fallback}`"
            @click.stop="toggle">
      <span class="material-icons">{{ modelValue || fallback }}</span>
      <span class="material-icons icon-sm chev">expand_more</span>
    </button>
    <div v-if="open" class="icon-menu" @click.stop>
      <!-- Search (top) -->
      <div class="flex gap-1 mb-2">
        <input v-model="query"
               class="input-field flex-1 text-xs py-1"
               placeholder="Search icons… (Enter to use name)"
               autofocus
               @keydown.enter="onSearchEnter" />
        <button v-if="showUseCustom"
                class="btn-sm bg-cyan-600 hover:bg-cyan-500 text-white whitespace-nowrap"
                type="button"
                :title="`Use “${customName}” as a custom icon`"
                @click="pick(customName)">Use</button>
      </div>

      <!-- Grid (middle) -->
      <div v-if="pageIcons.length" class="icon-grid">
        <button v-for="name in pageIcons" :key="name"
                :class="['icon-cell', name === modelValue ? 'is-active' : '']"
                :title="name"
                type="button"
                @click="pick(name)">
          <span class="material-icons">{{ name }}</span>
        </button>
      </div>
      <div v-else class="text-xs text-fg-faint italic py-6 text-center">
        No curated match. Press <span class="text-cyan-300">Enter</span> or
        <span class="text-cyan-300">Use</span> to apply the typed name.
      </div>

      <!-- Pagination (bottom) -->
      <div v-if="pageCount > 1"
           class="flex items-center justify-between border-t border-line/60 mt-2 pt-2">
        <button class="page-btn" type="button" :disabled="page === 0" @click="prevPage">
          <span class="material-icons icon-sm">chevron_left</span>
        </button>
        <span class="text-[11px] text-fg-faint tabular-nums">
          Page {{ page + 1 }} / {{ pageCount }}
        </span>
        <button class="page-btn" type="button" :disabled="page >= pageCount - 1" @click="nextPage">
          <span class="material-icons icon-sm">chevron_right</span>
        </button>
      </div>
    </div>
  </div>
</template>

<style scoped>
.icon-swatch {
  display: inline-flex; align-items: center; gap: 4px;
  padding: 4px 6px;
  background: var(--color-canvas);
  border: 1px solid var(--color-surface);
  border-radius: 6px;
  color: var(--color-fg);
  cursor: pointer;
  transition: border-color 0.1s;
  flex-shrink: 0;
}
.icon-swatch:hover { border-color: #0e7490; }
.icon-swatch .material-icons { font-size: 20px; }
.icon-swatch .chev { font-size: 14px; color: var(--color-fg-faint); }

.icon-menu {
  position: absolute;
  top: 100%;
  left: 0;
  margin-top: 4px;
  width: 280px;
  background: var(--color-canvas);
  border: 1px solid var(--color-surface);
  border-radius: 8px;
  box-shadow: 0 18px 50px -12px rgba(0,0,0,0.7);
  padding: 8px;
  z-index: 50;
}
.icon-grid {
  display: grid;
  grid-template-columns: repeat(7, 1fr);
  gap: 2px;
}
.icon-cell {
  display: inline-flex; align-items: center; justify-content: center;
  width: 100%; aspect-ratio: 1;
  border-radius: 4px;
  color: var(--color-fg-muted);
  cursor: pointer;
  transition: background-color 0.1s, color 0.1s;
}
.icon-cell:hover { background: var(--color-surface); color: var(--color-fg-strong); }
.icon-cell.is-active {
  background: rgba(6, 182, 212, 0.2);
  color: var(--color-cyan-300);
}
.icon-cell .material-icons { font-size: 18px; }

.page-btn {
  display: inline-flex; align-items: center; justify-content: center;
  height: 22px; width: 28px;
  border-radius: 4px;
  color: var(--color-fg-muted);
  cursor: pointer;
  transition: background-color 0.1s, color 0.1s;
}
.page-btn:hover:not(:disabled) { background: var(--color-surface); color: var(--color-fg-strong); }
.page-btn:disabled { opacity: 0.3; cursor: default; }
</style>
