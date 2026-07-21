<!--
    Preset panel overlay. Shown when a digital binding fires a
    show_panel action; displays a grid of presets the operator can
    navigate with d-pad / sticks and activate with A. The active
    panel + selection comes from useBindings; this component just
    renders + dispatches click→activatePreset.
-->
<script setup lang="ts">
import { computed, ref, watch, onMounted, onBeforeUnmount } from 'vue';
import { useBindings, type PanelItem } from '../composables/useBindings';
import { useConnection } from '../composables/useConnection';
import { useLibrary } from '../composables/useLibrary';
import { useDisplayPrefs, type IconLayout } from '../composables/useDisplayPrefs';

const bindings = useBindings();
const conn = useConnection();
const library = useLibrary();
const displayPrefs = useDisplayPrefs();

// Per-panel display options (icon layout + column count), set from the
// footer's display-options modal.
const prefs = computed(() => displayPrefs.prefsFor(panel.value?.id ?? ''));
const showDisplayModal = ref(false);
function setLayout(layout: IconLayout): void {
    if (panel.value) displayPrefs.setPrefs(panel.value.id, { layout });
}
function setColumns(columns: number): void {
    if (panel.value) displayPrefs.setPrefs(panel.value.id, { columns });
}

// The panel's title/icon, group filter, and page indicator now live in
// the app's main header (see App.vue) — this component renders just the
// grid + footer hints.

// For server-backed panels (Animations/Poses/Sounds), explain WHY the
// grid is empty instead of the generic "no presets": not connected vs
// still loading vs genuinely empty on the server.
const emptyState = computed(() => {
    const source = bindings.activePanelSource.value;
    if (!source) return { icon: 'folder_open', text: 'No presets in this panel' };
    const kind = source;   // 'animations' | 'poses' | 'sounds'
    if (!conn.isConnected.value) {
        return { icon: 'cloud_off', text: `Connect to the robot to load ${kind}` };
    }
    const loaded = source === 'animations'
        ? library.animationsLoaded.value
        : (source === 'poses'
            ? library.posesLoaded.value
            : library.soundsLoaded.value);
    if (!loaded) return { icon: 'sync', text: `Loading ${kind}…` };
    return { icon: 'folder_open', text: `No ${kind} saved on the server` };
});

const panel = computed(() => bindings.activePanel.value);
const panelState = computed(() => bindings.activePanelState.value);
const currentPage = computed(() => panelState.value.currentPage);
const selectedIndex = computed(() => panelState.value.selectedIndex);

// Items come from useBindings: server-backed panels (Animations/Poses)
// stream live from the server library; static panels use their presets.
const items = computed<PanelItem[]>(() => bindings.activePanelItems.value);

// Items per page is measured (rows that fit the grid at the current UI
// scale) and shared via useBindings so navigation math agrees. Falls back
// to the panel default before the first measurement.
const itemsPerPage = computed(() =>
    bindings.activePanelItemsPerPage.value || panel.value?.itemsPerPage || 8);

const totalPages = computed(() => {
    if (!panel.value) return 0;
    return Math.ceil(items.value.length / itemsPerPage.value);
});

const visiblePresets = computed(() => {
    if (!panel.value) return [];
    const start = currentPage.value * itemsPerPage.value;
    return items.value.slice(start, start + itemsPerPage.value);
});

function isSelected(pageIndex: number): boolean {
    if (!panel.value) return false;
    const globalIndex = currentPage.value * itemsPerPage.value + pageIndex;
    return globalIndex === selectedIndex.value;
}

// Animation playback state (only meaningful on the animations panel).
// `isPlaying` drives the "playing" highlight/badge; `isLooping` picks the
// loop badge so the operator knows tapping again will stop it.
function isPlaying(itemId: string): boolean {
    return bindings.activePanelSource.value === 'animations'
        && !!library.playing.value[itemId];
}

function isLooping(itemId: string): boolean {
    return isPlaying(itemId) && !!library.playing.value[itemId]?.loop;
}

function selectPreset(item: PanelItem): void {
    bindings.triggerActiveItem(item.id);
    // Sticky panels (keep_open) stay up so several presets can be fired
    // in a row; otherwise selecting dismisses the panel.
    if (!bindings.activePanelState.value.keepOpen) bindings.hidePanel();
}

function prevPage(): void { bindings.navigatePanel('prev_page'); }
function nextPage(): void { bindings.navigatePanel('next_page'); }

// ─── Fill-the-grid: measure how many rows fit and page by that ───────
// The grid is a fixed 4 columns (see template); we measure the visible
// content height against a rendered button to derive rows, so the page
// holds as many rows as fit at the current UI (webview) zoom. Recomputed
// on resize/zoom and when items first arrive.
const contentEl = ref<HTMLElement | null>(null);
let resizeObs: ResizeObserver | null = null;

function recomputeRows(): void {
    const el = contentEl.value;
    if (!el) return;
    const btn = el.querySelector('.preset-item') as HTMLElement | null;
    if (!btn) return; // nothing rendered to measure (empty list) — keep default
    const cs = getComputedStyle(el);
    const usable = el.clientHeight
        - (parseFloat(cs.paddingTop) || 0)
        - (parseFloat(cs.paddingBottom) || 0);
    const grid = btn.parentElement;
    const gap = grid ? (parseFloat(getComputedStyle(grid).rowGap) || 0) : 0;
    const rowH = btn.offsetHeight;
    if (rowH <= 0 || usable <= 0) return;
    const rows = Math.max(1, Math.floor((usable + gap) / (rowH + gap)));
    bindings.setPanelItemsPerPage(prefs.value.columns * rows);
}

function scheduleRecompute(): void {
    requestAnimationFrame(() => recomputeRows());
}

onMounted(() => {
    scheduleRecompute();
    if (contentEl.value && 'ResizeObserver' in window) {
        resizeObs = new ResizeObserver(() => scheduleRecompute());
        resizeObs.observe(contentEl.value);
    }
});

onBeforeUnmount(() => {
    resizeObs?.disconnect();
    resizeObs = null;
});

// The panel can open before the library list arrives (empty grid → no
// button to measure). Re-measure once items show up.
watch(() => items.value.length, (n, old) => {
    if (n > 0 && old === 0) scheduleRecompute();
});

// Layout/column changes alter button height and columns → re-measure.
watch(() => [prefs.value.layout, prefs.value.columns], () => scheduleRecompute());
</script>

<template>
    <div v-if="panel" class="h-full flex flex-col"
         :style="{ '--panel-color': panel.color }">

        <!-- Grid Content (title/group live in the app header now) -->
        <div ref="contentEl" class="panel-content flex-1 bg-saint-background p-6 overflow-auto">
            <div class="grid gap-3"
                 :style="{ gridTemplateColumns: `repeat(${prefs.columns}, minmax(0, 1fr))` }">
                <button v-for="(preset, i) in visiblePresets" :key="preset.id"
                        class="preset-item group relative rounded-xl transition-all duration-150"
                        :class="[
                            prefs.layout === 'vertical'
                                ? 'flex flex-col items-center justify-center p-4 text-center'
                                : 'flex items-center gap-3 px-4 py-3 text-left',
                            { 'preset-selected': isSelected(i), 'preset-playing': isPlaying(preset.id) },
                        ]"
                        :style="{ '--preset-color': preset.color || panel.color }"
                        @click="selectPreset(preset)">
                    <span class="material-symbols-outlined shrink-0 transition-transform group-hover:scale-110"
                          :class="prefs.layout === 'vertical' ? 'text-4xl mb-2' : 'text-2xl'"
                          :style="{ color: preset.color || panel.color }">
                        {{ preset.icon || 'radio_button_unchecked' }}
                    </span>
                    <span class="text-base font-medium text-saint-text truncate"
                          :class="prefs.layout === 'vertical' ? 'w-full' : 'flex-1 min-w-0'">
                        {{ preset.name }}
                    </span>
                    <!-- Playing badge: pulsing equalizer for a playing
                         animation; loopers add a loop glyph ("tap to stop"). -->
                    <span v-if="isPlaying(preset.id)"
                          class="flex items-center gap-1 text-saint-success"
                          :class="prefs.layout === 'vertical' ? 'absolute top-2 right-2' : 'shrink-0'">
                        <span class="material-symbols-outlined text-lg animate-pulse">graphic_eq</span>
                        <span v-if="isLooping(preset.id)" class="material-symbols-outlined text-sm">loop</span>
                    </span>
                    <div v-if="isSelected(i)" class="absolute inset-0 rounded-xl border-2 pointer-events-none"
                         :style="{ borderColor: preset.color || panel.color }"></div>
                </button>
            </div>

            <div v-if="visiblePresets.length === 0" class="text-center py-12 text-saint-text-muted">
                <span class="material-symbols-outlined text-4xl mb-2"
                      :class="{ 'animate-spin': emptyState.icon === 'sync' }">{{ emptyState.icon }}</span>
                <p>{{ emptyState.text }}</p>
            </div>
        </div>

        <!-- Footer with controls hint -->
        <div class="panel-footer px-6 py-3 bg-saint-surface border-t border-saint-surface-light
                    flex items-center justify-between text-sm text-saint-text-muted">
            <div class="flex items-center gap-4">
                <span class="flex items-center gap-1">
                    <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-saint-surface-light text-xs font-bold">D</span>
                    Navigate
                </span>
                <span class="flex items-center gap-1">
                    <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-green-600 text-white text-xs font-bold">A</span>
                    Select
                </span>
                <span class="flex items-center gap-1">
                    <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-red-500 text-white text-xs font-bold">B</span>
                    Close
                </span>
                <span v-if="totalPages > 1" class="flex items-center gap-1 ml-2 pl-2 border-l border-saint-surface-light">
                    <span class="inline-flex items-center justify-center px-2 h-6 rounded bg-saint-surface-light text-xs font-bold">LB</span>
                    <span class="inline-flex items-center justify-center px-2 h-6 rounded bg-saint-surface-light text-xs font-bold">RB</span>
                    Page
                </span>
            </div>
            <div class="flex items-center gap-2">
                <template v-if="totalPages > 1">
                    <button class="p-1.5 rounded hover:bg-saint-surface-light transition-colors"
                            :disabled="currentPage === 0"
                            :class="{ 'opacity-50': currentPage === 0 }"
                            @click="prevPage">
                        <span class="material-symbols-outlined text-lg">chevron_left</span>
                    </button>
                    <span class="text-saint-text">{{ currentPage + 1 }} / {{ totalPages }}</span>
                    <button class="p-1.5 rounded hover:bg-saint-surface-light transition-colors"
                            :disabled="currentPage >= totalPages - 1"
                            :class="{ 'opacity-50': currentPage >= totalPages - 1 }"
                            @click="nextPage">
                        <span class="material-symbols-outlined text-lg">chevron_right</span>
                    </button>
                </template>
                <!-- Display options: layout + column count -->
                <button class="p-1.5 rounded hover:bg-saint-surface-light transition-colors ml-1"
                        title="Display options"
                        @click="showDisplayModal = true">
                    <span class="material-symbols-outlined text-lg">grid_view</span>
                </button>
            </div>
        </div>

        <!-- Display-options modal -->
        <div v-if="showDisplayModal"
             class="fixed inset-0 z-50 flex items-center justify-center bg-black/50"
             @click.self="showDisplayModal = false">
            <div class="w-80 bg-saint-surface border border-saint-surface-light rounded-xl p-5 space-y-5">
                <div class="flex items-center justify-between">
                    <h3 class="text-lg font-semibold">Display options</h3>
                    <button class="p-1 rounded hover:bg-saint-surface-light transition-colors"
                            @click="showDisplayModal = false">
                        <span class="material-symbols-outlined">close</span>
                    </button>
                </div>

                <!-- Icon view -->
                <div>
                    <div class="text-sm text-saint-text-muted mb-2">Icon view</div>
                    <div class="grid grid-cols-2 gap-2">
                        <button class="px-3 py-2 rounded-lg text-sm flex items-center justify-center gap-2 transition-colors"
                                :class="prefs.layout === 'horizontal' ? 'bg-saint-primary text-white' : 'bg-saint-surface-light hover:bg-saint-surface text-saint-text'"
                                @click="setLayout('horizontal')">
                            <span class="material-symbols-outlined text-lg">view_list</span> Horizontal
                        </button>
                        <button class="px-3 py-2 rounded-lg text-sm flex items-center justify-center gap-2 transition-colors"
                                :class="prefs.layout === 'vertical' ? 'bg-saint-primary text-white' : 'bg-saint-surface-light hover:bg-saint-surface text-saint-text'"
                                @click="setLayout('vertical')">
                            <span class="material-symbols-outlined text-lg">grid_view</span> Vertical
                        </button>
                    </div>
                </div>

                <!-- Columns -->
                <div>
                    <div class="text-sm text-saint-text-muted mb-2">Columns</div>
                    <div class="grid grid-cols-3 gap-2">
                        <button v-for="n in [2, 3, 4]" :key="n"
                                class="px-3 py-2 rounded-lg text-sm font-mono transition-colors"
                                :class="prefs.columns === n ? 'bg-saint-primary text-white' : 'bg-saint-surface-light hover:bg-saint-surface text-saint-text'"
                                @click="setColumns(n)">
                            {{ n }}
                        </button>
                    </div>
                </div>
            </div>
        </div>
    </div>
</template>

<style scoped>
.preset-item {
    background: var(--saint-surface, #1e293b);
    border: 1px solid var(--saint-surface-light, #334155);
}
.preset-item:hover {
    background: var(--saint-surface-light, #334155);
    border-color: var(--preset-color);
}
.preset-selected {
    background: color-mix(in srgb, var(--preset-color) 15%, var(--saint-surface, #1e293b));
}
.preset-playing {
    background: color-mix(in srgb, #22c55e 12%, var(--saint-surface, #1e293b));
    box-shadow: inset 0 0 0 2px color-mix(in srgb, #22c55e 55%, transparent);
}
</style>
