/**
 * Bindings composable — Vue equivalent of the Angular
 * `BindingsService`. Manages the operator's binding profile (analog +
 * digital input mappings, preset panels, profile settings) and the
 * currently-active preset panel state.
 *
 * Profiles are persisted on the Rust side via `get_binding_profiles`
 * and `set_binding_profiles`. Panel navigation + preset activation are
 * driven from the frontend button path (App.vue: handleHardwareButton /
 * onVirtualButtonDown → executeDigitalAction), which owns panel state.
 * The Rust mapper's `action-event` stream is intentionally NOT consumed
 * here — doing so double-handled every press (see ensureInit). Commands,
 * analog control, and E-Stop are still executed Rust-side directly.
 *
 * The shape mirrors BindingsService 1:1 so the components mostly
 * translate verbatim — `signal` becomes `ref`, `computed` keeps its
 * name, `service.foo()` becomes `bindings.foo()`. The default profile
 * constant is identical between the two.
 */

import { computed, ref } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { useLibrary } from './useLibrary';
import { useConnection } from './useConnection';
import { useDisplayPrefs } from './useDisplayPrefs';

// ─── Input sources ───────────────────────────────────────────────────

export type AnalogInput =
    | 'left_stick_x'
    | 'left_stick_y'
    | 'right_stick_x'
    | 'right_stick_y'
    | 'left_trigger'
    | 'right_trigger'
    | 'left_pad_x'
    | 'left_pad_y'
    | 'right_pad_x'
    | 'right_pad_y';

export type DigitalInput =
    | 'a' | 'b' | 'x' | 'y'
    | 'lb' | 'rb'
    | 'd_pad_up' | 'd_pad_down' | 'd_pad_left' | 'd_pad_right'
    | 'start' | 'select'
    | 'left_stick' | 'right_stick'
    | 'l4' | 'r4' | 'l5' | 'r5' | 'steam';

export type ButtonTrigger = 'press' | 'release' | 'hold' | 'double_tap' | 'long_press';

export type NavigateDirection =
    | 'up' | 'down' | 'left' | 'right'
    | 'next_item' | 'prev_item' | 'next_page' | 'prev_page';

// ─── Targets + actions ───────────────────────────────────────────────

export type ControlTarget = WsInputTarget | TopicTarget;

export interface WsInputTarget {
    sheet_id: string;
    input_id: string;
    name?: string;
}

export interface TopicTarget {
    topic: string;
    channel: string;
    name?: string;
}

export function isWsInputTarget(t: ControlTarget): t is WsInputTarget {
    return (t as WsInputTarget).sheet_id !== undefined;
}

export function targetDisplayName(t: ControlTarget): string {
    if (t.name) return t.name;
    return isWsInputTarget(t)
        ? `${t.sheet_id}/${t.input_id}`
        : `${(t as TopicTarget).topic}:${(t as TopicTarget).channel}`;
}

export interface InputTransform {
    deadzone: number;
    scale: number;
    expo: number;
    invert: boolean;
}

export type ModifierEffect =
    | { type: 'precision_mode'; min_scale: number }
    | { type: 'speed_boost'; max_boost: number }
    | { type: 'scale_target'; target_id: string; scale: number };

export type AnalogAction =
    | { type: 'direct_control'; target: ControlTarget; transform: InputTransform }
    | {
        type: 'differential_drive';
        topic: string;
        left_channel: string;
        right_channel: string;
        throttle_transform: InputTransform;
        turn_transform: InputTransform;
    }
    | { type: 'modifier'; effect: ModifierEffect };

export type DigitalAction =
    | { type: 'show_panel'; panel_id: string; default_group?: string; keep_open?: boolean }
    | { type: 'hide_panel' }
    | { type: 'activate_preset'; preset_id: string }
    | { type: 'navigate_panel'; direction: NavigateDirection }
    | { type: 'select_panel_item' }
    | { type: 'toggle_output'; target_id: string }
    | { type: 'cycle_output'; target_id: string; values: string[] }
    | { type: 'direct_control'; target: ControlTarget; value: number }
    | { type: 'e_stop' }
    | { type: 'none' };

// ─── Bindings + presets ──────────────────────────────────────────────

export interface AnalogBinding {
    input: AnalogInput;
    action: AnalogAction;
    enabled: boolean;
}

export interface DigitalBinding {
    input: DigitalInput;
    trigger: ButtonTrigger;
    action: DigitalAction;
    enabled: boolean;
}

export type PresetType = 'servo' | 'animation' | 'sound';
export type EasingType = 'linear' | 'ease_in' | 'ease_out' | 'ease_in_out';
export type PanelLayout = 'grid' | 'list';

export interface ServoPosition {
    topic: string;
    channel: string;
    value: number;
}

export interface ServoPresetData {
    positions: ServoPosition[];
    transitionMs: number;
    easing: EasingType;
}

export interface AnimationKeyframe {
    timeMs: number;
    positions: ServoPosition[];
}

export interface AnimationPresetData {
    keyframes: AnimationKeyframe[];
    loop_animation: boolean;
    loop_count?: number;
}

export interface SoundPresetData {
    soundId: string;
    volume: number;
    priority: number;
}

export type PresetData =
    | { type: 'servo' } & ServoPresetData
    | { type: 'animation' } & AnimationPresetData
    | { type: 'sound' } & SoundPresetData;

export interface Preset {
    id: string;
    name: string;
    icon?: string;
    color?: string;
    type: PresetType;
    data: PresetData;
}

export interface PresetPanel {
    id: string;
    name: string;
    icon: string;
    color: string;
    presets: Preset[];
    layout: PanelLayout;
    columns: number;
    itemsPerPage: number;
    /** Live data source. Absent = static (use `presets`).
     *  'animations'/'poses'/'sounds' = populated from the server library
     *  (useLibrary); selecting fires start_animation / apply_pose /
     *  play_sound. */
    source?: 'animations' | 'poses' | 'sounds';
}

/** Minimal shape the panel grid renders. Both Preset and the server
 *  library's items satisfy it. */
export interface PanelItem {
    id: string;
    name: string;
    icon?: string;
    color?: string;
    // Server-library items may carry a group (sounds) used by the panel's
    // group filter. Absent for local presets.
    group?: string;
}

// ─── Profile + settings ──────────────────────────────────────────────

export type PanelActivationMode = 'press' | 'hold';

export interface ProfileSettings {
    globalDeadzone: number;
    hapticFeedback: boolean;
    doubleTapTimeMs: number;
    longPressTimeMs: number;
    panelActivation: PanelActivationMode;
}

export interface BindingProfile {
    id: string;
    name: string;
    description?: string;
    isDefault: boolean;
    analogBindings: AnalogBinding[];
    digitalBindings: DigitalBinding[];
    presetPanels: PresetPanel[];
    settings: ProfileSettings;
}

// ─── Action events (from the mapper) ─────────────────────────────────

export interface MappedCommand {
    topic: string;
    channel: string;
    value: unknown;
}

export type ActionEvent =
    | { type: 'command'; data: MappedCommand }
    | { type: 'show_panel'; panel_id: string; default_group?: string }
    | { type: 'hide_panel' }
    | { type: 'navigate_panel'; direction: NavigateDirection }
    | { type: 'select_panel_item' }
    | { type: 'activate_preset'; preset_id: string }
    | { type: 'toggle_output'; target_id: string }
    | { type: 'cycle_output'; target_id: string; value: string }
    | { type: 'e_stop' };

export interface PanelState {
    activePanelId: string | null;
    selectedIndex: number;
    currentPage: number;
    // Group filter for server-backed panels (currently sounds). 'All'
    // shows everything; otherwise only items whose group matches.
    selectedGroup: string;
    // When true, selecting an item does NOT close the panel — lets the
    // operator fire several presets (e.g. sounds) in a row. Comes from the
    // opening show_panel binding's keep_open flag.
    keepOpen: boolean;
}

// ─── Default profile ─────────────────────────────────────────────────
//
// Identical to BindingsService.getDefaultProfile(). Kept in sync
// during the migration; once Angular is dropped this is the only
// copy.

function getDefaultProfile(): BindingProfile {
    return {
        id: 'default',
        name: 'Default',
        description: 'Standard robot control layout',
        isDefault: true,
        analogBindings: [
            // Left stick — differential drive on the tracks topic.
            {
                input: 'left_stick_y',
                action: {
                    type: 'differential_drive',
                    topic: '/tracks',
                    left_channel: 'left_velocity',
                    right_channel: 'right_velocity',
                    throttle_transform: { deadzone: 0.1, scale: 1.0, expo: 1.0, invert: false },
                    turn_transform: { deadzone: 0.1, scale: 1.0, expo: 1.0, invert: false },
                },
                enabled: true,
            },
            {
                input: 'right_stick_x',
                action: {
                    type: 'direct_control',
                    target: { topic: '/saint/head', channel: 'pan', name: 'Head Pan' },
                    transform: { deadzone: 0.05, scale: 0.8, expo: 1.0, invert: false },
                },
                enabled: true,
            },
            {
                input: 'right_stick_y',
                action: {
                    type: 'direct_control',
                    target: { topic: '/saint/head', channel: 'tilt', name: 'Head Tilt' },
                    transform: { deadzone: 0.05, scale: 0.8, expo: 1.0, invert: false },
                },
                enabled: true,
            },
            {
                input: 'left_trigger',
                action: { type: 'modifier', effect: { type: 'precision_mode', min_scale: 0.3 } },
                enabled: true,
            },
            {
                input: 'right_trigger',
                action: { type: 'modifier', effect: { type: 'speed_boost', max_boost: 1.5 } },
                enabled: true,
            },
        ],
        digitalBindings: [
            { input: 'y', trigger: 'press', action: { type: 'show_panel', panel_id: 'poses' }, enabled: true },
            { input: 'x', trigger: 'press', action: { type: 'show_panel', panel_id: 'animations' }, enabled: true },
            { input: 'a', trigger: 'press', action: { type: 'select_panel_item' }, enabled: true },
            { input: 'b', trigger: 'press', action: { type: 'hide_panel' }, enabled: true },
            { input: 'd_pad_up', trigger: 'press', action: { type: 'navigate_panel', direction: 'up' }, enabled: true },
            { input: 'd_pad_down', trigger: 'press', action: { type: 'navigate_panel', direction: 'down' }, enabled: true },
            { input: 'd_pad_left', trigger: 'press', action: { type: 'navigate_panel', direction: 'left' }, enabled: true },
            { input: 'd_pad_right', trigger: 'press', action: { type: 'navigate_panel', direction: 'right' }, enabled: true },
            { input: 'lb', trigger: 'press', action: { type: 'navigate_panel', direction: 'prev_page' }, enabled: true },
            { input: 'rb', trigger: 'press', action: { type: 'navigate_panel', direction: 'next_page' }, enabled: true },
            { input: 'select', trigger: 'press', action: { type: 'e_stop' }, enabled: true },
            { input: 'start', trigger: 'press', action: { type: 'show_panel', panel_id: 'sounds' }, enabled: true },
        ],
        presetPanels: [
            {
                // Server-backed: items stream from list_animations.
                id: 'animations', name: 'Animations', icon: 'animation', color: '#8b5cf6',
                layout: 'grid', columns: 4, itemsPerPage: 8, source: 'animations',
                presets: [],
            },
            {
                // Server-backed: items stream from list_poses (Y opens this).
                id: 'poses', name: 'Poses', icon: 'accessibility', color: '#3b82f6',
                layout: 'grid', columns: 4, itemsPerPage: 8, source: 'poses',
                presets: [],
            },
            {
                id: 'sounds', name: 'Sounds', icon: 'volume_up', color: '#22c55e',
                layout: 'grid', columns: 4, itemsPerPage: 8, source: 'sounds',
                presets: [],
            },
        ],
        settings: {
            globalDeadzone: 0.1,
            hapticFeedback: true,
            doubleTapTimeMs: 300,
            longPressTimeMs: 500,
            panelActivation: 'press',
        },
    };
}

// ─── Module-level singleton state ────────────────────────────────────

const profilesRef = ref<BindingProfile[]>([getDefaultProfile()]);
const activeProfileIdRef = ref<string>('default');
const panelStateRef = ref<PanelState>({
    activePanelId: null,
    selectedIndex: 0,
    currentPage: 0,
    selectedGroup: 'All',
    keepOpen: false,
});

// Last item the operator activated in each panel, keyed by panel id. On
// reopen we restore the highlight to it (if still present in the current
// group filter) so the selection reflects the real last choice instead of
// a meaningless index-0 default. Session-scoped (not persisted).
const lastSelectedByPanel = ref<Record<string, string>>({});

// Runtime override for items-per-page, measured by the panel to fill the
// visible grid (rows that fit at the current UI scale) instead of the
// fixed panel default. null = use the panel's configured itemsPerPage.
// Shared here so navigation + pagination math agree with what's rendered.
const panelItemsPerPageRef = ref<number | null>(null);

function effectiveItemsPerPage(panel: PresetPanel): number {
    return panelItemsPerPageRef.value ?? panel.itemsPerPage;
}

// Called by the panel after measuring how many rows fit. Keeps the
// highlighted item visible by recomputing its page under the new size.
function setPanelItemsPerPage(count: number): void {
    const n = Math.max(1, Math.floor(count));
    if (panelItemsPerPageRef.value === n) return;
    panelItemsPerPageRef.value = n;
    const state = panelStateRef.value;
    panelStateRef.value = {
        ...state,
        currentPage: Math.floor(state.selectedIndex / n),
    };
}

let initialized = false;

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;
    await loadProfiles();
    // We deliberately do NOT subscribe to the Rust 'action-event' stream.
    // Physical button presses already drive panel/navigate/select/activate
    // through the frontend path (App.vue: handleHardwareButton →
    // onVirtualButtonDown → executeDigitalAction), which is the same path
    // the on-screen/touch buttons use. Applying the Rust mapper's
    // action-events on top double-handled every press — most visibly
    // toggling a panel open→closed on a single tap (the two toggles raced),
    // which is why panels came up only intermittently. Commands, analog
    // control, and E-Stop are still handled Rust-side directly (see
    // src-tauri/src/lib.rs) and are unaffected by this. If hold/DoubleTap/
    // release triggers are ever implemented, revisit ownership here.
}

// ─── Profile load / save ─────────────────────────────────────────────

async function loadProfiles(): Promise<void> {
    try {
        const profiles = await invoke<BindingProfile[]>('get_binding_profiles');
        if (profiles && profiles.length > 0) {
            profilesRef.value = profiles;
        }
    } catch (err) {
        console.error('[useBindings] Failed to load profiles, keeping defaults:', err);
    }
}

async function saveProfiles(): Promise<void> {
    await invoke('set_binding_profiles', { profiles: profilesRef.value });
}

function setActiveProfile(profileId: string): void {
    activeProfileIdRef.value = profileId;
    void invoke('set_active_profile', { profileId });
}

// ─── Panel state ─────────────────────────────────────────────────────

// While a server-backed panel is open, poll the library on an interval so
// a board left open picks up items added/removed on the server, and the
// header sync spinner shows it's actively pulling. Cleared when the panel
// closes so we don't poll in the background forever.
const PANEL_POLL_MS = 10000;
let panelPollTimer: ReturnType<typeof setInterval> | null = null;

function stopPanelPoll(): void {
    if (panelPollTimer) {
        clearInterval(panelPollTimer);
        panelPollTimer = null;
    }
}

function startPanelPoll(): void {
    stopPanelPoll();
    panelPollTimer = setInterval(() => { void library.refresh(); }, PANEL_POLL_MS);
}

// Where the highlight should land when (re)opening a panel: the last
// item the operator activated, if it's still in the current group-filtered
// list; otherwise the top. panelState isn't set yet here, so filter with
// the `group` we're about to apply rather than reading panelStateRef.
function restoreSelection(panelId: string, group: string): { index: number; page: number } {
    const profile = profilesRef.value.find(p => p.id === activeProfileIdRef.value);
    const panel = profile?.presetPanels.find(p => p.id === panelId);
    const lastId = lastSelectedByPanel.value[panelId];
    if (!panel || !lastId) return { index: 0, page: 0 };

    let items = panelItemsRaw(panel);
    if (group && group !== 'All') items = items.filter(it => (it.group ?? '') === group);
    const idx = items.findIndex(it => it.id === lastId);
    if (idx < 0) return { index: 0, page: 0 };
    return { index: idx, page: Math.floor(idx / effectiveItemsPerPage(panel)) };
}

function showPanel(panelId: string, defaultGroup?: string, keepOpen = false): void {
    const group = defaultGroup || 'All';
    const restored = restoreSelection(panelId, group);
    panelStateRef.value = {
        activePanelId: panelId,
        selectedIndex: restored.index,
        currentPage: restored.page,
        // Start on the button's configured group, or 'All' when unset.
        selectedGroup: group,
        // Sticky panels stay open after each selection.
        keepOpen,
    };
    // Pull a fresh server list when opening a server-backed panel so the
    // grid is current even if items changed since connect, then keep it
    // live with a background poll while the panel stays open. The panel
    // itself already renders instantly from the persisted cache.
    const profile = profilesRef.value.find(p => p.id === activeProfileIdRef.value);
    const panel = profile?.presetPanels.find(p => p.id === panelId);
    if (panel && panelSource(panel)) {
        void library.refresh();
        startPanelPoll();
    } else {
        stopPanelPoll();
    }
}

function hidePanel(): void {
    stopPanelPoll();
    panelStateRef.value = { ...panelStateRef.value, activePanelId: null };
}

function togglePanel(panelId: string, defaultGroup?: string, keepOpen = false): void {
    if (panelStateRef.value.activePanelId === panelId) hidePanel();
    else showPanel(panelId, defaultGroup, keepOpen);
}

// Change the active panel's group filter (from the header dropdown).
// Resets selection/page since the visible set changes.
function setActiveGroup(group: string): void {
    panelStateRef.value = {
        ...panelStateRef.value,
        selectedGroup: group,
        selectedIndex: 0,
        currentPage: 0,
    };
}

function navigatePanel(direction: NavigateDirection): void {
    const profile = profilesRef.value.find(p => p.id === activeProfileIdRef.value);
    const state = panelStateRef.value;
    if (!profile || !state.activePanelId) return;
    const panel = profile.presetPanels.find(p => p.id === state.activePanelId);
    if (!panel) return;

    const totalItems = panelItems(panel).length;
    const columns = displayPrefs.prefsFor(panel.id).columns;
    const itemsPerPage = effectiveItemsPerPage(panel);
    const totalPages = Math.ceil(totalItems / itemsPerPage);

    let { selectedIndex, currentPage } = state;

    switch (direction) {
        case 'up':
            if (selectedIndex - columns >= 0) {
                selectedIndex -= columns;
                currentPage = Math.floor(selectedIndex / itemsPerPage);
            }
            break;
        case 'down':
            if (selectedIndex + columns < totalItems) {
                selectedIndex += columns;
                currentPage = Math.floor(selectedIndex / itemsPerPage);
            }
            break;
        case 'left':
            if (selectedIndex > 0) {
                selectedIndex--;
                currentPage = Math.floor(selectedIndex / itemsPerPage);
            }
            break;
        case 'right':
            if (selectedIndex + 1 < totalItems) {
                selectedIndex++;
                currentPage = Math.floor(selectedIndex / itemsPerPage);
            }
            break;
        case 'next_item':
            if (selectedIndex + 1 < totalItems) {
                selectedIndex++;
                currentPage = Math.floor(selectedIndex / itemsPerPage);
            }
            break;
        case 'prev_item':
            if (selectedIndex > 0) {
                selectedIndex--;
                currentPage = Math.floor(selectedIndex / itemsPerPage);
            }
            break;
        case 'next_page':
            if (currentPage + 1 < totalPages) {
                currentPage++;
                selectedIndex = currentPage * itemsPerPage;
            }
            break;
        case 'prev_page':
            if (currentPage > 0) {
                currentPage--;
                selectedIndex = currentPage * itemsPerPage;
            }
            break;
    }

    panelStateRef.value = { ...state, selectedIndex, currentPage };
}

function selectCurrentItem(): void {
    const profile = profilesRef.value.find(p => p.id === activeProfileIdRef.value);
    const state = panelStateRef.value;
    if (!profile || !state.activePanelId) return;
    const panel = profile.presetPanels.find(p => p.id === state.activePanelId);
    if (!panel) return;
    const item = panelItems(panel)[state.selectedIndex];
    if (item) triggerPanelItem(panel, item.id);
    // Dismiss after selecting unless the panel is sticky (keep_open),
    // matching the touch path (PresetPanel.selectPreset).
    if (!state.keepOpen) hidePanel();
}

async function activatePreset(presetId: string): Promise<void> {
    await invoke('activate_preset', { presetId });
}

// Eager singletons: initializing the library here (rather than lazily
// inside a computed) means it fetches on connect even before any panel
// is opened, and panelItems can return stable computed refs.
const library = useLibrary();
const connection = useConnection();
const displayPrefs = useDisplayPrefs();

// Classify a panel's server backing. Honors the explicit `source`
// field, and also falls back to the built-in panel id so profiles
// saved before `source` existed still bind to the server lists without
// needing a profile reset.
function panelSource(panel: PresetPanel): 'animations' | 'poses' | 'sounds' | null {
    if (panel.source === 'animations' || panel.id === 'animations') return 'animations';
    if (panel.source === 'poses' || panel.id === 'poses') return 'poses';
    if (panel.source === 'sounds' || panel.id === 'sounds') return 'sounds';
    return null;
}

// Raw (unfiltered) items for a panel. Server-backed panels stream live
// from the server library; static panels use their stored presets.
function panelItemsRaw(panel: PresetPanel): PanelItem[] {
    const src = panelSource(panel);
    if (src === 'animations') return library.animations.value;
    if (src === 'poses') return library.poses.value;
    if (src === 'sounds') return library.sounds.value;
    return panel.presets;
}

// Items shown for a panel, after the active group filter. Only one panel
// is open at a time, so the global panelState.selectedGroup applies to it.
function panelItems(panel: PresetPanel): PanelItem[] {
    const items = panelItemsRaw(panel);
    const group = panelStateRef.value.selectedGroup;
    if (!group || group === 'All') return items;
    return items.filter(it => (it.group ?? '') === group);
}

// Distinct, sorted group names present in a panel's items (unfiltered).
// Drives the header group dropdown; empty when nothing is grouped.
function panelGroups(panel: PresetPanel): string[] {
    const groups = new Set<string>();
    for (const it of panelItemsRaw(panel)) {
        if (it.group) groups.add(it.group);
    }
    return Array.from(groups).sort();
}

// Groups available for a panel by id — used by the bindings editor to
// populate the "default group" dropdown for a show_panel button.
function groupsForPanel(panelId: string): string[] {
    const profile = profilesRef.value.find(p => p.id === activeProfileIdRef.value);
    const panel = profile?.presetPanels.find(p => p.id === panelId);
    return panel ? panelGroups(panel) : [];
}

// Fire the right action for a selected item: play/apply for the
// server-backed panels, the local preset path otherwise.
//
// Animations toggle: tapping one that's already playing AND loops stops
// it, so a looping animation is dismissed with the same button that
// started it. One-shots (and idle animations) just (re)start. We update
// the playing map optimistically so the badge flips instantly; the next
// server broadcast reconciles.
function triggerPanelItem(panel: PresetPanel, itemId: string): void {
    // Remember this as the panel's last selection so reopening restores
    // the highlight to it.
    lastSelectedByPanel.value = { ...lastSelectedByPanel.value, [panel.id]: itemId };

    const src = panelSource(panel);
    if (src === 'animations') {
        const playing = library.playing.value[itemId];
        if (playing && playing.loop) {
            void connection.stopAnimation(itemId);
            library.noteStopped(itemId);
        } else {
            const item = library.animations.value.find(a => a.id === itemId);
            void connection.startAnimation(itemId);
            library.noteStarted(itemId, item?.loop === true);
        }
    }
    else if (src === 'poses') void connection.applyPose(itemId);
    else if (src === 'sounds') void connection.playSound(itemId);
    else void activatePreset(itemId);
}

// ─── Mutators ────────────────────────────────────────────────────────

function mutateActive(fn: (p: BindingProfile) => BindingProfile): void {
    const activeId = activeProfileIdRef.value;
    profilesRef.value = profilesRef.value.map(p => p.id === activeId ? fn(p) : p);
    void saveProfiles();
}

function updateAnalogBinding(index: number, binding: AnalogBinding): void {
    mutateActive(p => {
        const analogBindings = [...p.analogBindings];
        analogBindings[index] = binding;
        return { ...p, analogBindings };
    });
}

function updateDigitalBinding(index: number, binding: DigitalBinding): void {
    mutateActive(p => {
        const digitalBindings = [...p.digitalBindings];
        digitalBindings[index] = binding;
        return { ...p, digitalBindings };
    });
}

function addAnalogBinding(binding: AnalogBinding): void {
    mutateActive(p => ({ ...p, analogBindings: [...p.analogBindings, binding] }));
}

function addDigitalBinding(binding: DigitalBinding): void {
    mutateActive(p => ({ ...p, digitalBindings: [...p.digitalBindings, binding] }));
}

function removeAnalogBinding(index: number): void {
    mutateActive(p => ({
        ...p,
        analogBindings: p.analogBindings.filter((_, i) => i !== index),
    }));
}

function removeDigitalBinding(index: number): void {
    mutateActive(p => ({
        ...p,
        digitalBindings: p.digitalBindings.filter((_, i) => i !== index),
    }));
}

function addPresetPanel(panel: PresetPanel): void {
    mutateActive(p => ({ ...p, presetPanels: [...p.presetPanels, panel] }));
}

function updatePresetPanel(panelId: string, panel: PresetPanel): void {
    mutateActive(p => ({
        ...p,
        presetPanels: p.presetPanels.map(pp => pp.id === panelId ? panel : pp),
    }));
}

function removePresetPanel(panelId: string): void {
    mutateActive(p => ({
        ...p,
        presetPanels: p.presetPanels.filter(pp => pp.id !== panelId),
    }));
}

function updateProfileSettings(settings: ProfileSettings): void {
    mutateActive(p => ({ ...p, settings }));
}

// ─── Composable export ───────────────────────────────────────────────

export function useBindings() {
    void ensureInit();

    const activeProfile = computed(() =>
        profilesRef.value.find(p => p.id === activeProfileIdRef.value) ?? profilesRef.value[0]);
    const activePanel = computed(() => {
        const profile = activeProfile.value;
        const state = panelStateRef.value;
        if (!profile || !state.activePanelId) return null;
        return profile.presetPanels.find(p => p.id === state.activePanelId) ?? null;
    });

    return {
        allProfiles: computed(() => profilesRef.value),
        activeProfile,
        activePanelState: computed(() => panelStateRef.value),
        activePanel,
        // Live items for the active panel (server-backed or static) and
        // the source-aware trigger the panel UI calls on select.
        activePanelItems: computed<PanelItem[]>(() =>
            activePanel.value ? panelItems(activePanel.value) : []),
        // 'animations' | 'poses' | 'sounds' | null — lets the panel UI
        // show the right loading/empty message for server-backed panels.
        activePanelSource: computed(() =>
            activePanel.value ? panelSource(activePanel.value) : null),
        // Group filter for the active panel: available groups, the current
        // selection, and a setter for the header dropdown.
        activePanelGroups: computed<string[]>(() =>
            activePanel.value ? panelGroups(activePanel.value) : []),
        activePanelSelectedGroup: computed(() => panelStateRef.value.selectedGroup),
        // Total pages for the active panel's (group-filtered) items — used
        // by the main header's page indicator now that it owns the panel chrome.
        activePanelTotalPages: computed(() => {
            const p = activePanel.value;
            if (!p) return 0;
            return Math.max(1, Math.ceil(panelItems(p).length / effectiveItemsPerPage(p)));
        }),
        // Effective items-per-page for the active panel (measured to fill
        // the grid). The panel uses this for slicing; setter below.
        activePanelItemsPerPage: computed(() =>
            activePanel.value ? effectiveItemsPerPage(activePanel.value) : 0),
        setPanelItemsPerPage,
        setActiveGroup,
        // Groups available for a panel by id (for the bindings editor).
        groupsForPanel,
        triggerActiveItem: (itemId: string) => {
            const p = activePanel.value;
            if (p) triggerPanelItem(p, itemId);
        },

        loadProfiles,
        saveProfiles,
        setActiveProfile,

        // Panel state
        showPanel,
        hidePanel,
        togglePanel,
        navigatePanel,
        selectCurrentItem,
        activatePreset,

        // Binding mutators
        addAnalogBinding,
        updateAnalogBinding,
        removeAnalogBinding,
        addDigitalBinding,
        updateDigitalBinding,
        removeDigitalBinding,

        // Preset panel mutators
        addPresetPanel,
        updatePresetPanel,
        removePresetPanel,

        // Settings
        updateProfileSettings,
    };
}
