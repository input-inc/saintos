/**
 * Bindings composable — Vue equivalent of the Angular
 * `BindingsService`. Manages the operator's binding profile (analog +
 * digital input mappings, preset panels, profile settings) and the
 * currently-active preset panel state.
 *
 * Profiles are persisted on the Rust side via `get_binding_profiles`
 * and `set_binding_profiles`. Action events from the input mapper
 * arrive on the `action-event` Tauri event and drive panel navigation
 * + preset activation.
 *
 * The shape mirrors BindingsService 1:1 so the components mostly
 * translate verbatim — `signal` becomes `ref`, `computed` keeps its
 * name, `service.foo()` becomes `bindings.foo()`. The default profile
 * constant is identical between the two.
 */

import { computed, ref } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { useLibrary } from './useLibrary';
import { useConnection } from './useConnection';

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
    | { type: 'show_panel'; panel_id: string }
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
     *  'animations'/'poses' = populated from the server library
     *  (useLibrary); selecting fires start_animation / apply_pose. */
    source?: 'animations' | 'poses';
}

/** Minimal shape the panel grid renders. Both Preset and the server
 *  library's items satisfy it. */
export interface PanelItem {
    id: string;
    name: string;
    icon?: string;
    color?: string;
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
    | { type: 'show_panel'; panel_id: string }
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
                layout: 'grid', columns: 4, itemsPerPage: 8,
                presets: [
                    { id: 'sound_hello',   name: 'Hello',   icon: 'record_voice_over',        type: 'sound', data: { type: 'sound', soundId: 'hello.wav',   volume: 1.0, priority: 1 } },
                    { id: 'sound_goodbye', name: 'Goodbye', icon: 'waving_hand',              type: 'sound', data: { type: 'sound', soundId: 'goodbye.wav', volume: 1.0, priority: 1 } },
                    { id: 'sound_yes',     name: 'Yes',     icon: 'thumb_up',                 type: 'sound', data: { type: 'sound', soundId: 'yes.wav',     volume: 1.0, priority: 1 } },
                    { id: 'sound_no',      name: 'No',      icon: 'thumb_down',               type: 'sound', data: { type: 'sound', soundId: 'no.wav',      volume: 1.0, priority: 1 } },
                    { id: 'sound_laugh',   name: 'Laugh',   icon: 'sentiment_very_satisfied', type: 'sound', data: { type: 'sound', soundId: 'laugh.wav',   volume: 1.0, priority: 1 } },
                    { id: 'sound_alert',   name: 'Alert',   icon: 'notifications',            type: 'sound', data: { type: 'sound', soundId: 'alert.wav',   volume: 1.0, priority: 1 } },
                ],
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
});

let initialized = false;
const unlistenFns: UnlistenFn[] = [];

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;
    await loadProfiles();
    unlistenFns.push(
        await listen<ActionEvent>('action-event', event => {
            handleActionEvent(event.payload);
        }),
    );
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

function showPanel(panelId: string): void {
    panelStateRef.value = {
        activePanelId: panelId,
        selectedIndex: 0,
        currentPage: 0,
    };
}

function hidePanel(): void {
    panelStateRef.value = { ...panelStateRef.value, activePanelId: null };
}

function togglePanel(panelId: string): void {
    if (panelStateRef.value.activePanelId === panelId) hidePanel();
    else showPanel(panelId);
}

function navigatePanel(direction: NavigateDirection): void {
    const profile = profilesRef.value.find(p => p.id === activeProfileIdRef.value);
    const state = panelStateRef.value;
    if (!profile || !state.activePanelId) return;
    const panel = profile.presetPanels.find(p => p.id === state.activePanelId);
    if (!panel) return;

    const totalItems = panelItems(panel).length;
    const columns = panel.columns;
    const itemsPerPage = panel.itemsPerPage;
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
}

async function activatePreset(presetId: string): Promise<void> {
    await invoke('activate_preset', { presetId });
}

// Items shown for a panel. Server-backed panels (source set) stream
// live from the server library; static panels use their stored presets.
function panelItems(panel: PresetPanel): PanelItem[] {
    if (panel.source === 'animations') return useLibrary().animations.value;
    if (panel.source === 'poses') return useLibrary().poses.value;
    return panel.presets;
}

// Fire the right action for a selected item: play/apply for the
// server-backed panels, the local preset path otherwise.
function triggerPanelItem(panel: PresetPanel, itemId: string): void {
    if (panel.source === 'animations') void useConnection().startAnimation(itemId);
    else if (panel.source === 'poses') void useConnection().applyPose(itemId);
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

// ─── Action event handler ───────────────────────────────────────────

function handleActionEvent(event: ActionEvent): void {
    switch (event.type) {
        case 'show_panel':       showPanel(event.panel_id); break;
        case 'hide_panel':       hidePanel(); break;
        case 'navigate_panel':   navigatePanel(event.direction); break;
        case 'select_panel_item': selectCurrentItem(); break;
        case 'activate_preset':  void activatePreset(event.preset_id); break;
        // 'command' / 'toggle_output' / 'cycle_output' / 'e_stop' are
        // already handled Rust-side (mapper → ws_client) and don't
        // need a frontend bounce.
    }
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
