<!--
    Bindings editor view. Tabs for analog / digital / preset-panels /
    settings, with modal editors for the binding rows. Translated from
    controller/src/app/features/bindings/bindings.component.ts.

    Every <select> dropdown is replaced with <SaintSelect> (Headless
    UI Listbox) — that's the change that motivated the entire Vue
    migration; webkit2gtk's native <select> popup ignores webview
    zoom and renders tiny on a Deck at any scale above 1×.
-->
<script setup lang="ts">
import { computed, onMounted, reactive, ref } from 'vue';
import SaintSelect from '../components/SaintSelect.vue';
import {
    useBindings,
    isWsInputTarget,
    targetDisplayName as defaultTargetDisplayName,
    type AnalogBinding,
    type DigitalBinding,
    type AnalogInput,
    type DigitalInput,
    type ButtonTrigger,
    type DigitalAction,
    type AnalogAction,
    type PresetPanel,
    type NavigateDirection,
    type ControlTarget,
    type ModifierEffect,
} from '../composables/useBindings';
import { useDiscovery } from '../composables/useDiscovery';

type TabId = 'analog' | 'digital' | 'panels' | 'settings';

const ANALOG_INPUTS: { value: AnalogInput; label: string }[] = [
    { value: 'left_stick_x', label: 'Left Stick X' },
    { value: 'left_stick_y', label: 'Left Stick Y' },
    { value: 'right_stick_x', label: 'Right Stick X' },
    { value: 'right_stick_y', label: 'Right Stick Y' },
    { value: 'left_trigger', label: 'Left Trigger' },
    { value: 'right_trigger', label: 'Right Trigger' },
];

const DIGITAL_INPUTS: { value: DigitalInput; label: string }[] = [
    { value: 'a', label: 'A Button' },
    { value: 'b', label: 'B Button' },
    { value: 'x', label: 'X Button' },
    { value: 'y', label: 'Y Button' },
    { value: 'lb', label: 'Left Bumper' },
    { value: 'rb', label: 'Right Bumper' },
    { value: 'd_pad_up', label: 'D-Pad Up' },
    { value: 'd_pad_down', label: 'D-Pad Down' },
    { value: 'd_pad_left', label: 'D-Pad Left' },
    { value: 'd_pad_right', label: 'D-Pad Right' },
    { value: 'start', label: 'Start' },
    { value: 'select', label: 'Select' },
    { value: 'left_stick', label: 'Left Stick Press' },
    { value: 'right_stick', label: 'Right Stick Press' },
];

const BUTTON_TRIGGERS: { value: ButtonTrigger; label: string }[] = [
    { value: 'press', label: 'Press' },
    { value: 'release', label: 'Release' },
    { value: 'hold', label: 'Hold' },
    { value: 'double_tap', label: 'Double Tap' },
    { value: 'long_press', label: 'Long Press' },
];

const DIGITAL_ACTION_TYPES = [
    { value: 'show_panel', label: 'Show Panel' },
    { value: 'hide_panel', label: 'Hide Panel' },
    { value: 'activate_preset', label: 'Activate Preset' },
    { value: 'navigate_panel', label: 'Navigate Panel' },
    { value: 'select_panel_item', label: 'Select Panel Item' },
    { value: 'toggle_output', label: 'Toggle Output' },
    { value: 'cycle_output', label: 'Cycle Output' },
    { value: 'direct_control', label: 'Direct Control' },
    { value: 'e_stop', label: 'Emergency Stop' },
    { value: 'none', label: 'None' },
];

const NAVIGATE_DIRECTIONS: { value: NavigateDirection; label: string }[] = [
    { value: 'up', label: 'Up (Grid)' },
    { value: 'down', label: 'Down (Grid)' },
    { value: 'left', label: 'Left (Grid)' },
    { value: 'right', label: 'Right (Grid)' },
    { value: 'next_item', label: 'Next Item (Linear)' },
    { value: 'prev_item', label: 'Previous Item (Linear)' },
    { value: 'next_page', label: 'Next Page' },
    { value: 'prev_page', label: 'Previous Page' },
];

const tabs: { id: TabId; label: string; icon: string }[] = [
    { id: 'analog', label: 'Analog', icon: 'joystick' },
    { id: 'digital', label: 'Buttons', icon: 'gamepad' },
    { id: 'panels', label: 'Preset Panels', icon: 'dashboard' },
    { id: 'settings', label: 'Settings', icon: 'settings' },
];

const bindings = useBindings();
const discovery = useDiscovery();

// ─── UI state ────────────────────────────────────────────────────────

const activeTab = ref<TabId>('analog');
const isNewBinding = ref(false);
const showAnalogEditor = ref(false);
const showDigitalEditor = ref(false);
let editingAnalogIndex = -1;
let editingDigitalIndex = -1;

const analogForm = reactive({
    input: 'left_stick_x' as AnalogInput,
    actionType: 'direct_control' as 'direct_control' | 'modifier' | 'differential_drive',
    targetKind: 'ws_input' as 'ws_input' | 'topic',
    targetSheetId: '',
    targetInputId: '',
    targetTopic: '',
    targetChannel: '',
    targetName: '',
    deadzone: 0.1,
    scale: 1.0,
    expo: 1.0,
    invert: false,
    modifierType: 'precision_mode' as 'precision_mode' | 'speed_boost' | 'scale_target',
    modifierMinScale: 0.3,
    modifierMaxBoost: 1.5,
    modifierTargetId: '',
    modifierScale: 1.0,
});

const digitalForm = reactive({
    input: 'a' as DigitalInput,
    trigger: 'press' as ButtonTrigger,
    actionType: 'show_panel',
    panelId: '',
    presetId: '',
    direction: 'next_item' as NavigateDirection,
    targetId: '',
    cycleValues: '',
    topic: '',
    channel: '',
    value: 1.0,
});

// ─── Reactive selectors ─────────────────────────────────────────────

const profileDescription = computed(() => bindings.activeProfile.value?.description);
const profileSettings = computed(() => bindings.activeProfile.value?.settings);
const analogBindings = computed<AnalogBinding[]>(() =>
    bindings.activeProfile.value?.analogBindings ?? []);
const digitalBindings = computed<DigitalBinding[]>(() =>
    bindings.activeProfile.value?.digitalBindings ?? []);
const presetPanels = computed<PresetPanel[]>(() =>
    bindings.activeProfile.value?.presetPanels ?? []);

// Headless-UI-friendly options.
const profileOptions = computed(() =>
    bindings.allProfiles.value.map(p => ({ value: p.id, label: p.name })));
const sheetOptions = computed(() =>
    [{ value: '', label: '-- Select Sheet --' }]
        .concat(discovery.wsInputSheets.value.map(s => ({ value: s.id, label: s.label }))));
const wsInputOptions = computed(() => {
    if (!analogForm.targetSheetId) return [{ value: '', label: '-- Select Input --' }];
    return [{ value: '', label: '-- Select Input --' }].concat(
        discovery.getWsInputsForSheet(analogForm.targetSheetId)
            .filter(slot => slot.kind === 'command')
            .map(slot => ({ value: slot.input_id, label: slot.label || slot.input_id })));
});
const topicOptionsAnalog = computed(() =>
    [{ value: '', label: '-- Select Topic --' }]
        .concat(discovery.availableTopics.value.map(t => ({ value: t, label: t }))));
const channelOptionsAnalog = computed(() => {
    if (!analogForm.targetTopic) return [{ value: '', label: '-- Select Channel --' }];
    return [{ value: '', label: '-- Select Channel --' }].concat(
        discovery.getChannelsForTopic(analogForm.targetTopic)
            .map(c => ({ value: c.field, label: c.label })));
});
const topicOptionsDigital = computed(() =>
    [{ value: '', label: '-- Select Topic --' }]
        .concat(discovery.availableTopics.value.map(t => ({ value: t, label: t }))));
const channelOptionsDigital = computed(() => {
    if (!digitalForm.topic) return [{ value: '', label: '-- Select Channel --' }];
    return [{ value: '', label: '-- Select Channel --' }].concat(
        discovery.getChannelsForTopic(digitalForm.topic)
            .map(c => ({ value: c.field, label: c.label })));
});
const panelOptions = computed(() =>
    presetPanels.value.map(p => ({ value: p.id, label: p.name })));
const analogInputOptions = ANALOG_INPUTS;
const digitalInputOptions = DIGITAL_INPUTS;
const buttonTriggerOptions = BUTTON_TRIGGERS;
const digitalActionTypeOptions = DIGITAL_ACTION_TYPES;
const navigateDirectionOptions = NAVIGATE_DIRECTIONS;

// ─── Display helpers ─────────────────────────────────────────────────

function getAnalogInputLabel(i: AnalogInput): string {
    return ANALOG_INPUTS.find(x => x.value === i)?.label ?? i;
}
function getDigitalInputLabel(i: DigitalInput): string {
    return DIGITAL_INPUTS.find(x => x.value === i)?.label ?? i;
}

/** Friendly target name. For WS inputs, resolve sheet_id to the
 *  node's display_name via discovery so the UI shows "Track Drive
 *  Right" instead of a raw node id like "teensy41_A1B2C3D4". */
function targetDisplayName(t: ControlTarget): string {
    if (t.name) return t.name;
    if (isWsInputTarget(t)) {
        const sheet = discovery.wsInputSheets.value.find(s => s.id === t.sheet_id);
        const slot = discovery.getWsInputsForSheet(t.sheet_id).find(s => s.input_id === t.input_id);
        const sheetLabel = sheet?.label || t.sheet_id;
        const slotLabel = slot?.label || t.input_id;
        return `${sheetLabel} · ${slotLabel}`;
    }
    return defaultTargetDisplayName(t);
}

function formatAnalogAction(action: AnalogAction): string {
    if (action.type === 'direct_control') {
        return `Direct Control → ${targetDisplayName(action.target)}`;
    }
    if (action.type === 'modifier') {
        switch (action.effect.type) {
            case 'precision_mode': return `Precision Mode (min: ${action.effect.min_scale})`;
            case 'speed_boost':    return `Speed Boost (max: ${action.effect.max_boost})`;
            case 'scale_target':   return `Scale ${action.effect.target_id}`;
        }
    }
    return 'Unknown';
}

function formatDigitalAction(action: DigitalAction): string {
    switch (action.type) {
        case 'show_panel':        return `Show Panel: ${action.panel_id}`;
        case 'hide_panel':        return 'Hide Panel';
        case 'activate_preset':   return `Activate Preset: ${action.preset_id}`;
        case 'navigate_panel':    return `Navigate: ${action.direction}`;
        case 'select_panel_item': return 'Select Panel Item';
        case 'toggle_output':     return `Toggle: ${action.target_id}`;
        case 'cycle_output':      return `Cycle: ${action.target_id} (${action.values.join(', ')})`;
        case 'direct_control':    return `Direct Control → ${targetDisplayName(action.target)} = ${action.value}`;
        case 'e_stop':            return 'Emergency Stop';
        case 'none':              return 'None';
    }
}

// ─── Mutators ────────────────────────────────────────────────────────

function toggleAnalogBinding(index: number, binding: AnalogBinding): void {
    bindings.updateAnalogBinding(index, { ...binding, enabled: !binding.enabled });
}

function toggleDigitalBinding(index: number, binding: DigitalBinding): void {
    bindings.updateDigitalBinding(index, { ...binding, enabled: !binding.enabled });
}

function removeAnalogBinding(i: number): void { bindings.removeAnalogBinding(i); }
function removeDigitalBinding(i: number): void { bindings.removeDigitalBinding(i); }

function addAnalogBinding(): void {
    isNewBinding.value = true;
    editingAnalogIndex = -1;
    resetAnalogForm();
    void discovery.refreshWsInputs();
    showAnalogEditor.value = true;
}

function editAnalogBinding(index: number, binding: AnalogBinding): void {
    isNewBinding.value = false;
    editingAnalogIndex = index;
    loadAnalogForm(binding);
    void discovery.refreshWsInputs();
    showAnalogEditor.value = true;
}

function addDigitalBinding(): void {
    isNewBinding.value = true;
    editingDigitalIndex = -1;
    resetDigitalForm();
    void discovery.refreshWsInputs();
    showDigitalEditor.value = true;
}

function editDigitalBinding(index: number, binding: DigitalBinding): void {
    isNewBinding.value = false;
    editingDigitalIndex = index;
    loadDigitalForm(binding);
    void discovery.refreshWsInputs();
    showDigitalEditor.value = true;
}

function resetAnalogForm(): void {
    analogForm.input = 'left_stick_x';
    analogForm.actionType = 'direct_control';
    analogForm.targetKind = 'ws_input';
    analogForm.targetSheetId = '';
    analogForm.targetInputId = '';
    analogForm.targetTopic = '';
    analogForm.targetChannel = '';
    analogForm.targetName = '';
    analogForm.deadzone = 0.1;
    analogForm.scale = 1.0;
    analogForm.expo = 1.0;
    analogForm.invert = false;
    analogForm.modifierType = 'precision_mode';
    analogForm.modifierMinScale = 0.3;
    analogForm.modifierMaxBoost = 1.5;
    analogForm.modifierTargetId = '';
    analogForm.modifierScale = 1.0;
}

function loadAnalogForm(binding: AnalogBinding): void {
    analogForm.input = binding.input;
    analogForm.actionType = binding.action.type;

    // The analog form only edits direct_control + modifier; bail on
    // differential_drive so we don't show stale state.
    if (binding.action.type === 'differential_drive') {
        console.warn('loadAnalogForm: differential_drive bindings are not editable here');
        return;
    }

    if (binding.action.type === 'direct_control') {
        const target = binding.action.target;
        if (isWsInputTarget(target)) {
            analogForm.targetKind = 'ws_input';
            analogForm.targetSheetId = target.sheet_id;
            analogForm.targetInputId = target.input_id;
        } else {
            analogForm.targetKind = 'topic';
            analogForm.targetTopic = target.topic;
            analogForm.targetChannel = target.channel;
        }
        analogForm.targetName = target.name || '';
        analogForm.deadzone = binding.action.transform.deadzone;
        analogForm.scale = binding.action.transform.scale;
        analogForm.expo = binding.action.transform.expo;
        analogForm.invert = binding.action.transform.invert;
    } else if (binding.action.type === 'modifier') {
        analogForm.modifierType = binding.action.effect.type;
        if (binding.action.effect.type === 'precision_mode') {
            analogForm.modifierMinScale = binding.action.effect.min_scale;
        } else if (binding.action.effect.type === 'speed_boost') {
            analogForm.modifierMaxBoost = binding.action.effect.max_boost;
        } else if (binding.action.effect.type === 'scale_target') {
            analogForm.modifierTargetId = binding.action.effect.target_id;
            analogForm.modifierScale = binding.action.effect.scale;
        }
    }
}

function saveAnalogBinding(): void {
    let action: AnalogAction;
    if (analogForm.actionType === 'direct_control') {
        const target: ControlTarget = analogForm.targetKind === 'ws_input'
            ? {
                sheet_id: analogForm.targetSheetId,
                input_id: analogForm.targetInputId,
                name: analogForm.targetName || undefined,
            }
            : {
                topic: analogForm.targetTopic,
                channel: analogForm.targetChannel,
                name: analogForm.targetName || undefined,
            };
        action = {
            type: 'direct_control',
            target,
            transform: {
                deadzone: analogForm.deadzone,
                scale: analogForm.scale,
                expo: analogForm.expo,
                invert: analogForm.invert,
            },
        };
    } else {
        let effect: ModifierEffect;
        switch (analogForm.modifierType) {
            case 'precision_mode':
                effect = { type: 'precision_mode', min_scale: analogForm.modifierMinScale }; break;
            case 'speed_boost':
                effect = { type: 'speed_boost', max_boost: analogForm.modifierMaxBoost }; break;
            case 'scale_target':
                effect = {
                    type: 'scale_target',
                    target_id: analogForm.modifierTargetId,
                    scale: analogForm.modifierScale,
                }; break;
        }
        action = { type: 'modifier', effect };
    }

    const binding: AnalogBinding = { input: analogForm.input, action, enabled: true };
    if (isNewBinding.value) bindings.addAnalogBinding(binding);
    else bindings.updateAnalogBinding(editingAnalogIndex, binding);
    cancelEdit();
}

function resetDigitalForm(): void {
    const panels = presetPanels.value;
    digitalForm.input = 'a';
    digitalForm.trigger = 'press';
    digitalForm.actionType = 'show_panel';
    digitalForm.panelId = panels[0]?.id ?? '';
    digitalForm.presetId = '';
    digitalForm.direction = 'next_item';
    digitalForm.targetId = '';
    digitalForm.cycleValues = '';
    digitalForm.topic = '';
    digitalForm.channel = '';
    digitalForm.value = 1.0;
}

function loadDigitalForm(binding: DigitalBinding): void {
    digitalForm.input = binding.input;
    digitalForm.trigger = binding.trigger;
    digitalForm.actionType = binding.action.type;

    switch (binding.action.type) {
        case 'show_panel':      digitalForm.panelId = binding.action.panel_id; break;
        case 'activate_preset': digitalForm.presetId = binding.action.preset_id; break;
        case 'navigate_panel':  digitalForm.direction = binding.action.direction; break;
        case 'toggle_output':   digitalForm.targetId = binding.action.target_id; break;
        case 'cycle_output':
            digitalForm.targetId = binding.action.target_id;
            digitalForm.cycleValues = binding.action.values.join(', ');
            break;
        case 'direct_control': {
            const t = binding.action.target;
            if (isWsInputTarget(t)) {
                console.warn('loadDigitalForm: WS-input digital direct_control not editable here');
                digitalForm.topic = '';
                digitalForm.channel = '';
            } else {
                digitalForm.topic = t.topic;
                digitalForm.channel = t.channel;
            }
            digitalForm.value = binding.action.value;
            break;
        }
    }
}

function saveDigitalBinding(): void {
    let action: DigitalAction;
    switch (digitalForm.actionType) {
        case 'show_panel':
            action = { type: 'show_panel', panel_id: digitalForm.panelId }; break;
        case 'hide_panel':
            action = { type: 'hide_panel' }; break;
        case 'activate_preset':
            action = { type: 'activate_preset', preset_id: digitalForm.presetId }; break;
        case 'navigate_panel':
            action = { type: 'navigate_panel', direction: digitalForm.direction }; break;
        case 'select_panel_item':
            action = { type: 'select_panel_item' }; break;
        case 'toggle_output':
            action = { type: 'toggle_output', target_id: digitalForm.targetId }; break;
        case 'cycle_output':
            action = {
                type: 'cycle_output',
                target_id: digitalForm.targetId,
                values: digitalForm.cycleValues.split(',').map(v => v.trim()).filter(v => v),
            }; break;
        case 'direct_control':
            action = {
                type: 'direct_control',
                target: { topic: digitalForm.topic, channel: digitalForm.channel },
                value: digitalForm.value,
            }; break;
        case 'e_stop':
            action = { type: 'e_stop' }; break;
        default:
            action = { type: 'none' };
    }

    const binding: DigitalBinding = {
        input: digitalForm.input,
        trigger: digitalForm.trigger,
        action,
        enabled: true,
    };

    if (isNewBinding.value) bindings.addDigitalBinding(binding);
    else bindings.updateDigitalBinding(editingDigitalIndex, binding);
    cancelEdit();
}

function cancelEdit(): void {
    showAnalogEditor.value = false;
    showDigitalEditor.value = false;
    isNewBinding.value = false;
}

// ─── Preset panel ────────────────────────────────────────────────────

function addPresetPanel(): void {
    const panel: PresetPanel = {
        id: `panel_${Date.now()}`,
        name: 'New Panel',
        icon: 'folder',
        color: '#8b5cf6',
        presets: [],
        layout: 'grid',
        columns: 4,
        itemsPerPage: 8,
    };
    bindings.addPresetPanel(panel);
}

function editPresetPanel(panel: PresetPanel): void {
    // TODO: panel editor modal — parity with the Angular TODO.
    console.log('Edit panel:', panel);
}

function updatePanelActivation(mode: 'press' | 'hold'): void {
    const profile = bindings.activeProfile.value;
    if (!profile) return;
    bindings.updateProfileSettings({ ...profile.settings, panelActivation: mode });
}

// Reset selection when the parent dropdown changes — same UX as the
// Angular onTopicChange / onSheetChange / onDigitalTopicChange.
function onTopicChange(value: string): void {
    analogForm.targetTopic = value;
    analogForm.targetChannel = '';
}
function onSheetChange(value: string): void {
    analogForm.targetSheetId = value;
    analogForm.targetInputId = '';
}
function onDigitalTopicChange(value: string): void {
    digitalForm.topic = value;
    digitalForm.channel = '';
}

onMounted(() => {
    void discovery.refresh();
});
</script>

<template>
    <div class="p-6 space-y-6">
        <!-- Profile Selector -->
        <div class="card">
            <div class="flex items-center justify-between">
                <h2 class="text-lg font-semibold">Binding Profile</h2>
                <div class="w-64">
                    <SaintSelect :model-value="bindings.activeProfile.value?.id ?? ''"
                                 :options="profileOptions"
                                 @update:model-value="(v: string) => bindings.setActiveProfile(v)" />
                </div>
            </div>
            <p v-if="profileDescription" class="text-sm text-saint-text-muted mt-2">
                {{ profileDescription }}
            </p>
        </div>

        <!-- Tabs -->
        <div class="flex border-b border-saint-surface-light">
            <button v-for="tab in tabs" :key="tab.id"
                    class="px-4 py-2 text-sm font-medium transition-colors border-b-2 -mb-px"
                    :class="activeTab === tab.id
                        ? 'border-saint-primary text-saint-primary'
                        : 'border-transparent text-saint-text-muted'"
                    @click="activeTab = tab.id">
                <span class="material-symbols-outlined text-lg align-middle mr-1">{{ tab.icon }}</span>
                {{ tab.label }}
            </button>
        </div>

        <!-- Analog tab -->
        <div v-if="activeTab === 'analog'" class="space-y-4">
            <div class="flex justify-between items-center">
                <h3 class="text-lg font-medium">Analog Bindings</h3>
                <button class="btn btn-primary text-sm" @click="addAnalogBinding">+ Add Binding</button>
            </div>
            <div v-for="(binding, i) in analogBindings" :key="i" class="card">
                <div class="flex items-start justify-between gap-4">
                    <div class="flex items-center gap-3">
                        <input type="checkbox" :checked="binding.enabled"
                               @change="toggleAnalogBinding(i, binding)"
                               class="w-4 h-4 rounded">
                        <div>
                            <div class="font-medium">{{ getAnalogInputLabel(binding.input) }}</div>
                            <div class="text-sm text-saint-text-muted">
                                {{ formatAnalogAction(binding.action) }}
                            </div>
                        </div>
                    </div>
                    <div class="flex gap-2">
                        <button class="btn btn-secondary text-sm" @click="editAnalogBinding(i, binding)">Edit</button>
                        <button class="btn btn-danger text-sm" @click="removeAnalogBinding(i)">Delete</button>
                    </div>
                </div>
                <div v-if="binding.action.type === 'direct_control'"
                     class="mt-3 pt-3 border-t border-saint-surface-light grid grid-cols-4 gap-4 text-sm">
                    <div>
                        <span class="text-saint-text-muted">Target:</span>
                        <span class="ml-1">{{ targetDisplayName(binding.action.target) }}</span>
                    </div>
                    <div>
                        <span class="text-saint-text-muted">Deadzone:</span>
                        <span class="ml-1">{{ binding.action.transform.deadzone }}</span>
                    </div>
                    <div>
                        <span class="text-saint-text-muted">Scale:</span>
                        <span class="ml-1">{{ binding.action.transform.scale }}</span>
                    </div>
                    <div>
                        <span class="text-saint-text-muted">Expo:</span>
                        <span class="ml-1">{{ binding.action.transform.expo }}</span>
                    </div>
                </div>
            </div>
            <p v-if="analogBindings.length === 0" class="text-saint-text-muted text-center py-8">
                No analog bindings configured
            </p>
        </div>

        <!-- Digital tab -->
        <div v-if="activeTab === 'digital'" class="space-y-4">
            <div class="flex justify-between items-center">
                <h3 class="text-lg font-medium">Digital Bindings</h3>
                <button class="btn btn-primary text-sm" @click="addDigitalBinding">+ Add Binding</button>
            </div>
            <div v-for="(binding, i) in digitalBindings" :key="i" class="card">
                <div class="flex items-start justify-between gap-4">
                    <div class="flex items-center gap-3">
                        <input type="checkbox" :checked="binding.enabled"
                               @change="toggleDigitalBinding(i, binding)"
                               class="w-4 h-4 rounded">
                        <div>
                            <div class="font-medium">
                                {{ getDigitalInputLabel(binding.input) }}
                                <span class="text-saint-text-muted text-sm ml-1">({{ binding.trigger }})</span>
                            </div>
                            <div class="text-sm text-saint-text-muted">
                                {{ formatDigitalAction(binding.action) }}
                            </div>
                        </div>
                    </div>
                    <div class="flex gap-2">
                        <button class="btn btn-secondary text-sm" @click="editDigitalBinding(i, binding)">Edit</button>
                        <button class="btn btn-danger text-sm" @click="removeDigitalBinding(i)">Delete</button>
                    </div>
                </div>
            </div>
            <p v-if="digitalBindings.length === 0" class="text-saint-text-muted text-center py-8">
                No digital bindings configured
            </p>
        </div>

        <!-- Panels tab -->
        <div v-if="activeTab === 'panels'" class="space-y-4">
            <div class="flex justify-between items-center">
                <h3 class="text-lg font-medium">Preset Panels</h3>
                <button class="btn btn-primary text-sm" @click="addPresetPanel">+ Add Panel</button>
            </div>
            <div class="grid grid-cols-2 lg:grid-cols-3 gap-4">
                <div v-for="panel in presetPanels" :key="panel.id"
                     class="card cursor-pointer hover:border-saint-primary transition-colors"
                     @click="editPresetPanel(panel)">
                    <div class="flex items-center gap-3 mb-3">
                        <span class="material-symbols-outlined text-2xl" :style="{ color: panel.color }">
                            {{ panel.icon }}
                        </span>
                        <div>
                            <div class="font-medium">{{ panel.name }}</div>
                            <div class="text-sm text-saint-text-muted">{{ panel.presets.length }} presets</div>
                        </div>
                    </div>
                    <div class="flex flex-wrap gap-1">
                        <span v-for="preset in panel.presets.slice(0, 4)" :key="preset.id"
                              class="inline-flex items-center gap-1 px-2 py-1 rounded-full text-xs bg-saint-surface-light">
                            <span class="material-symbols-outlined text-sm"
                                  :style="{ color: preset.color || panel.color }">
                                {{ preset.icon }}
                            </span>
                            {{ preset.name }}
                        </span>
                        <span v-if="panel.presets.length > 4"
                              class="px-2 py-1 rounded-full text-xs bg-saint-surface-light text-saint-text-muted">
                            +{{ panel.presets.length - 4 }} more
                        </span>
                    </div>
                </div>
                <p v-if="presetPanels.length === 0" class="text-saint-text-muted text-center py-8 col-span-full">
                    No preset panels configured
                </p>
            </div>
        </div>

        <!-- Settings tab -->
        <div v-if="activeTab === 'settings'" class="space-y-4">
            <h3 class="text-lg font-medium">Profile Settings</h3>
            <div v-if="profileSettings" class="card space-y-6">
                <div class="grid grid-cols-2 gap-4">
                    <div class="col-span-2">
                        <span class="block text-sm font-medium text-saint-text mb-2">Panel Activation Mode</span>
                        <div class="flex gap-2">
                            <button type="button" class="btn"
                                    :class="profileSettings.panelActivation === 'press' ? 'btn-primary' : 'btn-secondary'"
                                    @click="updatePanelActivation('press')">
                                Press (stays open)
                            </button>
                            <button type="button" class="btn"
                                    :class="profileSettings.panelActivation === 'hold' ? 'btn-primary' : 'btn-secondary'"
                                    @click="updatePanelActivation('hold')">
                                Hold (release to close)
                            </button>
                        </div>
                    </div>
                    <div>
                        <label class="block text-sm text-saint-text-muted mb-1">Global Deadzone</label>
                        <input type="number" step="0.01" min="0" max="1" class="input w-full"
                               :value="profileSettings.globalDeadzone" disabled>
                    </div>
                    <div>
                        <label class="block text-sm text-saint-text-muted mb-1">Double Tap Time (ms)</label>
                        <input type="number" step="50" min="100" max="500" class="input w-full"
                               :value="profileSettings.doubleTapTimeMs" disabled>
                    </div>
                    <div>
                        <label class="block text-sm text-saint-text-muted mb-1">Long Press Time (ms)</label>
                        <input type="number" step="50" min="200" max="1000" class="input w-full"
                               :value="profileSettings.longPressTimeMs" disabled>
                    </div>
                    <div class="flex items-center gap-2 pt-6">
                        <input type="checkbox" class="w-4 h-4 rounded"
                               :checked="profileSettings.hapticFeedback" disabled>
                        <label class="text-sm">Haptic Feedback</label>
                    </div>
                </div>
            </div>
        </div>

        <!-- Analog Binding Editor Modal -->
        <div v-if="showAnalogEditor" class="fixed inset-0 bg-black/50 flex items-center justify-center z-50 p-4">
            <div class="card w-full max-w-lg flex flex-col max-h-full">
                <h3 class="text-lg font-semibold mb-4 shrink-0">
                    {{ isNewBinding ? 'Add Analog Binding' : 'Edit Analog Binding' }}
                </h3>
                <div class="space-y-4 overflow-y-auto min-h-0">
                    <div>
                        <label class="block text-sm text-saint-text-muted mb-1">Input</label>
                        <SaintSelect v-model="analogForm.input" :options="analogInputOptions" />
                    </div>
                    <div>
                        <label class="block text-sm text-saint-text-muted mb-1">Action Type</label>
                        <SaintSelect v-model="analogForm.actionType" :options="[
                            { value: 'direct_control', label: 'Direct Control' },
                            { value: 'modifier', label: 'Modifier' },
                        ]" />
                    </div>

                    <template v-if="analogForm.actionType === 'direct_control'">
                        <div>
                            <label class="block text-sm text-saint-text-muted mb-1">Target Kind</label>
                            <SaintSelect v-model="analogForm.targetKind" :options="[
                                { value: 'ws_input', label: 'WebSocket Input (routing sheet)' },
                                { value: 'topic',    label: 'ROS Topic / Channel (legacy)' },
                            ]" />
                        </div>

                        <div v-if="analogForm.targetKind === 'ws_input'" class="grid grid-cols-2 gap-4">
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Sheet</label>
                                <SaintSelect :model-value="analogForm.targetSheetId"
                                             :options="sheetOptions"
                                             @update:model-value="onSheetChange" />
                            </div>
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">WS Input</label>
                                <SaintSelect v-model="analogForm.targetInputId"
                                             :options="wsInputOptions"
                                             :disabled="!analogForm.targetSheetId" />
                            </div>
                        </div>

                        <div v-if="analogForm.targetKind === 'topic'" class="grid grid-cols-2 gap-4">
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Topic</label>
                                <SaintSelect :model-value="analogForm.targetTopic"
                                             :options="topicOptionsAnalog"
                                             @update:model-value="onTopicChange" />
                            </div>
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Channel</label>
                                <SaintSelect v-model="analogForm.targetChannel"
                                             :options="channelOptionsAnalog"
                                             :disabled="!analogForm.targetTopic" />
                            </div>
                        </div>

                        <div>
                            <label class="block text-sm text-saint-text-muted mb-1">Display Name</label>
                            <input type="text" class="input w-full" v-model="analogForm.targetName">
                        </div>
                        <div class="grid grid-cols-2 gap-4">
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Deadzone</label>
                                <input type="number" step="0.01" min="0" max="1" class="input w-full"
                                       v-model.number="analogForm.deadzone">
                            </div>
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Scale</label>
                                <input type="number" step="0.1" class="input w-full"
                                       v-model.number="analogForm.scale">
                            </div>
                        </div>
                        <div class="grid grid-cols-2 gap-4">
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Expo Curve</label>
                                <input type="number" step="0.1" min="1" max="3" class="input w-full"
                                       v-model.number="analogForm.expo">
                            </div>
                            <div class="flex items-center gap-2 pt-6">
                                <input type="checkbox" class="w-4 h-4 rounded" v-model="analogForm.invert">
                                <label class="text-sm">Invert</label>
                            </div>
                        </div>
                    </template>

                    <template v-if="analogForm.actionType === 'modifier'">
                        <div>
                            <label class="block text-sm text-saint-text-muted mb-1">Modifier Effect</label>
                            <SaintSelect v-model="analogForm.modifierType" :options="[
                                { value: 'precision_mode', label: 'Precision Mode' },
                                { value: 'speed_boost',    label: 'Speed Boost' },
                                { value: 'scale_target',   label: 'Scale Target' },
                            ]" />
                        </div>
                        <div v-if="analogForm.modifierType === 'precision_mode'">
                            <label class="block text-sm text-saint-text-muted mb-1">Minimum Scale (at full input)</label>
                            <input type="number" step="0.1" min="0" max="1" class="input w-full"
                                   v-model.number="analogForm.modifierMinScale">
                        </div>
                        <div v-if="analogForm.modifierType === 'speed_boost'">
                            <label class="block text-sm text-saint-text-muted mb-1">Maximum Boost</label>
                            <input type="number" step="0.1" min="1" max="3" class="input w-full"
                                   v-model.number="analogForm.modifierMaxBoost">
                        </div>
                        <div v-if="analogForm.modifierType === 'scale_target'" class="grid grid-cols-2 gap-4">
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Target ID</label>
                                <input type="text" class="input w-full" v-model="analogForm.modifierTargetId">
                            </div>
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Scale</label>
                                <input type="number" step="0.1" class="input w-full"
                                       v-model.number="analogForm.modifierScale">
                            </div>
                        </div>
                    </template>
                </div>
                <div class="flex justify-end gap-3 mt-6 shrink-0">
                    <button class="btn btn-secondary" @click="cancelEdit">Cancel</button>
                    <button class="btn btn-primary" @click="saveAnalogBinding">Save</button>
                </div>
            </div>
        </div>

        <!-- Digital Binding Editor Modal -->
        <div v-if="showDigitalEditor" class="fixed inset-0 bg-black/50 flex items-center justify-center z-50 p-4">
            <div class="card w-full max-w-lg flex flex-col max-h-full">
                <h3 class="text-lg font-semibold mb-4 shrink-0">
                    {{ isNewBinding ? 'Add Digital Binding' : 'Edit Digital Binding' }}
                </h3>
                <div class="space-y-4 overflow-y-auto min-h-0">
                    <div class="grid grid-cols-2 gap-4">
                        <div>
                            <label class="block text-sm text-saint-text-muted mb-1">Input</label>
                            <SaintSelect v-model="digitalForm.input" :options="digitalInputOptions" />
                        </div>
                        <div>
                            <label class="block text-sm text-saint-text-muted mb-1">Trigger</label>
                            <SaintSelect v-model="digitalForm.trigger" :options="buttonTriggerOptions" />
                        </div>
                    </div>
                    <div>
                        <label class="block text-sm text-saint-text-muted mb-1">Action Type</label>
                        <SaintSelect v-model="digitalForm.actionType" :options="digitalActionTypeOptions" />
                    </div>

                    <div v-if="digitalForm.actionType === 'show_panel'">
                        <label class="block text-sm text-saint-text-muted mb-1">Panel</label>
                        <SaintSelect v-model="digitalForm.panelId" :options="panelOptions" />
                    </div>
                    <div v-if="digitalForm.actionType === 'activate_preset'">
                        <label class="block text-sm text-saint-text-muted mb-1">Preset ID</label>
                        <input type="text" class="input w-full" v-model="digitalForm.presetId">
                    </div>
                    <div v-if="digitalForm.actionType === 'navigate_panel'">
                        <label class="block text-sm text-saint-text-muted mb-1">Direction</label>
                        <SaintSelect v-model="digitalForm.direction" :options="navigateDirectionOptions" />
                    </div>
                    <div v-if="digitalForm.actionType === 'toggle_output'">
                        <label class="block text-sm text-saint-text-muted mb-1">Target ID</label>
                        <input type="text" class="input w-full" v-model="digitalForm.targetId">
                    </div>
                    <template v-if="digitalForm.actionType === 'cycle_output'">
                        <div>
                            <label class="block text-sm text-saint-text-muted mb-1">Target ID</label>
                            <input type="text" class="input w-full" v-model="digitalForm.targetId">
                        </div>
                        <div>
                            <label class="block text-sm text-saint-text-muted mb-1">Values (comma-separated)</label>
                            <input type="text" class="input w-full" v-model="digitalForm.cycleValues">
                        </div>
                    </template>
                    <template v-if="digitalForm.actionType === 'direct_control'">
                        <div class="grid grid-cols-2 gap-4">
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Topic</label>
                                <SaintSelect :model-value="digitalForm.topic"
                                             :options="topicOptionsDigital"
                                             @update:model-value="onDigitalTopicChange" />
                            </div>
                            <div>
                                <label class="block text-sm text-saint-text-muted mb-1">Channel</label>
                                <SaintSelect v-model="digitalForm.channel"
                                             :options="channelOptionsDigital"
                                             :disabled="!digitalForm.topic" />
                            </div>
                        </div>
                        <div>
                            <label class="block text-sm text-saint-text-muted mb-1">Value</label>
                            <input type="number" step="0.1" class="input w-full"
                                   v-model.number="digitalForm.value">
                        </div>
                    </template>
                </div>
                <div class="flex justify-end gap-3 mt-6 shrink-0">
                    <button class="btn btn-secondary" @click="cancelEdit">Cancel</button>
                    <button class="btn btn-primary" @click="saveDigitalBinding">Save</button>
                </div>
            </div>
        </div>
    </div>
</template>
