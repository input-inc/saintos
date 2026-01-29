import { Injectable, signal, computed } from '@angular/core';
import { TauriService } from './tauri.service';

// ============================================================================
// Input Sources
// ============================================================================

export type AnalogInput =
  | 'left_stick_x'
  | 'left_stick_y'
  | 'right_stick_x'
  | 'right_stick_y'
  | 'left_trigger'
  | 'right_trigger';

export type DigitalInput =
  | 'a' | 'b' | 'x' | 'y'
  | 'lb' | 'rb'
  | 'd_pad_up' | 'd_pad_down' | 'd_pad_left' | 'd_pad_right'
  | 'start' | 'select'
  | 'left_stick' | 'right_stick';

export type ButtonTrigger = 'press' | 'release' | 'hold' | 'double_tap' | 'long_press';

export type NavigateDirection = 'next_item' | 'prev_item' | 'next_page' | 'prev_page';

// ============================================================================
// Actions
// ============================================================================

export interface ControlTarget {
  nodeId: string;
  pinId: number;
  name?: string;
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

// ============================================================================
// Bindings
// ============================================================================

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

// ============================================================================
// Presets
// ============================================================================

export type PresetType = 'servo' | 'animation' | 'sound';
export type EasingType = 'linear' | 'ease_in' | 'ease_out' | 'ease_in_out';
export type PanelLayout = 'grid' | 'list';

export interface ServoPosition {
  nodeId: string;
  pinId: number;
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
}

// ============================================================================
// Profile
// ============================================================================

export interface ProfileSettings {
  globalDeadzone: number;
  hapticFeedback: boolean;
  doubleTapTimeMs: number;
  longPressTimeMs: number;
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

// ============================================================================
// Action Events (from mapper)
// ============================================================================

export interface MappedCommand {
  node_id: string;
  pin_id: number;
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

// ============================================================================
// Service
// ============================================================================

@Injectable({
  providedIn: 'root'
})
export class BindingsService {
  private profiles = signal<BindingProfile[]>([]);
  private activeProfileId = signal<string>('default');
  private panelState = signal<PanelState>({
    activePanelId: null,
    selectedIndex: 0,
    currentPage: 0
  });

  readonly allProfiles = computed(() => this.profiles());
  readonly activeProfile = computed(() =>
    this.profiles().find(p => p.id === this.activeProfileId()) ?? this.profiles()[0]
  );
  readonly activePanelState = computed(() => this.panelState());
  readonly activePanel = computed(() => {
    const profile = this.activeProfile();
    const state = this.panelState();
    if (!profile || !state.activePanelId) return null;
    return profile.presetPanels.find(p => p.id === state.activePanelId) ?? null;
  });

  constructor(private tauri: TauriService) {
    // Initialize with default profile immediately so bindings work before async load
    this.profiles.set([this.getDefaultProfile()]);
    this.loadProfiles();
    this.subscribeToActionEvents();
  }

  async loadProfiles(): Promise<void> {
    try {
      const profiles = await this.tauri.invoke<BindingProfile[]>('get_binding_profiles');
      // Only update if we got profiles from backend; otherwise keep the default
      if (profiles && profiles.length > 0) {
        this.profiles.set(profiles);
      }
    } catch (err) {
      console.error('Failed to load profiles from backend, using default:', err);
      // Default profile is already set in constructor, so no action needed
    }
  }

  async saveProfiles(): Promise<void> {
    await this.tauri.invoke('set_binding_profiles', { profiles: this.profiles() });
  }

  setActiveProfile(profileId: string): void {
    this.activeProfileId.set(profileId);
    this.tauri.invoke('set_active_profile', { profileId });
  }

  // Panel state management
  showPanel(panelId: string): void {
    console.log('BindingsService.showPanel:', panelId);
    const profile = this.activeProfile();
    const panel = profile?.presetPanels.find(p => p.id === panelId);
    console.log('Panel found:', panel?.name ?? 'NOT FOUND', 'in profile:', profile?.name);
    this.panelState.set({
      activePanelId: panelId,
      selectedIndex: 0,
      currentPage: 0
    });
  }

  hidePanel(): void {
    this.panelState.update(s => ({ ...s, activePanelId: null }));
  }

  navigatePanel(direction: NavigateDirection): void {
    const panel = this.activePanel();
    if (!panel) return;

    this.panelState.update(state => {
      const totalItems = panel.presets.length;
      const itemsPerPage = panel.itemsPerPage;
      const totalPages = Math.ceil(totalItems / itemsPerPage);

      let { selectedIndex, currentPage } = state;

      switch (direction) {
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

      return { ...state, selectedIndex, currentPage };
    });
  }

  selectCurrentItem(): void {
    const panel = this.activePanel();
    const state = this.panelState();
    if (!panel) return;

    const preset = panel.presets[state.selectedIndex];
    if (preset) {
      this.activatePreset(preset.id);
    }
  }

  async activatePreset(presetId: string): Promise<void> {
    await this.tauri.invoke('activate_preset', { presetId });
  }

  // Binding management
  updateAnalogBinding(index: number, binding: AnalogBinding): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        const analogBindings = [...p.analogBindings];
        analogBindings[index] = binding;
        return { ...p, analogBindings };
      });
    });
    this.saveProfiles();
  }

  updateDigitalBinding(index: number, binding: DigitalBinding): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        const digitalBindings = [...p.digitalBindings];
        digitalBindings[index] = binding;
        return { ...p, digitalBindings };
      });
    });
    this.saveProfiles();
  }

  addAnalogBinding(binding: AnalogBinding): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        return { ...p, analogBindings: [...p.analogBindings, binding] };
      });
    });
    this.saveProfiles();
  }

  addDigitalBinding(binding: DigitalBinding): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        return { ...p, digitalBindings: [...p.digitalBindings, binding] };
      });
    });
    this.saveProfiles();
  }

  removeAnalogBinding(index: number): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        const analogBindings = p.analogBindings.filter((_, i) => i !== index);
        return { ...p, analogBindings };
      });
    });
    this.saveProfiles();
  }

  removeDigitalBinding(index: number): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        const digitalBindings = p.digitalBindings.filter((_, i) => i !== index);
        return { ...p, digitalBindings };
      });
    });
    this.saveProfiles();
  }

  // Preset panel management
  addPresetPanel(panel: PresetPanel): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        return { ...p, presetPanels: [...p.presetPanels, panel] };
      });
    });
    this.saveProfiles();
  }

  updatePresetPanel(panelId: string, panel: PresetPanel): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        return {
          ...p,
          presetPanels: p.presetPanels.map(pp => pp.id === panelId ? panel : pp)
        };
      });
    });
    this.saveProfiles();
  }

  removePresetPanel(panelId: string): void {
    this.profiles.update(profiles => {
      return profiles.map(p => {
        if (p.id !== this.activeProfileId()) return p;
        return {
          ...p,
          presetPanels: p.presetPanels.filter(pp => pp.id !== panelId)
        };
      });
    });
    this.saveProfiles();
  }

  private subscribeToActionEvents(): void {
    this.tauri.listen<ActionEvent>('action-event').subscribe(event => {
      this.handleActionEvent(event);
    });
  }

  private handleActionEvent(event: ActionEvent): void {
    switch (event.type) {
      case 'show_panel':
        this.showPanel(event.panel_id);
        break;
      case 'hide_panel':
        this.hidePanel();
        break;
      case 'navigate_panel':
        this.navigatePanel(event.direction);
        break;
      case 'select_panel_item':
        this.selectCurrentItem();
        break;
      case 'activate_preset':
        this.activatePreset(event.preset_id);
        break;
    }
  }

  private getDefaultProfile(): BindingProfile {
    return {
      id: 'default',
      name: 'Default',
      description: 'Standard robot control layout',
      isDefault: true,
      analogBindings: [
        {
          input: 'left_stick_x',
          action: {
            type: 'direct_control',
            target: { nodeId: 'track', pinId: 1, name: 'Track Angular' },
            transform: { deadzone: 0.1, scale: 1.0, expo: 1.0, invert: false }
          },
          enabled: true
        },
        {
          input: 'left_stick_y',
          action: {
            type: 'direct_control',
            target: { nodeId: 'track', pinId: 0, name: 'Track Linear' },
            transform: { deadzone: 0.1, scale: 1.0, expo: 1.0, invert: false }
          },
          enabled: true
        },
        {
          input: 'right_stick_x',
          action: {
            type: 'direct_control',
            target: { nodeId: 'head', pinId: 0, name: 'Head Pan' },
            transform: { deadzone: 0.05, scale: 0.8, expo: 1.0, invert: false }
          },
          enabled: true
        },
        {
          input: 'right_stick_y',
          action: {
            type: 'direct_control',
            target: { nodeId: 'head', pinId: 1, name: 'Head Tilt' },
            transform: { deadzone: 0.05, scale: 0.8, expo: 1.0, invert: false }
          },
          enabled: true
        },
        {
          input: 'left_trigger',
          action: { type: 'modifier', effect: { type: 'precision_mode', min_scale: 0.3 } },
          enabled: true
        },
        {
          input: 'right_trigger',
          action: { type: 'modifier', effect: { type: 'speed_boost', max_boost: 1.5 } },
          enabled: true
        }
      ],
      digitalBindings: [
        { input: 'y', trigger: 'press', action: { type: 'show_panel', panel_id: 'moods' }, enabled: true },
        { input: 'x', trigger: 'press', action: { type: 'show_panel', panel_id: 'animations' }, enabled: true },
        { input: 'a', trigger: 'press', action: { type: 'select_panel_item' }, enabled: true },
        { input: 'b', trigger: 'press', action: { type: 'hide_panel' }, enabled: true },
        { input: 'd_pad_up', trigger: 'press', action: { type: 'navigate_panel', direction: 'prev_item' }, enabled: true },
        { input: 'd_pad_down', trigger: 'press', action: { type: 'navigate_panel', direction: 'next_item' }, enabled: true },
        { input: 'd_pad_left', trigger: 'press', action: { type: 'navigate_panel', direction: 'prev_page' }, enabled: true },
        { input: 'd_pad_right', trigger: 'press', action: { type: 'navigate_panel', direction: 'next_page' }, enabled: true },
        { input: 'lb', trigger: 'press', action: { type: 'cycle_output', target_id: 'speed_mode', values: ['slow', 'normal', 'fast'] }, enabled: true },
        { input: 'rb', trigger: 'press', action: { type: 'toggle_output', target_id: 'head_tracking' }, enabled: true },
        { input: 'select', trigger: 'press', action: { type: 'e_stop' }, enabled: true },
        { input: 'start', trigger: 'press', action: { type: 'show_panel', panel_id: 'sounds' }, enabled: true }
      ],
      presetPanels: [
        {
          id: 'moods',
          name: 'Moods',
          icon: 'mood',
          color: '#f59e0b',
          layout: 'grid',
          columns: 4,
          itemsPerPage: 8,
          presets: [
            { id: 'mood_happy', name: 'Happy', icon: 'sentiment_very_satisfied', color: '#22c55e', type: 'servo', data: { type: 'servo', positions: [], transitionMs: 500, easing: 'ease_in_out' } },
            { id: 'mood_sad', name: 'Sad', icon: 'sentiment_dissatisfied', color: '#3b82f6', type: 'servo', data: { type: 'servo', positions: [], transitionMs: 500, easing: 'ease_in_out' } },
            { id: 'mood_angry', name: 'Angry', icon: 'mood_bad', color: '#ef4444', type: 'servo', data: { type: 'servo', positions: [], transitionMs: 500, easing: 'ease_in_out' } },
            { id: 'mood_curious', name: 'Curious', icon: 'psychology', color: '#8b5cf6', type: 'servo', data: { type: 'servo', positions: [], transitionMs: 500, easing: 'ease_in_out' } },
            { id: 'mood_sleepy', name: 'Sleepy', icon: 'bedtime', color: '#6b7280', type: 'servo', data: { type: 'servo', positions: [], transitionMs: 500, easing: 'ease_in_out' } },
            { id: 'mood_surprised', name: 'Surprised', icon: 'sentiment_excited', color: '#f59e0b', type: 'servo', data: { type: 'servo', positions: [], transitionMs: 500, easing: 'ease_in_out' } },
            { id: 'mood_love', name: 'Love', icon: 'favorite', color: '#ec4899', type: 'servo', data: { type: 'servo', positions: [], transitionMs: 500, easing: 'ease_in_out' } },
            { id: 'mood_neutral', name: 'Neutral', icon: 'sentiment_neutral', color: '#9ca3af', type: 'servo', data: { type: 'servo', positions: [], transitionMs: 500, easing: 'ease_in_out' } }
          ]
        },
        {
          id: 'animations',
          name: 'Animations',
          icon: 'animation',
          color: '#8b5cf6',
          layout: 'grid',
          columns: 4,
          itemsPerPage: 8,
          presets: [
            { id: 'anim_wave', name: 'Wave', icon: 'waving_hand', type: 'animation', data: { type: 'animation', keyframes: [], loop_animation: false } },
            { id: 'anim_nod', name: 'Nod', icon: 'check_circle', type: 'animation', data: { type: 'animation', keyframes: [], loop_animation: false } },
            { id: 'anim_shake', name: 'Shake Head', icon: 'cancel', type: 'animation', data: { type: 'animation', keyframes: [], loop_animation: false } },
            { id: 'anim_dance', name: 'Dance', icon: 'music_note', type: 'animation', data: { type: 'animation', keyframes: [], loop_animation: false } },
            { id: 'anim_look_around', name: 'Look Around', icon: 'visibility', type: 'animation', data: { type: 'animation', keyframes: [], loop_animation: false } },
            { id: 'anim_bow', name: 'Bow', icon: 'arrow_downward', type: 'animation', data: { type: 'animation', keyframes: [], loop_animation: false } }
          ]
        },
        {
          id: 'sounds',
          name: 'Sounds',
          icon: 'volume_up',
          color: '#22c55e',
          layout: 'grid',
          columns: 4,
          itemsPerPage: 8,
          presets: [
            { id: 'sound_hello', name: 'Hello', icon: 'record_voice_over', type: 'sound', data: { type: 'sound', soundId: 'hello.wav', volume: 1.0, priority: 1 } },
            { id: 'sound_goodbye', name: 'Goodbye', icon: 'waving_hand', type: 'sound', data: { type: 'sound', soundId: 'goodbye.wav', volume: 1.0, priority: 1 } },
            { id: 'sound_yes', name: 'Yes', icon: 'thumb_up', type: 'sound', data: { type: 'sound', soundId: 'yes.wav', volume: 1.0, priority: 1 } },
            { id: 'sound_no', name: 'No', icon: 'thumb_down', type: 'sound', data: { type: 'sound', soundId: 'no.wav', volume: 1.0, priority: 1 } },
            { id: 'sound_laugh', name: 'Laugh', icon: 'sentiment_very_satisfied', type: 'sound', data: { type: 'sound', soundId: 'laugh.wav', volume: 1.0, priority: 1 } },
            { id: 'sound_alert', name: 'Alert', icon: 'notifications', type: 'sound', data: { type: 'sound', soundId: 'alert.wav', volume: 1.0, priority: 1 } }
          ]
        }
      ],
      settings: {
        globalDeadzone: 0.1,
        hapticFeedback: true,
        doubleTapTimeMs: 300,
        longPressTimeMs: 500
      }
    };
  }
}
