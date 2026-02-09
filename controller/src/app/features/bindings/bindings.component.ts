import { Component, signal, computed, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import {
  BindingsService,
  AnalogBinding,
  DigitalBinding,
  AnalogInput,
  DigitalInput,
  ButtonTrigger,
  DigitalAction,
  AnalogAction,
  PresetPanel,
  NavigateDirection,
  ControlTarget,
  InputTransform,
  ModifierEffect
} from '../../core/services/bindings.service';
import { DiscoveryService } from '../../core/services/discovery.service';

type TabId = 'analog' | 'digital' | 'panels' | 'settings';

const ANALOG_INPUTS: { value: AnalogInput; label: string }[] = [
  { value: 'left_stick_x', label: 'Left Stick X' },
  { value: 'left_stick_y', label: 'Left Stick Y' },
  { value: 'right_stick_x', label: 'Right Stick X' },
  { value: 'right_stick_y', label: 'Right Stick Y' },
  { value: 'left_trigger', label: 'Left Trigger' },
  { value: 'right_trigger', label: 'Right Trigger' }
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
  { value: 'right_stick', label: 'Right Stick Press' }
];

const BUTTON_TRIGGERS: { value: ButtonTrigger; label: string }[] = [
  { value: 'press', label: 'Press' },
  { value: 'release', label: 'Release' },
  { value: 'hold', label: 'Hold' },
  { value: 'double_tap', label: 'Double Tap' },
  { value: 'long_press', label: 'Long Press' }
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
  { value: 'none', label: 'None' }
];

const NAVIGATE_DIRECTIONS: { value: NavigateDirection; label: string }[] = [
  { value: 'up', label: 'Up (Grid)' },
  { value: 'down', label: 'Down (Grid)' },
  { value: 'left', label: 'Left (Grid)' },
  { value: 'right', label: 'Right (Grid)' },
  { value: 'next_item', label: 'Next Item (Linear)' },
  { value: 'prev_item', label: 'Previous Item (Linear)' },
  { value: 'next_page', label: 'Next Page' },
  { value: 'prev_page', label: 'Previous Page' }
];

@Component({
  selector: 'app-bindings',
  standalone: true,
  imports: [CommonModule, FormsModule],
  template: `
    <div class="p-6 space-y-6">
      <!-- Profile Selector -->
      <div class="card">
        <div class="flex items-center justify-between">
          <h2 class="text-lg font-semibold">Binding Profile</h2>
          <select class="input"
                  [ngModel]="bindingsService.activeProfile()?.id"
                  (ngModelChange)="bindingsService.setActiveProfile($event)">
            @for (profile of bindingsService.allProfiles(); track profile.id) {
              <option [value]="profile.id">{{ profile.name }}</option>
            }
          </select>
        </div>
        @if (profileDescription()) {
          <p class="text-sm text-saint-text-muted mt-2">{{ profileDescription() }}</p>
        }
      </div>

      <!-- Tabs -->
      <div class="flex border-b border-saint-border">
        @for (tab of tabs; track tab.id) {
          <button class="px-4 py-2 text-sm font-medium transition-colors border-b-2 -mb-px"
                  [class.border-saint-accent]="activeTab() === tab.id"
                  [class.text-saint-accent]="activeTab() === tab.id"
                  [class.border-transparent]="activeTab() !== tab.id"
                  [class.text-saint-text-muted]="activeTab() !== tab.id"
                  (click)="activeTab.set(tab.id)">
            <span class="material-symbols-outlined text-lg align-middle mr-1">{{ tab.icon }}</span>
            {{ tab.label }}
          </button>
        }
      </div>

      <!-- Tab Content -->
      @switch (activeTab()) {
        @case ('analog') {
          <div class="space-y-4">
            <div class="flex justify-between items-center">
              <h3 class="text-lg font-medium">Analog Bindings</h3>
              <button class="btn btn-primary text-sm" (click)="addAnalogBinding()">
                + Add Binding
              </button>
            </div>

            @for (binding of analogBindings(); track $index; let i = $index) {
              <div class="card">
                <div class="flex items-start justify-between gap-4">
                  <div class="flex items-center gap-3">
                    <input type="checkbox"
                           [checked]="binding.enabled"
                           (change)="toggleAnalogBinding(i, binding)"
                           class="w-4 h-4 rounded">
                    <div>
                      <div class="font-medium">{{ getAnalogInputLabel(binding.input) }}</div>
                      <div class="text-sm text-saint-text-muted">
                        {{ formatAnalogAction(binding.action) }}
                      </div>
                    </div>
                  </div>
                  <div class="flex gap-2">
                    <button class="btn btn-secondary text-sm" (click)="editAnalogBinding(i, binding)">
                      Edit
                    </button>
                    <button class="btn btn-danger text-sm" (click)="removeAnalogBinding(i)">
                      Delete
                    </button>
                  </div>
                </div>

                @if (binding.action.type === 'direct_control') {
                  <div class="mt-3 pt-3 border-t border-saint-border grid grid-cols-4 gap-4 text-sm">
                    <div>
                      <span class="text-saint-text-muted">Target:</span>
                      <span class="ml-1">{{ binding.action.target.name || (binding.action.target.role + ':' + binding.action.target.function) }}</span>
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
                }
              </div>
            } @empty {
              <p class="text-saint-text-muted text-center py-8">No analog bindings configured</p>
            }
          </div>
        }

        @case ('digital') {
          <div class="space-y-4">
            <div class="flex justify-between items-center">
              <h3 class="text-lg font-medium">Digital Bindings</h3>
              <button class="btn btn-primary text-sm" (click)="addDigitalBinding()">
                + Add Binding
              </button>
            </div>

            @for (binding of digitalBindings(); track $index; let i = $index) {
              <div class="card">
                <div class="flex items-start justify-between gap-4">
                  <div class="flex items-center gap-3">
                    <input type="checkbox"
                           [checked]="binding.enabled"
                           (change)="toggleDigitalBinding(i, binding)"
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
                    <button class="btn btn-secondary text-sm" (click)="editDigitalBinding(i, binding)">
                      Edit
                    </button>
                    <button class="btn btn-danger text-sm" (click)="removeDigitalBinding(i)">
                      Delete
                    </button>
                  </div>
                </div>
              </div>
            } @empty {
              <p class="text-saint-text-muted text-center py-8">No digital bindings configured</p>
            }
          </div>
        }

        @case ('panels') {
          <div class="space-y-4">
            <div class="flex justify-between items-center">
              <h3 class="text-lg font-medium">Preset Panels</h3>
              <button class="btn btn-primary text-sm" (click)="addPresetPanel()">
                + Add Panel
              </button>
            </div>

            <div class="grid grid-cols-2 lg:grid-cols-3 gap-4">
              @for (panel of presetPanels(); track panel.id) {
                <div class="card cursor-pointer hover:border-saint-accent transition-colors"
                     (click)="editPresetPanel(panel)">
                  <div class="flex items-center gap-3 mb-3">
                    <span class="material-symbols-outlined text-2xl" [style.color]="panel.color">
                      {{ panel.icon }}
                    </span>
                    <div>
                      <div class="font-medium">{{ panel.name }}</div>
                      <div class="text-sm text-saint-text-muted">
                        {{ panel.presets.length }} presets
                      </div>
                    </div>
                  </div>
                  <div class="flex flex-wrap gap-1">
                    @for (preset of panel.presets.slice(0, 4); track preset.id) {
                      <span class="inline-flex items-center gap-1 px-2 py-1 rounded-full text-xs
                                   bg-saint-surface-light">
                        <span class="material-symbols-outlined text-sm" [style.color]="preset.color || panel.color">
                          {{ preset.icon }}
                        </span>
                        {{ preset.name }}
                      </span>
                    }
                    @if (panel.presets.length > 4) {
                      <span class="px-2 py-1 rounded-full text-xs bg-saint-surface-light text-saint-text-muted">
                        +{{ panel.presets.length - 4 }} more
                      </span>
                    }
                  </div>
                </div>
              } @empty {
                <p class="text-saint-text-muted text-center py-8 col-span-full">No preset panels configured</p>
              }
            </div>
          </div>
        }

        @case ('settings') {
          <div class="space-y-4">
            <h3 class="text-lg font-medium">Profile Settings</h3>
            @if (profileSettings()) {
              <div class="card space-y-6">
                <div class="grid grid-cols-2 gap-4">
                  <div class="col-span-2">
                    <span class="block text-sm font-medium text-saint-text mb-2">Panel Activation Mode</span>
                    <div class="flex gap-2">
                      <button type="button" class="btn" [class.btn-primary]="profileSettings()?.panelActivation === 'press'" [class.btn-secondary]="profileSettings()?.panelActivation !== 'press'" (click)="updatePanelActivation('press')">Press (stays open)</button>
                      <button type="button" class="btn" [class.btn-primary]="profileSettings()?.panelActivation === 'hold'" [class.btn-secondary]="profileSettings()?.panelActivation !== 'hold'" (click)="updatePanelActivation('hold')">Hold (release to close)</button>
                    </div>
                  </div>
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Global Deadzone</label>
                    <input type="number" step="0.01" min="0" max="1" class="input w-full"
                           [ngModel]="profileSettings()?.globalDeadzone">
                  </div>
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Double Tap Time (ms)</label>
                    <input type="number" step="50" min="100" max="500" class="input w-full"
                           [ngModel]="profileSettings()?.doubleTapTimeMs">
                  </div>
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Long Press Time (ms)</label>
                    <input type="number" step="50" min="200" max="1000" class="input w-full"
                           [ngModel]="profileSettings()?.longPressTimeMs">
                  </div>
                  <div class="flex items-center gap-2 pt-6">
                    <input type="checkbox" class="w-4 h-4 rounded"
                           [ngModel]="profileSettings()?.hapticFeedback">
                    <label class="text-sm">Haptic Feedback</label>
                  </div>
                </div>
              </div>
            }
          </div>
        }
      }

      <!-- Analog Binding Editor Modal -->
      @if (showAnalogEditor()) {
        <div class="fixed inset-0 bg-black/50 flex items-center justify-center z-50 p-4">
          <div class="card w-full max-w-lg flex flex-col max-h-full">
            <h3 class="text-lg font-semibold mb-4 shrink-0">
              {{ isNewBinding() ? 'Add Analog Binding' : 'Edit Analog Binding' }}
            </h3>

            <div class="space-y-4 overflow-y-auto min-h-0">
              <div>
                <label class="block text-sm text-saint-text-muted mb-1">Input</label>
                <select class="input w-full" [(ngModel)]="analogForm.input">
                  @for (opt of analogInputs; track opt.value) {
                    <option [value]="opt.value">{{ opt.label }}</option>
                  }
                </select>
              </div>

              <div>
                <label class="block text-sm text-saint-text-muted mb-1">Action Type</label>
                <select class="input w-full" [(ngModel)]="analogForm.actionType">
                  <option value="direct_control">Direct Control</option>
                  <option value="modifier">Modifier</option>
                </select>
              </div>

              @if (analogForm.actionType === 'direct_control') {
                <div class="grid grid-cols-2 gap-4">
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Role</label>
                    <select class="input w-full" [(ngModel)]="analogForm.targetRole"
                            (ngModelChange)="onRoleChange()">
                      <option value="">-- Select Role --</option>
                      @for (role of availableRoles(); track role) {
                        <option [value]="role">{{ role }}</option>
                      }
                    </select>
                  </div>
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Function</label>
                    <select class="input w-full" [(ngModel)]="analogForm.targetFunction"
                            [disabled]="!analogForm.targetRole">
                      <option value="">-- Select Function --</option>
                      @for (fn of availableFunctionsForSelectedRole(); track fn) {
                        <option [value]="fn">{{ fn }}</option>
                      }
                    </select>
                  </div>
                </div>
                <div>
                  <label class="block text-sm text-saint-text-muted mb-1">Display Name</label>
                  <input type="text" class="input w-full" [(ngModel)]="analogForm.targetName">
                </div>
                <div class="grid grid-cols-2 gap-4">
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Deadzone</label>
                    <input type="number" step="0.01" min="0" max="1" class="input w-full"
                           [(ngModel)]="analogForm.deadzone">
                  </div>
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Scale</label>
                    <input type="number" step="0.1" class="input w-full" [(ngModel)]="analogForm.scale">
                  </div>
                </div>
                <div class="grid grid-cols-2 gap-4">
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Expo Curve</label>
                    <input type="number" step="0.1" min="1" max="3" class="input w-full"
                           [(ngModel)]="analogForm.expo">
                  </div>
                  <div class="flex items-center gap-2 pt-6">
                    <input type="checkbox" class="w-4 h-4 rounded" [(ngModel)]="analogForm.invert">
                    <label class="text-sm">Invert</label>
                  </div>
                </div>
              }

              @if (analogForm.actionType === 'modifier') {
                <div>
                  <label class="block text-sm text-saint-text-muted mb-1">Modifier Effect</label>
                  <select class="input w-full" [(ngModel)]="analogForm.modifierType">
                    <option value="precision_mode">Precision Mode</option>
                    <option value="speed_boost">Speed Boost</option>
                    <option value="scale_target">Scale Target</option>
                  </select>
                </div>

                @if (analogForm.modifierType === 'precision_mode') {
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Minimum Scale (at full input)</label>
                    <input type="number" step="0.1" min="0" max="1" class="input w-full"
                           [(ngModel)]="analogForm.modifierMinScale">
                  </div>
                }

                @if (analogForm.modifierType === 'speed_boost') {
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Maximum Boost</label>
                    <input type="number" step="0.1" min="1" max="3" class="input w-full"
                           [(ngModel)]="analogForm.modifierMaxBoost">
                  </div>
                }

                @if (analogForm.modifierType === 'scale_target') {
                  <div class="grid grid-cols-2 gap-4">
                    <div>
                      <label class="block text-sm text-saint-text-muted mb-1">Target ID</label>
                      <input type="text" class="input w-full" [(ngModel)]="analogForm.modifierTargetId">
                    </div>
                    <div>
                      <label class="block text-sm text-saint-text-muted mb-1">Scale</label>
                      <input type="number" step="0.1" class="input w-full" [(ngModel)]="analogForm.modifierScale">
                    </div>
                  </div>
                }
              }
            </div>

            <div class="flex justify-end gap-3 mt-6 shrink-0">
              <button class="btn btn-secondary" (click)="cancelEdit()">Cancel</button>
              <button class="btn btn-primary" (click)="saveAnalogBinding()">Save</button>
            </div>
          </div>
        </div>
      }

      <!-- Digital Binding Editor Modal -->
      @if (showDigitalEditor()) {
        <div class="fixed inset-0 bg-black/50 flex items-center justify-center z-50 p-4">
          <div class="card w-full max-w-lg flex flex-col max-h-full">
            <h3 class="text-lg font-semibold mb-4 shrink-0">
              {{ isNewBinding() ? 'Add Digital Binding' : 'Edit Digital Binding' }}
            </h3>

            <div class="space-y-4 overflow-y-auto min-h-0">
              <div class="grid grid-cols-2 gap-4">
                <div>
                  <label class="block text-sm text-saint-text-muted mb-1">Input</label>
                  <select class="input w-full" [(ngModel)]="digitalForm.input">
                    @for (opt of digitalInputs; track opt.value) {
                      <option [value]="opt.value">{{ opt.label }}</option>
                    }
                  </select>
                </div>
                <div>
                  <label class="block text-sm text-saint-text-muted mb-1">Trigger</label>
                  <select class="input w-full" [(ngModel)]="digitalForm.trigger">
                    @for (opt of buttonTriggers; track opt.value) {
                      <option [value]="opt.value">{{ opt.label }}</option>
                    }
                  </select>
                </div>
              </div>

              <div>
                <label class="block text-sm text-saint-text-muted mb-1">Action Type</label>
                <select class="input w-full" [(ngModel)]="digitalForm.actionType">
                  @for (opt of digitalActionTypes; track opt.value) {
                    <option [value]="opt.value">{{ opt.label }}</option>
                  }
                </select>
              </div>

              <!-- Action-specific fields -->
              @switch (digitalForm.actionType) {
                @case ('show_panel') {
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Panel</label>
                    <select class="input w-full" [(ngModel)]="digitalForm.panelId">
                      @for (panel of presetPanels(); track panel.id) {
                        <option [value]="panel.id">{{ panel.name }}</option>
                      }
                    </select>
                  </div>
                }
                @case ('activate_preset') {
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Preset ID</label>
                    <input type="text" class="input w-full" [(ngModel)]="digitalForm.presetId">
                  </div>
                }
                @case ('navigate_panel') {
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Direction</label>
                    <select class="input w-full" [(ngModel)]="digitalForm.direction">
                      @for (opt of navigateDirections; track opt.value) {
                        <option [value]="opt.value">{{ opt.label }}</option>
                      }
                    </select>
                  </div>
                }
                @case ('toggle_output') {
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Target ID</label>
                    <input type="text" class="input w-full" [(ngModel)]="digitalForm.targetId">
                  </div>
                }
                @case ('cycle_output') {
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Target ID</label>
                    <input type="text" class="input w-full" [(ngModel)]="digitalForm.targetId">
                  </div>
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Values (comma-separated)</label>
                    <input type="text" class="input w-full" [(ngModel)]="digitalForm.cycleValues">
                  </div>
                }
                @case ('direct_control') {
                  <div class="grid grid-cols-2 gap-4">
                    <div>
                      <label class="block text-sm text-saint-text-muted mb-1">Role</label>
                      <select class="input w-full" [(ngModel)]="digitalForm.role"
                              (ngModelChange)="onDigitalRoleChange()">
                        <option value="">-- Select Role --</option>
                        @for (role of availableRoles(); track role) {
                          <option [value]="role">{{ role }}</option>
                        }
                      </select>
                    </div>
                    <div>
                      <label class="block text-sm text-saint-text-muted mb-1">Function</label>
                      <select class="input w-full" [(ngModel)]="digitalForm.function"
                              [disabled]="!digitalForm.role">
                        <option value="">-- Select Function --</option>
                        @for (fn of availableFunctionsForDigitalRole(); track fn) {
                          <option [value]="fn">{{ fn }}</option>
                        }
                      </select>
                    </div>
                  </div>
                  <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Value</label>
                    <input type="number" step="0.1" class="input w-full" [(ngModel)]="digitalForm.value">
                  </div>
                }
              }
            </div>

            <div class="flex justify-end gap-3 mt-6 shrink-0">
              <button class="btn btn-secondary" (click)="cancelEdit()">Cancel</button>
              <button class="btn btn-primary" (click)="saveDigitalBinding()">Save</button>
            </div>
          </div>
        </div>
      }
    </div>
  `
})
export class BindingsComponent {
  readonly tabs: { id: TabId; label: string; icon: string }[] = [
    { id: 'analog', label: 'Analog', icon: 'joystick' },
    { id: 'digital', label: 'Buttons', icon: 'gamepad' },
    { id: 'panels', label: 'Preset Panels', icon: 'dashboard' },
    { id: 'settings', label: 'Settings', icon: 'settings' }
  ];

  activeTab = signal<TabId>('analog');
  isNewBinding = signal(false);
  showAnalogEditor = signal(false);
  showDigitalEditor = signal(false);
  editingAnalogIndex = -1;
  editingDigitalIndex = -1;

  // Form state for analog binding editor
  analogForm = {
    input: 'left_stick_x' as AnalogInput,
    actionType: 'direct_control' as 'direct_control' | 'modifier',
    targetRole: '',
    targetFunction: '',
    targetName: '',
    deadzone: 0.1,
    scale: 1.0,
    expo: 1.0,
    invert: false,
    modifierType: 'precision_mode' as 'precision_mode' | 'speed_boost' | 'scale_target',
    modifierMinScale: 0.3,
    modifierMaxBoost: 1.5,
    modifierTargetId: '',
    modifierScale: 1.0
  };

  // Form state for digital binding editor
  digitalForm = {
    input: 'a' as DigitalInput,
    trigger: 'press' as ButtonTrigger,
    actionType: 'show_panel',
    panelId: '',
    presetId: '',
    direction: 'next_item' as NavigateDirection,
    targetId: '',
    cycleValues: '',
    role: '',
    function: '',
    value: 1.0
  };

  // Reference data
  readonly analogInputs = ANALOG_INPUTS;
  readonly digitalInputs = DIGITAL_INPUTS;
  readonly buttonTriggers = BUTTON_TRIGGERS;
  readonly digitalActionTypes = DIGITAL_ACTION_TYPES;
  readonly navigateDirections = NAVIGATE_DIRECTIONS;

  // Discovery-based role/function lists
  readonly availableRoles = computed(() => this.discoveryService.activeRoles());

  constructor(
    public bindingsService: BindingsService,
    public discoveryService: DiscoveryService
  ) {
    // Refresh discovery data when component initializes
    this.discoveryService.refresh();
  }

  // Get functions available for the currently selected role in analog form
  availableFunctionsForSelectedRole(): string[] {
    if (!this.analogForm.targetRole) return [];
    return this.discoveryService.getFunctionsForRole(this.analogForm.targetRole);
  }

  // Get functions available for the currently selected role in digital form
  availableFunctionsForDigitalRole(): string[] {
    if (!this.digitalForm.role) return [];
    return this.discoveryService.getFunctionsForRole(this.digitalForm.role);
  }

  // When analog role changes, reset function selection
  onRoleChange(): void {
    this.analogForm.targetFunction = '';
  }

  // When digital role changes, reset function selection
  onDigitalRoleChange(): void {
    this.digitalForm.function = '';
  }

  // Computed helpers
  profileDescription(): string | undefined {
    return this.bindingsService.activeProfile()?.description;
  }

  profileSettings() {
    return this.bindingsService.activeProfile()?.settings;
  }

  analogBindings(): AnalogBinding[] {
    return this.bindingsService.activeProfile()?.analogBindings ?? [];
  }

  digitalBindings(): DigitalBinding[] {
    return this.bindingsService.activeProfile()?.digitalBindings ?? [];
  }

  presetPanels(): PresetPanel[] {
    return this.bindingsService.activeProfile()?.presetPanels ?? [];
  }

  getAnalogInputLabel(input: AnalogInput): string {
    return ANALOG_INPUTS.find(i => i.value === input)?.label || input;
  }

  getDigitalInputLabel(input: DigitalInput): string {
    return DIGITAL_INPUTS.find(i => i.value === input)?.label || input;
  }

  formatAnalogAction(action: AnalogAction): string {
    if (action.type === 'direct_control') {
      return `Direct Control → ${action.target.name || `${action.target.role}:${action.target.function}`}`;
    }
    if (action.type === 'modifier') {
      switch (action.effect.type) {
        case 'precision_mode': return `Precision Mode (min: ${action.effect.min_scale})`;
        case 'speed_boost': return `Speed Boost (max: ${action.effect.max_boost})`;
        case 'scale_target': return `Scale ${action.effect.target_id}`;
      }
    }
    return 'Unknown';
  }

  formatDigitalAction(action: DigitalAction): string {
    switch (action.type) {
      case 'show_panel': return `Show Panel: ${action.panel_id}`;
      case 'hide_panel': return 'Hide Panel';
      case 'activate_preset': return `Activate Preset: ${action.preset_id}`;
      case 'navigate_panel': return `Navigate: ${action.direction}`;
      case 'select_panel_item': return 'Select Panel Item';
      case 'toggle_output': return `Toggle: ${action.target_id}`;
      case 'cycle_output': return `Cycle: ${action.target_id} (${action.values.join(', ')})`;
      case 'direct_control': return `Direct Control → ${action.target.role}:${action.target.function} = ${action.value}`;
      case 'e_stop': return 'Emergency Stop';
      case 'none': return 'None';
    }
  }

  // Analog binding methods
  toggleAnalogBinding(index: number, binding: AnalogBinding): void {
    this.bindingsService.updateAnalogBinding(index, { ...binding, enabled: !binding.enabled });
  }

  addAnalogBinding(): void {
    this.isNewBinding.set(true);
    this.editingAnalogIndex = -1;
    this.resetAnalogForm();
    this.showAnalogEditor.set(true);
  }

  editAnalogBinding(index: number, binding: AnalogBinding): void {
    this.isNewBinding.set(false);
    this.editingAnalogIndex = index;
    this.loadAnalogForm(binding);
    this.showAnalogEditor.set(true);
  }

  removeAnalogBinding(index: number): void {
    this.bindingsService.removeAnalogBinding(index);
  }

  resetAnalogForm(): void {
    this.analogForm = {
      input: 'left_stick_x',
      actionType: 'direct_control',
      targetRole: '',
      targetFunction: '',
      targetName: '',
      deadzone: 0.1,
      scale: 1.0,
      expo: 1.0,
      invert: false,
      modifierType: 'precision_mode',
      modifierMinScale: 0.3,
      modifierMaxBoost: 1.5,
      modifierTargetId: '',
      modifierScale: 1.0
    };
  }

  loadAnalogForm(binding: AnalogBinding): void {
    this.analogForm.input = binding.input;
    this.analogForm.actionType = binding.action.type;

    if (binding.action.type === 'direct_control') {
      this.analogForm.targetRole = binding.action.target.role;
      this.analogForm.targetFunction = binding.action.target.function;
      this.analogForm.targetName = binding.action.target.name || '';
      this.analogForm.deadzone = binding.action.transform.deadzone;
      this.analogForm.scale = binding.action.transform.scale;
      this.analogForm.expo = binding.action.transform.expo;
      this.analogForm.invert = binding.action.transform.invert;
    } else if (binding.action.type === 'modifier') {
      this.analogForm.modifierType = binding.action.effect.type;
      if (binding.action.effect.type === 'precision_mode') {
        this.analogForm.modifierMinScale = binding.action.effect.min_scale;
      } else if (binding.action.effect.type === 'speed_boost') {
        this.analogForm.modifierMaxBoost = binding.action.effect.max_boost;
      } else if (binding.action.effect.type === 'scale_target') {
        this.analogForm.modifierTargetId = binding.action.effect.target_id;
        this.analogForm.modifierScale = binding.action.effect.scale;
      }
    }
  }

  saveAnalogBinding(): void {
    let action: AnalogAction;

    if (this.analogForm.actionType === 'direct_control') {
      action = {
        type: 'direct_control',
        target: {
          role: this.analogForm.targetRole,
          function: this.analogForm.targetFunction,
          name: this.analogForm.targetName || undefined
        },
        transform: {
          deadzone: this.analogForm.deadzone,
          scale: this.analogForm.scale,
          expo: this.analogForm.expo,
          invert: this.analogForm.invert
        }
      };
    } else {
      let effect: ModifierEffect;
      switch (this.analogForm.modifierType) {
        case 'precision_mode':
          effect = { type: 'precision_mode', min_scale: this.analogForm.modifierMinScale };
          break;
        case 'speed_boost':
          effect = { type: 'speed_boost', max_boost: this.analogForm.modifierMaxBoost };
          break;
        case 'scale_target':
          effect = { type: 'scale_target', target_id: this.analogForm.modifierTargetId, scale: this.analogForm.modifierScale };
          break;
      }
      action = { type: 'modifier', effect };
    }

    const binding: AnalogBinding = {
      input: this.analogForm.input,
      action,
      enabled: true
    };

    if (this.isNewBinding()) {
      this.bindingsService.addAnalogBinding(binding);
    } else {
      this.bindingsService.updateAnalogBinding(this.editingAnalogIndex, binding);
    }
    this.cancelEdit();
  }

  // Digital binding methods
  toggleDigitalBinding(index: number, binding: DigitalBinding): void {
    this.bindingsService.updateDigitalBinding(index, { ...binding, enabled: !binding.enabled });
  }

  addDigitalBinding(): void {
    this.isNewBinding.set(true);
    this.editingDigitalIndex = -1;
    this.resetDigitalForm();
    this.showDigitalEditor.set(true);
  }

  editDigitalBinding(index: number, binding: DigitalBinding): void {
    this.isNewBinding.set(false);
    this.editingDigitalIndex = index;
    this.loadDigitalForm(binding);
    this.showDigitalEditor.set(true);
  }

  removeDigitalBinding(index: number): void {
    this.bindingsService.removeDigitalBinding(index);
  }

  resetDigitalForm(): void {
    const panels = this.presetPanels();
    this.digitalForm = {
      input: 'a',
      trigger: 'press',
      actionType: 'show_panel',
      panelId: panels[0]?.id || '',
      presetId: '',
      direction: 'next_item',
      targetId: '',
      cycleValues: '',
      role: '',
      function: '',
      value: 1.0
    };
  }

  loadDigitalForm(binding: DigitalBinding): void {
    this.digitalForm.input = binding.input;
    this.digitalForm.trigger = binding.trigger;
    this.digitalForm.actionType = binding.action.type;

    switch (binding.action.type) {
      case 'show_panel':
        this.digitalForm.panelId = binding.action.panel_id;
        break;
      case 'activate_preset':
        this.digitalForm.presetId = binding.action.preset_id;
        break;
      case 'navigate_panel':
        this.digitalForm.direction = binding.action.direction;
        break;
      case 'toggle_output':
        this.digitalForm.targetId = binding.action.target_id;
        break;
      case 'cycle_output':
        this.digitalForm.targetId = binding.action.target_id;
        this.digitalForm.cycleValues = binding.action.values.join(', ');
        break;
      case 'direct_control':
        this.digitalForm.role = binding.action.target.role;
        this.digitalForm.function = binding.action.target.function;
        this.digitalForm.value = binding.action.value;
        break;
    }
  }

  saveDigitalBinding(): void {
    let action: DigitalAction;

    switch (this.digitalForm.actionType) {
      case 'show_panel':
        action = { type: 'show_panel', panel_id: this.digitalForm.panelId };
        break;
      case 'hide_panel':
        action = { type: 'hide_panel' };
        break;
      case 'activate_preset':
        action = { type: 'activate_preset', preset_id: this.digitalForm.presetId };
        break;
      case 'navigate_panel':
        action = { type: 'navigate_panel', direction: this.digitalForm.direction };
        break;
      case 'select_panel_item':
        action = { type: 'select_panel_item' };
        break;
      case 'toggle_output':
        action = { type: 'toggle_output', target_id: this.digitalForm.targetId };
        break;
      case 'cycle_output':
        action = {
          type: 'cycle_output',
          target_id: this.digitalForm.targetId,
          values: this.digitalForm.cycleValues.split(',').map(v => v.trim()).filter(v => v)
        };
        break;
      case 'direct_control':
        action = {
          type: 'direct_control',
          target: { role: this.digitalForm.role, function: this.digitalForm.function },
          value: this.digitalForm.value
        };
        break;
      case 'e_stop':
        action = { type: 'e_stop' };
        break;
      default:
        action = { type: 'none' };
    }

    const binding: DigitalBinding = {
      input: this.digitalForm.input,
      trigger: this.digitalForm.trigger,
      action,
      enabled: true
    };

    if (this.isNewBinding()) {
      this.bindingsService.addDigitalBinding(binding);
    } else {
      this.bindingsService.updateDigitalBinding(this.editingDigitalIndex, binding);
    }
    this.cancelEdit();
  }

  // Preset panel methods
  addPresetPanel(): void {
    const panel: PresetPanel = {
      id: `panel_${Date.now()}`,
      name: 'New Panel',
      icon: 'folder',
      color: '#8b5cf6',
      presets: [],
      layout: 'grid',
      columns: 4,
      itemsPerPage: 8
    };
    this.bindingsService.addPresetPanel(panel);
  }

  editPresetPanel(panel: PresetPanel): void {
    // TODO: Open preset panel editor modal
    console.log('Edit panel:', panel);
  }

  updatePanelActivation(mode: 'press' | 'hold'): void {
    const profile = this.bindingsService.activeProfile();
    if (!profile) return;

    const updatedSettings = { ...profile.settings, panelActivation: mode };

    // Update through binding service - we need to add this method
    this.bindingsService.updateProfileSettings(updatedSettings);
  }

  cancelEdit(): void {
    this.showAnalogEditor.set(false);
    this.showDigitalEditor.set(false);
    this.isNewBinding.set(false);
  }
}
