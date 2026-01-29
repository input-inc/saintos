import { Component, signal, computed } from '@angular/core';
import { RouterOutlet, RouterLink, RouterLinkActive } from '@angular/router';
import { CommonModule } from '@angular/common';
import { ConnectionService, ConnectionStatus } from './core/services/connection.service';
import { InputService } from './core/services/input.service';
import { BindingsService, DigitalInput } from './core/services/bindings.service';
import { VirtualJoystickComponent, JoystickPosition } from './shared/components/virtual-joystick/virtual-joystick.component';
import { PresetPanelComponent } from './shared/components/preset-panel/preset-panel.component';

@Component({
  selector: 'app-root',
  standalone: true,
  imports: [CommonModule, RouterOutlet, RouterLink, RouterLinkActive, VirtualJoystickComponent, PresetPanelComponent],
  template: `
    <div class="h-screen flex flex-col">
      <!-- Header -->
      <header class="bg-saint-surface border-b border-saint-surface-light px-4 py-3 flex items-center justify-between">
        <nav class="flex gap-2">
          <a routerLink="/controller" routerLinkActive="bg-saint-primary"
             class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors">
            Controller
          </a>
          <a routerLink="/bindings" routerLinkActive="bg-saint-primary"
             class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors">
            Bindings
          </a>
          <a routerLink="/settings" routerLinkActive="bg-saint-primary"
             class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors">
            Settings
          </a>
        </nav>

        <div class="flex items-center gap-4">
          <!-- Connection Status -->
          <div class="flex items-center gap-2">
            <div [class]="getStatusClass()"></div>
            <span class="text-sm text-saint-text-muted">{{ getStatusText() }}</span>
          </div>

          <!-- E-Stop Button -->
          <button
            (click)="emergencyStop()"
            class="flex items-center gap-2 px-4 py-2 bg-red-600 hover:bg-red-700 text-white font-semibold rounded-lg transition-colors uppercase tracking-wide"
            title="Emergency Stop">
            <span class="material-icons icon-sm">front_hand</span>
            <span>E-Stop</span>
          </button>
        </div>
      </header>

      <!-- Main Content -->
      <main class="flex-1 overflow-auto">
        <router-outlet></router-outlet>
      </main>

      <!-- Preset Panel Overlay -->
      <app-preset-panel></app-preset-panel>

      <!-- Footer Navigation Bar -->
      <footer class="bg-saint-surface border-t border-saint-surface-light">
        <!-- Tab Buttons -->
        <div class="flex border-b border-saint-surface-light">
          <button
            (click)="togglePanel('controls')"
            [class]="getTabClass('controls')"
            class="flex items-center gap-2 px-4 py-2 text-sm transition-colors">
            <span class="material-icons icon-sm">sports_esports</span>
            <span>Controls</span>
            @if (gamepadConnected()) {
              <span class="px-2 py-0.5 bg-saint-success/20 text-saint-success text-xs rounded-full">
                {{ gamepadName() }}
              </span>
            } @else {
              <span class="px-2 py-0.5 bg-saint-text-muted/20 text-saint-text-muted text-xs rounded-full">
                No Controller
              </span>
            }
            <span class="material-icons icon-sm transition-transform"
                  [class.rotate-180]="activePanel() === 'controls'">
              expand_less
            </span>
          </button>
        </div>

        <!-- Sliding Panel -->
        <div class="overflow-hidden transition-all duration-300 ease-in-out"
             [style.height]="activePanel() ? '280px' : '0'">

          <!-- Controls Panel -->
          @if (activePanel() === 'controls') {
            <div class="p-4 h-full">
              <div class="flex justify-center items-center h-full">
                <!-- Virtual Controller Layout -->
                <div class="relative w-full max-w-2xl h-56">

                  <!-- Left Side -->
                  <div class="absolute left-0 top-0 bottom-0 w-1/2 flex flex-col justify-between py-2">
                    <!-- Left Shoulder/Trigger -->
                    <div class="flex flex-col gap-1 ml-8">
                      <button class="w-20 h-6 rounded-t-lg text-xs font-medium transition-colors relative overflow-hidden"
                              [class.bg-saint-surface-light]="leftTrigger() < 0.1"
                              [class.bg-saint-primary]="leftTrigger() >= 0.1">
                        <div class="absolute inset-0 bg-saint-primary transition-all"
                             [style.width.%]="leftTrigger() * 100"></div>
                        <span class="relative z-10">LT</span>
                      </button>
                      <button class="w-20 h-5 rounded-b text-xs font-medium transition-colors active:scale-95"
                              [class.bg-saint-surface-light]="!gamepadButtons()['LB'] && !virtualPressed()['lb']"
                              [class.bg-saint-primary]="gamepadButtons()['LB'] || virtualPressed()['lb']"
                              (click)="onVirtualButtonPress('lb')">
                        LB
                      </button>
                    </div>

                    <!-- Left Stick and D-Pad Row -->
                    <div class="flex items-center gap-6 ml-4">
                      <!-- Left Analog Stick -->
                      <app-virtual-joystick
                        [size]="80"
                        [knobSize]="40"
                        [externalPosition]="physicalLeftStick()"
                        (positionChange)="onLeftStickChange($event)">
                      </app-virtual-joystick>

                      <!-- D-Pad -->
                      <div class="relative w-20 h-20">
                        <!-- Up -->
                        <button class="absolute top-0 left-1/2 -translate-x-1/2 w-6 h-6 rounded-t transition-colors active:scale-95"
                                [class.bg-saint-primary]="gamepadButtons()['DPadUp'] || virtualPressed()['d_pad_up']"
                                [class.bg-saint-surface-light]="!gamepadButtons()['DPadUp'] && !virtualPressed()['d_pad_up']"
                                (click)="onVirtualButtonPress('d_pad_up')">
                          <span class="material-icons text-sm">keyboard_arrow_up</span>
                        </button>
                        <!-- Down -->
                        <button class="absolute bottom-0 left-1/2 -translate-x-1/2 w-6 h-6 rounded-b transition-colors active:scale-95"
                                [class.bg-saint-primary]="gamepadButtons()['DPadDown'] || virtualPressed()['d_pad_down']"
                                [class.bg-saint-surface-light]="!gamepadButtons()['DPadDown'] && !virtualPressed()['d_pad_down']"
                                (click)="onVirtualButtonPress('d_pad_down')">
                          <span class="material-icons text-sm">keyboard_arrow_down</span>
                        </button>
                        <!-- Left -->
                        <button class="absolute left-0 top-1/2 -translate-y-1/2 w-6 h-6 rounded-l transition-colors active:scale-95"
                                [class.bg-saint-primary]="gamepadButtons()['DPadLeft'] || virtualPressed()['d_pad_left']"
                                [class.bg-saint-surface-light]="!gamepadButtons()['DPadLeft'] && !virtualPressed()['d_pad_left']"
                                (click)="onVirtualButtonPress('d_pad_left')">
                          <span class="material-icons text-sm">keyboard_arrow_left</span>
                        </button>
                        <!-- Right -->
                        <button class="absolute right-0 top-1/2 -translate-y-1/2 w-6 h-6 rounded-r transition-colors active:scale-95"
                                [class.bg-saint-primary]="gamepadButtons()['DPadRight'] || virtualPressed()['d_pad_right']"
                                [class.bg-saint-surface-light]="!gamepadButtons()['DPadRight'] && !virtualPressed()['d_pad_right']"
                                (click)="onVirtualButtonPress('d_pad_right')">
                          <span class="material-icons text-sm">keyboard_arrow_right</span>
                        </button>
                        <!-- Center -->
                        <div class="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-6 h-6 bg-saint-surface-light"></div>
                      </div>
                    </div>

                    <!-- Spacer -->
                    <div class="h-6"></div>
                  </div>

                  <!-- Right Side -->
                  <div class="absolute right-0 top-0 bottom-0 w-1/2 flex flex-col justify-between py-2">
                    <!-- Right Shoulder/Trigger -->
                    <div class="flex flex-col gap-1 ml-auto mr-8">
                      <button class="w-20 h-6 rounded-t-lg text-xs font-medium transition-colors relative overflow-hidden"
                              [class.bg-saint-surface-light]="rightTrigger() < 0.1"
                              [class.bg-saint-primary]="rightTrigger() >= 0.1">
                        <div class="absolute inset-0 bg-saint-primary transition-all origin-right"
                             [style.width.%]="rightTrigger() * 100"></div>
                        <span class="relative z-10">RT</span>
                      </button>
                      <button class="w-20 h-5 rounded-b text-xs font-medium transition-colors active:scale-95"
                              [class.bg-saint-surface-light]="!gamepadButtons()['RB'] && !virtualPressed()['rb']"
                              [class.bg-saint-primary]="gamepadButtons()['RB'] || virtualPressed()['rb']"
                              (click)="onVirtualButtonPress('rb')">
                        RB
                      </button>
                    </div>

                    <!-- Face Buttons and Right Stick Row -->
                    <div class="flex items-center gap-6 justify-end mr-4">
                      <!-- Face Buttons (ABXY) -->
                      <div class="relative w-20 h-20">
                        <!-- Y (top) -->
                        <button class="absolute top-0 left-1/2 -translate-x-1/2 w-8 h-8 rounded-full text-xs font-bold transition-colors shadow-md active:scale-90"
                                [class.bg-yellow-400]="gamepadButtons()['Y'] || virtualPressed()['y']"
                                [class.bg-yellow-600]="!gamepadButtons()['Y'] && !virtualPressed()['y']"
                                [class.scale-95]="gamepadButtons()['Y'] || virtualPressed()['y']"
                                (click)="onVirtualButtonPress('y')">
                          Y
                        </button>
                        <!-- A (bottom) -->
                        <button class="absolute bottom-0 left-1/2 -translate-x-1/2 w-8 h-8 rounded-full text-xs font-bold transition-colors shadow-md active:scale-90"
                                [class.bg-green-400]="gamepadButtons()['A'] || virtualPressed()['a']"
                                [class.bg-green-600]="!gamepadButtons()['A'] && !virtualPressed()['a']"
                                [class.scale-95]="gamepadButtons()['A'] || virtualPressed()['a']"
                                (click)="onVirtualButtonPress('a')">
                          A
                        </button>
                        <!-- X (left) -->
                        <button class="absolute left-0 top-1/2 -translate-y-1/2 w-8 h-8 rounded-full text-xs font-bold transition-colors shadow-md active:scale-90"
                                [class.bg-blue-400]="gamepadButtons()['X'] || virtualPressed()['x']"
                                [class.bg-blue-600]="!gamepadButtons()['X'] && !virtualPressed()['x']"
                                [class.scale-95]="gamepadButtons()['X'] || virtualPressed()['x']"
                                (click)="onVirtualButtonPress('x')">
                          X
                        </button>
                        <!-- B (right) -->
                        <button class="absolute right-0 top-1/2 -translate-y-1/2 w-8 h-8 rounded-full text-xs font-bold transition-colors shadow-md active:scale-90"
                                [class.bg-red-400]="gamepadButtons()['B'] || virtualPressed()['b']"
                                [class.bg-red-600]="!gamepadButtons()['B'] && !virtualPressed()['b']"
                                [class.scale-95]="gamepadButtons()['B'] || virtualPressed()['b']"
                                (click)="onVirtualButtonPress('b')">
                          B
                        </button>
                      </div>

                      <!-- Right Analog Stick -->
                      <app-virtual-joystick
                        [size]="80"
                        [knobSize]="40"
                        [externalPosition]="physicalRightStick()"
                        (positionChange)="onRightStickChange($event)">
                      </app-virtual-joystick>
                    </div>

                    <!-- Spacer -->
                    <div class="h-6"></div>
                  </div>

                </div>
              </div>
            </div>
          }
        </div>
      </footer>
    </div>
  `
})
export class AppComponent {
  activePanel = signal<string | null>(null);
  leftStick = signal<JoystickPosition>({ x: 0, y: 0 });
  rightStick = signal<JoystickPosition>({ x: 0, y: 0 });
  virtualPressed = signal<Record<string, boolean>>({});

  // Gamepad state from physical controller
  gamepadConnected = computed(() => this.inputService.isGamepadConnected());
  gamepadName = computed(() => this.inputService.gamepad().name);
  physicalLeftStick = computed(() => this.inputService.gamepad().leftStick);
  physicalRightStick = computed(() => this.inputService.gamepad().rightStick);
  gamepadButtons = computed(() => this.inputService.gamepad().buttons);
  leftTrigger = computed(() => this.inputService.gamepad().leftTrigger);
  rightTrigger = computed(() => this.inputService.gamepad().rightTrigger);

  constructor(
    public connectionService: ConnectionService,
    public inputService: InputService,
    public bindingsService: BindingsService
  ) {}

  /**
   * Handle virtual button press from onscreen controls
   */
  onVirtualButtonPress(button: DigitalInput): void {
    console.log('Virtual button pressed:', button);

    // Visual feedback - show button as pressed briefly
    this.virtualPressed.update(state => ({ ...state, [button]: true }));
    setTimeout(() => {
      this.virtualPressed.update(state => ({ ...state, [button]: false }));
    }, 150);

    // Find the binding for this button and execute it
    const profile = this.bindingsService.activeProfile();
    console.log('Active profile:', profile?.name, 'digitalBindings count:', profile?.digitalBindings?.length);

    if (!profile) {
      console.log('No active profile found!');
      return;
    }

    const binding = profile.digitalBindings.find(
      b => b.input === button && b.trigger === 'press' && b.enabled
    );

    if (!binding) {
      console.log(`No binding found for button: ${button}`);
      console.log('Available bindings:', profile.digitalBindings.map(b => `${b.input}:${b.trigger}`));
      return;
    }

    console.log('Found binding, executing action:', binding.action.type);

    // Execute the action
    this.executeDigitalAction(binding.action);
  }

  /**
   * Execute a digital action from a binding
   */
  private executeDigitalAction(action: typeof this.bindingsService.activeProfile extends () => infer P ? P extends { digitalBindings: { action: infer A }[] } ? A : never : never): void {
    switch (action.type) {
      case 'show_panel':
        this.bindingsService.showPanel(action.panel_id);
        break;
      case 'hide_panel':
        this.bindingsService.hidePanel();
        break;
      case 'navigate_panel':
        this.bindingsService.navigatePanel(action.direction);
        break;
      case 'select_panel_item':
        this.bindingsService.selectCurrentItem();
        break;
      case 'activate_preset':
        this.bindingsService.activatePreset(action.preset_id);
        break;
      case 'e_stop':
        this.emergencyStop();
        break;
      case 'toggle_output':
        console.log('Toggle output:', action.target_id);
        // TODO: Implement toggle output
        break;
      case 'cycle_output':
        console.log('Cycle output:', action.target_id, action.values);
        // TODO: Implement cycle output
        break;
      case 'direct_control':
        console.log('Direct control:', action.target, action.value);
        // TODO: Send command to server
        break;
      case 'none':
        break;
    }
  }

  onLeftStickChange(position: JoystickPosition): void {
    this.leftStick.set(position);
    // TODO: Send control command based on left stick position
    console.log('Left stick:', position);
  }

  onRightStickChange(position: JoystickPosition): void {
    this.rightStick.set(position);
    // TODO: Send control command based on right stick position
    console.log('Right stick:', position);
  }

  togglePanel(panel: string): void {
    if (this.activePanel() === panel) {
      this.activePanel.set(null);
    } else {
      this.activePanel.set(panel);
    }
  }

  getTabClass(panel: string): string {
    const base = 'border-r border-saint-surface-light';
    if (this.activePanel() === panel) {
      return `${base} bg-saint-primary text-white`;
    }
    return `${base} hover:bg-saint-surface-light text-saint-text-muted`;
  }

  getStatusClass(): string {
    const base = 'status-indicator';
    switch (this.connectionService.status()) {
      case ConnectionStatus.Connected:
        return `${base} status-connected`;
      case ConnectionStatus.Connecting:
        return `${base} status-connecting`;
      default:
        return `${base} status-disconnected`;
    }
  }

  getStatusText(): string {
    switch (this.connectionService.status()) {
      case ConnectionStatus.Connected:
        return 'Connected';
      case ConnectionStatus.Connecting:
        return 'Connecting...';
      case ConnectionStatus.Authenticating:
        return 'Authenticating...';
      default:
        return 'Disconnected';
    }
  }

  async emergencyStop(): Promise<void> {
    try {
      await this.connectionService.emergencyStop();
    } catch (err) {
      console.error('Emergency stop failed:', err);
    }
  }
}
