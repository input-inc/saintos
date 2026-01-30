import { Component, computed } from '@angular/core';
import { CommonModule } from '@angular/common';
import { InputService, GamepadState, GyroState, TouchpadState } from '../../core/services/input.service';
import { ConnectionService } from '../../core/services/connection.service';

@Component({
  selector: 'app-controller',
  standalone: true,
  imports: [CommonModule],
  template: `
    <div class="p-6 space-y-6">
      <!-- Gamepad Status -->
      <div class="card">
        <div class="flex items-center justify-between mb-4">
          <h2 class="text-lg font-semibold">Gamepad</h2>
          <div class="flex items-center gap-2">
            <div [class]="gamepad().connected ? 'status-indicator status-connected' : 'status-indicator status-disconnected'"></div>
            <span class="text-sm text-saint-text-muted">
              {{ gamepad().connected ? gamepad().name : 'No gamepad detected' }}
            </span>
          </div>
        </div>

        @if (gamepad().connected) {
          <div class="grid grid-cols-2 lg:grid-cols-4 gap-6">
            <!-- Left Stick -->
            <div class="flex flex-col items-center">
              <span class="text-sm text-saint-text-muted mb-2">Left Stick</span>
              <div class="relative w-32 h-32 bg-saint-background rounded-full border-2 border-saint-surface-light">
                <div class="absolute w-4 h-4 bg-saint-primary rounded-full transform -translate-x-1/2 -translate-y-1/2"
                     [style.left.%]="50 + gamepad().leftStick.x * 40"
                     [style.top.%]="50 - gamepad().leftStick.y * 40"></div>
                <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                  <div class="w-1 h-full bg-saint-surface-light opacity-30"></div>
                </div>
                <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                  <div class="w-full h-1 bg-saint-surface-light opacity-30"></div>
                </div>
              </div>
              <span class="text-xs text-saint-text-muted mt-2">
                {{ formatAxis(gamepad().leftStick.x) }}, {{ formatAxis(gamepad().leftStick.y) }}
              </span>
            </div>

            <!-- Right Stick -->
            <div class="flex flex-col items-center">
              <span class="text-sm text-saint-text-muted mb-2">Right Stick</span>
              <div class="relative w-32 h-32 bg-saint-background rounded-full border-2 border-saint-surface-light">
                <div class="absolute w-4 h-4 bg-saint-primary rounded-full transform -translate-x-1/2 -translate-y-1/2"
                     [style.left.%]="50 + gamepad().rightStick.x * 40"
                     [style.top.%]="50 - gamepad().rightStick.y * 40"></div>
                <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                  <div class="w-1 h-full bg-saint-surface-light opacity-30"></div>
                </div>
                <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                  <div class="w-full h-1 bg-saint-surface-light opacity-30"></div>
                </div>
              </div>
              <span class="text-xs text-saint-text-muted mt-2">
                {{ formatAxis(gamepad().rightStick.x) }}, {{ formatAxis(gamepad().rightStick.y) }}
              </span>
            </div>

            <!-- Triggers -->
            <div class="flex flex-col items-center">
              <span class="text-sm text-saint-text-muted mb-2">Triggers</span>
              <div class="flex gap-4">
                <div class="flex flex-col items-center">
                  <span class="text-xs text-saint-text-muted mb-1">LT</span>
                  <div class="w-8 h-32 bg-saint-background rounded border border-saint-surface-light relative overflow-hidden">
                    <div class="absolute bottom-0 left-0 right-0 bg-saint-primary transition-all"
                         [style.height.%]="gamepad().leftTrigger * 100"></div>
                  </div>
                  <span class="text-xs text-saint-text-muted mt-1">{{ formatAxis(gamepad().leftTrigger) }}</span>
                </div>
                <div class="flex flex-col items-center">
                  <span class="text-xs text-saint-text-muted mb-1">RT</span>
                  <div class="w-8 h-32 bg-saint-background rounded border border-saint-surface-light relative overflow-hidden">
                    <div class="absolute bottom-0 left-0 right-0 bg-saint-primary transition-all"
                         [style.height.%]="gamepad().rightTrigger * 100"></div>
                  </div>
                  <span class="text-xs text-saint-text-muted mt-1">{{ formatAxis(gamepad().rightTrigger) }}</span>
                </div>
              </div>
            </div>

            <!-- Buttons -->
            <div class="flex flex-col items-center">
              <span class="text-sm text-saint-text-muted mb-2">Buttons</span>
              <div class="grid grid-cols-4 gap-2">
                @for (btn of buttonList; track btn) {
                  <div [class]="getButtonClass(btn)"
                       class="w-8 h-8 rounded flex items-center justify-center text-xs font-medium">
                    {{ btn }}
                  </div>
                }
              </div>
            </div>
          </div>
        }
      </div>

      <!-- Touchpads (Steam Deck) -->
      <div class="card">
        <h2 class="text-lg font-semibold mb-4">Touchpads</h2>
        <div class="grid grid-cols-2 gap-6">
          <!-- Left Touchpad -->
          <div class="flex flex-col items-center">
            <span class="text-sm text-saint-text-muted mb-2">Left Touchpad</span>
            <div class="relative w-32 h-32 bg-saint-background rounded-lg border-2 border-saint-surface-light"
                 [class.border-saint-primary]="leftTouchpad().touched">
              <div class="absolute w-3 h-3 bg-saint-accent rounded-full transform -translate-x-1/2 -translate-y-1/2"
                   [class.bg-saint-primary]="leftTouchpad().clicked"
                   [style.left.%]="50 + leftTouchpad().x * 45"
                   [style.top.%]="50 - leftTouchpad().y * 45"
                   [style.opacity]="leftTouchpad().touched ? 1 : 0.3"></div>
            </div>
            <span class="text-xs text-saint-text-muted mt-2">
              {{ formatAxis(leftTouchpad().x) }}, {{ formatAxis(leftTouchpad().y) }}
              {{ leftTouchpad().touched ? '(touched)' : '' }}
              {{ leftTouchpad().clicked ? '(clicked)' : '' }}
            </span>
          </div>

          <!-- Right Touchpad -->
          <div class="flex flex-col items-center">
            <span class="text-sm text-saint-text-muted mb-2">Right Touchpad</span>
            <div class="relative w-32 h-32 bg-saint-background rounded-lg border-2 border-saint-surface-light"
                 [class.border-saint-primary]="rightTouchpad().touched">
              <div class="absolute w-3 h-3 bg-saint-accent rounded-full transform -translate-x-1/2 -translate-y-1/2"
                   [class.bg-saint-primary]="rightTouchpad().clicked"
                   [style.left.%]="50 + rightTouchpad().x * 45"
                   [style.top.%]="50 - rightTouchpad().y * 45"
                   [style.opacity]="rightTouchpad().touched ? 1 : 0.3"></div>
            </div>
            <span class="text-xs text-saint-text-muted mt-2">
              {{ formatAxis(rightTouchpad().x) }}, {{ formatAxis(rightTouchpad().y) }}
              {{ rightTouchpad().touched ? '(touched)' : '' }}
              {{ rightTouchpad().clicked ? '(clicked)' : '' }}
            </span>
          </div>
        </div>
      </div>

      <!-- Gyroscope -->
      <div class="card">
        <h2 class="text-lg font-semibold mb-4">Gyroscope</h2>
        <div class="grid grid-cols-3 gap-4">
          <div class="flex flex-col items-center">
            <span class="text-sm text-saint-text-muted mb-2">Pitch</span>
            <div class="text-2xl font-mono">{{ formatAngle(gyro().pitch) }}</div>
          </div>
          <div class="flex flex-col items-center">
            <span class="text-sm text-saint-text-muted mb-2">Roll</span>
            <div class="text-2xl font-mono">{{ formatAngle(gyro().roll) }}</div>
          </div>
          <div class="flex flex-col items-center">
            <span class="text-sm text-saint-text-muted mb-2">Yaw</span>
            <div class="text-2xl font-mono">{{ formatAngle(gyro().yaw) }}</div>
          </div>
        </div>
      </div>

      <!-- Command Log -->
      <div class="card">
        <h2 class="text-lg font-semibold mb-4">Command Log</h2>
        <div class="bg-saint-background rounded-lg p-3 h-48 overflow-y-auto font-mono text-sm">
          @if (!connectionService.isConnected()) {
            <p class="text-saint-text-muted">Connect to server to see commands...</p>
          } @else {
            <p class="text-saint-text-muted">Commands will appear here...</p>
          }
        </div>
      </div>
    </div>
  `
})
export class ControllerComponent {
  readonly buttonList = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'Select', 'Start', 'L3', 'R3', 'L4', 'R4', 'L5', 'R5', 'Steam', 'QAM'];

  constructor(
    private inputService: InputService,
    public connectionService: ConnectionService
  ) {}

  readonly gamepad = computed(() => this.inputService.gamepad());
  readonly gyro = computed(() => this.inputService.gyro());
  readonly leftTouchpad = computed(() => this.inputService.leftTouchpad());
  readonly rightTouchpad = computed(() => this.inputService.rightTouchpad());

  formatAxis(value: number): string {
    return value.toFixed(2);
  }

  formatAngle(degrees: number): string {
    return `${degrees.toFixed(1)}°`;
  }

  getButtonClass(button: string): string {
    const pressed = this.gamepad().buttons[button];
    return pressed
      ? 'bg-saint-primary text-white'
      : 'bg-saint-surface-light text-saint-text-muted';
  }
}
