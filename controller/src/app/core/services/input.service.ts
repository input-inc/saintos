import { Injectable, OnDestroy, signal, computed } from '@angular/core';
import { TauriService } from './tauri.service';
import { Subject, Subscription } from 'rxjs';

export interface AxisState {
  x: number;
  y: number;
}

export interface GyroState {
  pitch: number;
  roll: number;
  yaw: number;
}

export interface TouchpadState {
  x: number;
  y: number;
  touched: boolean;
  clicked: boolean;
}

export interface GamepadState {
  connected: boolean;
  name: string;
  leftStick: AxisState;
  rightStick: AxisState;
  leftTrigger: number;
  rightTrigger: number;
  buttons: { [key: string]: boolean };
}

export interface InputState {
  gamepad: GamepadState;
  gyro: GyroState;
  leftTouchpad: TouchpadState;
  rightTouchpad: TouchpadState;
}

export interface ButtonEvent {
  button: string;
  pressed: boolean;
}

const DEFAULT_INPUT_STATE: InputState = {
  gamepad: {
    connected: false,
    name: '',
    leftStick: { x: 0, y: 0 },
    rightStick: { x: 0, y: 0 },
    leftTrigger: 0,
    rightTrigger: 0,
    buttons: {}
  },
  gyro: {
    pitch: 0,
    roll: 0,
    yaw: 0
  },
  leftTouchpad: {
    x: 0,
    y: 0,
    touched: false,
    clicked: false
  },
  rightTouchpad: {
    x: 0,
    y: 0,
    touched: false,
    clicked: false
  }
};

@Injectable({
  providedIn: 'root'
})
export class InputService implements OnDestroy {
  private subscription?: Subscription;
  private previousButtons: { [key: string]: boolean } = {};

  private inputState = signal<InputState>(DEFAULT_INPUT_STATE);

  // Subject for button press/release events
  private buttonEventSubject = new Subject<ButtonEvent>();
  readonly buttonEvents$ = this.buttonEventSubject.asObservable();

  readonly gamepad = computed(() => this.inputState().gamepad);
  readonly gyro = computed(() => this.inputState().gyro);
  readonly leftTouchpad = computed(() => this.inputState().leftTouchpad);
  readonly rightTouchpad = computed(() => this.inputState().rightTouchpad);
  readonly isGamepadConnected = computed(() => this.inputState().gamepad.connected);

  constructor(private tauri: TauriService) {
    this.subscribeToInputEvents();
  }

  private subscribeToInputEvents(): void {
    this.subscription = this.tauri.listen<InputState>('input-state').subscribe(state => {
      // Detect button state changes
      this.detectButtonChanges(state.gamepad.buttons);
      this.inputState.set(state);
    });
  }

  private detectButtonChanges(currentButtons: { [key: string]: boolean }): void {
    // Check all buttons in current state
    for (const [button, pressed] of Object.entries(currentButtons)) {
      const wasPressed = this.previousButtons[button] || false;
      if (pressed !== wasPressed) {
        this.buttonEventSubject.next({ button, pressed });
      }
    }

    // Check for buttons that were released (no longer in current state)
    for (const [button, wasPressed] of Object.entries(this.previousButtons)) {
      if (wasPressed && !currentButtons[button]) {
        this.buttonEventSubject.next({ button, pressed: false });
      }
    }

    // Update previous state
    this.previousButtons = { ...currentButtons };
  }

  async getInputState(): Promise<InputState> {
    try {
      return await this.tauri.invoke<InputState>('get_input_state');
    } catch {
      return DEFAULT_INPUT_STATE;
    }
  }

  ngOnDestroy(): void {
    this.subscription?.unsubscribe();
    this.buttonEventSubject.complete();
  }
}
