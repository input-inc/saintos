import { Injectable, OnDestroy, signal, computed } from '@angular/core';
import { TauriService } from './tauri.service';
import { Subscription } from 'rxjs';

export interface AxisState {
  x: number;
  y: number;
}

export interface GyroState {
  pitch: number;
  roll: number;
  yaw: number;
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
  }
};

@Injectable({
  providedIn: 'root'
})
export class InputService implements OnDestroy {
  private subscription?: Subscription;

  private inputState = signal<InputState>(DEFAULT_INPUT_STATE);

  readonly gamepad = computed(() => this.inputState().gamepad);
  readonly gyro = computed(() => this.inputState().gyro);
  readonly isGamepadConnected = computed(() => this.inputState().gamepad.connected);

  constructor(private tauri: TauriService) {
    this.subscribeToInputEvents();
  }

  private subscribeToInputEvents(): void {
    this.subscription = this.tauri.listen<InputState>('input-state').subscribe(state => {
      this.inputState.set(state);
    });
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
  }
}
