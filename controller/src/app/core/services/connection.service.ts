import { Injectable, OnDestroy, signal, computed } from '@angular/core';
import { TauriService } from './tauri.service';
import { Subscription } from 'rxjs';

export enum ConnectionStatus {
  Disconnected = 'disconnected',
  Connecting = 'connecting',
  Authenticating = 'authenticating',
  Connected = 'connected',
  Error = 'error'
}

export interface ConnectionConfig {
  host: string;
  port: number;
  password: string;
}

export interface ConnectionState {
  status: ConnectionStatus;
  error?: string;
  lastConnected?: Date;
}

@Injectable({
  providedIn: 'root'
})
export class ConnectionService implements OnDestroy {
  private subscriptions: Subscription[] = [];

  private connectionState = signal<ConnectionState>({
    status: ConnectionStatus.Disconnected
  });

  readonly status = computed(() => this.connectionState().status);
  readonly error = computed(() => this.connectionState().error);
  readonly isConnected = computed(() => this.connectionState().status === ConnectionStatus.Connected);

  constructor(private tauri: TauriService) {
    this.subscribeToConnectionEvents();
  }

  private subscribeToConnectionEvents(): void {
    this.subscriptions.push(
      this.tauri.listen<ConnectionState>('connection-status').subscribe(state => {
        this.connectionState.set(state);
      })
    );
  }

  async connect(config: ConnectionConfig): Promise<void> {
    this.connectionState.update(s => ({ ...s, status: ConnectionStatus.Connecting, error: undefined }));
    try {
      await this.tauri.invoke('connect', {
        host: config.host,
        port: config.port,
        password: config.password
      });
    } catch (err) {
      this.connectionState.update(s => ({
        ...s,
        status: ConnectionStatus.Error,
        error: String(err)
      }));
      throw err;
    }
  }

  async disconnect(): Promise<void> {
    try {
      await this.tauri.invoke('disconnect');
      this.connectionState.update(s => ({
        ...s,
        status: ConnectionStatus.Disconnected,
        error: undefined
      }));
    } catch (err) {
      console.error('Disconnect error:', err);
    }
  }

  async sendCommand(target: string, value: unknown): Promise<void> {
    if (!this.isConnected()) {
      throw new Error('Not connected');
    }
    await this.tauri.invoke('send_command', { target, value });
  }

  async emergencyStop(): Promise<void> {
    await this.tauri.invoke('emergency_stop');
  }

  ngOnDestroy(): void {
    this.subscriptions.forEach(s => s.unsubscribe());
  }
}
