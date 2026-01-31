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
  private static readonly CONFIG_KEY = 'saint-controller-config';

  private connectionState = signal<ConnectionState>({
    status: ConnectionStatus.Disconnected
  });

  readonly status = computed(() => this.connectionState().status);
  readonly error = computed(() => this.connectionState().error);
  readonly isConnected = computed(() => this.connectionState().status === ConnectionStatus.Connected);

  constructor(private tauri: TauriService) {
    this.subscribeToConnectionEvents();
    // Attempt auto-connect after a short delay to let the app initialize
    setTimeout(() => this.autoConnect(), 500);
  }

  /**
   * Get saved connection config from localStorage.
   */
  getSavedConfig(): ConnectionConfig | null {
    const saved = localStorage.getItem(ConnectionService.CONFIG_KEY);
    if (!saved) return null;

    try {
      const parsed = JSON.parse(saved);
      if (parsed.host && parsed.port) {
        return {
          host: parsed.host,
          port: parsed.port,
          password: parsed.password || ''
        };
      }
    } catch {
      // Ignore parse errors
    }
    return null;
  }

  /**
   * Attempt to auto-connect using saved credentials.
   * Only connects if a password is saved (indicates user wants auto-connect).
   */
  async autoConnect(): Promise<void> {
    const config = this.getSavedConfig();
    if (!config || !config.password) {
      console.log('[ConnectionService] No saved credentials for auto-connect');
      return;
    }

    console.log('[ConnectionService] Attempting auto-connect to', config.host);
    try {
      await this.connect(config);
    } catch (err) {
      console.error('[ConnectionService] Auto-connect failed:', err);
      // Don't throw - auto-connect failure is not critical
    }
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
