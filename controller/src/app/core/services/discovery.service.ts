import { Injectable, signal, computed } from '@angular/core';
import { TauriService } from './tauri.service';
import { ConnectionService } from './connection.service';

/**
 * Represents a controllable function on a node
 */
export interface ControllableFunction {
  function: string;
  mode: string;  // 'pwm', 'servo', 'digital_out'
  gpio: number;  // For debugging only
}

/**
 * Represents a node with controllable functions
 */
export interface ControllableNode {
  role: string;
  node_id: string;
  display_name: string;
  online: boolean;
  functions: ControllableFunction[];
}

/**
 * Role definition from the server
 */
export interface RoleDefinition {
  name: string;
  display_name: string;
  description?: string;
  pins: Record<string, unknown>;
}

@Injectable({
  providedIn: 'root'
})
export class DiscoveryService {
  private _controllable = signal<ControllableNode[]>([]);
  private _roles = signal<RoleDefinition[]>([]);
  private _loading = signal(false);
  private _lastFetched = signal<Date | null>(null);

  /** All controllable nodes with their functions */
  readonly controllable = this._controllable.asReadonly();

  /** All role definitions */
  readonly roles = this._roles.asReadonly();

  /** Loading state */
  readonly loading = this._loading.asReadonly();

  /** Unique list of active roles (roles that have nodes assigned) */
  readonly activeRoles = computed(() => {
    const roles = new Set<string>();
    for (const node of this._controllable()) {
      roles.add(node.role);
    }
    return Array.from(roles).sort();
  });

  /** Get functions available for a specific role */
  getFunctionsForRole(role: string): string[] {
    const functions = new Set<string>();
    for (const node of this._controllable()) {
      if (node.role === role) {
        for (const fn of node.functions) {
          functions.add(fn.function);
        }
      }
    }
    return Array.from(functions).sort();
  }

  /** Get all unique functions across all roles */
  readonly allFunctions = computed(() => {
    const functions = new Set<string>();
    for (const node of this._controllable()) {
      for (const fn of node.functions) {
        functions.add(fn.function);
      }
    }
    return Array.from(functions).sort();
  });

  constructor(
    private tauri: TauriService,
    private connection: ConnectionService
  ) {
    // Auto-refresh when connection status changes to connected
    this.tauri.listen<{ status: string }>('connection-status').subscribe(state => {
      if (state.status === 'connected') {
        // Small delay to let the connection stabilize
        setTimeout(() => this.refresh(), 500);
      }
    });
  }

  /**
   * Fetch discovery data from the server
   */
  async refresh(): Promise<void> {
    if (!this.connection.isConnected()) {
      console.log('[DiscoveryService] Not connected, skipping refresh');
      return;
    }

    this._loading.set(true);
    try {
      // Fetch controllable functions
      await this.connection.discoverControllable();

      // Listen for the response
      const controllablePromise = new Promise<ControllableNode[]>((resolve) => {
        const sub = this.tauri.listen<{ controllable: ControllableNode[] }>('discovery-controllable')
          .subscribe(data => {
            sub.unsubscribe();
            resolve(data.controllable || []);
          });

        // Timeout after 5 seconds
        setTimeout(() => {
          sub.unsubscribe();
          resolve([]);
        }, 5000);
      });

      // For now, we'll use a simpler approach - invoke and get direct response
      // The current implementation sends the discovery request but doesn't return data directly
      // We need to update the backend to return data or listen for events

      // Temporary: Set some default controllable items for testing
      // In production, this should come from the server response
      this._controllable.set([
        {
          role: 'head',
          node_id: 'head-node',
          display_name: 'Head Controller',
          online: true,
          functions: [
            { function: 'pan', mode: 'servo', gpio: 0 },
            { function: 'tilt', mode: 'servo', gpio: 1 },
            { function: 'eye_left_lr', mode: 'servo', gpio: 2 },
            { function: 'eye_left_ud', mode: 'servo', gpio: 3 },
            { function: 'eye_right_lr', mode: 'servo', gpio: 4 },
            { function: 'eye_right_ud', mode: 'servo', gpio: 5 },
            { function: 'eyelid_left', mode: 'servo', gpio: 6 },
            { function: 'eyelid_right', mode: 'servo', gpio: 7 },
          ]
        },
        {
          role: 'tracks',
          node_id: 'tracks-node',
          display_name: 'Track Drive',
          online: true,
          functions: [
            { function: 'linear_velocity', mode: 'pwm', gpio: 0 },
            { function: 'angular_velocity', mode: 'pwm', gpio: 1 },
          ]
        },
        {
          role: 'sound',
          node_id: 'sound-node',
          display_name: 'Sound System',
          online: true,
          functions: [
            { function: 'play', mode: 'digital_out', gpio: 0 },
            { function: 'volume', mode: 'pwm', gpio: 1 },
          ]
        }
      ]);

      this._lastFetched.set(new Date());
      console.log('[DiscoveryService] Discovery data refreshed');
    } catch (err) {
      console.error('[DiscoveryService] Failed to refresh discovery data:', err);
    } finally {
      this._loading.set(false);
    }
  }

  /**
   * Get display name for a role
   */
  getRoleDisplayName(role: string): string {
    const roleDef = this._roles().find(r => r.name === role);
    return roleDef?.display_name || role;
  }

  /**
   * Check if a role+function combination is valid
   */
  isValidTarget(role: string, functionName: string): boolean {
    for (const node of this._controllable()) {
      if (node.role === role) {
        for (const fn of node.functions) {
          if (fn.function === functionName) {
            return true;
          }
        }
      }
    }
    return false;
  }
}
