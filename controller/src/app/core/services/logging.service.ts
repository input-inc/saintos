import { Injectable, ErrorHandler } from '@angular/core';
import { error, warn, info, debug, trace, attachConsole } from '@tauri-apps/plugin-log';

@Injectable({
  providedIn: 'root'
})
export class LoggingService {
  private initialized = false;

  async init(): Promise<void> {
    if (this.initialized) return;

    try {
      // Attach console to forward all console.log/error/etc to the log file
      const detach = await attachConsole();
      this.initialized = true;
      console.log('[LoggingService] Frontend logging initialized - console attached to Tauri log');
      // Note: After this point, console.log calls should appear in the log file
    } catch (err) {
      console.error('[LoggingService] Failed to initialize logging:', err);
      // Still mark as initialized to avoid repeated attempts
      this.initialized = true;
    }
  }

  async error(context: string, message: string, err?: unknown): Promise<void> {
    const fullMessage = err ? `${message}: ${this.formatError(err)}` : message;
    console.error(`[${context}]`, fullMessage);
    try {
      await error(`[${context}] ${fullMessage}`);
    } catch {
      // Logging failed, already logged to console
    }
  }

  async warn(context: string, message: string): Promise<void> {
    console.warn(`[${context}]`, message);
    try {
      await warn(`[${context}] ${message}`);
    } catch {
      // Logging failed, already logged to console
    }
  }

  async info(context: string, message: string): Promise<void> {
    console.info(`[${context}]`, message);
    try {
      await info(`[${context}] ${message}`);
    } catch {
      // Logging failed, already logged to console
    }
  }

  async debug(context: string, message: string): Promise<void> {
    console.debug(`[${context}]`, message);
    try {
      await debug(`[${context}] ${message}`);
    } catch {
      // Logging failed, already logged to console
    }
  }

  async trace(context: string, message: string): Promise<void> {
    console.trace(`[${context}]`, message);
    try {
      await trace(`[${context}] ${message}`);
    } catch {
      // Logging failed, already logged to console
    }
  }

  private formatError(err: unknown): string {
    if (err instanceof Error) {
      return `${err.message}\n${err.stack || ''}`;
    }
    return String(err);
  }
}

/**
 * Global error handler that logs uncaught errors
 */
@Injectable()
export class GlobalErrorHandler implements ErrorHandler {
  constructor(private logging: LoggingService) {}

  handleError(error: unknown): void {
    this.logging.error('GlobalErrorHandler', 'Uncaught error', error);
  }
}
