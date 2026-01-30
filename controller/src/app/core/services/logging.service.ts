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
      await attachConsole();
      this.initialized = true;
      await this.info('LoggingService', 'Frontend logging initialized');
    } catch (err) {
      console.error('Failed to initialize logging:', err);
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
