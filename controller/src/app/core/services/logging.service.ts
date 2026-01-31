import { Injectable, ErrorHandler } from '@angular/core';
import { invoke } from '@tauri-apps/api/core';

@Injectable({
  providedIn: 'root'
})
export class LoggingService {
  private initialized = false;

  // Store original console methods
  private originalConsole = {
    log: console.log.bind(console),
    info: console.info.bind(console),
    warn: console.warn.bind(console),
    error: console.error.bind(console),
    debug: console.debug.bind(console),
  };

  async init(): Promise<void> {
    if (this.initialized) return;

    try {
      // Override console methods to forward to Rust log
      this.interceptConsole();
      this.initialized = true;
      console.log('[LoggingService] Frontend logging initialized - console intercepted');
    } catch (err) {
      this.originalConsole.error('[LoggingService] Failed to initialize logging:', err);
      this.initialized = true;
    }
  }

  private interceptConsole(): void {
    // Override console.log
    console.log = (...args: unknown[]) => {
      this.originalConsole.log(...args);
      this.forwardToRust('debug', args);
    };

    // Override console.info
    console.info = (...args: unknown[]) => {
      this.originalConsole.info(...args);
      this.forwardToRust('info', args);
    };

    // Override console.warn
    console.warn = (...args: unknown[]) => {
      this.originalConsole.warn(...args);
      this.forwardToRust('warn', args);
    };

    // Override console.error
    console.error = (...args: unknown[]) => {
      this.originalConsole.error(...args);
      this.forwardToRust('error', args);
    };

    // Override console.debug
    console.debug = (...args: unknown[]) => {
      this.originalConsole.debug(...args);
      this.forwardToRust('debug', args);
    };
  }

  private forwardToRust(level: string, args: unknown[]): void {
    try {
      // Format all arguments into a single message string
      const message = args.map(arg => {
        if (typeof arg === 'string') return arg;
        if (arg instanceof Error) return `${arg.message}\n${arg.stack || ''}`;
        try {
          return JSON.stringify(arg);
        } catch {
          return String(arg);
        }
      }).join(' ');

      // Extract context if message starts with [ContextName]
      let context: string | undefined;
      let finalMessage = message;
      const contextMatch = message.match(/^\[([^\]]+)\]\s*/);
      if (contextMatch) {
        context = contextMatch[1];
        finalMessage = message.slice(contextMatch[0].length);
      }

      // Fire and forget - don't await to avoid blocking
      invoke('log_frontend', { level, message: finalMessage, context }).catch(() => {
        // Silently ignore errors to prevent infinite loops
      });
    } catch {
      // Silently ignore to prevent infinite loops
    }
  }

  // Direct logging methods for explicit use
  async error(context: string, message: string, err?: unknown): Promise<void> {
    const fullMessage = err ? `${message}: ${this.formatError(err)}` : message;
    console.error(`[${context}]`, fullMessage);
  }

  async warn(context: string, message: string): Promise<void> {
    console.warn(`[${context}]`, message);
  }

  async info(context: string, message: string): Promise<void> {
    console.info(`[${context}]`, message);
  }

  async debug(context: string, message: string): Promise<void> {
    console.debug(`[${context}]`, message);
  }

  async trace(context: string, message: string): Promise<void> {
    console.debug(`[${context}]`, message); // Use debug for trace since console.trace shows stack
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
