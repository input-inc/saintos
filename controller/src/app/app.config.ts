import { ApplicationConfig, provideZoneChangeDetection, ErrorHandler, APP_INITIALIZER } from '@angular/core';
import { provideRouter } from '@angular/router';
import { routes } from './app.routes';
import { LoggingService, GlobalErrorHandler } from './core/services/logging.service';
import { KeyboardService } from './core/services/keyboard.service';

async function initializeApp(logging: LoggingService, keyboard: KeyboardService) {
  // Initialize logging first so console.log goes to log file
  await logging.init();
  // KeyboardService constructor already set up listeners
  // Just log that we're ready
  console.log('[AppInit] Application initialized');
}

export const appConfig: ApplicationConfig = {
  providers: [
    provideZoneChangeDetection({ eventCoalescing: true }),
    provideRouter(routes),
    LoggingService,
    KeyboardService,
    {
      provide: APP_INITIALIZER,
      useFactory: (logging: LoggingService, keyboard: KeyboardService) => () => initializeApp(logging, keyboard),
      deps: [LoggingService, KeyboardService],
      multi: true
    },
    {
      provide: ErrorHandler,
      useClass: GlobalErrorHandler
    }
  ]
};
