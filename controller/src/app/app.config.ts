import { ApplicationConfig, provideZoneChangeDetection, ErrorHandler, APP_INITIALIZER } from '@angular/core';
import { provideRouter } from '@angular/router';
import { routes } from './app.routes';
import { LoggingService, GlobalErrorHandler } from './core/services/logging.service';
import { KeyboardService } from './core/services/keyboard.service';

function initializeLogging(logging: LoggingService) {
  return () => logging.init();
}

function initializeKeyboard(keyboard: KeyboardService) {
  // Just injecting the service is enough - it sets up listeners in constructor
  return () => {};
}

export const appConfig: ApplicationConfig = {
  providers: [
    provideZoneChangeDetection({ eventCoalescing: true }),
    provideRouter(routes),
    LoggingService,
    KeyboardService,
    {
      provide: APP_INITIALIZER,
      useFactory: initializeLogging,
      deps: [LoggingService],
      multi: true
    },
    {
      provide: APP_INITIALIZER,
      useFactory: initializeKeyboard,
      deps: [KeyboardService],
      multi: true
    },
    {
      provide: ErrorHandler,
      useClass: GlobalErrorHandler
    }
  ]
};
