import { ApplicationConfig, provideZoneChangeDetection, ErrorHandler, APP_INITIALIZER } from '@angular/core';
import { provideRouter } from '@angular/router';
import { routes } from './app.routes';
import { LoggingService, GlobalErrorHandler } from './core/services/logging.service';

function initializeLogging(logging: LoggingService) {
  return () => logging.init();
}

export const appConfig: ApplicationConfig = {
  providers: [
    provideZoneChangeDetection({ eventCoalescing: true }),
    provideRouter(routes),
    LoggingService,
    {
      provide: APP_INITIALIZER,
      useFactory: initializeLogging,
      deps: [LoggingService],
      multi: true
    },
    {
      provide: ErrorHandler,
      useClass: GlobalErrorHandler
    }
  ]
};
