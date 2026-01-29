import { Routes } from '@angular/router';

export const routes: Routes = [
  {
    path: '',
    redirectTo: 'controller',
    pathMatch: 'full'
  },
  {
    path: 'controller',
    loadComponent: () => import('./features/controller/controller.component')
      .then(m => m.ControllerComponent)
  },
  {
    path: 'bindings',
    loadComponent: () => import('./features/bindings/bindings.component')
      .then(m => m.BindingsComponent)
  },
  {
    path: 'settings',
    loadComponent: () => import('./features/settings/settings.component')
      .then(m => m.SettingsComponent)
  }
];
