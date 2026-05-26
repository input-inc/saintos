/**
 * Vue Router config. Routes mirror the Angular app.routes.ts.
 * Components are loaded eagerly for the Controller/Settings paths
 * (the operator hits these from boot) and lazily for Bindings
 * (heavy form UI, only opened when actively editing).
 *
 * Hash history is used because Tauri serves the built bundle from a
 * non-HTTP scheme where path-based history fights with the webview's
 * URL handling.
 */

import { createRouter, createWebHashHistory, type RouteRecordRaw } from 'vue-router';
import ControllerView from './views/ControllerView.vue';
import SettingsView from './views/SettingsView.vue';

const BindingsView = () => import('./views/BindingsView.vue');

const routes: RouteRecordRaw[] = [
    { path: '/',           redirect: '/controller' },
    { path: '/controller', component: ControllerView, meta: { title: 'Controller' } },
    { path: '/bindings',   component: BindingsView,   meta: { title: 'Bindings' } },
    { path: '/settings',   component: SettingsView,   meta: { title: 'Settings' } },
];

export const router = createRouter({
    history: createWebHashHistory(),
    routes,
});
