import { createRouter, createWebHashHistory } from 'vue-router'

// Hash mode mirrors the legacy `#node/<id>` URLs so existing bookmarks
// keep working. Switch to history mode after the cutover if desired.
export const router = createRouter({
  history: createWebHashHistory(),
  routes: [
    { path: '/',                redirect: '/dashboard' },
    { path: '/dashboard',       name: 'dashboard',   component: () => import('@/views/Dashboard.vue') },
    { path: '/nodes',           name: 'nodes',       component: () => import('@/views/Nodes.vue') },
    { path: '/node/:id',        name: 'node-detail', component: () => import('@/views/NodeDetail.vue'), props: true,
      children: [
        { path: '',             redirect: { name: 'node-overview' } },
        { path: 'overview',     name: 'node-overview',    component: () => import('@/views/node/Overview.vue') },
        { path: 'peripherals',  name: 'node-peripherals', component: () => import('@/views/node/Peripherals.vue') },
        { path: 'state',        name: 'node-state',       component: () => import('@/views/node/State.vue') },
        { path: 'live',         name: 'node-live',        component: () => import('@/views/node/Live.vue') },
        { path: 'logs',         name: 'node-logs',        component: () => import('@/views/node/Logs.vue') },
        { path: 'boards',       name: 'node-boards',      component: () => import('@/views/node/Boards.vue') },
      ],
    },
    { path: '/routes',          name: 'routes',      component: () => import('@/views/Routes.vue') },
    { path: '/widgets',         name: 'widgets',     component: () => import('@/views/Widgets.vue') },
    { path: '/inputs',          name: 'inputs',      component: () => import('@/views/Inputs.vue') },
    { path: '/livelink',        name: 'livelink',    component: () => import('@/views/LiveLink.vue') },
    { path: '/control',         name: 'control',     component: () => import('@/views/Control.vue') },
    { path: '/moods',           name: 'moods',       component: () => import('@/views/Moods.vue') },
    { path: '/logs',            name: 'logs',        component: () => import('@/views/Logs.vue') },
    { path: '/terminal',        name: 'terminal',    component: () => import('@/views/Terminal.vue') },
    { path: '/updates',         name: 'updates',     component: () => import('@/views/Updates.vue') },
    { path: '/settings',        name: 'settings',    component: () => import('@/views/Settings.vue') },
    { path: '/:catchAll(.*)',   redirect: '/dashboard' },
  ],
})
