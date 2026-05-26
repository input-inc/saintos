/**
 * Vue 3 entry point for the SAINT.OS controller.
 *
 * The Angular app boots from controller/src/main.ts. This module
 * mirrors it for the Vue rewrite during the migration window —
 * controller/docs/VUE_MIGRATION.md has the full picture. After
 * cutover, this becomes the only entry point and src-vue/ replaces
 * src/.
 */

import { createApp } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import App from './App.vue';
import { router } from './router';
import { installDragScroll } from './composables/useDragScroll';
import './styles.css';

// Apply saved UI scale via Tauri's webview-level zoom as early as
// possible (before App is mounted) so the first paint comes up at
// the right size. CSS-zoom-on-documentElement is intentionally NOT
// used here — it stacks with the webview zoom that Settings later
// applies, and CSS zoom doesn't reach native form controls anyway.
// See the inline comment in the old app.component.ts for the long
// version of this story.
const savedScale = localStorage.getItem('saint-controller-ui-scale');
if (savedScale) {
    const scale = parseFloat(savedScale);
    if (!Number.isNaN(scale) && scale > 0) {
        void invoke('set_zoom', { scale }).catch(err => {
            console.error('[main] set_zoom failed on startup, falling back to CSS zoom:', err);
            document.documentElement.style.zoom = savedScale;
        });
    }
}

// Document-level drag-scroll handler. Needed on Steam Deck Game Mode
// where gamescope reports every screen touch as `pointerType: "mouse"`
// — WebKit's native pan-scroll only fires for touch pointers, so we
// emulate it ourselves. Idempotent; safe to call once per app boot.
installDragScroll();

const app = createApp(App);
app.use(router);
app.mount('#app');
