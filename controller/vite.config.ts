/**
 * Vite config for the SAINT.OS controller (Vue 3).
 *
 * The Tauri build runs `npm run build`, which version-syncs and then
 * invokes `vite build`. Output lands in `dist/`; tauri.conf.json's
 * `frontendDist` points there for both `tauri dev` and `tauri build`.
 */

import { defineConfig } from 'vite';
import vue from '@vitejs/plugin-vue';
import path from 'node:path';

// https://vitejs.dev/config/
export default defineConfig({
    root: path.resolve(__dirname, 'src'),
    base: './',

    plugins: [vue()],

    resolve: {
        alias: {
            '@': path.resolve(__dirname, 'src'),
        },
    },

    build: {
        outDir: path.resolve(__dirname, 'dist'),
        emptyOutDir: true,
        // Smaller chunks help Deck startup latency over the bundled
        // squashfs mount. Tauri's webview cache helps on subsequent
        // launches, but first launch is fastest with fewer + smaller
        // assets to fault in.
        target: 'esnext',
        sourcemap: true,
    },

    server: {
        // Tauri's dev shell talks to localhost; bind explicitly to
        // 127.0.0.1 so we don't accidentally expose the dev server
        // on the LAN during development.
        port: 4200,
        strictPort: true,
        host: '127.0.0.1',
    },

    optimizeDeps: {
        // Pre-bundle these to avoid the noisy "discovered new
        // dependency" reloads during early dev.
        include: ['vue', 'vue-router', '@headlessui/vue'],
    },
});
