/**
 * Vitest config for the SAINT.OS controller frontend.
 *
 * Separate from vite.config.ts (which sets root=src for the Tauri build)
 * so tests run from the controller dir with normal relative paths. The
 * Vue plugin + `@` alias mirror the build config so SFCs compile and
 * imports resolve the same way. happy-dom gives components a DOM without
 * a full browser.
 *
 * Run: `npm test` (CI) or `npm run test:watch`.
 */
import { defineConfig } from 'vitest/config';
import vue from '@vitejs/plugin-vue';
import path from 'node:path';

export default defineConfig({
    plugins: [vue()],
    resolve: {
        alias: {
            '@': path.resolve(__dirname, 'src'),
        },
    },
    test: {
        environment: 'happy-dom',
        setupFiles: ['./vitest.setup.ts'],
        include: ['src/**/*.{test,spec}.ts'],
        // Each spec file runs in its own module registry, so the
        // module-singleton composables (useInput, useBindings, …) start
        // fresh per file — important since they hold process-wide state.
        clearMocks: true,
    },
});
