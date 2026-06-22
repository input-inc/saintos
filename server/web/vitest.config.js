import { defineConfig } from 'vitest/config'
import vue from '@vitejs/plugin-vue'
import { fileURLToPath, URL } from 'node:url'

// Vitest config for the SAINT.OS web dashboard. Mirrors vite.config.js's
// Vue plugin, `@` alias, and the __APP_VERSION__ define so modules import
// and compile identically under test. Tailwind/Histoire plugins are left
// out — they're irrelevant to logic tests and only slow startup.
export default defineConfig({
  plugins: [vue()],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  define: {
    __APP_VERSION__: JSON.stringify('test'),
  },
  test: {
    environment: 'happy-dom',
    setupFiles: ['./vitest.setup.js'],
    include: ['src/**/*.{test,spec}.js'],
    clearMocks: true,
  },
})
