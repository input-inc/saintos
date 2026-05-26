import { defineConfig, loadEnv } from 'vite'
import vue from '@vitejs/plugin-vue'
import { fileURLToPath, URL } from 'node:url'

// Vite config for the SAINT.OS operator UI.
//
// Dev: `npm run dev` serves on http://localhost:5173 and proxies
//   /api → the running Pi (host configured via SAINT_HOST env var,
//   defaults to localhost:8080). WebSocket frames are forwarded too.
//
// Build: `npm run build` emits to web/dist/ — the Pi's http_server
//   picks that up as its `web_root` once we flip the cutover at deploy time.
export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  const apiTarget = env.SAINT_HOST || 'http://localhost:8080'
  const wsTarget = apiTarget.replace(/^http/, 'ws')

  return {
    plugins: [vue()],
    resolve: {
      alias: {
        '@': fileURLToPath(new URL('./src', import.meta.url)),
      },
    },
    server: {
      port: 5173,
      proxy: {
        '/api/ws': { target: wsTarget, ws: true, changeOrigin: true },
        '/api': { target: apiTarget, changeOrigin: true },
      },
    },
    build: {
      outDir: 'dist',
      emptyOutDir: true,
      sourcemap: true,
    },
    define: {
      __APP_VERSION__: JSON.stringify(process.env.npm_package_version || '0.0.0'),
    },
  }
})
