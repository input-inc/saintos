import { defineConfig } from 'histoire'
import { HstVue } from '@histoire/plugin-vue'

// Histoire — component sandbox for local dev only.
//
//   npm run story        # dev server on http://localhost:6006
//   npm run story:build  # static build (not deployed; lives outside dist/)
//
// Picks up vite.config.js automatically (including the @ → ./src alias
// and the Tailwind plugin). The shared setup file installs Pinia and
// the same mock-WS/catalog/display stores every widget story leans on.
export default defineConfig({
  plugins: [HstVue()],
  setupFile: 'histoire.setup.js',
  storyMatch: ['src/**/*.story.vue'],
  outDir: '.histoire',
  theme: {
    title: 'SAINT.OS UI sandbox',
  },
  vite: {
    server: { port: 6006 },
  },
})
