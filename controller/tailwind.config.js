/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{html,ts}",
  ],
  theme: {
    extend: {
      colors: {
        'saint': {
          'primary': '#3b82f6',
          'secondary': '#1e40af',
          'background': '#0f172a',
          'surface': '#1e293b',
          'surface-light': '#334155',
          'text': '#f1f5f9',
          'text-muted': '#94a3b8',
          'success': '#22c55e',
          'warning': '#f59e0b',
          'error': '#ef4444',
        }
      },
      fontFamily: {
        'sans': ['Inter', 'system-ui', 'sans-serif'],
        'mono': ['JetBrains Mono', 'Fira Code', 'monospace'],
      }
    },
  },
  plugins: [],
}
