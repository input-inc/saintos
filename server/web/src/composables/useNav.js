// Shared navigation model + mobile-drawer state.
//
// The operator nav renders two ways off the SAME link list:
//   • ≥ lg  — an inline horizontal bar (NavBar.vue)
//   • < lg  — a slide-out drawer (NavDrawer.vue) toggled from AppHeader's
//             hamburger button.
//
// `drawerOpen` is module-scoped singleton state so the hamburger (in
// AppHeader) and the drawer (rendered at the App shell root) can share
// it without prop-drilling through the layout.
import { ref } from 'vue'

export const NAV_LINKS = [
  { to: '/dashboard',  icon: 'dashboard',    label: 'Dashboard' },
  { to: '/nodes',      icon: 'dns',          label: 'Nodes' },
  { to: '/routes',     icon: 'bolt',         label: 'Routes' },
  { to: '/control',    icon: 'tune',         label: 'Control' },
  { to: '/inputs',     icon: 'sensors',      label: 'Inputs' },
  { to: '/boards',     icon: 'animation',    label: 'Boards' },
  { to: '/settings',   icon: 'settings',     label: 'Settings' },
  { to: '/logs',       icon: 'description',  label: 'Logs' },
  { to: '/terminal',   icon: 'terminal',     label: 'Terminal' },
]

const drawerOpen = ref(false)

export function useNavDrawer () {
  return {
    drawerOpen,
    openDrawer:   () => { drawerOpen.value = true },
    closeDrawer:  () => { drawerOpen.value = false },
    toggleDrawer: () => { drawerOpen.value = !drawerOpen.value },
  }
}
