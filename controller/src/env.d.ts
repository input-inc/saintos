/// <reference types="vite/client" />

// SFC imports — lets TS recognize `.vue` files as modules.
declare module '*.vue' {
    import type { DefineComponent } from 'vue';
    const component: DefineComponent<{}, {}, any>;
    export default component;
}
