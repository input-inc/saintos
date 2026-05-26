<!--
    Virtual joystick component. Tracks pointer drags inside a circular
    "base" and emits a normalized (x, y) position in [-1, 1].
    Optionally mirrors an external position (e.g. live gamepad axis
    state) when not being dragged, so on-screen visualization tracks
    physical input.
-->
<script setup lang="ts">
import { computed, ref, onMounted, onBeforeUnmount } from 'vue';

export interface JoystickPosition {
    x: number; // -1 to 1
    y: number; // -1 to 1
}

const props = withDefaults(defineProps<{
    size?: number;
    knobSize?: number;
    /** Live external position (e.g. gamepad axis). When the operator
     *  isn't dragging, the knob renders from this. */
    externalPosition?: JoystickPosition | null;
}>(), {
    size: 80,
    knobSize: 40,
    externalPosition: null,
});

const emit = defineEmits<{
    'position-change': [position: JoystickPosition];
}>();

const baseEl = ref<HTMLDivElement | null>(null);
const isDragging = ref(false);
const localPosition = ref<JoystickPosition>({ x: 0, y: 0 });

const hasExternalInput = computed(() => {
    const e = props.externalPosition;
    return e !== null && (Math.abs(e.x) > 0.05 || Math.abs(e.y) > 0.05);
});

// Drag in progress → show the local (operator-driven) position.
// External signal above noise floor → show it.
// Neither → centered.
const position = computed<JoystickPosition>(() => {
    if (isDragging.value) return localPosition.value;
    if (props.externalPosition && hasExternalInput.value) return props.externalPosition;
    return localPosition.value;
});

const knobLeft = computed(() => {
    const maxOffset = (props.size - props.knobSize) / 2;
    return (props.size - props.knobSize) / 2 + position.value.x * maxOffset;
});

const knobTop = computed(() => {
    const maxOffset = (props.size - props.knobSize) / 2;
    return (props.size - props.knobSize) / 2 - position.value.y * maxOffset;
});

function updatePosition(clientX: number, clientY: number): void {
    const el = baseEl.value;
    if (!el) return;
    const rect = el.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    const maxRadius = (props.size - props.knobSize) / 2;

    let deltaX = clientX - centerX;
    let deltaY = centerY - clientY; // Invert Y so up is positive

    const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    if (distance > maxRadius) {
        deltaX = (deltaX / distance) * maxRadius;
        deltaY = (deltaY / distance) * maxRadius;
    }

    const normalizedX = deltaX / maxRadius;
    const normalizedY = deltaY / maxRadius;
    localPosition.value = { x: normalizedX, y: normalizedY };
    emit('position-change', { x: normalizedX, y: normalizedY });
}

function onStart(event: MouseEvent): void {
    event.preventDefault();
    isDragging.value = true;
    updatePosition(event.clientX, event.clientY);
}

function onTouchStart(event: TouchEvent): void {
    event.preventDefault();
    if (event.touches.length > 0) {
        isDragging.value = true;
        updatePosition(event.touches[0].clientX, event.touches[0].clientY);
    }
}

function onMouseMove(event: MouseEvent): void {
    if (isDragging.value) updatePosition(event.clientX, event.clientY);
}

function onTouchMove(event: TouchEvent): void {
    if (isDragging.value && event.touches.length > 0) {
        updatePosition(event.touches[0].clientX, event.touches[0].clientY);
    }
}

function onEnd(): void {
    if (isDragging.value) {
        isDragging.value = false;
        localPosition.value = { x: 0, y: 0 };
        emit('position-change', { x: 0, y: 0 });
    }
}

// Window-level listeners so drags that wander outside the base still
// track. Vue's @-syntax handlers are bound to the element; for global
// drag tracking we register on window in a lifecycle hook.
onMounted(() => {
    window.addEventListener('mousemove', onMouseMove);
    window.addEventListener('touchmove', onTouchMove);
    window.addEventListener('mouseup', onEnd);
    window.addEventListener('touchend', onEnd);
    window.addEventListener('touchcancel', onEnd);
});

onBeforeUnmount(() => {
    window.removeEventListener('mousemove', onMouseMove);
    window.removeEventListener('touchmove', onTouchMove);
    window.removeEventListener('mouseup', onEnd);
    window.removeEventListener('touchend', onEnd);
    window.removeEventListener('touchcancel', onEnd);
});
</script>

<template>
    <div ref="baseEl"
         class="relative bg-saint-background rounded-full border-2 select-none touch-none"
         :class="hasExternalInput ? 'border-saint-success' : 'border-saint-surface-light'"
         :style="{ width: `${size}px`, height: `${size}px` }"
         @mousedown="onStart($event)"
         @touchstart="onTouchStart($event)">
        <div class="absolute rounded-full cursor-pointer transition-colors shadow-lg"
             :class="[
                 isDragging ? 'bg-saint-primary' :
                 (hasExternalInput ? 'bg-saint-success' : 'bg-saint-surface-light')
             ]"
             :style="{
                 width: `${knobSize}px`,
                 height: `${knobSize}px`,
                 left: `${knobLeft}px`,
                 top: `${knobTop}px`,
             }">
        </div>
    </div>
</template>
