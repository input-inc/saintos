<!--
    Dropdown component built on @headlessui/vue's Listbox primitive.
    Replaces native `<select>` elements throughout the app — the
    native popup is rendered by webkit2gtk as a separate GTK window
    that ignores the webview's zoom level, so on Steam Deck at any
    scale other than 1× the dropdown menu items render tiny while the
    rest of the UI scales correctly. This component renders the
    options as a regular HTML overlay so the zoom applies uniformly.

    API mirrors a basic `<select v-model>` with `options: { value, label }`:
        <SaintSelect v-model="choice" :options="[
            { value: 1.0,  label: '100%' },
            { value: 1.25, label: '125%' },
        ]" />
-->
<script setup lang="ts" generic="T">
import { computed } from 'vue';
import {
    Listbox,
    ListboxButton,
    ListboxOption,
    ListboxOptions,
} from '@headlessui/vue';

interface Option {
    value: T;
    label: string;
    disabled?: boolean;
}

const props = defineProps<{
    modelValue: T;
    options: Option[];
    placeholder?: string;
    disabled?: boolean;
}>();

const emit = defineEmits<{
    'update:modelValue': [value: T];
}>();

const selected = computed(() => props.options.find(o => o.value === props.modelValue));
const displayLabel = computed(() => selected.value?.label ?? props.placeholder ?? '');
</script>

<template>
    <Listbox :model-value="modelValue"
             :disabled="disabled"
             @update:model-value="(v: T) => emit('update:modelValue', v)">
        <div class="relative">
            <ListboxButton
                class="input w-full text-left flex items-center justify-between gap-2"
                :class="{ 'opacity-50 cursor-not-allowed': disabled }">
                <span class="truncate">{{ displayLabel }}</span>
                <span class="material-icons text-saint-text-muted text-base shrink-0">
                    expand_more
                </span>
            </ListboxButton>

            <transition leave-active-class="transition duration-100 ease-in"
                        leave-from-class="opacity-100"
                        leave-to-class="opacity-0">
                <ListboxOptions
                    class="absolute z-50 mt-1 w-full max-h-60 overflow-auto rounded-lg
                           bg-saint-surface border border-saint-surface-light shadow-xl
                           focus:outline-none">
                    <ListboxOption v-for="opt in options" :key="String(opt.value)"
                                   :value="opt.value"
                                   :disabled="opt.disabled"
                                   v-slot="{ active, selected: isSelected, disabled: isDisabled }"
                                   as="template">
                        <li class="cursor-pointer px-3 py-2 flex items-center justify-between gap-2"
                            :class="[
                                active ? 'bg-saint-primary text-white' : 'text-saint-text',
                                isDisabled ? 'opacity-50 cursor-not-allowed' : '',
                            ]">
                            <span class="truncate" :class="{ 'font-medium': isSelected }">
                                {{ opt.label }}
                            </span>
                            <span v-if="isSelected" class="material-icons text-base shrink-0">
                                check
                            </span>
                        </li>
                    </ListboxOption>
                </ListboxOptions>
            </transition>
        </div>
    </Listbox>
</template>
