import { describe, it, expect } from 'vitest'
import { mount } from '@vue/test-utils'
import { nextTick, ref, h } from 'vue'
import NumberField from '../NumberField.vue'

/**
 * Regression tests for the animation editor's "the field won't let me
 * type my number" bug. See NumberField.vue's header for the mechanism.
 *
 * A note on fidelity: happy-dom only partially implements `<input
 * type="number">` value sanitization. It blanks "-" and "abc" like a
 * browser does, but keeps "2." and "1e", where a real browser reports
 * `el.value === ""`. So the browser is STRICTER than what these tests
 * run against, and the failure being fixed is correspondingly worse in
 * production than it looks here.
 *
 * What survives that gap is the part that matters: the component must
 * never substitute its own text for what the operator typed. That is
 * asserted directly below, and it is the behaviour that makes both the
 * sanitized and unsanitized intermediates safe.
 */

/* Type a string one character at a time into a mounted field, the way
 * an operator does: each keystroke appends to whatever the box holds
 * and fires `input`. Returns the text visible at the end. */
async function typeInto (input, text, { clear = true } = {}) {
  input.element.focus()
  await input.trigger('focus')
  if (clear) {
    input.element.value = ''
    await input.trigger('input')
  }
  for (const ch of text) {
    input.element.value = input.element.value + ch
    await input.trigger('input')
  }
  return input.element.value
}

describe('NumberField', () => {
  it('lets a decimal be typed end to end', async () => {
    const w = mount(NumberField, {
      props: { modelValue: 1, decimals: 2, 'onUpdate:modelValue': v => w.setProps({ modelValue: v }) },
    })
    const input = w.find('input')

    const visible = await typeInto(input, '2.5')
    expect(visible).toBe('2.5')

    await input.trigger('blur')
    await nextTick()
    expect(w.props('modelValue')).toBe(2.5)
  })

  it('never rewrites the box while it is being typed in', async () => {
    const w = mount(NumberField, {
      props: { modelValue: 1, decimals: 2, 'onUpdate:modelValue': v => w.setProps({ modelValue: v }) },
    })
    const input = w.find('input')
    input.element.focus()
    await input.trigger('focus')

    // "0.05" passes through "0", "0." and "0.0" — each of which the old
    // code rounded to "0.00" and slammed back into the element.
    for (const partial of ['0', '0.', '0.0', '0.05']) {
      input.element.value = partial
      await input.trigger('input')
      await nextTick()
      // Exactly what was typed, still there after Vue has re-rendered.
      expect(input.element.value).toBe(partial)
    }

    await input.trigger('blur')
    await nextTick()
    expect(w.props('modelValue')).toBe(0.05)
  })

  it('streams parseable input live for preview', async () => {
    const seen = []
    const w = mount(NumberField, {
      props: { modelValue: 0, decimals: 2, 'onUpdate:modelValue': v => seen.push(v) },
    })
    await typeInto(w.find('input'), '12')
    expect(seen).toEqual([1, 12])
  })

  it('does not emit for a partially-typed number', async () => {
    const seen = []
    const w = mount(NumberField, {
      props: { modelValue: 7, decimals: 2, 'onUpdate:modelValue': v => seen.push(v) },
    })
    const input = w.find('input')
    input.element.focus()
    await input.trigger('focus')
    input.element.value = '-'        // sanitized to "" by the element
    await input.trigger('input')
    expect(seen).toEqual([])
  })

  it('keeps the last good value when the field is left blank', async () => {
    const w = mount(NumberField, {
      props: { modelValue: 3.25, decimals: 2, 'onUpdate:modelValue': v => w.setProps({ modelValue: v }) },
    })
    const input = w.find('input')
    input.element.focus()
    await input.trigger('focus')
    input.element.value = ''
    await input.trigger('input')
    await input.trigger('blur')
    await nextTick()
    expect(w.props('modelValue')).toBe(3.25)
    expect(input.element.value).toBe('3.25')
  })

  it('rounds and clamps only on commit', async () => {
    const seen = []
    const w = mount(NumberField, {
      props: {
        modelValue: 0, decimals: 2, min: 0, max: 5,
        'onUpdate:modelValue': v => { seen.push(v); w.setProps({ modelValue: v }) },
      },
    })
    const input = w.find('input')
    input.element.focus()
    await input.trigger('focus')
    input.element.value = '9.999'
    await input.trigger('input')
    // Live pass-through: untouched, so typing toward a value inside the
    // range isn't fought on the way.
    expect(seen.at(-1)).toBe(9.999)

    await input.trigger('blur')
    await nextTick()
    expect(w.props('modelValue')).toBe(5)
    expect(input.element.value).toBe('5.00')
  })

  it('reformats to the configured precision after commit', async () => {
    const w = mount(NumberField, {
      props: { modelValue: 0, decimals: 2, 'onUpdate:modelValue': v => w.setProps({ modelValue: v }) },
    })
    const input = w.find('input')
    await typeInto(input, '1.239')
    await input.trigger('blur')
    await nextTick()
    expect(w.props('modelValue')).toBe(1.24)
    expect(input.element.value).toBe('1.24')
  })

  it('follows the model while unfocused', async () => {
    const w = mount(NumberField, { props: { modelValue: 1, decimals: 2 } })
    await w.setProps({ modelValue: 4.5 })
    await nextTick()
    expect(w.find('input').element.value).toBe('4.50')
  })
})

/**
 * The shape the old code had, kept as a live demonstration that the bug
 * was real and that the component is what fixes it. If this ever starts
 * passing, `<input type="number">` sanitization changed and the
 * component's premise is worth re-reading.
 */
describe('the controlled pattern NumberField replaces', () => {
  const Controlled = {
    props: ['modelValue'],
    emits: ['update:modelValue'],
    setup (props, { emit }) {
      const fmt2 = n => (Number.isFinite(Number(n)) ? Number(n).toFixed(2) : '0.00')
      return () => h('input', {
        type: 'number',
        value: fmt2(props.modelValue),
        onInput: e => emit('update:modelValue', Number(Number(e.target.value).toFixed(2))),
      })
    },
  }

  it('rewrites the box out from under the typist', async () => {
    const model = ref(1)
    const w = mount(Controlled, {
      props: { modelValue: model.value, 'onUpdate:modelValue': v => { model.value = v; w.setProps({ modelValue: v }) } },
    })
    const input = w.find('input')

    // Operator selects all and types "2", intending "2.5".
    input.element.value = '2'
    await input.trigger('input')
    await nextTick()

    // The box no longer holds what they typed — ".00" was appended and
    // the caret is now behind it. Every further keystroke lands in the
    // wrong place, which is what "it auto-completes my number" is.
    expect(input.element.value).toBe('2.00')

    // Carrying on: the "." keystroke produces "2.00." which is not a
    // valid number, so the element blanks it and the handler stores 0.
    input.element.value = ''
    await input.trigger('input')
    await nextTick()
    expect(model.value).toBe(0)
    expect(input.element.value).toBe('0.00')
  })

  it('NumberField leaves the same keystroke alone', async () => {
    const w = mount(NumberField, {
      props: { modelValue: 1, decimals: 2, 'onUpdate:modelValue': v => w.setProps({ modelValue: v }) },
    })
    const input = w.find('input')
    input.element.focus()
    await input.trigger('focus')
    input.element.value = '2'
    await input.trigger('input')
    await nextTick()
    expect(input.element.value).toBe('2')
  })
})
