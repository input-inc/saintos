// useThrottledSend rate-limits streaming control writes (slider/joystick
// drags) to the server, but ALWAYS delivers the final value via a
// trailing call so a motor isn't left at a stale setpoint when the user
// stops dragging mid-window. Fake timers let us assert the leading +
// trailing-coalesce behaviour deterministically.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { useThrottledSend } from '../useThrottledSend'

describe('useThrottledSend', () => {
  beforeEach(() => {
    vi.useFakeTimers()
    // Start the clock well past 0 so the very first call (lastAt=0) is
    // immediately eligible to fire (now - 0 >= ms).
    vi.setSystemTime(1_000_000)
  })
  afterEach(() => {
    vi.useRealTimers()
  })

  it('fires the leading call immediately', () => {
    const sent = []
    const call = useThrottledSend(v => sent.push(v), 50)
    call('a')
    expect(sent).toEqual(['a'])
  })

  it('throttles within the window and delivers the LAST value as a trailing call', () => {
    const sent = []
    const call = useThrottledSend(v => sent.push(v), 50)
    call('a')        // leading → sent immediately
    call('b')        // within window → scheduled
    call('c')        // within window → coalesced (replaces pending)
    expect(sent).toEqual(['a'])

    vi.advanceTimersByTime(50)
    expect(sent).toEqual(['a', 'c']) // 'b' was superseded by 'c'
  })

  it('fires the leading edge again once the window has elapsed', () => {
    const sent = []
    const call = useThrottledSend(v => sent.push(v), 50)
    call('a')                 // leading
    vi.advanceTimersByTime(60) // window fully elapses, nothing pending
    call('b')                 // eligible immediately again
    expect(sent).toEqual(['a', 'b'])
  })

  it('does not double-send when no further calls arrive after the leading edge', () => {
    const sent = []
    const call = useThrottledSend(v => sent.push(v), 50)
    call('a')
    vi.advanceTimersByTime(200)
    expect(sent).toEqual(['a']) // no spurious trailing fire
  })
})
