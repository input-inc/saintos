// Canonical RoboClaw fault decode — mirrors the ROBOCLAW_FAULT_* bits the
// firmware emits on the `error_flags` telemetry channel
// (firmware/shared/include/roboclaw_protocol.h). The firmware normalizes
// both the 16- and 32-bit RoboClaw status words into THIS stable set, so
// the UI decodes one table regardless of the controller's firmware rev.

export const ROBOCLAW_FAULTS = [
  { bit: 0x0001, label: 'E-Stop',                     severity: 'error' },
  { bit: 0x0002, label: 'Over-current',               severity: 'error' },
  { bit: 0x0004, label: 'Over-temperature',           severity: 'error' },
  { bit: 0x0008, label: 'Temperature warning',        severity: 'warn'  },
  { bit: 0x0010, label: 'Main battery over-voltage',  severity: 'error' },
  { bit: 0x0020, label: 'Main battery under-voltage', severity: 'error' },
  { bit: 0x0040, label: 'Logic battery out of range', severity: 'error' },
  { bit: 0x0080, label: 'Driver fault',               severity: 'error' },
  { bit: 0x0100, label: 'Speed error limit',          severity: 'error' },
  { bit: 0x0200, label: 'Position error limit',       severity: 'error' },
]

// Decode a canonical bitmask into the list of active faults.
export function decodeRoboclawFaults (bits) {
  const b = (Number(bits) | 0) & 0xFFFF
  if (!b) return []
  return ROBOCLAW_FAULTS.filter(f => b & f.bit)
}

// True if any decoded fault is a hard error (vs. a warning) — used to
// pick badge/severity colouring.
export function roboclawHasError (bits) {
  return decodeRoboclawFaults(bits).some(f => f.severity === 'error')
}
