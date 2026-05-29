# Pololu Maestro Servo Controller Protocol

The SAINT.OS firmware speaks the Pololu Maestro's serial protocol on a UART
(or USB CDC) link. This note captures everything the driver needs to know
about the wire format. The authoritative source is Pololu's *Maestro Servo
Controller User's Guide*; this is the working subset.

## Variants

| Variant                       | Channels | PWM channels | Hardware UART | Catalog id    |
| ----------------------------- | -------- | ------------ | ------------- | ------------- |
| Micro Maestro 6-Channel       | 6        | 6 (all)      | 5 V / 3.3 V   | `maestro_6`   |
| Mini Maestro 12-Channel       | 12       | 6            | 5 V / 3.3 V   | `maestro_12`  |
| Mini Maestro 18-Channel       | 18       | 12           | 5 V / 3.3 V   | `maestro_18`  |
| Mini Maestro 24-Channel       | 24       | 18           | 5 V / 3.3 V   | `maestro_24`  |

Channels that aren't dedicated PWM can still drive a servo on the Mini
Maestros, but the position resolution drops to ~5 µs vs the dedicated
channels' 0.25 µs.

## Serial modes

The Maestro Control Center utility configures one of:

* **USB Dual Port** — two virtual COM ports over USB: one for commands,
  one for diagnostics.
* **USB Chained** — single USB serial; TTL UART receives the same byte
  stream so a host can sniff or co-issue commands.
* **UART, detect baud** — TTL only; auto-detects 50–115 200 bps from the
  first 0xAA byte.
* **UART, fixed baud** — TTL only; supports 50 – 200 000 bps. Use this in
  production — the auto-detect path needs a 0xAA byte at startup which our
  driver doesn't always emit before the first real command.

`device_number` in the catalog params matches the Maestro Control Center
*Device Number* setting. Default is **12** (0x0C).

## Protocols

Three framings share the same set of commands. Pick one per UART bus:

### Compact (single device)

Just the command byte followed by data. No device-number prefix.

```
<command> <data...>
```

Use when one Maestro owns the UART.

### Pololu (multi-device, daisy chain or shared bus)

```
0xAA <device_number> <command & 0x7F> <data...>
```

The leading `0xAA` is what the auto-detect baud-rate mode locks onto.
`device_number` is 0–127; the Maestro ignores commands whose device byte
doesn't match its configured value.

Use this when multiple Maestros (or any other Pololu device) share the
UART, or when you want robustness against bus noise.

### Mini-SSC II

```
0xFF <channel> <position 0–254>
```

Legacy compatibility for hosts that already drive Scott Edwards Mini SSC
II boards. The position byte maps linearly between the channel's
`MiniSSC Offset` and `MiniSSC Offset + 254 × 4 µs` configured per channel
in the Maestro. We don't use this in SAINT.OS — the resolution is too
coarse.

## Commands (Compact framing)

All multi-byte data values are **little-endian, 7-bit-per-byte**: split
the value `v` into `lo = v & 0x7F` and `hi = (v >> 7) & 0x7F`. This is
why command bytes use bit 7 as a "command vs data" flag.

| Command            | Byte   | Body                                                  | Notes                                                                                                       |
| ------------------ | ------ | ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| Set Target         | `0x84` | `channel, target_lo, target_hi`                       | `target` in 0.25-µs units. 1500 µs = 6000 = (lo=0x70, hi=0x2E). 0 = "off" (no pulses).                       |
| Set Multiple Targets | `0x9F` | `count, first_channel, t0_lo, t0_hi, t1_lo, t1_hi, …` | **Mini Maestro 12/18/24 only.** One packet sets `count` consecutive channels.                                |
| Set Speed          | `0x87` | `channel, speed_lo, speed_hi`                         | Speed in 0.25-µs / 10-ms. `0` = unlimited. `70` ≈ 1750 µs/s = "graceful" for a normal servo.                  |
| Set Acceleration   | `0x89` | `channel, accel_lo, accel_hi`                         | Accel in 0.25-µs / 10-ms / 80-ms. Range 0–255 (top byte is ignored). `0` = unlimited.                        |
| Set PWM            | `0x8A` | `on_time_lo, on_time_hi, period_lo, period_hi`        | **Mini Maestro 12/18/24 only.** PWM on channel 8 / 12 / 12 respectively (last channel — check the manual). |
| Get Position       | `0x90` | `channel`                                              | Reply: two bytes, little-endian (NOT 7-bit-split — it's the raw 16-bit position in 0.25-µs).                |
| Get Moving State   | `0x93` | —                                                     | Reply: one byte. `0x00` = idle, `0x01` = at least one servo still moving.                                    |
| Get Errors         | `0xA1` | —                                                     | Reply: two bytes (little-endian 16-bit bitmap; reading also clears).                                         |
| Go Home            | `0xA2` | —                                                     | Snap every channel to its configured Home position.                                                          |

### Encoding helper

```c
static inline void encode_split(uint16_t v, uint8_t out[2]) {
    out[0] = (uint8_t)(v & 0x7F);
    out[1] = (uint8_t)((v >> 7) & 0x7F);
}
```

### Pololu framing wrapper

To send the same command on the Pololu bus, prefix and strip bit 7 of the
command byte:

```
Set Target (compact)  : 0x84 <ch> <lo> <hi>
Set Target (Pololu)   : 0xAA <dev> 0x04 <ch> <lo> <hi>
```

## Mapping operator-facing values

The catalog parameters `min_pulse_us` and `max_pulse_us` (default
1000 / 2000) describe the operator-facing 0.0–1.0 range. The driver
converts:

```c
uint16_t target_quarter_us = (uint16_t)((min_us + (max_us - min_us) * v) * 4);
```

A trigger keyframe value of `0.5` with the defaults becomes 1500 µs →
6000 quarter-µs → `Set Target 0x84 ch 0x70 0x2E`.

## Error bitmap (Get Errors reply)

| Bit | Name                       | Meaning                                                                              |
| --- | -------------------------- | ------------------------------------------------------------------------------------ |
| 0   | Serial Signal Error        | A hardware-level UART framing error.                                                  |
| 1   | Serial Overrun Error       | UART RX FIFO overflowed before the firmware could read it.                            |
| 2   | Serial Buffer Full         | The internal 256-byte serial buffer filled.                                           |
| 3   | Serial CRC Error           | Only set when CRC mode is enabled in the Maestro Control Center.                      |
| 4   | Serial Protocol Error      | Command byte unrecognized, or a Pololu-mode device number didn't match.               |
| 5   | Serial Timeout             | The serial-timeout-fired safety latch (channels reset to home).                       |
| 6   | Script Stack Error         | Bytecode stack overflow / underflow (only relevant if Maestro's script is in use).    |
| 7   | Script Call Stack Error    | Bytecode call stack overflow.                                                         |
| 8   | Script Program Counter Err | Bytecode jumped to an out-of-range address.                                           |

## Driver tips

* Use **Pololu protocol** in production. The 0xAA prefix is also the
  byte the auto-baud-detect path locks onto, so the same wire format
  works whether the Maestro is in fixed-baud or auto-baud mode.
* On Mini Maestros, prefer **Set Multiple Targets** for bulk updates: at
  9600 baud each Set Target costs 4.2 ms, so 24 individual sets would
  miss a 100 Hz tick. One Set Multiple Targets packet is ~25 ms for 24
  channels — still tight, but workable.
* The Maestro's **serial timeout** safety feature (configurable, default
  off) will home all channels if no command arrives within the timeout
  window. Enable it for failsafe behavior, and ensure the driver pings
  at least once per timeout interval.
* The `idle_value` catalog param drives a Set Target on every channel at
  peripheral init so the Maestro doesn't sit at 0 (= "no pulses, servo
  goes limp") when a sheet doesn't connect a wire to the channel.
