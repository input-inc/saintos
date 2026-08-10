# Pimoroni Servo 2040 — bring-up

The [Pimoroni Servo 2040](https://shop.pimoroni.com/products/servo-2040)
is an RP2040-based servo controller (18 servos, aggregate current sense,
6 onboard RGB LEDs). SAINT.OS treats it as an **I2C peripheral**,
alongside the Pololu Maestro and the native PWM servo type.

## Architecture — two pieces, one wire contract

Unlike the Maestro (which ships a host protocol in ROM), the Servo 2040
is a bare RP2040, so this integration has **two** firmware pieces that
stay in lockstep via a single shared header:

| Piece | Where | Role |
|-------|-------|------|
| **Board image** | `firmware/pimoroni_servo2040/` | Runs *on the Servo 2040*. I2C **target**. Drives servos/LEDs/current-sense; persists home to flash. Built on Pimoroni's C++ SDK. **Provisioned once, by hand** — not in OTA/CI. |
| **Host driver** | `firmware/shared/src/pimoroni_servo2040_driver.c` (Teensy + RP2040) and `firmware/raspberrypi/saint_node/peripherals/pimoroni_servo2040.py` (Pi) | Runs on the SAINT.OS controller node. I2C **master**. Holds extents, maps −1..+1 → pulse, pushes home, reads telemetry. |
| **Wire contract** | `firmware/shared/include/pimoroni_servo2040_protocol.h` | I2C register map + constants. Included by **both** ends. |

The board is never a SAINT.OS node — it's a peripheral, exactly like the
Maestro. See the peripheral-first model in `docs/PERIPHERAL_FIRST_MIGRATION.md`.

## Wiring

I2C on the board's **Qwiic / STEMMA QT** connector:

| Servo 2040 | Signal | Controller |
|------------|--------|------------|
| GP20 | SDA | node's I2C SDA |
| GP21 | SCL | node's I2C SCL |
| —    | GND | common ground |

- Board target address `0x30`, 400 kHz.
- Default host pins: **RP2040** GP2/GP3 (Feather STEMMA-QT, i2c1);
  **Teensy 4.1** Wire (SDA 18 / SCL 19); **Pi** bus 1 (GPIO2/3). Override
  from the Peripherals tab (`I2C SDA pin` / `I2C SCL pin`).
- Power the servos from the board's own servo-rail supply, not the logic
  rail.

## Flashing the board image (one-time)

```sh
cd firmware/pimoroni_servo2040
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
cp $PIMORONI_PICO_PATH/pimoroni_pico_import.cmake .
cmake -B build -DPICO_BOARD=pico -DPICO_SDK_PATH=$PICO_SDK_PATH -DPIMORONI_PICO_PATH=$PIMORONI_PICO_PATH
cmake --build build
```

Hold **BOOT/USER SW** while plugging in USB, then drag
`build/saint_servo2040.uf2` onto the `RPI-RP2` drive. See
`firmware/pimoroni_servo2040/README.md`.

## Adding it in the UI

Server **Peripherals** tab → **Add peripheral** → **Pimoroni Servo 2040**.
Set the SDA/SCL pins, per-channel extents + home, LED brightness, then
**Sync**. No controller-app (Steam Deck) changes are needed — it
discovers the channels abstractly.

Channels: `ch0..ch17` (servos, bipolar −1..+1), `led0..led5` (onboard RGB
color pickers), plus telemetry `connected` / `current_a` / `error_flags`
(rendered in the Live tab's Servo 2040 card).

## Feature parity with the Maestro

- **Extents** live host-side (`start_us` / `center_us` / `end_us` per
  channel). The driver maps the routed −1..+1 through them and clamps to
  the SDK hard limits [400, 2600] µs; the board only ever receives an
  absolute pulse. Same model as the Maestro's per-channel min/max.
- **Home** mirrors the Maestro's EEPROM `HomeMode=Goto`: the driver
  writes `SERVO_HOME[ch]` + `COMMIT`, and the board **persists home
  pulses to its own flash and drives them on power-on** — so the rig
  comes up at known positions before the host connects. The driver also
  re-homes on connect. `home_us = 0` leaves a channel relaxed at boot.

## Telemetry

The board samples aggregate servo current (3 mΩ shunt × 69 gain) and a
status bitmask; the driver polls them over I2C (~5 Hz) and emits
`current_a`, `connected`, `error_flags` via `state_emit_channels`. Status
flags: `0x01` over-current, `0x02` failsafe (host heartbeat lost),
`0x04` homed-from-flash. The board relaxes all servos if the host
heartbeat stops (~1 s).

## Wire-size budget

The 18-entry per-channel extents array is slimmed for the wire exactly
like the Maestro (`pimoroni_slim_channels_for_wire` — all-default
channels collapse to `{}`, otherwise a per-field diff). Budgets
(regression-guarded by `server/test/test_pimoroni_servo2040_wire_size_budget.py`):

| Config | Bytes | Cap |
|--------|-------|-----|
| all default | ~232 | 512 (single frame) |
| 3 customized | ~253 | 1024 |
| 18 fully customized | ~1260 | 2048 (XRCE reassembly) |

Labels/icons are display-only and never reach the wire. See the same
analysis for the Maestro in `docs/MAESTRO_BRINGUP.md`.

## Firmware integration points (for maintainers)

- Virtual-GPIO base **168** (18 servo + 6 LED channels = slab 168–191).
  Chosen below the Maestro's 200 and above every real pin so `uint8_t`
  `pin_config_t.gpio` never wraps (unlike Tic 300 / Kangaroo 364).
- `pin_types.h`: `PIN_MODE_PIMORONI_SERVO`, `PIN_CAP_PIMORONI_SERVO`,
  `params.pimoroni_servo2040`.
- `flash_types.h`: `flash_pimoroni_servo2040_config_t`, `pimoroni_tx/rx_pin`
  (reused as SDA/SCL), **flash version 12 → 13** with migration.
- Registered in both `main.c` / `main.cpp`; type map + mode-string +
  channel-offset (`ch<N>`/`led<N>`) added to both `pin_config` and
  `pin_control`.
