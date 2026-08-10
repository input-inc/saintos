# SAINT.OS — Pimoroni Servo 2040 board firmware

A small **fixed image** that turns a [Pimoroni Servo 2040](https://shop.pimoroni.com/products/servo-2040)
into an **I2C peripheral** for SAINT.OS. The board runs this image; a
SAINT.OS controller node (Teensy 4.1 / RP2040 / Raspberry Pi) drives it
over I2C as the bus master.

> **This is not a SAINT.OS node target.** It is provisioned onto the
> board **once, by hand** (drag-and-drop `.uf2`) and is deliberately kept
> out of the SAINT.OS OTA/CI firmware pipeline. From SAINT.OS's
> perspective the board arrives "pre-flashed and ready to communicate."
> The matching host-side driver is
> `firmware/shared/src/pimoroni_servo2040_driver.c`; the two share the
> wire contract in `firmware/shared/include/pimoroni_servo2040_protocol.h`.

## What it does

- Drives the **18 servo outputs** (`ServoCluster`, pio0).
- Drives the **6 onboard WS2812 RGB LEDs** (`plasma::WS2812`, pio1).
- Reports **aggregate servo current** (Analog + AnalogMux over the shared
  current-sense ADC).
- Persists per-servo **home pulses** to its own flash and **drives them
  on power-on** — mirroring the Pololu Maestro's EEPROM `HomeMode=Goto`,
  so the rig comes up at known positions before the host connects.
- **Failsafe:** relaxes all servos if the host heartbeat stops.

## Wire link

I2C on the **Qwiic / STEMMA QT** connector:

| Board pin | Signal |
|-----------|--------|
| GP20      | SDA (i2c0) |
| GP21      | SCL (i2c0) |

- Target address: `0x30` (`PIMORONI_SERVO2040_I2C_ADDR`), 400 kHz.
- Register map + semantics: `../shared/include/pimoroni_servo2040_protocol.h`.
- Servo **extents** live host-side (the driver maps −1..+1 → pulse and
  clamps); the board only ever receives absolute pulse µs. **Home**
  positions are pushed to the board (`SERVO_HOME` + `COMMIT`) and
  persisted so they survive power cycles.

Wire the controller node's Qwiic/I2C bus to the board's Qwiic port (SDA,
SCL, GND — the board is powered separately for servos).

## Building

Depends on the [Pico SDK](https://github.com/raspberrypi/pico-sdk) and
[pimoroni-pico](https://github.com/pimoroni/pimoroni-pico). Copy the two
standard import shims into this directory first (they ship with the SDKs):

```sh
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
cp $PIMORONI_PICO_PATH/pimoroni_pico_import.cmake .
```

Then:

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
export PIMORONI_PICO_PATH=/path/to/pimoroni-pico
cmake -B build -DPICO_BOARD=pico \
      -DPICO_SDK_PATH=$PICO_SDK_PATH \
      -DPIMORONI_PICO_PATH=$PIMORONI_PICO_PATH
cmake --build build
```

Produces `build/saint_servo2040.uf2`.

## Flashing (one-time provisioning)

1. Hold **BOOT/USER SW** while plugging the Servo 2040 into USB (it
   mounts as the `RPI-RP2` drive).
2. Drag `saint_servo2040.uf2` onto that drive. The board reboots running
   this firmware.

That's it — the board is now a SAINT.OS I2C peripheral. Add it from the
server **Peripherals** tab (type **Pimoroni Servo 2040**), set the
per-channel extents + home, and Sync.

## Design note

The I2C target IRQ is a dumb **register-file mem-slave** (fast, timing
safe); **all** application logic — applying servo targets, LED colors,
home commit, current sampling, failsafe — runs in the main loop by
diffing the register file. This keeps bus timing decoupled from the
servo/LED/flash work.
