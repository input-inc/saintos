# FlashTxx (vendored)

In-application flash write/erase for Teensy 3.x / 4.x / TMM, derived from
NXP/Freescale's original Kinetis flash routines + Paul Stoffregen /
Frank Boesing / Jon Zeeff / Deb Hollenback / Joe Pasquariello's adaptations.

Sources copied verbatim from https://github.com/joepasquariello/FlasherX
(commit at vendoring time). The author(s) released this code into the
public domain. The Freescale/NXP portions remain under their original
license — see the header comments inside `FlashTxx.c` / `FlashTxx.h`.

We use only `flash_write_block()`, `flash_erase_block()`,
`firmware_buffer_init()`, `firmware_buffer_free()`, and `flash_move()`
from the C library. We deliberately do NOT vendor `FlasherX.ino`,
`FXUtil.cpp`, or `FXUtil.h` — those handle Intel HEX parsing and serial
I/O. SAINT.OS streams the new image as raw bytes over HTTP, so the HEX
layer isn't needed.
