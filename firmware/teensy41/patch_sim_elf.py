"""
PlatformIO post-build script (simulation env only): rewrite the entry
of several Teensy-core startup functions to `bx lr` (Thumb `0x4770`) so
firmware.elf can run under Renode without touching hardware Renode
doesn't model.

What gets neutered, and why
---------------------------
* `configure_cache` — walks all 16 MPU regions, enables MPU, invalidates
  I-cache via ICIALLU, then enables I/D cache via CCR. Renode's
  Cortex-M model faults somewhere in that sequence and dumps us into
  `unused_interrupt_vector`.
* `usb_pll_start` — spin-polls `CCM_ANALOG_PLL_USB1` waiting for ENABLE,
  POWER, LOCK, BYPASS, EN_USB_CLKS to settle. Renode doesn't model the
  PLL, so the bits never change and the loop is infinite.
* `set_arm_clock` — clockspeed.c, same story: polls PLL lock bits while
  reprogramming the ARM core PLL. The sim runs at "as fast as the JIT
  can go" regardless of these registers.
* `configure_external_ram` — Teensy 4.1 PSRAM init over FlexSPI2.
  FlexSPI2 isn't connected to anything in our .repl.
* `usb_init` — kicks the USB device controller, which Renode doesn't
  model. The startup path also has two `while (millis() < N)` delay
  loops bracketing this call — those depend on SysTick (modeled, fine).

Why not `-Wl,--wrap=<sym>`?
---------------------------
All of these are called from `ResetHandler2` / inline siblings inside
the framework's `startup.o`, i.e. the calls are resolved within the same
translation unit. `--wrap` only redirects *undefined* references, so it
silently does nothing for same-TU calls.

What this script does
---------------------
1. Resolves each target symbol's file offset in firmware.elf
   (pyelftools — present in PlatformIO's penv).
2. Overwrites the first 2 bytes of each with `0x70 0x47` (Thumb `bx lr`).
   The rest of the function body stays as dead code in flash — only the
   entry instruction changes.

All targets are `void (void)` (or have their return value safely ignored
by the call sites in startup.c), so the immediate-return shim is
semantically benign for the sim. The hardware build is untouched —
this script is wired in via `extra_scripts` only on `env:simulation`.
"""

import sys

from elftools.elf.elffile import ELFFile

Import("env")  # noqa: F821 — provided by PlatformIO at runtime

THUMB_BX_LR = b"\x70\x47"  # `bx lr`, Thumb encoding, little-endian

# `bx lr` neuter: just return immediately. Right for functions whose
# side effects (PLL config, USB init, cache enable, ...) only make sense
# against real hardware.
NEUTERED_SYMBOLS = (
    "configure_cache",
    "usb_pll_start",
    "set_arm_clock",
    "configure_external_ram",
    "usb_init",
)

# Synchronous-direct-write shim for `HardwareSerialIMXRT::write9bit(uint32_t)`.
#
# The original buffers the byte in a software ring and relies on the
# LPUART TX interrupt to drain it. Renode's NXP_LPUART asserts the IRQ
# correctly, but the dispatched handler never runs (NVIC ISER0 bit-25
# write doesn't appear to take effect under Renode for this peripheral),
# so the ring fills, `write9bit` spins on `tx_buffer_tail_ == head`, and
# no bytes ever land in Renode's UART analyzer.
#
# Replacement (14 bytes — fits inside the original ~200 B function):
#   ldr  r3, [r0, #16]    ; r3 = this->port_addr (offset 16 — confirmed
#                         ;       from disasm at HardwareSerialIMXRT::begin,
#                         ;       which uses the same offset).
#   .poll:
#   ldr  r2, [r3, #20]    ; r2 = port->STAT (offset 0x14)
#   lsls r2, r2, #8       ; shift TDRE (bit 23) up into N flag; only
#                         ; r2 is touched (caller-save, fine to clobber).
#   bpl  .poll            ; loop while TDRE=0 (TX FIFO full)
#   str  r1, [r3, #28]    ; *(uint32_t*)(port_addr + 0x1C) = c (DATA)
#   movs r0, #1           ; return 1 (one byte "transmitted")
#   bx   lr
#
# r0 = `this` (AAPCS hidden first arg for non-static method).
# r1 = c (first explicit `uint32_t` arg).
# r0..r3 + r12 are caller-save; r4..r11 must be preserved across a
# function. The earlier shim used `lsls r4, r2, #8` for the flag
# shift, which corrupted r4 without saving it — that played havoc
# with the caller's locals and we ended up writing zeros to DATA
# every iteration. Shifting in-place on r2 keeps the whole shim
# inside caller-save registers, so no push/pop is needed.
#
# The TDRE poll is critical: Renode's NXP_LPUART exposes a 4-deep TX
# FIFO and emits "Trying to write to a full Tx FIFO" when it's
# overflowed, silently dropping bytes. Without the poll, any
# Serial.printf burst longer than ~4 bytes loses the tail. The .repl
# wires `lpuart6 IRQ -> nvic@25` but the NVIC doesn't dispatch the
# IRQHandler, so the firmware can't drain by interrupt — polling is
# the only synchronous option that survives.
#
# Reading port_addr from `this` rather than hard-coding LPUART6 means
# Serial2/3/... still work if the firmware ever opens them.
WRITE9BIT_SYNC_SHIM = bytes((
    0x03, 0x69,  # ldr  r3, [r0, #16]
    0x5A, 0x69,  # .poll: ldr  r2, [r3, #20]
    0x12, 0x02,  # lsls r2, r2, #8
    0xFC, 0xD5,  # bpl  .poll   (back to offset 2)
    0xD9, 0x61,  # str  r1, [r3, #28]
    0x01, 0x20,  # movs r0, #1
    0x70, 0x47,  # bx   lr
))

# Mangled name of `HardwareSerialIMXRT::write9bit(unsigned long)` —
# verified via `arm-none-eabi-nm` against the current build. If g++
# ever changes mangling rules or the signature, this lookup will fail
# loudly (good) rather than silently doing nothing.
WRITE9BIT_SYMBOL = "_ZN19HardwareSerialIMXRT9write9bitEm"


def _find_symbol_file_offset(elf, name: str) -> int:
    """Resolve `name`'s file offset inside the open ELFFile.

    Returns the offset at which the symbol's bytes live on disk (not the
    runtime virtual address). pyelftools handles the section/segment math.
    """
    symtab = elf.get_section_by_name(".symtab")
    if symtab is None:
        raise RuntimeError("ELF has no symbol table (.symtab) — unstripped build expected")

    symbols = symtab.get_symbol_by_name(name)
    if not symbols:
        raise RuntimeError(f"symbol '{name}' not found")
    sym = symbols[0]

    # Thumb function addresses carry the LSB set as a state marker; the
    # actual code starts one byte lower.
    vaddr = sym["st_value"] & ~1

    section = elf.get_section(sym["st_shndx"])
    return section["sh_offset"] + (vaddr - section["sh_addr"])


def patch_sim_elf(source, target, env):  # noqa: ARG001 — PIO callback signature
    elf_path = str(target[0])

    bx_lr_patches: list[tuple[str, int]] = []
    write9bit_offset: int | None = None
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        for name in NEUTERED_SYMBOLS:
            try:
                bx_lr_patches.append((name, _find_symbol_file_offset(elf, name)))
            except Exception as exc:  # noqa: BLE001
                print(f"[patch_sim_elf] ERROR: {exc}", file=sys.stderr)
                sys.exit(1)
        try:
            write9bit_offset = _find_symbol_file_offset(elf, WRITE9BIT_SYMBOL)
        except Exception as exc:  # noqa: BLE001
            print(f"[patch_sim_elf] ERROR locating write9bit: {exc}", file=sys.stderr)
            sys.exit(1)

    with open(elf_path, "r+b") as f:
        for name, offset in bx_lr_patches:
            f.seek(offset)
            original = f.read(2)
            f.seek(offset)
            f.write(THUMB_BX_LR)
            print(
                f"[patch_sim_elf] {name:>24s} @ file offset 0x{offset:06x}: "
                f"{original.hex()} -> {THUMB_BX_LR.hex()} (bx lr)"
            )

        assert write9bit_offset is not None  # set above or we'd have exited
        f.seek(write9bit_offset)
        original = f.read(len(WRITE9BIT_SYNC_SHIM))
        f.seek(write9bit_offset)
        f.write(WRITE9BIT_SYNC_SHIM)
        print(
            f"[patch_sim_elf] {'write9bit (sync shim)':>24s} @ file offset 0x{write9bit_offset:06x}: "
            f"{original.hex()} -> {WRITE9BIT_SYNC_SHIM.hex()}"
        )


env.AddPostAction("$BUILD_DIR/firmware.elf", patch_sim_elf)  # noqa: F821
