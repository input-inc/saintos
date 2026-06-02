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

# Functions to neuter. Order is purely cosmetic (sets the log order).
NEUTERED_SYMBOLS = (
    "configure_cache",
    "usb_pll_start",
    "set_arm_clock",
    "configure_external_ram",
    "usb_init",
)


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

    patches: list[tuple[str, int]] = []
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        for name in NEUTERED_SYMBOLS:
            try:
                patches.append((name, _find_symbol_file_offset(elf, name)))
            except Exception as exc:  # noqa: BLE001
                print(f"[patch_sim_elf] ERROR: {exc}", file=sys.stderr)
                sys.exit(1)

    with open(elf_path, "r+b") as f:
        for name, offset in patches:
            f.seek(offset)
            original = f.read(2)
            f.seek(offset)
            f.write(THUMB_BX_LR)
            print(
                f"[patch_sim_elf] {name:>24s} @ file offset 0x{offset:06x}: "
                f"{original.hex()} -> {THUMB_BX_LR.hex()} (bx lr)"
            )


env.AddPostAction("$BUILD_DIR/firmware.elf", patch_sim_elf)  # noqa: F821
