"""
PlatformIO post-build script: produce + stage Teensy 4.1 firmware artifacts.

After the hardware ELF is linked, this script writes three things into
server/resources/firmware/teensy41/ atomically on every build, so the
staging area can never drift out of sync with the running build:

  - saint_node.bin       — raw flash image, streamed by the in-app OTA
                           path; the server uses this to compute the
                           OTA control message's `size` + `crc32`.
  - firmware.hex         — Intel HEX, what Teensy Loader / the manual
                           flashing tool consumes via HalfKay USB.
                           Copied directly from the build output —
                           overwrites whatever was staged previously,
                           which is the only thing keeping
                           resources/firmware/teensy41/ from accruing
                           stale HEX files from old builds.
  - generated/version.h  — sidecar so the server can read out
                           FIRMWARE_VERSION_FULL for the OTA up-to-date
                           comparison.

Mirrors what the RP2040 build pipeline stages for parity. Without
saint_node.bin the OTA falls back to a no-op reboot; without an up-to-
date firmware.hex, anyone doing a manual flash via Teensy Loader gets
old code.
"""

import os
import shutil
import struct
import zlib

Import("env")

# Resources directory is rooted at the source repo's server/resources/firmware/<type>/.
# Walk up from firmware/teensy41/ to source/, then back down.
PROJECT_DIR  = env["PROJECT_DIR"]
SOURCE_DIR   = os.path.abspath(os.path.join(PROJECT_DIR, "..", ".."))
RESOURCES    = os.path.join(SOURCE_DIR, "server", "resources", "firmware", "teensy41")


def _stage_bin(source, target, env):
    # Hooked on $BUILD_DIR/${PROGNAME}.hex — target[0] is the freshly
    # built HEX. The matching ELF lives next to it; objcopy reads the
    # ELF to produce the OTA .bin. Hooking on .hex (not .elf) is what
    # guarantees the HEX exists at the moment we try to copy it to
    # resources — the .elf post-action fires too early in PlatformIO's
    # build chain.
    hex_src = str(target[0])
    elf = hex_src[:-4] + ".elf" if hex_src.endswith(".hex") else hex_src + ".elf"
    bin_path = hex_src[:-4] + ".bin" if hex_src.endswith(".hex") else hex_src + ".bin"

    # ELF -> raw flash image. The Teensy 4.1 in-app OTA streams this
    # verbatim into FlashTxx's staging buffer, then flash_move() copies
    # it down over the running app.
    objcopy = env.subst("$OBJCOPY")
    rc = env.Execute(f'"{objcopy}" -O binary "{elf}" "{bin_path}"')
    if rc:
        print(f"!! stage_firmware: objcopy failed (rc={rc})")
        return

    with open(bin_path, "rb") as f:
        data = f.read()
    size = len(data)
    crc = zlib.crc32(data) & 0xFFFFFFFF

    print(f"** stage_firmware: {os.path.basename(bin_path)} "
          f"size={size} crc32=0x{crc:08x}")

    # Stage into server/resources/firmware/teensy41/. The server
    # recomputes size + CRC32 itself when it builds the firmware_update
    # message, so we don't need to ship a metadata file alongside.
    os.makedirs(RESOURCES, exist_ok=True)
    dst = os.path.join(RESOURCES, "saint_node.bin")
    shutil.copy2(bin_path, dst)
    print(f"** stage_firmware: staged -> {dst}")

    # Refresh firmware.hex too. shutil.copy2 overwrites, so the staged
    # copy can never drift behind the build output — solves the stale-
    # .hex problem where a HEX from a previous build would sit in
    # resources/firmware/teensy41/ untouched (because stage_firmware
    # used to ignore .hex entirely).
    if os.path.exists(hex_src):
        hex_dst = os.path.join(RESOURCES, "firmware.hex")
        shutil.copy2(hex_src, hex_dst)
        print(f"** stage_firmware: staged -> {hex_dst}")

    # Also stage the generated version.h so the server can parse the
    # FIRMWARE_VERSION_STRING / FIRMWARE_VERSION_FULL out of it for the
    # OTA control message.
    gen_dir = os.path.join(RESOURCES, "generated")
    os.makedirs(gen_dir, exist_ok=True)
    version_src = os.path.join(PROJECT_DIR, "include", "version.h")
    if os.path.exists(version_src):
        shutil.copy2(version_src, os.path.join(gen_dir, "version.h"))

    # Stage the hand-maintained info.json (board metadata: name, features,
    # architecture, etc.) from its source-of-truth location in
    # firmware/teensy41/. This file used to live at the staging path and
    # was tracked there, which mixed source with artifacts. Now the
    # staging path is purely build output (gitignored), and the source
    # file is copied here on every build so the dist tarball always has
    # the latest metadata bundled with the .bin / .hex / version.h.
    info_src = os.path.join(PROJECT_DIR, "info.json")
    if os.path.exists(info_src):
        shutil.copy2(info_src, os.path.join(RESOURCES, "info.json"))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", _stage_bin)
