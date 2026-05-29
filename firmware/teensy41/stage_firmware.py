"""
PlatformIO post-build script: produce + stage saint_node.bin

After the hardware ELF is linked, objcopy it to a raw .bin (what the
in-app OTA path streams over HTTP) and copy it — plus the auto-generated
version.h — into server/resources/firmware/teensy41/ where the server's
state_manager picks them up.

Mirrors what the RP2040 build pipeline does for parity: server
get_firmware_info_for_type('teensy41') expects saint_node.bin to exist
in order to populate the OTA control message's `size` + `crc32` fields.
Without it the node sees no metadata and falls back to a no-op reboot.
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
    elf = str(target[0])
    bin_path = elf[:-4] + ".bin" if elf.endswith(".elf") else elf + ".bin"

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

    # Also stage the generated version.h so the server can parse the
    # FIRMWARE_VERSION_STRING out of it for the update message.
    gen_dir = os.path.join(RESOURCES, "generated")
    os.makedirs(gen_dir, exist_ok=True)
    version_src = os.path.join(PROJECT_DIR, "include", "version.h")
    if os.path.exists(version_src):
        shutil.copy2(version_src, os.path.join(gen_dir, "version.h"))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", _stage_bin)
