"""Put a native `teensy_size` on the build PATH, ahead of PJRC's.

The Teensy platform reports post-link size, and enforces the flash
limit, by shelling out to `teensy_size` and failing the build on a
non-zero exit:

    result = exec_command(["teensy_size", str(source[0])], env=sysenv)
    if result["returncode"] != 0:
        env.Exit(1)

(platforms/teensy/builder/main.py, the `build_core == "teensy4"` block.)

PJRC ships that helper x86_64-only. Every published tool-teensy version
— 1.158 through 1.162 — declares its macOS file as
`darwin_x86_64,darwin_arm64`: one Intel binary claimed to serve both
architectures. On Apple Silicon it only ever ran through Rosetta, so a
macOS upgrade that removes Rosetta breaks the Teensy build at
`Calculating size` with "Bad CPU type in executable" — after every
source has compiled and firmware.elf has linked. Bumping the platform
does not help: no version of tool-teensy is native.

Interception happens through PATH rather than by replacing the SCons
method. `env.AddMethod(..., "CheckUploadSize")` from a post: script is
too late — the `checkprogsize` alias has already captured the
platform's bound method, so the override is silently ignored (observed:
the platform's own "Advanced Memory Usage is available via..." line
still printed). But the platform resolves the *binary* via PATH at
build time, so prepending our own directory wins cleanly and leaves the
platform's logic — including its treatment of a non-zero exit as fatal
— exactly as written.

tools/teensy_size does the reporting with `arm-none-eabi-size` (shipped
as a genuine darwin_arm64 build in toolchain-gccarmnoneeabi-teensy
1.150201.0+) and keeps the flash-overflow check, which is the only
thing standing between a too-large image and a bricked flash.
"""

import os

Import("env")  # noqa: F821  (injected by SCons)

def _install_shim(env):
    # SCons exec()s extra scripts without setting __file__, so derive the
    # location from the project dir the way stage_firmware.py does.
    shim_dir = os.path.join(env["PROJECT_DIR"], "tools")
    shim = os.path.join(shim_dir, "teensy_size")
    if not os.path.exists(shim):
        print("patch_teensy_size: {} missing; leaving PJRC's teensy_size in "
              "place".format(shim))
        return

    # Prepend, so we resolve ahead of the tool-teensy package.
    build_path = env["ENV"].get("PATH", "")
    if shim_dir not in build_path.split(os.pathsep):
        env["ENV"]["PATH"] = (
            shim_dir + os.pathsep + build_path if build_path else shim_dir)

    # The shim can't reach the board config, so pass the limits it needs to
    # keep enforcing the flash ceiling.
    board = env.BoardConfig()
    env["ENV"]["SAINT_TEENSY_MAX_FLASH"] = str(
        board.get("upload.maximum_size", 0) or 0)
    env["ENV"]["SAINT_TEENSY_MAX_RAM"] = str(
        board.get("upload.maximum_ram_size", 0) or 0)

    print("patch_teensy_size: native teensy_size shim ahead of tool-teensy "
          "on PATH")


_install_shim(env)  # noqa: F821
