#!/bin/bash
# SAINT.OS Node Firmware Build Script for Teensy 4.1
#
# Builds both simulation and hardware versions of the firmware
# using PlatformIO.
#
# Usage:
#   ./build.sh          # Build both versions
#   ./build.sh sim      # Build simulation only
#   ./build.sh hw       # Build hardware only
#   ./build.sh clean    # Clean build directories
#   ./build.sh all      # Build both (same as no argument)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# PlatformIO installs `pio` into ~/.platformio/penv/bin, which isn't on PATH
# under a non-interactive shell (or any shell that hasn't sourced the VSCode
# extension's profile). Make the script self-sufficient.
if ! command -v pio >/dev/null 2>&1 && [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
    export PATH="$HOME/.platformio/penv/bin:$PATH"
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# PlatformIO compiles the out-of-tree shared sources (../../shared/src/*,
# pulled in via build_src_filter) into a single build/shared/ object
# directory that is NOT namespaced per environment. Both `simulation`
# (-DSIMULATION) and `hardware` read/write the SAME object files, and
# PlatformIO only recompiles when the *source* is newer than the object
# — it can't tell the build flags changed. So a hardware build that runs
# after a simulation build silently links the SIMULATION-compiled
# shared objects.
#
# That is not cosmetic: with -DSIMULATION the shared code does
# `#define Serial Serial1` (platform.h) and takes other `#ifdef
# SIMULATION` branches (flash/transport). The Serial1 case alone bricks
# hardware — the first shared-code PLATFORM_PRINTF (registering the
# Maestro driver) writes to the unwired LPUART6, fills its TX FIFO, and
# blocks forever, so the node watchdog-loops at boot.
#
# Force the shared objects to rebuild with the target env's flags by
# clearing build/shared before every build. The shared tree is small
# (~10 s) — cheap insurance against a bug that builds clean and bricks
# on hardware.
clean_shared_objects() {
    rm -rf build/shared
}

build_sim() {
    echo -e "${YELLOW}Building SIMULATION firmware (Teensy 4.1)...${NC}"
    clean_shared_objects
    pio run -e simulation
    echo -e "${GREEN}Simulation build complete${NC}"
}

build_hw() {
    echo -e "${YELLOW}Building HARDWARE firmware (Teensy 4.1)...${NC}"
    clean_shared_objects
    pio run -e hardware
    echo -e "${GREEN}Hardware build complete: build/hardware/firmware.hex${NC}"
    echo -e "${GREEN}OTA artifact staged: server/resources/firmware/teensy41/saint_node.bin${NC}"
}

clean_builds() {
    echo -e "${YELLOW}Cleaning build directories...${NC}"
    pio run -t clean
    echo -e "${GREEN}Clean complete${NC}"
}

case "${1:-all}" in
    sim|simulation)
        build_sim
        ;;
    hw|hardware)
        build_hw
        ;;
    clean)
        clean_builds
        ;;
    all|"")
        build_sim
        echo ""
        build_hw
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}Both builds complete!${NC}"
        echo -e "${GREEN}  Simulation: build/simulation/${NC}"
        echo -e "${GREEN}  Hardware:   build/hardware/firmware.hex${NC}"
        echo -e "${GREEN}  OTA stage:  server/resources/firmware/teensy41/saint_node.bin${NC}"
        echo -e "${GREEN}========================================${NC}"
        ;;
    *)
        echo "Usage: $0 [sim|hw|clean|all]"
        echo "  sim   - Build simulation firmware only"
        echo "  hw    - Build hardware firmware only"
        echo "  clean - Clean build directories"
        echo "  all   - Build both (default)"
        exit 1
        ;;
esac
