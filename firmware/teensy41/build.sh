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

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

build_sim() {
    echo -e "${YELLOW}Building SIMULATION firmware (Teensy 4.1)...${NC}"
    pio run -e simulation
    echo -e "${GREEN}Simulation build complete${NC}"
}

build_hw() {
    echo -e "${YELLOW}Building HARDWARE firmware (Teensy 4.1)...${NC}"
    pio run -e hardware
    echo -e "${GREEN}Hardware build complete: .pio/build/hardware/firmware.hex${NC}"
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
        echo -e "${GREEN}  Simulation: .pio/build/simulation/${NC}"
        echo -e "${GREEN}  Hardware:   .pio/build/hardware/firmware.hex${NC}"
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
