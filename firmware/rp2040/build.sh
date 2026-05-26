#!/bin/bash
# SAINT.OS Node Firmware Build Script
#
# Builds both simulation and hardware versions of the firmware.
#
# Usage:
#   ./build.sh          # Build both versions
#   ./build.sh sim      # Build simulation only
#   ./build.sh hw       # Build hardware only
#   ./build.sh clean    # Clean both build directories

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

build_sim() {
    echo -e "${YELLOW}Building SIMULATION firmware...${NC}"
    mkdir -p build_sim
    cd build_sim
    cmake -DSIMULATION=ON ..
    make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
    cd ..
    echo -e "${GREEN}Simulation build complete: build_sim/saint_node.elf${NC}"
}

build_hw() {
    echo -e "${YELLOW}Building HARDWARE firmware...${NC}"
    mkdir -p build
    cd build
    cmake -DSIMULATION=OFF ..
    make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
    cd ..
    echo -e "${GREEN}Hardware build complete: build/saint_node.elf, build/saint_node.uf2${NC}"
}

clean_builds() {
    echo -e "${YELLOW}Cleaning build directories...${NC}"
    rm -rf build build_sim
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
        echo -e "${GREEN}  Simulation: build_sim/saint_node.elf${NC}"
        echo -e "${GREEN}  Hardware:   build/saint_node.uf2${NC}"
        echo -e "${GREEN}========================================${NC}"
        ;;
    *)
        echo "Usage: $0 [sim|hw|clean|all]"
        echo "  sim   - Build simulation firmware only"
        echo "  hw    - Build hardware firmware only"
        echo "  clean - Remove all build directories"
        echo "  all   - Build both (default)"
        exit 1
        ;;
esac
