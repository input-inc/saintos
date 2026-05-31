#!/usr/bin/env bash
#
# Host-runnable tests for RP2040 drivers. Each test runner is a single
# self-contained .c file that #includes the driver source directly,
# stubs the platform/log headers, and runs with -DSIMULATION=1 so no
# Pico SDK is needed.
#
# Usage: ./run_tests.sh
#
# Adding a new driver: drop a test_<driver>.c next to the existing ones
# and append its name to the TEST_BINARIES list below.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_INC="${SCRIPT_DIR}/../../shared/include"
DRIVER_INC="${SCRIPT_DIR}/../include"

CC="${CC:-cc}"

# -Wno-unused-* because the driver's peripheral_driver_t fields + flash
# helpers aren't all exercised by each test runner — that's fine, the
# test code just calls the entry points it cares about directly.
# -DSIMULATION drops the hardware UART/GPIO code so we don't need Pico
# SDK headers.
CFLAGS=(
    -std=c11 -O0 -g
    -Wall -Wextra
    -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter
    -Wno-unused-but-set-variable
    -DSIMULATION=1
    -I"${DRIVER_INC}"
    -I"${SHARED_INC}"
)

TEST_BINARIES=(
    "test_fas100_parser"
    "test_roboclaw_driver"
    "test_maestro_driver"
    "test_tic_driver"
    "test_tmc2208_driver"
)

overall_rc=0
for name in "${TEST_BINARIES[@]}"; do
    src="${SCRIPT_DIR}/${name}.c"
    bin="${SCRIPT_DIR}/${name}"
    echo "=== ${name} ==="
    "${CC}" "${CFLAGS[@]}" -o "${bin}" "${src}"
    if ! "${bin}"; then
        overall_rc=1
    fi
    echo
done

exit "${overall_rc}"
