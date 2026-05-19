#!/usr/bin/env bash
#
# Host-runnable tests for the FAS100 driver's S.Port/FBUS parsers.
# Compiles the driver source against host stubs (SIMULATION=1) — no
# Pico SDK required. See test_fas100_parser.c for the test cases.
#
# Usage: ./run_tests.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_INC="${SCRIPT_DIR}/../../shared/include"
DRIVER_INC="${SCRIPT_DIR}/../include"

CC="${CC:-cc}"

# -Wno-unused-* because the driver's peripheral_driver_t fields + flash
# helpers aren't exercised by the parser tests — that's fine, the test
# runner just calls the parser entry points directly. -DSIMULATION
# drops the hardware UART/GPIO code so we don't need Pico SDK headers.
"${CC}" \
    -std=c11 -O0 -g \
    -Wall -Wextra \
    -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter \
    -Wno-unused-but-set-variable \
    -DSIMULATION=1 \
    -I"${DRIVER_INC}" \
    -I"${SHARED_INC}" \
    -o "${SCRIPT_DIR}/test_fas100_parser" \
    "${SCRIPT_DIR}/test_fas100_parser.c"

"${SCRIPT_DIR}/test_fas100_parser"
