#!/usr/bin/env bash
#
# Host-runnable tests for shared firmware modules. Each test runner is
# a self-contained .c file that #includes the source under test (with
# -DSIMULATION=1 so hardware paths drop) and exercises it through the
# public API with stubbed neighbors.
#
# Usage: ./run_tests.sh
#
# Adding a test: drop a test_<name>.c next to the existing ones and
# add its basename to TEST_BINARIES below.
#
# Mirrors the convention in firmware/rp2040/tests/run_tests.sh so a
# new shared module gets covered the same way a new RP2040 driver
# does today.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_INC="${SCRIPT_DIR}/../include"

CC="${CC:-cc}"

CFLAGS=(
    -std=c11 -O0 -g
    -Wall -Wextra
    -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter
    -Wno-unused-but-set-variable
    -DSIMULATION=1
    -I"${SHARED_INC}"
    -I"${SCRIPT_DIR}"
)

TEST_BINARIES=(
    "test_crc32"
    "test_saint_log"
    "test_maestro_driver"
    "test_ota_streamer"
    "test_peripheral_state_emit"
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
