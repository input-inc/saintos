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
    # Force-include the host platform shim FIRST so every test TU has
    # PLATFORM_H + the PLATFORM_* macros defined before any shared source
    # pulls in the guard-stub platform.h. Without this, sources that use
    # PLATFORM_MILLIS() (maestro_driver.c, saint_log.c) won't compile
    # off-target. See host_platform.h.
    -include "${SCRIPT_DIR}/host_platform.h"
    -I"${SHARED_INC}"
    -I"${SCRIPT_DIR}"
)

TEST_BINARIES=(
    "test_crc32"
    "test_saint_log"
    "test_maestro_driver"
    "test_ota_streamer"
    "test_peripheral_state_emit"
    "test_control_message"
)

# Don't let one compile/run failure abort the whole sweep — we want to
# see EVERY test's status in a single run, not just the first breakage
# (that masking is how the harness silently rotted). Track + report all.
overall_rc=0
failed=()
for name in "${TEST_BINARIES[@]}"; do
    src="${SCRIPT_DIR}/${name}.c"
    bin="${SCRIPT_DIR}/${name}"
    echo "=== ${name} ==="
    if ! "${CC}" "${CFLAGS[@]}" -o "${bin}" "${src}"; then
        echo "  BUILD FAILED"
        overall_rc=1; failed+=("${name} (build)")
        echo; continue
    fi
    if ! "${bin}"; then
        overall_rc=1; failed+=("${name} (run)")
    fi
    echo
done

if [ "${overall_rc}" -ne 0 ]; then
    echo "FAILURES: ${failed[*]}"
fi
exit "${overall_rc}"
