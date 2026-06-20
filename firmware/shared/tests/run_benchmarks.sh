#!/usr/bin/env bash
#
# Performance benchmarks for the platform-agnostic shared C code.
#
# Companion to run_tests.sh (which proves correctness). This compiles
# every bench_<module>.c at -O2 (we want optimized code — that's what
# ships) with the host platform shim force-included, and runs it.
#
# Usage:
#   ./run_benchmarks.sh            # build + run every bench_*.c
#   ./run_benchmarks.sh maestro    # only benches whose name matches
#
# Adding a benchmark: drop a bench_<module>.c next to the tests that
# #includes bench_harness.h + the shared source under test (with any
# stubs it needs — same pattern as the test_*.c files) and calls
# BENCH()/BENCH_DIST() from main(). It's auto-discovered; no list to
# edit.
#
# NOTE: numbers are RELATIVE host measurements — deterministic and good
# for proving an optimization's delta, NOT predictive wall-clock for the
# RP2040/Teensy MCU. For on-target numbers, measure with the hardware
# cycle counter.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_INC="${SCRIPT_DIR}/../include"
CC="${CC:-cc}"
FILTER="${1:-}"

CFLAGS=(
    -std=c11 -O2          # optimized: bench the code as it ships
    -Wall -Wextra
    -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter
    -Wno-unused-but-set-variable
    -DSIMULATION=1
    # Same host platform shim the test runner uses — defines PLATFORM_H
    # + PLATFORM_* before any shared source pulls in the guard stub.
    -include "${SCRIPT_DIR}/host_platform.h"
    -I"${SHARED_INC}"
    -I"${SCRIPT_DIR}"
    -lm
)

shopt -s nullglob
benches=("${SCRIPT_DIR}"/bench_*.c)
if [ ${#benches[@]} -eq 0 ]; then
    echo "No bench_*.c files found."
    exit 0
fi

overall_rc=0
failed=()
for src in "${benches[@]}"; do
    name="$(basename "${src}" .c)"
    [ -n "${FILTER}" ] && [[ "${name}" != *"${FILTER}"* ]] && continue
    bin="${SCRIPT_DIR}/${name}"
    echo "=== ${name} ==="
    if ! "${CC}" "${CFLAGS[@]}" -o "${bin}" "${src}"; then
        echo "  BUILD FAILED"
        overall_rc=1; failed+=("${name} (build)"); echo; continue
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
