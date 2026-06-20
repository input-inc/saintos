/**
 * Tiny benchmark harness for the platform-agnostic shared C code.
 *
 * Drop a `bench_<module>.c` next to the tests, #include this, #include
 * the shared source under test (with whatever stubs it needs — same
 * pattern as the test_*.c files), and call BENCH()/BENCH_DIST() from
 * main(). run_benchmarks.sh compiles every bench_*.c at -O2 with the
 * host platform shim and runs it.
 *
 * Two timing modes:
 *
 *   BENCH(label, iters, STMT)
 *       Amortized: runs STMT `iters` times inside ONE timed region and
 *       reports mean ns/op + ops/sec. Use for sub-µs operations — per-
 *       call timing there is dominated by clock_gettime resolution
 *       (~1 µs on macOS), so the batch mean is the only honest number.
 *
 *   BENCH_DIST(label, iters, STMT)
 *       Distribution: times each call separately and reports
 *       p50/p90/p99/max. Use for operations ≳1 µs where per-call jitter
 *       matters and is above the clock floor.
 *
 * IMPORTANT — defeat dead-code elimination: at -O2 the compiler will
 * delete STMT if its result is unused. Either bench a function with
 * observable side effects (e.g. one that writes through a stubbed
 * transport), or fold the result into BENCH_SINK(x) inside STMT.
 *
 * ALSO — beware loop-invariant hoisting: a PURE function of CONSTANT
 * inputs is computed once and lifted out of the loop even WITH
 * BENCH_SINK (you'll see an implausible sub-ns/op). Vary an input each
 * iteration (use `_i`, the loop index) so every call does real work.
 * Functions with internal/static state (a parser fed byte-by-byte) are
 * naturally non-invariant and don't need this.
 *
 * These numbers are RELATIVE (host CPU, deterministic, good for proving
 * an optimization's delta) — NOT predictive wall-clock for the MCU.
 */
#ifndef SAINT_BENCH_HARNESS_H
#define SAINT_BENCH_HARNESS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Volatile sink so a benched expression's result can't be optimized
 * away. Usage inside STMT:  BENCH_SINK(some_result); */
static volatile uint64_t g_bench_sink;
#define BENCH_SINK(x)  (g_bench_sink += (uint64_t)(x))

static inline long long bench_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int _bench_cmp_ll(const void* a, const void* b)
{
    long long x = *(const long long*)a, y = *(const long long*)b;
    return (x > y) - (x < y);
}

/* Amortized batch timing — mean ns/op + throughput.
 *
 * The statement is variadic (trailing args) so a multi-statement body
 * with commas — e.g. an initializer list `uint8_t d[2]={a,b};` — passes
 * through intact; the preprocessor would otherwise split those commas
 * into separate macro arguments. Pass either a single expression or a
 * `{ ...; ...; }` block. `_i` is the loop index, usable for variation. */
#define BENCH(label, iters, ...)                                              \
    do {                                                                       \
        long _n = (long)(iters);                                               \
        long _warm = _n < 10000 ? _n : 10000;                                  \
        for (long _i = 0; _i < _warm; _i++) { __VA_ARGS__; }                   \
        long long _t0 = bench_now_ns();                                        \
        for (long _i = 0; _i < _n; _i++) { __VA_ARGS__; }                      \
        long long _el = bench_now_ns() - _t0;                                  \
        double _mean = (double)_el / (double)_n;                               \
        double _ops  = _el > 0 ? (double)_n / ((double)_el / 1e9) : 0.0;       \
        printf("  %-38s %8.1f ns/op   %12.0f ops/sec  (n=%ld)\n",              \
               (label), _mean, _ops, _n);                                      \
    } while (0)

/* Per-call distribution — p50/p90/p99/max. Statement is variadic for
 * the same reason as BENCH (see above). */
#define BENCH_DIST(label, iters, ...)                                         \
    do {                                                                       \
        long _n = (long)(iters);                                               \
        long _warm = _n < 10000 ? _n : 10000;                                  \
        for (long _i = 0; _i < _warm; _i++) { __VA_ARGS__; }                   \
        long long* _s = (long long*)malloc(sizeof(long long) * (size_t)_n);    \
        if (!_s) { perror("malloc"); break; }                                  \
        for (long _i = 0; _i < _n; _i++) {                                     \
            long long _a = bench_now_ns(); __VA_ARGS__;                        \
            _s[_i] = bench_now_ns() - _a;                                      \
        }                                                                      \
        qsort(_s, (size_t)_n, sizeof(long long), _bench_cmp_ll);               \
        double _sum = 0; for (long _i = 0; _i < _n; _i++) _sum += (double)_s[_i];\
        printf("  %-38s mean %.0f | p50 %lld | p90 %lld | p99 %lld | max %lld ns\n",\
               (label), _sum / (double)_n,                                     \
               _s[(long)(_n * 0.50)], _s[(long)(_n * 0.90)],                   \
               _s[(long)(_n * 0.99)], _s[_n - 1]);                             \
        free(_s);                                                              \
    } while (0)

#endif /* SAINT_BENCH_HARNESS_H */
