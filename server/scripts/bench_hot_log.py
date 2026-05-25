#!/usr/bin/env python3
"""Local throughput bench for the hot-path logging changes.

NOT a unit test. NOT run in CI. Run it on your dev machine and on the
target Pi to confirm the changes (default WARN log level + _hot_log
1-of-N sampling + eval-trace → DEBUG) actually buy throughput in the
control pipeline.

The bench exercises the routing-evaluator path that the latency-
reduction doc names as the streaming hot path:

  set_ws_input(...)  →  _evaluate_sheet  →  _dispatch_sink(set_channel)

This is the per-tick work the controller's joystick stream drives at
~50 Hz. We loop ``--iters`` calls in a tight Python loop and report
ops/sec under four configurations so each lever's contribution is
visible:

  1. before        — INFO level, sampling disabled    (every call emits)
  2. level-only    — WARN level, sampling disabled    (filter at root)
  3. sampling-only — INFO level, sampling enabled     (1-of-N emit)
  4. current       — WARN level, sampling enabled     (default we ship)

What this DOES measure
  - Time spent inside the Python routing-evaluator hot path
  - The cost of the underlying Python logger emit through a real
    file handler (the same kind of handler file_log.py uses in prod)

What this DOES NOT measure
  - rclpy / rcutils emit cost (no ROS bus here)
  - The DDS publisher path (no actual ROS publish happens — we hand a
    dummy send_channel that just appends to a list)
  - Filesystem cost on the Pi (use the --logfile flag to write to a
    real path on the target's storage rather than /tmp)

If "current" beats "before" by a meaningful margin on the Pi, the
changes are doing what we claim. If it doesn't, time was being spent
somewhere else and we should re-investigate.

Usage:
  python3 -m server.scripts.bench_hot_log               # 100k iters
  python3 -m server.scripts.bench_hot_log --iters 50000
  python3 -m server.scripts.bench_hot_log --logfile /tmp/bench.log
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import tempfile
import time
from pathlib import Path


# Allow running directly from the repo (`python3 server/scripts/bench_hot_log.py`)
# without installing the package — same dance the test suite uses.
_THIS = Path(__file__).resolve()
_SERVER_ROOT = _THIS.parents[1]
if str(_SERVER_ROOT) not in sys.path:
    sys.path.insert(0, str(_SERVER_ROOT))

# `saint_server/__init__.py` eagerly imports `SaintServerNode`, which
# pulls in `rclpy` etc. The test suite stubs those in
# `server/test/conftest.py` so unit tests run on a host without ROS.
# Run that same stubbing here when ROS isn't installed so the bench
# works on a dev laptop. When you run on the actual Pi target, rclpy
# is already importable and we don't take this branch.
try:
    import rclpy  # noqa: F401
except ImportError:
    _CONFTEST = _SERVER_ROOT / "test" / "conftest.py"
    if _CONFTEST.exists():
        import importlib.util as _ilu
        _spec = _ilu.spec_from_file_location("_bench_conftest_stubs",
                                             str(_CONFTEST))
        _mod = _ilu.module_from_spec(_spec)
        # Side-effect: registers rclpy / aiohttp / etc. stubs into
        # sys.modules at module-load time so the subsequent imports
        # below succeed without a real ROS install.
        _spec.loader.exec_module(_mod)


def _build_routing(sheet_id: str = "bench-node"):
    """Realistic shape: WS Input → Add → Clamp → Peripheral channel.

    Mirrors the chain in test_routing_evaluator.py so what the bench
    measures matches what the latency-reduction doc calls the hot path.
    """
    from saint_server.peripheral_model import (
        OperatorNode, RouteEndpoint, SystemRouting,
        WebSocketInputNode, Wire,
    )
    routing = SystemRouting()
    sheet = routing.get_sheet(sheet_id)
    sheet.ws_inputs.append(WebSocketInputNode(id="wsin1", label="bench stick"))
    sheet.operators.append(OperatorNode(id="add1", op="add"))
    sheet.operators.append(OperatorNode(id="clamp1", op="clamp"))
    sheet.wires.append(Wire(
        id="w1",
        source=RouteEndpoint(kind="ws_input", parts=["wsin1"]),
        sink=RouteEndpoint(kind="operator", parts=["add1", "a"]),
    ))
    sheet.wires.append(Wire(
        id="w2",
        source=RouteEndpoint(kind="operator", parts=["add1", "out"]),
        sink=RouteEndpoint(kind="operator", parts=["clamp1", "value"]),
    ))
    sheet.wires.append(Wire(
        id="w3",
        source=RouteEndpoint(kind="operator", parts=["clamp1", "out"]),
        sink=RouteEndpoint(kind="peripheral",
                           parts=[sheet_id, "roboclaw-1", "motor"]),
    ))
    return routing, sheet_id


def _build_evaluator(logger):
    from saint_server.router.routing_evaluator import RoutingEvaluator
    routing, _ = _build_routing()
    ev = RoutingEvaluator(
        ros_bridge=None,
        send_channel=lambda *_a: None,   # cheap sink, avoid measuring list-append cost
        peripheral_type_lookup=lambda *_: "roboclaw",
        logger=logger,
    )
    ev.reconcile(routing)
    return ev


def _make_logger(path: str, level: int) -> logging.Logger:
    """Real Python logger with a real FileHandler. The handler kind is
    the same shape file_log.py uses in production so the emit cost is
    representative — just without the daily-rotation wrapper, which
    doesn't kick in mid-bench anyway."""
    lg = logging.getLogger("saint_os.bench")
    lg.setLevel(level)
    # Wipe handlers from a previous run so we don't double-emit when
    # the bench is invoked multiple times in the same interpreter.
    for h in list(lg.handlers):
        lg.removeHandler(h)
    h = logging.FileHandler(path, mode="w", encoding="utf-8")
    h.setFormatter(logging.Formatter("%(message)s"))
    lg.addHandler(h)
    lg.propagate = False
    return lg


def _run_pass(label: str, iters: int, logfile: str,
              log_level: int, sample_n: int) -> dict:
    """One bench pass. Resets the routing-evaluator module's sampling
    constant so we can fairly compare with/without sampling without
    code-patching the evaluator itself."""
    import saint_server.router.routing_evaluator as evmod

    original_n = evmod._HOT_LOG_SAMPLE_N
    evmod._HOT_LOG_SAMPLE_N = sample_n
    try:
        lg = _make_logger(logfile, log_level)
        ev = _build_evaluator(lg)

        # Warm-up to amortize first-touch costs (ROS-free, but Python
        # method resolution + dict lookups still cache better after a
        # few iterations).
        for i in range(min(500, iters)):
            ev.set_ws_input("bench-node", "wsin1", 0.5 + (i % 7) * 0.01)

        # Flush the handler so the warm-up file I/O is done before we
        # start the clock.
        for h in lg.handlers:
            h.flush()

        start = time.perf_counter()
        for i in range(iters):
            # Vary the value slightly so the call isn't optimized away
            # and the dispatch actually fires every iteration.
            ev.set_ws_input("bench-node", "wsin1", 0.5 + (i % 17) * 0.01)
        elapsed = time.perf_counter() - start

        # Force any buffered writes to disk before we measure size, so
        # the on-disk byte count reflects what actually got emitted.
        for h in lg.handlers:
            h.flush()
        emitted_bytes = os.path.getsize(logfile)
        try:
            with open(logfile, "r", encoding="utf-8", errors="replace") as f:
                emitted_lines = sum(1 for _ in f)
        except OSError:
            emitted_lines = -1

        return {
            "label": label,
            "iters": iters,
            "elapsed_s": elapsed,
            "ops_per_s": iters / elapsed if elapsed > 0 else float("inf"),
            "us_per_op": (elapsed / iters) * 1e6 if iters else 0.0,
            "lines_written": emitted_lines,
            "bytes_written": emitted_bytes,
        }
    finally:
        evmod._HOT_LOG_SAMPLE_N = original_n


def _print_results(rows):
    """Tabular output. Sized so it fits in an 80-column terminal even
    with the Pi's typical SSH font."""
    header = ("config", "iters", "elapsed s", "ops/s",
              "µs/op", "lines", "bytes")
    fmt = "{:<14} {:>8} {:>10} {:>12} {:>9} {:>8} {:>10}"
    print(fmt.format(*header))
    print("-" * 80)
    for r in rows:
        print(fmt.format(
            r["label"],
            f"{r['iters']:,}",
            f"{r['elapsed_s']:.3f}",
            f"{r['ops_per_s']:>12,.0f}",
            f"{r['us_per_op']:>9.2f}",
            f"{r['lines_written']:,}" if r['lines_written'] >= 0 else "?",
            f"{r['bytes_written']:,}",
        ))

    if len(rows) >= 2:
        baseline = rows[0]
        print()
        for r in rows[1:]:
            speedup = (r["ops_per_s"] / baseline["ops_per_s"]
                       if baseline["ops_per_s"] else float("nan"))
            saved_us = baseline["us_per_op"] - r["us_per_op"]
            print(f"  {r['label']:<14} vs {baseline['label']}:"
                  f"  {speedup:.2f}× throughput,"
                  f"  -{saved_us:.2f} µs/op")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Throughput bench for the routing-evaluator hot path."
    )
    parser.add_argument("--iters", type=int, default=100_000,
                        help="Iterations per pass (default 100k).")
    parser.add_argument("--logfile", type=str, default=None,
                        help="File to write log emits to. Default: per-pass "
                             "temp file. Pass a real path on the Pi to "
                             "measure storage cost.")
    args = parser.parse_args()

    configs = [
        # (label,         level,           sample_n)
        ("before",        logging.INFO,    1),    # every call emits
        ("level-only",    logging.WARNING, 1),    # filter at root, no sampling
        ("sampling-only", logging.INFO,    20),   # 1-of-N, no level filter
        ("current",       logging.WARNING, 20),   # what we ship by default
    ]

    rows = []
    for label, level, n in configs:
        with tempfile.NamedTemporaryFile(
                prefix=f"bench_hot_log_{label}_", suffix=".log",
                delete=False) as f:
            path = args.logfile or f.name
        try:
            row = _run_pass(label, args.iters, path, level, n)
            rows.append(row)
        finally:
            if not args.logfile:
                try:
                    os.unlink(path)
                except OSError:
                    pass

    _print_results(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
