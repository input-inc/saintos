"""Server-side routing hot-path benchmark.

Measures the per-tick latency of the controller→peripheral translation
that lives entirely on the server: a WebSocket input value entering
``RoutingEvaluator.set_ws_input`` → sheet evaluation (operator chains)
→ peripheral output dispatch. ROS and the network are stubbed out
(``send_channel`` just counts calls), so the numbers isolate SERVER
COMPUTE — exactly the segment we want as tight as possible.

Why a standalone harness: it lets us baseline the current code, make a
change, and re-run to prove the delta locally without a robot or the
full ROS stack.

Two scenarios are reported so we can attribute cost:
  • control-only          — ``on_values_changed=None``. Pure
                            evaluate + dispatch: the latency a peripheral
                            write actually experiences.
  • with-values-broadcast — ``on_values_changed`` wired (like
                            production). The evaluator builds a full
                            value snapshot for the routing-canvas UI on
                            EVERY tick; the gap vs control-only is that
                            snapshot-build overhead riding the hot path.

Run directly:
    python3 test/bench_routing_evaluator.py
    python3 test/bench_routing_evaluator.py --iters 200000 --channels 18 --depth 3

Or via pytest (uses conftest's rclpy stubs automatically):
    python3 -m pytest test/bench_routing_evaluator.py -s
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Callable, List, Tuple


# ── representative robot graph ──────────────────────────────────────


def build_robot_routing(channels_per_sheet: int, chain_depth: int):
    """A graph shaped like a real SAINT rig.

    `head`  — a Maestro with `channels_per_sheet` servo channels, each
              driven by its own WS input through a `chain_depth`-long
              operator chain (deadband → scale → clamp → …), the common
              "shape the stick then send it" pattern.
    `track_l` / `track_r` — one RoboClaw motor each, single deadband.

    Returns (routing, drive_points) where drive_points is the list of
    (sheet_id, ws_input_id) the benchmark rotates through.
    """
    from saint_server.peripheral_model import (
        OperatorNode,
        RouteEndpoint,
        SystemRouting,
        WebSocketInputNode,
        Wire,
    )

    routing = SystemRouting()
    drive_points: List[Tuple[str, str]] = []
    # Cycle of single-input operators so chains are realistic shapes.
    chain_ops = ["deadband", "scale", "clamp", "curve", "invert"]

    def add_channel(sheet, sheet_id, idx, sink_parts):
        ws_id = f"in{idx}"
        sheet.ws_inputs.append(WebSocketInputNode(id=ws_id, label=f"axis {idx}"))
        drive_points.append((sheet_id, ws_id))

        prev_source = RouteEndpoint(kind="ws_input", parts=[ws_id])
        for d in range(chain_depth):
            op_id = f"op{idx}_{d}"
            op = chain_ops[d % len(chain_ops)]
            sheet.operators.append(OperatorNode(id=op_id, op=op))
            # Single-input ops read pin "value"; feed the prior stage in.
            sheet.wires.append(Wire(
                id=f"w{idx}_{d}_in",
                source=prev_source,
                sink=RouteEndpoint(kind="operator", parts=[op_id, "value"]),
            ))
            prev_source = RouteEndpoint(kind="operator", parts=[op_id, "out"])
        # Final stage → peripheral sink.
        sheet.wires.append(Wire(
            id=f"w{idx}_sink",
            source=prev_source,
            sink=RouteEndpoint(kind="peripheral", parts=sink_parts),
        ))

    head = routing.get_sheet("head")
    for i in range(channels_per_sheet):
        add_channel(head, "head", i, ["head", "maestro-1", f"ch{i}"])

    for side in ("track_l", "track_r"):
        sheet = routing.get_sheet(side)
        add_channel(sheet, side, 0, [side, "roboclaw-1", "motor"])

    return routing, drive_points


# ── measurement ─────────────────────────────────────────────────────


def _percentiles(samples_ns: List[int], ps=(50, 90, 99)) -> dict:
    s = sorted(samples_ns)
    n = len(s)
    out = {}
    for p in ps:
        # nearest-rank
        k = min(n - 1, max(0, int(math.ceil(p / 100.0 * n)) - 1))
        out[p] = s[k]
    out["max"] = s[-1]
    out["mean"] = sum(s) / n
    return out


def run_scenario(label: str, on_values_changed: Callable | None,
                 channels: int, depth: int, iters: int) -> dict:
    from saint_server.router.routing_evaluator import RoutingEvaluator

    routing, drive_points = build_robot_routing(channels, depth)

    sends = {"n": 0}

    def send_channel(*_a):
        sends["n"] += 1

    evaluator = RoutingEvaluator(
        ros_bridge=None,
        send_channel=send_channel,
        peripheral_type_lookup=lambda *_: "maestro",
        on_values_changed=on_values_changed,
    )
    evaluator.reconcile(routing)

    npoints = len(drive_points)
    # Warm up (first eval allocates per-sheet snapshot dicts, JIT of
    # interpreter caches, etc.) so we measure steady state.
    for i in range(min(2000, iters)):
        sid, wid = drive_points[i % npoints]
        evaluator.set_ws_input(sid, wid, math.sin(i * 0.05))

    samples = [0] * iters
    for i in range(iters):
        sid, wid = drive_points[i % npoints]
        val = math.sin(i * 0.05)
        t0 = time.perf_counter_ns()
        evaluator.set_ws_input(sid, wid, val)
        samples[i] = time.perf_counter_ns() - t0

    stats = _percentiles(samples)
    total_s = sum(samples) / 1e9
    return {
        "label": label,
        "iters": iters,
        "sends": sends["n"],
        "ticks_per_s": iters / total_s if total_s else float("inf"),
        "p50_us": stats[50] / 1000.0,
        "p90_us": stats[90] / 1000.0,
        "p99_us": stats[99] / 1000.0,
        "max_us": stats["max"] / 1000.0,
        "mean_us": stats["mean"] / 1000.0,
    }


def _print_report(rows: List[dict], channels: int, depth: int) -> None:
    print()
    print(f"Routing hot-path benchmark  "
          f"(head: {channels} Maestro ch × depth {depth} + 2 RoboClaw sheets)")
    print("  per call = set_ws_input → evaluate sheet → dispatch peripheral sinks")
    print("-" * 78)
    print(f"{'scenario':<24}{'mean µs':>9}{'p50 µs':>9}{'p90 µs':>9}"
          f"{'p99 µs':>9}{'max µs':>9}{'ticks/s':>10}")
    for r in rows:
        print(f"{r['label']:<24}{r['mean_us']:>9.2f}{r['p50_us']:>9.2f}"
              f"{r['p90_us']:>9.2f}{r['p99_us']:>9.2f}{r['max_us']:>9.2f}"
              f"{r['ticks_per_s']:>10.0f}")
    if len(rows) == 2:
        delta = rows[1]["mean_us"] - rows[0]["mean_us"]
        print("-" * 78)
        print(f"snapshot-build overhead per tick (with-values − control): "
              f"{delta:+.2f} µs mean")
    print()


def run_benchmark(channels: int = 12, depth: int = 3, iters: int = 100000):
    rows = [
        run_scenario("control-only", None, channels, depth, iters),
        run_scenario("with-values-broadcast",
                     lambda _snap: None, channels, depth, iters),
    ]
    _print_report(rows, channels, depth)
    return rows


# ── pytest entry (conftest stubs rclpy so this imports cleanly) ─────


def test_routing_hot_path_smoke():
    """Smoke + propagation guard. Runs a short benchmark, prints metrics
    (visible with `pytest -s`), and asserts the chain actually reaches
    the peripheral sink — no hard timing ceiling here so CI stays
    non-flaky; the printed numbers are the local comparison artifact."""
    rows = run_benchmark(channels=12, depth=3, iters=20000)
    for r in rows:
        assert r["sends"] > 0, f"{r['label']}: no peripheral writes — chain broke"
        assert r["p99_us"] > 0


if __name__ == "__main__":
    # Direct-run path: replicate the rclpy/aiohttp/etc. stubs that
    # conftest installs for pytest, so `import saint_server.*` works
    # without the ROS stack present.
    import os
    import sys
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, _here)                     # find conftest
    sys.path.insert(0, os.path.dirname(_here))    # find saint_server
    import conftest  # noqa: F401  — executing it installs the stubs

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iters", type=int, default=100000,
                    help="timed set_ws_input calls per scenario")
    ap.add_argument("--channels", type=int, default=12,
                    help="Maestro channels on the head sheet")
    ap.add_argument("--depth", type=int, default=3,
                    help="operator-chain length per channel")
    args = ap.parse_args()
    run_benchmark(channels=args.channels, depth=args.depth, iters=args.iters)
