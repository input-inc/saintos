"""Animation-playback hot-path benchmark (before/after).

Isolates the per-tick cost an ``AnimationPlayer`` imposes on the routing
evaluator. An animation samples N value tracks per tick and pushes each
sampled value into the evaluator as a URDF-joint setpoint. The thing we
want to measure — and shrink — is how that fan-out of N joint writes
turns into sheet evaluations and UI-snapshot builds.

Why this is the interesting segment: a rig's joints typically all live
on ONE node sheet (the "arm" sheet wires every joint of the arm). With
the per-track dispatch the player does today, each of the N joints calls
``set_urdf_joint_value`` independently, and EACH call re-evaluates that
shared sheet AND rebuilds + broadcasts the full value snapshot. So an
N-joint animation frame costs ~N sheet evaluations and ~N snapshot
builds per tick — quadratic-ish in the worst case, and entirely
redundant since all N values are known at the same instant.

Two scenarios are timed per frame (a frame = one animation tick = N
joint values delivered):

  • per-track-dispatch   — the CURRENT path: N separate
                           ``set_urdf_joint_value`` calls, each
                           re-evaluating the shared sheet + rebuilding
                           the snapshot. This is the BEFORE number.

  • batch-frame          — the proposed path: one
                           ``apply_animation_frame(joint_values)`` call
                           that writes all N caches, evaluates each
                           touched sheet ONCE, and broadcasts ONCE. This
                           is the AFTER number. Skipped automatically if
                           the evaluator doesn't implement the method yet
                           (so this file runs cleanly before the fix and
                           reports both after it).

Both scenarios run with ``on_values_changed`` wired, matching production
— the snapshot build is part of what the per-track path multiplies, so
leaving it out would understate the win.

Run directly:
    python3 test/bench_animation_frame.py
    python3 test/bench_animation_frame.py --iters 50000 --joints 12 --depth 3

Or via pytest (uses conftest's rclpy stubs automatically):
    python3 -m pytest test/bench_animation_frame.py -s
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Callable, List, Tuple


# ── representative animated rig ─────────────────────────────────────


def build_animated_routing(joints: int, chain_depth: int):
    """A graph shaped like an animated multi-DOF rig.

    One ``arm`` sheet carries ``joints`` URDF-joint inputs (joint_0 …
    joint_{N-1}), each driven through a ``chain_depth``-long operator
    chain (deadband → scale → clamp → …) into its own Maestro servo
    channel — i.e. every joint of the rig lives on the SAME sheet, which
    is what makes per-track dispatch re-evaluate that sheet N times per
    tick.

    Returns (routing, joint_names) where joint_names is the ordered list
    of URDF joint names an animation frame delivers values for.
    """
    from saint_server.peripheral_model import (
        InputNode,
        OperatorNode,
        RouteEndpoint,
        SystemRouting,
        Wire,
    )

    routing = SystemRouting()
    joint_names: List[str] = []
    chain_ops = ["deadband", "scale", "clamp", "curve", "invert"]

    arm = routing.get_sheet("arm")
    for i in range(joints):
        joint = f"joint_{i}"
        joint_names.append(joint)
        inp_id = f"jin{i}"
        arm.inputs.append(
            InputNode(id=inp_id, topic="", field="value",
                      label=joint, kind="urdf_joint", joint=joint)
        )

        prev_source = RouteEndpoint(kind="input", parts=[inp_id])
        for d in range(chain_depth):
            op_id = f"op{i}_{d}"
            op = chain_ops[d % len(chain_ops)]
            arm.operators.append(OperatorNode(id=op_id, op=op))
            arm.wires.append(Wire(
                id=f"w{i}_{d}_in",
                source=prev_source,
                sink=RouteEndpoint(kind="operator", parts=[op_id, "value"]),
            ))
            prev_source = RouteEndpoint(kind="operator", parts=[op_id, "out"])
        arm.wires.append(Wire(
            id=f"w{i}_sink",
            source=prev_source,
            sink=RouteEndpoint(kind="peripheral",
                               parts=["arm", "maestro-1", f"ch{i}"]),
        ))

    return routing, joint_names


# ── measurement ─────────────────────────────────────────────────────


def _percentiles(samples_ns: List[int], ps=(50, 90, 99)) -> dict:
    s = sorted(samples_ns)
    n = len(s)
    out = {}
    for p in ps:
        k = min(n - 1, max(0, int(math.ceil(p / 100.0 * n)) - 1))
        out[p] = s[k]
    out["max"] = s[-1]
    out["mean"] = sum(s) / n
    return out


def _make_evaluator(routing):
    from saint_server.router.routing_evaluator import RoutingEvaluator

    sends = {"n": 0}

    def send_channel(*_a):
        sends["n"] += 1

    # Production-like: a values-changed broadcaster that builds (and here,
    # just drops) the snapshot. The per-track path triggers this once per
    # joint; the batch path triggers it once per frame.
    broadcasts = {"n": 0}

    def on_values_changed(_snap):
        broadcasts["n"] += 1

    ev = RoutingEvaluator(
        ros_bridge=None,
        send_channel=send_channel,
        peripheral_type_lookup=lambda *_: "maestro",
        on_values_changed=on_values_changed,
    )
    ev.reconcile(routing)
    return ev, sends, broadcasts


def _frame_values(joint_names: List[str], tick: int) -> List[Tuple[str, float]]:
    """The N (joint, value) pairs an animation delivers on tick `tick`.

    Phase-offset sinusoids so every joint changes every frame (worst
    case for any change-detection) and no two joints share a value."""
    n = len(joint_names)
    return [
        (jn, math.sin(tick * 0.05 + i * (math.pi / max(1, n))))
        for i, jn in enumerate(joint_names)
    ]


def run_per_track(joints: int, depth: int, iters: int) -> dict:
    """BEFORE: N separate set_urdf_joint_value calls per frame."""
    routing, joint_names = build_animated_routing(joints, depth)
    ev, sends, broadcasts = _make_evaluator(routing)

    for i in range(min(500, iters)):
        for jn, v in _frame_values(joint_names, i):
            ev.set_urdf_joint_value(jn, v)

    samples = [0] * iters
    for i in range(iters):
        frame = _frame_values(joint_names, i)
        t0 = time.perf_counter_ns()
        for jn, v in frame:
            ev.set_urdf_joint_value(jn, v)
        samples[i] = time.perf_counter_ns() - t0

    return _summarize("per-track-dispatch", samples, iters, joints,
                      sends["n"], broadcasts["n"])


def run_batch_frame(joints: int, depth: int, iters: int):
    """AFTER: one apply_animation_frame call per frame. Returns None if
    the evaluator doesn't implement the batch method yet."""
    routing, joint_names = build_animated_routing(joints, depth)
    ev, sends, broadcasts = _make_evaluator(routing)
    if not hasattr(ev, "apply_animation_frame"):
        return None

    for i in range(min(500, iters)):
        ev.apply_animation_frame(dict(_frame_values(joint_names, i)))

    samples = [0] * iters
    for i in range(iters):
        frame = dict(_frame_values(joint_names, i))
        t0 = time.perf_counter_ns()
        ev.apply_animation_frame(frame)
        samples[i] = time.perf_counter_ns() - t0

    return _summarize("batch-frame", samples, iters, joints,
                      sends["n"], broadcasts["n"])


def _summarize(label, samples, iters, joints, sends, broadcasts) -> dict:
    stats = _percentiles(samples)
    total_s = sum(samples) / 1e9
    return {
        "label": label,
        "iters": iters,
        "joints": joints,
        "sends": sends,
        "broadcasts": broadcasts,
        "frames_per_s": iters / total_s if total_s else float("inf"),
        "p50_us": stats[50] / 1000.0,
        "p90_us": stats[90] / 1000.0,
        "p99_us": stats[99] / 1000.0,
        "max_us": stats["max"] / 1000.0,
        "mean_us": stats["mean"] / 1000.0,
    }


def _print_report(rows: List[dict], joints: int, depth: int) -> None:
    print()
    print(f"Animation frame benchmark  "
          f"({joints} URDF joints on one sheet × depth {depth})")
    print("  per frame = one animation tick = N joint values → sheet eval(s) "
          "→ peripheral sinks")
    print("-" * 86)
    print(f"{'scenario':<22}{'mean µs':>9}{'p50 µs':>9}{'p90 µs':>9}"
          f"{'p99 µs':>9}{'max µs':>9}{'frames/s':>10}{'broadcasts':>11}")
    for r in rows:
        print(f"{r['label']:<22}{r['mean_us']:>9.2f}{r['p50_us']:>9.2f}"
              f"{r['p90_us']:>9.2f}{r['p99_us']:>9.2f}{r['max_us']:>9.2f}"
              f"{r['frames_per_s']:>10.0f}{r['broadcasts']:>11}")
    if len(rows) == 2:
        before, after = rows[0]["mean_us"], rows[1]["mean_us"]
        speedup = before / after if after else float("inf")
        print("-" * 86)
        print(f"speedup (per-track ÷ batch): {speedup:.2f}×  "
              f"({before:.2f} → {after:.2f} µs/frame mean); "
              f"broadcasts/frame {rows[0]['broadcasts'] // rows[0]['iters']} "
              f"→ {rows[1]['broadcasts'] // rows[1]['iters']}")
    print()


def run_benchmark(joints: int = 12, depth: int = 3, iters: int = 50000):
    rows = [run_per_track(joints, depth, iters)]
    after = run_batch_frame(joints, depth, iters)
    if after is not None:
        rows.append(after)
    else:
        print("\n[batch-frame skipped: RoutingEvaluator.apply_animation_frame "
              "not implemented yet — this is the BEFORE-only baseline]")
    _print_report(rows, joints, depth)
    return rows


# ── pytest entry (conftest stubs rclpy so this imports cleanly) ─────


def test_animation_frame_hot_path_smoke():
    """Smoke + propagation guard. Runs a short benchmark, prints metrics
    (visible with `pytest -s`), and asserts the chain actually reaches
    the peripheral sink. No hard timing ceiling so CI stays non-flaky;
    the printed numbers are the local before/after comparison artifact."""
    rows = run_benchmark(joints=12, depth=3, iters=8000)
    for r in rows:
        assert r["sends"] > 0, f"{r['label']}: no peripheral writes — chain broke"
        assert r["p99_us"] > 0


if __name__ == "__main__":
    import os
    import sys
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, _here)                     # find conftest
    sys.path.insert(0, os.path.dirname(_here))    # find saint_server
    import conftest  # noqa: F401  — executing it installs the stubs

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iters", type=int, default=50000,
                    help="timed animation frames per scenario")
    ap.add_argument("--joints", type=int, default=12,
                    help="URDF joints (value tracks) on the arm sheet")
    ap.add_argument("--depth", type=int, default=3,
                    help="operator-chain length per joint")
    args = ap.parse_args()
    run_benchmark(joints=args.joints, depth=args.depth, iters=args.iters)
