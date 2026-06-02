#!/usr/bin/env python3
"""Adopt the running Teensy sim node so the entrypoint's hang-repro
mode can put the firmware into ACTIVE state before triggering the
server restart that's supposed to wedge it.

Standalone from `test_sync_recovery.py` on purpose: that harness owns
node_manager lifecycle (it tears the sim down at the end), which we
explicitly do NOT want for hang repro — the firmware has to keep
running across the restart so we can watch its counters wedge or
recover. This script only:

  1. Connects + authenticates over the dashboard WebSocket.
  2. Waits for `list_unadopted` to surface a `teensy41_*` node (the
     firmware names itself from its chip ID, which on Renode is
     0000…000, so we match by prefix).
  3. Calls `adopt_node` with the SAINT.OS board id for the Teensy
     `native_eth` variant.
  4. Calls `sync_node_peripherals` so the firmware actually transitions
     UNADOPTED → ACTIVE (just adopting doesn't push a /config).
  5. Exits 0 — leaving the node running. The entrypoint does the
     restart-and-watch sequence after this exits.

Reuses the `WsClient` from the main harness so the WS protocol shape
stays in one place; this script only sequences the calls.
"""
from __future__ import annotations

import asyncio
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SIM_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(SIM_DIR))

from test_sync_recovery import (  # noqa: E402
    WsClient,
    DEFAULT_WS_URL,
    DEFAULT_PASSWORD,
    ADOPT_WAIT_S,
)


async def adopt(ws_url: str, password: str, chip_prefix: str = "teensy41_",
                board_id: str = "teensy41_native_eth") -> int:
    client = WsClient(ws_url, password)
    await client.connect()
    await asyncio.sleep(0.5)

    # Wait for the firmware to announce. list_unadopted is the same
    # API the harness uses; polling it for up to ADOPT_WAIT_S matches
    # what the e2e considers "the node should have shown up by now".
    import time
    deadline = time.monotonic() + ADOPT_WAIT_S
    announced_id = None
    while time.monotonic() < deadline:
        resp = await client.request("list_unadopted")
        nodes = (resp.get("data") or {}).get("nodes") or []
        match = next(
            (n for n in nodes
             if (n.get("node_id") or "").startswith(chip_prefix)),
            None,
        )
        if match:
            announced_id = match["node_id"]
            break
        await asyncio.sleep(1.0)

    if not announced_id:
        print(f"[hang_repro_adopt] no {chip_prefix}* node announced within "
              f"{ADOPT_WAIT_S}s — sim/agent may be wedged before adoption")
        await client.close()
        return 1

    print(f"[hang_repro_adopt] adopting {announced_id} (board={board_id})")
    await client.request("adopt_node", {
        "node_id": announced_id,
        "role": "cradle_base",
        "display_name": "HangRepro",
        "board_id": board_id,
    })
    # Give the agent's DDS endpoints time to match — same 1 s settle
    # the main harness uses to dodge the just-after-adopt empty-/log-
    # frame race.
    await asyncio.sleep(1.0)
    print(f"[hang_repro_adopt] pushing config (sync_node_peripherals)")
    await client.request("sync_node_peripherals", {"node_id": announced_id})

    # Best effort — wait for the firmware to log "Config saved to flash"
    # via the /log forwarding so we know it actually entered ACTIVE.
    # If we don't see it, continue anyway: hang-repro should still run
    # the restart even if adopt didn't fully take, so the operator can
    # see which step regressed.
    entry = await client.wait_for_node_log_containing(
        announced_id, "Config saved to flash", 10.0)
    if entry:
        print(f"[hang_repro_adopt] firmware reached ACTIVE; ready for restart")
    else:
        print(f"[hang_repro_adopt] adopt issued but no 'Config saved to flash' "
              f"ack within 10s — proceeding anyway")
    await client.close()
    return 0


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ws-url", default=DEFAULT_WS_URL)
    p.add_argument("--password", default=DEFAULT_PASSWORD)
    p.add_argument("--chip-prefix", default="teensy41_")
    p.add_argument("--board-id", default="teensy41_native_eth")
    args = p.parse_args()
    sys.exit(asyncio.run(adopt(args.ws_url, args.password,
                                args.chip_prefix, args.board_id)))
