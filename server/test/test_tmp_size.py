import json, os, sys, yaml
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from saint_server.peripheral_model import maestro_slim_channels_for_wire

SCRATCH = "/private/tmp/claude-503/-Users-hackman-Projects-OpenSAINT-SaintOS-source/92d501c7-776c-4e9b-a798-0fc684df54aa/scratchpad/head.yaml"

def test_size():
    d = yaml.safe_load(open(SCRATCH))
    out = []
    for p in d["peripherals"]["peripherals"]:
        if p.get("builtin"):
            continue
        params = dict(p.get("params") or {})
        if p.get("type") == "maestro":
            params = maestro_slim_channels_for_wire(params)
        out.append({"id": p["id"], "type": p["type"],
                    "pins": p.get("pins") or {}, "params": params})
    payload = {"action": "configure",
               "version": d["peripherals"]["version"], "peripherals": out}
    blob = json.dumps(payload, separators=(",", ":"))
    print(f"\nWIRE SIZE: {len(blob)} bytes (cap ~2048)")
    for per in out:
        chans = (per["params"] or {}).get("channels")
        if chans is None:
            continue
        print(f"  {per['id']}: {len(chans)} channels, "
              f"channels array = {len(json.dumps(chans, separators=(',',':')))} bytes")
        nonempty = [(i, c) for i, c in enumerate(chans) if c]
        print(f"  non-default channels: {len(nonempty)}")
        for i, c in nonempty[:30]:
            print(f"    ch{i}: {json.dumps(c, separators=(',',':'))}")
