#!/usr/bin/env python3
"""Combine two RP2040 UF2 files into one.

Each UF2 block is 512 bytes with its own target flash address. The
"merge" is just concatenation of the block streams from each input,
with `block_no` (offset 24) and `num_blocks` (offset 28) renumbered so
the combined file presents as a single sequence.

Usage:
    combine_uf2.py OUTPUT.uf2 INPUT1.uf2 [INPUT2.uf2 ...]
"""

import struct
import sys


UF2_BLOCK_SIZE = 512
UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30


def read_blocks(path):
    blocks = []
    with open(path, "rb") as f:
        while True:
            chunk = f.read(UF2_BLOCK_SIZE)
            if not chunk:
                break
            if len(chunk) != UF2_BLOCK_SIZE:
                raise SystemExit(f"{path}: truncated UF2 block ({len(chunk)} bytes)")
            blocks.append(bytearray(chunk))
    # Verify the magic on every block.
    for i, b in enumerate(blocks):
        m0, m1 = struct.unpack_from("<II", b, 0)
        me = struct.unpack_from("<I", b, 508)[0]
        if m0 != UF2_MAGIC_START0 or m1 != UF2_MAGIC_START1 or me != UF2_MAGIC_END:
            raise SystemExit(f"{path}: block {i} has bad UF2 magic")
    return blocks


def main(argv):
    if len(argv) < 3:
        raise SystemExit("usage: combine_uf2.py OUTPUT.uf2 INPUT1.uf2 [INPUT2.uf2 ...]")
    out_path = argv[1]
    inputs = argv[2:]

    all_blocks = []
    for p in inputs:
        all_blocks.extend(read_blocks(p))

    total = len(all_blocks)
    for i, b in enumerate(all_blocks):
        struct.pack_into("<I", b, 24, i)       # block_no
        struct.pack_into("<I", b, 28, total)   # num_blocks

    with open(out_path, "wb") as f:
        for b in all_blocks:
            f.write(b)

    print(f"Combined {total} blocks from {len(inputs)} input(s) into {out_path}")


if __name__ == "__main__":
    main(sys.argv)
