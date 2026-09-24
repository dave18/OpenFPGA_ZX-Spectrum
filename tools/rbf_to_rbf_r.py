#!/usr/bin/env python3
"""Convert a Quartus .rbf into the Analogue Pocket's bitstream.rbf_r.

The Pocket expects the raw binary bitstream with every byte bit-reversed
(this is the standard, community-verified transform used across openFPGA
Pocket cores - see e.g. https://github.com/syltendo/pocket-image-viewer).

Usage:
    python3 tools/rbf_to_rbf_r.py input.rbf output.rbf_r
"""
import sys


def bit_reverse_byte(b: int) -> int:
    r = 0
    for _ in range(8):
        r = (r << 1) | (b & 1)
        b >>= 1
    return r


TABLE = bytes(bit_reverse_byte(b) for b in range(256))


def main() -> None:
    if len(sys.argv) != 3:
        print("usage: rbf_to_rbf_r.py input.rbf output.rbf_r")
        sys.exit(2)
    src, dst = sys.argv[1], sys.argv[2]
    with open(src, "rb") as f:
        data = f.read()
    with open(dst, "wb") as f:
        f.write(data.translate(TABLE))
    print(f"wrote {dst} ({len(data)} bytes)")


if __name__ == "__main__":
    main()
