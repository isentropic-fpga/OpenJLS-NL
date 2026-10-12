#!/usr/bin/env python3
"""Synthetic test image: gradients, noise, flat areas (runs) and edges.
Usage: gen_image.py W H BITS SEED out.pgm out.hex"""
import random
import sys

w, h, bits, seed = map(int, sys.argv[1:5])
rng = random.Random(seed)
mx = (1 << bits) - 1
pix = []
for y in range(h):
    for x in range(w):
        reg = (x * 5 // w + y * 3 // max(h, 1) + seed) % 5
        if reg == 0:
            v = mx // 3                                      # flat -> runs
        elif reg == 1:
            v = (x * mx) // max(w - 1, 1)                    # ramp
        elif reg == 2:
            v = rng.randint(0, mx)                           # noise
        elif reg == 3:
            v = mx // 2 + rng.randint(-mx // 32, mx // 32)   # low noise
        else:
            v = mx if (x // 3 + y // 2) % 2 else 0           # edges
        pix.append(max(0, min(mx, v)))
with open(sys.argv[5], "wb") as f:
    f.write(b"P5\n%d %d\n%d\n" % (w, h, mx))
    for v in pix:
        f.write(v.to_bytes(2, "big") if mx > 255 else bytes([v]))
with open(sys.argv[6], "w") as f:
    f.writelines("%04X\n" % v for v in pix)
