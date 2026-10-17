#!/usr/bin/env python3
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#
"""NL corpus for the golden suite: seeded synthetic PGMs aimed at the 4-lane
split, plus a manifest of the NEAR values to run each at.

    gen_nl_images.py <out_dir>

Writes <out_dir>/<stem>.pgm and <out_dir>/manifest.txt ("stem near ...").
Output is deterministic; a file is rewritten only when its bytes change, so
build_run.sh keeps cached results for unchanged images.

Sets:
  seam   per stripe textures: runs that end on the stripe edge, runs whose
         value carries into the next stripe's first pixel, flat rows across
         all 4 stripes, noise and gradients, at every BITNESS 8..16
  noise  full-range noise, every BITNESS (escape-heavy, few runs)
  edge   0 / MAXVAL checkerboards, every BITNESS (largest errors, LIMIT path)
  shape  W = 32 (minimal stripe), W = 36 (odd stripe width), one row,
         32 x 8192 (tall narrow), 65532 x 4 (max lane line, RUNindex 30)
"""
import os
import random
import sys

BITS_ALL = range(8, 17)
BITS_SHAPE = (8, 12, 16)


def nmax(bits):
    return min(255, ((1 << bits) - 1) // 2)


def seam(bits, w, h, rng):
    mx = (1 << bits) - 1
    ws = w // 4
    img = []
    for y in range(h):
        row = []
        mode = y % 4
        for x in range(w):
            lane, c = divmod(x, ws)
            if mode == 0:                       # flat across every stripe
                v = mx // 3
            elif mode == 1:                     # run ends on each stripe edge
                v = (lane * mx) // 4 if c < ws - 1 else (lane * mx) // 4 + 1 + y % 3
            elif mode == 2:                     # stripe value carries into next stripe's col 0
                v = ((lane + (c > 0)) * mx) // 5
            elif lane == 0:
                v = mx // 2 if c % 7 else rng.randint(0, mx)
            elif lane == 1:
                v = min(mx, (c * mx) // ws + y)
            elif lane == 2:
                v = rng.randint(0, mx)
            else:
                v = mx if c == 0 else (mx // 4 if (c // 3) % 2 else 3 * mx // 4)
            row.append(v)
        img.append(row)
    return img


def noise(bits, w, h, rng):
    mx = (1 << bits) - 1
    return [[rng.randint(0, mx) for _ in range(w)] for _ in range(h)]


def edge(bits, w, h, rng):
    mx = (1 << bits) - 1
    return [[mx if (x + y + (rng.random() < 0.05)) % 2 else 0 for x in range(w)] for y in range(h)]


def smooth(bits, w, h, rng):
    mx = (1 << bits) - 1
    base = rng.randint(0, mx)
    return [[max(0, min(mx, base + x // 3 - y // 5 + rng.randint(-2, 2))) for x in range(w)] for y in range(h)]


def maxline(bits, w, h, rng):
    """Row 0 flat (one 16383-pixel run per lane), then flat rows broken once per stripe."""
    mx = (1 << bits) - 1
    ws = w // 4
    img = [[mx // 2] * w]
    for y in range(1, h):
        row = [mx // 2] * w
        for lane in range(4):
            row[lane * ws + rng.randrange(ws)] = rng.randint(0, mx)
        img.append(row)
    return img


def corpus():
    """(stem, gen, bits, w, h, seed, nears)"""
    out = []
    for b in BITS_ALL:
        n = nmax(b)
        out.append(("nl-seam-b%d" % b, seam, b, 128, 64, 1, (0, 1, 3, n // 2, n)))
        out.append(("nl-noise-b%d" % b, noise, b, 64, 64, 2, (0, 3, n)))
        out.append(("nl-edge-b%d" % b, edge, b, 64, 32, 3, (0, 7, n)))
    for b in BITS_SHAPE:
        n = nmax(b)
        out.append(("nl-w32-h1-b%d" % b, noise, b, 32, 1, 4, (0, 3, n)))
        out.append(("nl-w32-b%d" % b, seam, b, 32, 256, 5, (0, 3, n)))
        out.append(("nl-w36-b%d" % b, smooth, b, 36, 64, 6, (0, 3, n)))
        out.append(("nl-row-b%d" % b, smooth, b, 4096, 1, 7, (0, 3, n)))
        out.append(("nl-tall-b%d" % b, seam, b, 32, 8192, 8, (0, 3)))
        out.append(("nl-maxline-b%d" % b, maxline, b, 65532, 4, 9, (0, 3)))
    return out


def pgm_bytes(bits, img):
    mx = (1 << bits) - 1
    h, w = len(img), len(img[0])
    body = bytearray()
    for row in img:
        if mx > 255:
            for v in row:
                body += bytes((v >> 8, v & 0xFF))
        else:
            body += bytes(row)
    return b"P5\n%d %d\n%d\n" % (w, h, mx) + bytes(body)


def write_if_changed(path, data):
    try:
        with open(path, "rb") as f:
            if f.read() == data:
                return False
    except FileNotFoundError:
        pass
    with open(path, "wb") as f:
        f.write(data)
    return True


def main():
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    lines, changed = [], 0
    for stem, gen, bits, w, h, seed, nears in corpus():
        img = gen(bits, w, h, random.Random(seed * 100 + bits))
        changed += write_if_changed(os.path.join(out_dir, stem + ".pgm"), pgm_bytes(bits, img))
        lines.append(stem + " " + " ".join(map(str, sorted(set(nears)))))
    write_if_changed(os.path.join(out_dir, "manifest.txt"), ("\n".join(lines) + "\n").encode())
    print("NL corpus: %d images (%d rewritten) in %s" % (len(lines), changed, out_dir))


if __name__ == "__main__":
    sys.exit(main())
