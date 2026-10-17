#!/usr/bin/env python3
"""Copy a binary PGM, cropping the width down to a multiple of 4."""
import sys

src, dst = sys.argv[1:3]
with open(src, "rb") as f:
    data = f.read()
tok, pos = [], 0
while len(tok) < 4:
    while data[pos:pos + 1].isspace():
        pos += 1
    if data[pos:pos + 1] == b"#":
        pos = data.index(b"\n", pos) + 1
        continue
    end = pos
    while not data[end:end + 1].isspace():
        end += 1
    tok.append(data[pos:end])
    pos = end
pos += 1
w, h, mx = (int(t) for t in tok[1:])
bps = 2 if mx > 255 else 1
wc = w - w % 4
body = data[pos:pos + w * h * bps]
rows = (body[y * w * bps:(y * w + wc) * bps] for y in range(h))
with open(dst, "wb") as f:
    f.write(b"P5\n%d %d\n%d\n" % (wc, h, mx))
    f.write(b"".join(rows))
