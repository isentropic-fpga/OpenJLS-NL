#!/usr/bin/env python3
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#
"""nl_unpack: split an nl_top chunk stream into one .jls per band, image and slice.

Reference unpacker for Docs/output_format.md.

Usage: nl_unpack.py STREAM [-o FORMAT]
FORMAT fields: {band} {image} {slice}; default "b{band}_i{image}_s{slice}.jls".
Exit status 1 if the stream had to be resynced or an image is incomplete.
"""

import argparse
import sys

MARKER = b"\xff\xb0"
VERSION = 1
HDR = 8


def parse_header(b, i):
    """(band, image, slice, last, length) at offset i, or None."""
    if i + HDR > len(b) or b[i:i + 2] != MARKER:
        return None
    v = b[i + 2]
    if v >> 4 != VERSION or v & 1:
        return None
    n = int.from_bytes(b[i + 6:i + 8], "big")
    if n == 0:
        return None
    return b[i + 3], int.from_bytes(b[i + 4:i + 6], "big"), (v >> 2) & 3, bool(v & 2), n


def unpack(b):
    """Completed slices as [((band, image, slice), jls)], and the errors."""
    errors = []
    parts = {}
    done = []
    i = 0
    synced = True
    while i < len(b):
        h = parse_header(b, i)
        end = i + HDR + h[4] if h else 0
        # Out of sync, an FF B0 inside a header could be taken: also need the next header
        ok = h is not None and end <= len(b) and (synced or end == len(b) or parse_header(b, end) is not None)
        if not ok:
            if synced:
                errors.append("offset %d: no valid header, resyncing" % i)
            synced = False
            i += 1
            continue
        synced = True
        band, image, sl, last, n = h
        key = (band, image, sl)
        parts.setdefault(key, bytearray()).extend(b[i + HDR:end])
        if last:
            done.append((key, bytes(parts.pop(key))))
        i = end
    for band, image, sl in parts:
        errors.append("band %d image %d slice %d: no last chunk" % (band, image, sl))
    return done, errors


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("stream")
    ap.add_argument("-o", "--output", default="b{band}_i{image}_s{slice}.jls")
    a = ap.parse_args()
    with open(a.stream, "rb") as f:
        b = f.read()
    done, errors = unpack(b)
    written = set()
    for (band, image, sl), jls in done:
        name = a.output.format(band=band, image=image, slice=sl)
        if name in written:
            errors.append("%s written twice (image_count wrapped?)" % name)
        written.add(name)
        with open(name, "wb") as f:
            f.write(jls)
    for e in errors:
        print("nl_unpack: " + e, file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
