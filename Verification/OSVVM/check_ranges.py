#!/usr/bin/env python3
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# check_ranges: G2.Range / G2.MinMax cross-check of Sources/ against range_map.tsv and the TBs.
#
# Every arithmetic signal/variable in Sources/ needs a range (comment [lo, hi], "full type", or a
# ranged subtype) and a range_map.tsv row: min and max bin labels found in the TB, "N/A <reason>",
# or "= <node>" for a copy of another node. A label must equal a string literal of the TB; a bin
# built with "&" is named by one of its literals. Array types, record fields and std_logic_vector
# arithmetic are not scanned.

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "Sources"
TBS = [ROOT / "Verification/OSVVM/Modules", ROOT / "Verification/OSVVM/Top"]
MAP = Path(__file__).resolve().parent / "range_map.tsv"

DECL = re.compile(r"^\s*(?:shared\s+)?(?:signal|variable)\s+([\w\s,]+?)\s*:\s*"
                  r"(unsigned|signed|integer|natural|positive)\b([^;]*);?\s*(?:--(.*))?$", re.I)
RANGE = re.compile(r"\[[^\]]*(,|\.\.)[^\]]*\]|full type", re.I)


def nodes():
    for f in sorted(SRC.glob("*.vhd")):
        for no, line in enumerate(f.read_text().splitlines(), 1):
            m = DECL.match(line)
            if not m:
                continue
            ranged = bool(RANGE.search(m.group(4) or "")) or " range " in m.group(3).lower()
            for name in m.group(1).split(","):
                yield f.stem, name.strip(), f"{f.name}:{no}", ranged


def tb_text(tb):
    for d in TBS:
        p = d / f"{tb}.vhd"
        if p.exists():
            return set(re.findall(r'"([^"]*)"', p.read_text()))
    return None


def main():
    rows = {}
    for no, line in enumerate(MAP.read_text().splitlines(), 1):
        if not line.strip() or line.startswith("#"):
            continue
        f = line.split("\t")
        rows[(f[0], f[1])] = (no, f[2:])

    gaps = []
    seen = set()
    for unit, name, where, ranged in nodes():
        key = (unit, name)
        seen.add(key)
        if not ranged:
            gaps.append(f"{where}: {name}: no range comment")
            continue
        if key not in rows:
            gaps.append(f"{where}: {name}: not in range_map.tsv")
            continue
        no, f = rows[key]
        if f[0].startswith("N/A") or f[0].startswith("="):
            if f[0].startswith("=") and (unit, f[0][1:].strip()) not in rows:
                gaps.append(f"range_map.tsv:{no}: {name}: alias of unknown node {f[0][1:].strip()}")
            elif f[0] == "N/A" or f[0] == "N/A ":
                gaps.append(f"range_map.tsv:{no}: {name}: N/A without a reason")
            continue
        if len(f) < 3:
            gaps.append(f"range_map.tsv:{no}: {name}: needs <tb> <min bin> <max bin>")
            continue
        text = tb_text(f[0])
        if text is None:
            gaps.append(f"range_map.tsv:{no}: {name}: TB {f[0]} not found")
            continue
        for side, label in (("min", f[1]), ("max", f[2])):
            if label not in text:
                gaps.append(f"range_map.tsv:{no}: {name}: {side} bin '{label}' not in {f[0]}")

    for key, (no, _) in rows.items():
        if key not in seen:
            gaps.append(f"range_map.tsv:{no}: {key[0]}.{key[1]}: no such ranged node in Sources/")

    for g in gaps:
        print(g)
    print(f"check_ranges: {len(seen)} nodes, {len(gaps)} gaps")
    return 1 if gaps else 0


if __name__ == "__main__":
    sys.exit(main())
