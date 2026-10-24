#!/usr/bin/env bash
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

# Post-synthesis smoke test. nl_top is synthesized (OOC, Vivado) at BITNESS 8
# and the funcsim netlist runs under NVC on the small images of the golden
# corpus: its stream, split by Scripts/nl_unpack.py, must match CharLS per stripe, byte-exact.
#
# Images: the OpenJLS corpus, the T.87 PGMs and the NL corpus
# (Golden/gen_nl_images.py), 8 bit, <= MAX_MP, cropped to
# W % 4 = 0, W >= 32. Each image runs at one NEAR, rotating over 0, 3 and NMAX.
# Jobs are split into shards; one NVC elaboration per shard runs the images
# of its manifest back to back (tb_nl_postsynth resets nl_top between them).
#
# BITS="8 12 16" adds bitnesses. Netlists are reused while the RTL hash is unchanged (--synth forces).
# Needs `vivado` on PATH for synthesis and the NVC Xilinx libraries
# (one-time `nvc --install vivado`).
#
# Usage:  [MAX_MP=0.01] [NUMBER_OF_THREADS=20] [PER_MEM_MB=1500] [MEM_BUDGET_MB=25000] ./build_run.sh [--synth]
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
GOLD="$HERE/../Golden"
OJ_FULL="${OJ_FULL:-$HOME/Repos/OpenJLS}"
CHARLS="$OJ_FULL/ThirdParty/charls"
REF_DIR="$OJ_FULL/Verification/T87 conformance/Reference Images"
IMAGES_DIR="${IMAGES_DIR:-$OJ_FULL/Verification/Golden model/Images}"

BITS="${BITS:-8}"
MAX_W=4096
MAX_H=4096
OUT_WIDTH=64
MAX_MP="${MAX_MP:-0.01}"
NT="${NUMBER_OF_THREADS:-$(( $(nproc) < 20 ? $(nproc) : 20 ))}"
PER_MEM_MB="${PER_MEM_MB:-1500}"     # measured ~1.2 GB per shard (8 bit)
MEM_BUDGET_MB="${MEM_BUDGET_MB:-25000}"
NS=$(( NT < MEM_BUDGET_MB / PER_MEM_MB ? NT : MEM_BUDGET_MB / PER_MEM_MB ))
NVC_HEAP="${NVC_HEAP:-1g}"

OUT="$HERE/Output"
SYN="$OUT/synth"
CROP="$OUT/cropped"
REFD="$OUT/ref"
RUND="$OUT/runs"
LOGD="$OUT/logs"
mkdir -p "$SYN" "$CROP" "$REFD" "$RUND" "$LOGD"

# 1. Netlists (synth.sh: rebuilt when the RTL hash changes).
BITS="$BITS" "$HERE/synth.sh" "$@"

# 2. Reference encoder, gated on the official T16E0 / T16E3.
g++ -std=c++17 -O2 -DCHARLS_STATIC -I"$CHARLS/include" "$GOLD/ref/nl_charls.cpp" \
  "$CHARLS/build/libcharls.a" -o "$OUT/nl_charls"
for n in 0 3; do
  "$OUT/nl_charls" "$REF_DIR/TEST16.PGM" $n 1 "$OUT/gate"
  cmp -s "$OUT/gate0.jls" "$REF_DIR/T16E$n.JLS" \
    || { echo "FATAL: CharLS does not reproduce T16E$n.JLS" >&2; exit 1; }
done

# 3. Jobs: "bits pixels stem cropped near", one per eligible image.
python3 "$GOLD/gen_nl_images.py" "$OUT/Images_nl" > /dev/null
JOBS="$OUT/jobs.tsv"
python3 - "$MAX_MP" "$BITS" "$CROP" "$IMAGES_DIR" "$REF_DIR" "$OUT/Images_nl" > "$JOBS" <<'EOF'
import glob, os, subprocess, sys
cap, bits, crop = float(sys.argv[1]), set(map(int, sys.argv[2].split())), sys.argv[3]
here = os.path.dirname(crop)
seen, jobs = set(), []
for d in sys.argv[4:]:
    for p in sorted(glob.glob(os.path.join(d, "*.pgm")) + glob.glob(os.path.join(d, "*.PGM"))):
        stem = os.path.splitext(os.path.basename(p))[0]
        if stem in seen:
            continue
        seen.add(stem)
        t = open(p, "rb").read(256).split(None, 4)
        w, h, mx = int(t[1]), int(t[2]), int(t[3])
        b, wc = mx.bit_length(), w - w % 4
        if b not in bits or wc < 32 or wc > 4096 or h > 4096 or wc * h > cap * 1e6:
            continue
        jobs.append((b, wc * h, stem, p))
for b in sorted(bits):
    nmax = min(255, ((1 << b) - 1) // 2)
    for i, (_, px, stem, p) in enumerate(sorted((j for j in jobs if j[0] == b), key=lambda j: j[2])):
        c = os.path.join(crop, stem + ".pgm")
        if not os.path.exists(c) or os.path.getmtime(p) > os.path.getmtime(c):
            subprocess.run([sys.executable, os.path.join(here, "..", "..", "Golden", "crop4.py"), p, c], check=True)
        print("%d\t%d\t%s\t%s\t%d" % (b, px, stem, c, (0, 3, nmax)[i % 3]))
EOF
NJ=$(wc -l < "$JOBS")
echo "Jobs: $NJ image(s) <= $MAX_MP MP ($(cut -f1 "$JOBS" | sort -n | uniq -c | awk '{printf "%s%d x %d bit", (NR > 1 ? ", " : ""), $1, $2}'))"

# CharLS reference per job (4 stripes), in parallel. A failure leaves no
# ref file and is reported by the compare below.
while IFS=$'\t' read -r _ _ stem c near; do
  mkdir -p "$REFD/${stem}_N$near" "$RUND/${stem}_N$near"
  echo "$c $near 4 $REFD/${stem}_N$near/ref"
done < "$JOBS" | xargs -P "$NT" -L 1 "$OUT/nl_charls" > /dev/null 2>&1 || true

# 4. Shards: NS split over the bitnesses by pixel count, then LPT inside each.
rm -f "$OUT"/shard_*.txt "$LOGD"/shard_*.log
python3 - "$JOBS" "$NS" "$OUT" "$RUND" <<'EOF'
import sys
jobs_f, nt, out, rund = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
jobs = [l.rstrip("\n").split("\t") for l in open(jobs_f) if l.strip()]
by = {}
for b, px, stem, c, near in jobs:
    by.setdefault(int(b), []).append((int(px), stem, c, near))
tot = sum(int(j[1]) for j in jobs)
share = {b: max(1, round(nt * sum(p for p, *_ in v) / tot)) for b, v in by.items()}
for b, v in sorted(by.items()):
    n = min(share[b], len(v))
    load, bins = [0] * n, [[] for _ in range(n)]
    for px, stem, c, near in sorted(v, reverse=True):
        k = load.index(min(load))
        load[k] += px
        bins[k].append("%s %s %s/%s_N%s/out" % (c, near, rund, stem, near))
    for k, lines in enumerate(bins):
        with open("%s/shard_%d_%d.txt" % (out, b, k), "w") as f:
            f.write("\n".join(reversed(lines)) + "\n")
EOF

# 5. Compile one work library per netlist, then run the shards in parallel.
NVC=(nvc --std=2008 --ieee-warnings=off -H "$NVC_HEAP" -M 256m)
for b in $BITS; do
  lib="$OUT/work-lib/b$b"; mkdir -p "$lib"
  "${NVC[@]}" -L "$lib" --work=work:"$lib/work.08" -a --relaxed \
    "$SYN/nl_top_${b}bit_funcsim.vhd" "$HERE/tb_nl_postsynth.vhd"
done
run_shard() {
  local f="$1" b k lib
  k="$(basename "$f" .txt)"; b="${k#shard_}"; b="${b%%_*}"; lib="$OUT/work-lib/b$b"
  local cmd=("${NVC[@]}" -L "$lib" --work=work:"$lib/work.08" -e --jit --no-save
             -g MANIFEST="$f" -g BITNESS="$b" -g OUT_WIDTH="$OUT_WIDTH" tb_nl_postsynth
             -r --exit-severity=error)
  if command -v systemd-run > /dev/null 2>&1; then
    systemd-run --user --scope -q -p MemoryMax="${PER_MEM_MB}M" -p MemorySwapMax=0 "${cmd[@]}" > "$LOGD/$k.log" 2>&1
  else
    "${cmd[@]}" > "$LOGD/$k.log" 2>&1
  fi
}
NSH=$(ls "$OUT"/shard_*.txt | wc -l)
echo "Running $NSH shard(s), per-shard memory cap ${PER_MEM_MB} MB"
rc=0
for f in "$OUT"/shard_*.txt; do run_shard "$f" & done
for f in "$OUT"/shard_*.txt; do wait -n || rc=1; done

# 6. Per-job compare and results table.
RES="$OUT/ps_results.tsv"
{
  printf 'Image\tDims\tBits\tNEAR\tBytes\tResult\n'
  while IFS=$'\t' read -r b px stem c near; do
    d="$RUND/${stem}_N$near"; r="$REFD/${stem}_N$near"; res=PASS; mt=0; tt=0
    if [ -f "$d/out.bin" ] && ! python3 "$HERE/../../../Scripts/nl_unpack.py" "$d/out.bin" -o "$d/out{slice}.jls" 2>>"$LOGD/unpack.log"; then
      res="FAIL (unpack)"
    fi
    for l in 0 1 2 3; do
      t=$(stat -c %s "$r/ref$l.jls" 2>/dev/null || echo 0)
      if [ "$t" -eq 0 ]; then res="FAIL (charls)"; m=0
      elif [ ! -f "$d/out$l.jls" ]; then res="FAIL (no output)"; m=0
      elif cmp -s "$r/ref$l.jls" "$d/out$l.jls"; then m=$t
      else
        m=$(cmp "$r/ref$l.jls" "$d/out$l.jls" 2>&1 | grep -oE 'byte [0-9]+' | grep -oE '[0-9]+' | head -1)
        m=${m:-0}; cmp "$r/ref$l.jls" "$d/out$l.jls" 2>&1 | grep -q differ && m=$((m - 1))
        [ "$res" = PASS ] && res="FAIL (lane $l)"
      fi
      mt=$((mt + m)); tt=$((tt + t))
    done
    dims=$(head -c 64 "$c" | tr '\n' ' ' | awk '{print $2 "x" $3}')
    printf '%s\t%s\t%s\t%s\t%s/%s\t%s\n' "$stem" "$dims" "$b" "$near" "$mt" "$tt" "$res"
  done < "$JOBS" | sort
} > "$RES"
pass=$(grep -c $'\tPASS$' "$RES" || true); fail=$((NJ - pass))
[ "$rc" -eq 0 ] || echo "some shard exited with an error, see $LOGD"

{
  echo "NAME=\"Post-synth golden\""
  echo "NOTE=\"funcsim netlists (BITNESS $BITS) byte-exact vs CharLS per stripe\""
  echo "STATUS=$([ "$fail" -eq 0 ] && [ "$rc" -eq 0 ] && echo PASS || echo FAIL)"
  echo "PCT=\"$(awk -v p="$pass" -v t="$NJ" 'BEGIN{ if (t > 0) printf "%.0f%%", 100 * p / t }')\""
  echo "SUMMARY=\"$pass of $NJ images byte-exact vs CharLS (<= $MAX_MP MP)\""
  echo "DATE=\"$(date -Iseconds)\""
} > "$OUT/report_status.env"

echo "Post-synth golden: PASS=$pass FAIL=$fail of $NJ (RTL $(cat "$SYN/rtl.hash")), table $RES"
[ "$fail" -eq 0 ] && [ "$rc" -eq 0 ] || { grep -v $'\tPASS$' "$RES" | tail -n +2 | head -20; exit 1; }
