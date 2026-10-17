#!/usr/bin/env bash
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

# Golden-model cross-check for nl_top (NVC). Each image is cropped to a width
# multiple of 4, split into 4 stripes and encoded per stripe with CharLS at the
# given NEAR; the 4 nl_top lane streams must match byte-exact.
#
# Three batches:
#   wide   every image under MAX_MP, at NEAR_WIDE (default "3 7")
#   sweep  the images in sweep_images.txt, at NEAR_SWEEP (default 0..16)
#   nl     the NL corpus from gen_nl_images.py (stripe seams, BITNESS 8..16,
#          minimal / odd / max lane widths), each at its manifest NEARs
#
# Images come from the full OpenJLS checkout (its prepare_images.sh fills
# Images/), the T.87 reference PGMs and Output/Images_nl. Widths below 32 are
# skipped (W/4 >= 8).
#
# Incremental: a (image, NEAR) job that passed against the same RTL hash
# (NL + reused OpenJLS sources, TB, nl_charls.cpp) is not re-run, unless its
# PGM is newer than the results table. RERUN=1 forces every job. Results merge
# into Output/golden_results.tsv; Output/report_status.env summarizes it.
#
# Usage:  [BATCH=wide|sweep|nl|all] [MAX_MP=0.5] [NUMBER_OF_THREADS=20] [RERUN=1] ./build_run.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
OJ_FULL="${OJ_FULL:-$HOME/Repos/OpenJLS}"
CHARLS="$OJ_FULL/ThirdParty/charls"
REF_DIR="$OJ_FULL/Verification/T87 conformance/Reference Images"
IMAGES_DIR="${IMAGES_DIR:-$OJ_FULL/Verification/Golden model/Images}"
PGM_INFO="$OJ_FULL/Verification/Golden model/imageprep/pgm_info.py"

BATCH="${BATCH:-all}"
MAX_MP="${MAX_MP:-0.5}"
NUMBER_OF_THREADS="${NUMBER_OF_THREADS:-$(( $(nproc) < 20 ? $(nproc) : 20 ))}"
NEAR_WIDE="${NEAR_WIDE:-3 7}"
NEAR_SWEEP="${NEAR_SWEEP:-$(seq -s ' ' 0 16)}"

OUT="$HERE/Output"
NL_IMG="$OUT/Images_nl"
LIBS="$OUT/work-lib"
CROP="$OUT/cropped"
LOGD="$OUT/logs"
RUND="$OUT/runs"
rm -rf "$LOGD" "$RUND"
mkdir -p "$LIBS" "$CROP" "$LOGD" "$RUND"

# 0. Reference encoder + gate: CharLS must reproduce the official T16E0/T16E3.
g++ -std=c++17 -O2 -DCHARLS_STATIC -I"$CHARLS/include" "$HERE/ref/nl_charls.cpp" \
  "$CHARLS/build/libcharls.a" -o "$OUT/nl_charls"
for n in 0 3; do
  "$OUT/nl_charls" "$REF_DIR/TEST16.PGM" $n 1 "$OUT/gate"
  cmp -s "$OUT/gate0.jls" "$REF_DIR/T16E$n.JLS" \
    || { echo "FATAL: CharLS does not reproduce T16E$n.JLS" >&2; exit 1; }
done
echo "CharLS gate: reproduces T16E0.JLS and T16E3.JLS byte-exact OK"

# 1. Compile (--relaxed: OpenLogic shared variables; --psl: contract assertions)
NVC=(nvc --std=2008 --ieee-warnings=off -H "${NVC_HEAP:-1g}" -L "$LIBS")
OLO="$ROOT/OpenJLS/ThirdParty/open-logic/src/base/vhdl"
OJ="$ROOT/OpenJLS/Sources"
SRC="$ROOT/Sources"
RTL_FILES=(
  "$OLO"/olo_base_pkg_{array,math,string,logic,attribute}.vhd
  "$OLO/olo_base_ram_sdp.vhd" "$OLO/olo_base_fifo_sync.vhd"
  "$OJ/openjls_pkg.vhd" "$OJ/A11_2_bit_packer.vhd"
  "$OJ"/{A10_compute_k,A11_error_mapping,A12_variables_update}.vhd
  "$OJ"/{A20_compute_temp,A21_compute_map,A22_errval_mapping,A23_run_interruption_update}.vhd
  "$SRC"/{A11_1_nl_golomb_encoder,A15_A16_nl_encode_run}.vhd
  "$SRC"/{nl_pkg,nl_loop,nl_config,nl_frontend,nl_backend,byte_stuffer_nl,nl_framer,nl_top}.vhd
)
"${NVC[@]}" --work=work:"$LIBS/work.08" -a --relaxed --psl "${RTL_FILES[@]}" "$HERE/tb_nl_golden.vhd"
RTL_HASH="$(cat "${RTL_FILES[@]}" "$HERE/tb_nl_golden.vhd" "$HERE/ref/nl_charls.cpp" | sha256sum | cut -c1-12)"

# 2. Image set: stem -> path, cropped to W % 4 = 0 (cached under Output/cropped)
python3 "$HERE/gen_nl_images.py" "$NL_IMG"
declare -A PATHS
shopt -s nullglob
for p in "$IMAGES_DIR"/*.pgm "$REF_DIR"/*.PGM "$NL_IMG"/*.pgm; do
  s="$(basename "$p")"; PATHS["${s%.*}"]="$p"
done
shopt -u nullglob
printf '%s\n' "${!PATHS[@]}" > "$OUT/live.txt"

# Cached keys: "stem<TAB>near" that passed against this RTL hash.
TSV="$OUT/golden_results.tsv"
declare -A CACHED SEEN
if [ "${RERUN:-0}" != 1 ] && [ -f "$TSV" ]; then
  while IFS=$'\t' read -r s _ _ n _ r _ h; do
    [ "$r" = PASS ] && [ "$h" = "$RTL_HASH" ] && CACHED["$s"$'\t'"$n"]=1
  done < <(tail -n +2 "$TSV")
fi

JOBS="$OUT/jobs.tsv"; : > "$JOBS"
SKIPS="$OUT/skips.tsv"; : > "$SKIPS"
NCACHED=0
add_jobs() { # stem nears...
  local stem="$1" p W H MX B mp c n; shift
  p="${PATHS[$stem]:-}"
  [ -n "$p" ] || { echo "missing image: $stem" >&2; exit 1; }
  read -r W H MX B < <(python3 "$PGM_INFO" "$p")
  mp=$(awk -v w="$W" -v h="$H" 'BEGIN{printf "%.3f", w*h/1e6}')
  if [ "$W" -lt 32 ] || awk -v mp="$mp" -v cap="$MAX_MP" 'BEGIN{exit !(mp>cap)}'; then
    printf '%s\t%sx%s\t%s\t-\t-\tSKIP\t-\t-\n' "$stem" "$W" "$H" "$B" >>"$SKIPS"; return
  fi
  c="$CROP/$stem.pgm"
  { [ -f "$c" ] && [ ! "$p" -nt "$c" ]; } || python3 "$HERE/crop4.py" "$p" "$c"
  for n in "$@"; do
    [ -z "${SEEN[$stem$'\t'$n]:-}" ] || continue
    SEEN["$stem"$'\t'"$n"]=1
    if [ -n "${CACHED[$stem$'\t'$n]:-}" ] && [ ! "$p" -nt "$TSV" ]; then
      NCACHED=$((NCACHED + 1)); continue
    fi
    # leading pixel count: largest jobs start first (LPT-like over xargs -P)
    printf '%s\t%s\t%s\t%sx%s\t%s\t%s\n' "$((W / 4 * 4 * H))" "$stem" "$c" "$((W / 4 * 4))" "$H" "$B" "$n" >>"$JOBS"
  done
}
if [ "$BATCH" = wide ] || [ "$BATCH" = all ]; then
  for stem in $(printf '%s\n' "${!PATHS[@]}" | grep -v '^nl-' | sort); do add_jobs "$stem" $NEAR_WIDE; done
fi
if [ "$BATCH" = sweep ] || [ "$BATCH" = all ]; then
  while read -r stem; do
    case "$stem" in ''|'#'*) continue ;; esac
    add_jobs "$stem" $NEAR_SWEEP
  done < "$HERE/sweep_images.txt"
fi
if [ "$BATCH" = nl ] || [ "$BATCH" = all ]; then
  while read -r stem nears; do add_jobs "$stem" $nears; done < "$NL_IMG/manifest.txt"
fi
sort -u "$JOBS" | sort -t$'\t' -k1,1nr | cut -f2- > "$JOBS.tmp"; mv "$JOBS.tmp" "$JOBS"
sort -u -o "$SKIPS" "$SKIPS"

# 3. One job: CharLS reference per stripe, nl_top sim, per-lane byte compare.
#    Prints one row: Image Dims Bits NEAR Bytes(match/total) Result
run_job() {
  local stem="$1" pgm="$2" dims="$3" bits="$4" near="$5" tag d r l m t mt=0 tt=0
  local -a NVC=(nvc --std=2008 --ieee-warnings=off -H "${NVC_HEAP:-1g}" -L "$LIBS")
  tag="${stem}_N$near"; d="$RUND/$tag"; mkdir -p "$d"
  if ! "$OUT/nl_charls" "$pgm" "$near" 4 "$d/ref" >"$LOGD/$tag.log" 2>&1; then
    r="FAIL (charls)"
  else
    if ! "${NVC[@]}" --work=work:"$LIBS/work.08" -e --jit --no-save \
        -g PGM_PATH="$pgm" -g OUT_PREFIX="$d/out" -g BITNESS="$bits" -g NEAR="$near" tb_nl_golden \
        -r --exit-severity=error >>"$LOGD/$tag.log" 2>&1; then
      r="FAIL (sim)"
    else
      r=PASS
    fi
    # matched bytes per lane = common prefix length with the CharLS stripe
    for l in 0 1 2 3; do
      t=$(stat -c %s "$d/ref$l.jls")
      if [ ! -f "$d/out$l.jls" ]; then m=0
      elif cmp -s "$d/ref$l.jls" "$d/out$l.jls"; then m=$t
      else
        m=$(cmp "$d/ref$l.jls" "$d/out$l.jls" 2>&1 | grep -oE '(byte|after byte) [0-9]+' | grep -oE '[0-9]+' | head -1)
        m=${m:-0}; cmp "$d/ref$l.jls" "$d/out$l.jls" 2>&1 | grep -q differ && m=$((m - 1))
        [ "$r" = PASS ] && r="FAIL (lane $l)"
      fi
      mt=$((mt + m)); tt=$((tt + t))
    done
  fi
  printf '%s\t%s\t%s\t%s\t%s/%s\t%s\n' "$stem" "$dims" "$bits" "$near" "$mt" "$tt" "$r"
  [ "$r" = PASS ] && rm -rf "$d"
  return 0
}
export -f run_job
export OUT LOGD RUND LIBS NVC_HEAP

NJ=$(wc -l < "$JOBS")
echo "BATCH=$BATCH: $NJ job(s), $NCACHED cached, $(wc -l < "$SKIPS") image(s) skipped, $NUMBER_OF_THREADS thread(s), RTL $RTL_HASH"
NEW="$OUT/new_results.tsv"
tr '\t' '\n' < "$JOBS" | xargs -d '\n' -n 5 -P "$NUMBER_OF_THREADS" bash -c 'run_job "$@"' _ \
  | tee "$NEW" | grep --line-buffered -v $'\tPASS$' || true

pass=$(grep -c $'\tPASS$' "$NEW" || true)
fail=$((NJ - pass))

# 4. Merge into the results table, keyed by (Image, NEAR): this run's rows
#    replace old ones, rows not run now are kept, images gone from every
#    source drop out.
today="$(date +%F)"
{
  printf 'Image\tDims\tBits\tNEAR\tBytes\tResult\tRun\tRTL\n'
  {
    [ -f "$TSV" ] && tail -n +2 "$TSV" | sed 's/^/old\t/'
    awk -v d="$today" -v h="$RTL_HASH" '{ print "new\t" $0 "\t" d "\t" h }' "$NEW"
    sed 's/^/new\t/' "$SKIPS"
    :
  } | awk -F'\t' -v OFS='\t' -v live="$OUT/live.txt" '
      BEGIN { while ((getline f < live) > 0) ok[f] = 1 }
      { src = $1; sub(/^[^\t]*\t/, ""); split($0, c, "\t"); k = c[1] "\t" c[4]
        if (!(c[1] in ok)) next
        if (src == "new" || !(k in row)) row[k] = $0 }
      END { for (k in row) print row[k] }' | sort -t$'\t' -k1,1 -k4,4n
} > "$TSV.tmp" && mv "$TSV.tmp" "$TSV"

# Status line for a published report, over the merged table.
read -r tpass tfail tskip < <(awk -F'\t' 'NR > 1 { if ($6 == "PASS") p++; else if ($6 == "SKIP") s++; else f++ }
  END { print p + 0, f + 0, s + 0 }' "$TSV")
gpct=$(awk -v p="$tpass" -v t="$((tpass + tfail))" 'BEGIN{ if (t > 0) printf "%.0f%%", 100 * p / t }')
{
  echo "NAME=\"Golden model\""
  echo "NOTE=\"CharLS per-stripe byte-exact cross-check\""
  echo "STATUS=$([ "$tfail" -eq 0 ] && echo PASS || echo FAIL)"
  echo "PCT=\"$gpct\""
  echo "SUMMARY=\"$tpass of $((tpass + tfail)) image x NEAR jobs byte-exact vs CharLS$([ "$tskip" -gt 0 ] && echo " ($tskip images skipped)")\""
  echo "DATE=\"$(date -Iseconds)\""
} > "$OUT/report_status.env"

echo "Golden NL suite: PASS=$pass FAIL=$fail CACHED=$NCACHED SKIP=$(wc -l < "$SKIPS") images (cap $MAX_MP MP, RTL $RTL_HASH)"
echo "Table: $tpass PASS, $tfail FAIL, $tskip SKIP in $TSV"
[ "$fail" -eq 0 ] || { echo "Failures in $NEW, logs under $LOGD"; exit 1; }
