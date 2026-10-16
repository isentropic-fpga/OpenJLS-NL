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
# Two batches:
#   wide   every image under MAX_MP, at NEAR_WIDE (default "3 7")
#   sweep  the images in sweep_images.txt, at NEAR_SWEEP (default 0..16)
#
# Images come from the full OpenJLS checkout (its prepare_images.sh fills
# Images/) plus the T.87 reference PGMs. Widths below 32 are skipped (W/4 >= 8).
#
# Usage:  [BATCH=wide|sweep|all] [MAX_MP=0.5] [NUMBER_OF_THREADS=1] ./build_run.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OJ_FULL="${OJ_FULL:-$HOME/Repos/OpenJLS}"
CHARLS="$OJ_FULL/ThirdParty/charls"
REF_DIR="$OJ_FULL/Verification/T87 conformance/Reference Images"
IMAGES_DIR="${IMAGES_DIR:-$OJ_FULL/Verification/Golden model/Images}"
PGM_INFO="$OJ_FULL/Verification/Golden model/imageprep/pgm_info.py"

BATCH="${BATCH:-all}"
MAX_MP="${MAX_MP:-0.5}"
NUMBER_OF_THREADS="${NUMBER_OF_THREADS:-1}"
NEAR_WIDE="${NEAR_WIDE:-3 7}"
NEAR_SWEEP="${NEAR_SWEEP:-$(seq -s ' ' 0 16)}"

OUT="$HERE/Output"
LIBS="$OUT/work-lib"
CROP="$OUT/cropped"
LOGD="$OUT/logs"
RUND="$OUT/runs"
rm -rf "$LOGD" "$RUND"
mkdir -p "$LIBS" "$CROP" "$LOGD" "$RUND"

# 0. Reference encoder + gate: CharLS must reproduce the official T16E0/T16E3.
g++ -std=c++17 -O2 -DCHARLS_STATIC -I"$CHARLS/include" "$ROOT/Verification/ref/nl_charls.cpp" \
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
"${NVC[@]}" --work=work:"$LIBS/work.08" -a --relaxed --psl \
  "$OLO"/olo_base_pkg_{array,math,string,logic,attribute}.vhd \
  "$OLO/olo_base_ram_sdp.vhd" "$OLO/olo_base_fifo_sync.vhd" \
  "$OJ/openjls_pkg.vhd" "$OJ/A11_2_bit_packer.vhd" \
  "$OJ"/{A10_compute_k,A11_error_mapping,A12_variables_update}.vhd \
  "$OJ"/{A20_compute_temp,A21_compute_map,A22_errval_mapping,A23_run_interruption_update}.vhd \
  "$SRC"/{A11_1_nl_golomb_encoder,A15_A16_nl_encode_run}.vhd \
  "$SRC"/{nl_pkg,nl_loop,nl_config,nl_frontend,nl_backend,byte_stuffer_nl,nl_framer,nl_top}.vhd \
  "$HERE/tb_nl_golden.vhd"

# 2. Image set: stem -> path, cropped to W % 4 = 0 (cached under Output/cropped)
declare -A PATHS
shopt -s nullglob
for p in "$IMAGES_DIR"/*.pgm "$REF_DIR"/*.PGM; do
  s="$(basename "$p")"; PATHS["${s%.*}"]="$p"
done
shopt -u nullglob

JOBS="$OUT/jobs.tsv"; : > "$JOBS"
SKIPS="$OUT/skips.tsv"; : > "$SKIPS"
add_jobs() { # stem nears...
  local stem="$1" p W H MX B mp c; shift
  p="${PATHS[$stem]:-}"
  [ -n "$p" ] || { echo "missing image: $stem" >&2; exit 1; }
  read -r W H MX B < <(python3 "$PGM_INFO" "$p")
  mp=$(awk -v w="$W" -v h="$H" 'BEGIN{printf "%.3f", w*h/1e6}')
  if [ "$W" -lt 32 ] || awk -v mp="$mp" -v cap="$MAX_MP" 'BEGIN{exit !(mp>cap)}'; then
    printf '%s\t%sx%s\t%s\n' "$stem" "$W" "$H" "$mp" >>"$SKIPS"; return
  fi
  c="$CROP/$stem.pgm"
  [ -f "$c" ] || python3 "$HERE/crop4.py" "$p" "$c"
  for n in "$@"; do printf '%s\t%s\t%s\t%s\n' "$stem" "$c" "$B" "$n" >>"$JOBS"; done
}
if [ "$BATCH" = wide ] || [ "$BATCH" = all ]; then
  for stem in $(printf '%s\n' "${!PATHS[@]}" | sort); do add_jobs "$stem" $NEAR_WIDE; done
fi
if [ "$BATCH" = sweep ] || [ "$BATCH" = all ]; then
  while read -r stem; do
    case "$stem" in ''|'#'*) continue ;; esac
    add_jobs "$stem" $NEAR_SWEEP
  done < "$HERE/sweep_images.txt"
fi
sort -u -o "$JOBS" "$JOBS"

# 3. One job: CharLS reference per stripe, nl_top sim, per-lane byte compare.
run_job() {
  local stem="$1" pgm="$2" bits="$3" near="$4" tag d r l
  local -a NVC=(nvc --std=2008 --ieee-warnings=off -H "${NVC_HEAP:-1g}" -L "$LIBS")
  tag="${stem}_N$near"; d="$RUND/$tag"; mkdir -p "$d"
  if ! "$OUT/nl_charls" "$pgm" "$near" 4 "$d/ref" >"$LOGD/$tag.log" 2>&1; then
    r="FAIL (charls)"
  elif ! "${NVC[@]}" --work=work:"$LIBS/work.08" -e --jit --no-save \
      -g PGM_PATH="$pgm" -g OUT_PREFIX="$d/out" -g BITNESS="$bits" -g NEAR="$near" tb_nl_golden \
      -r --exit-severity=error >>"$LOGD/$tag.log" 2>&1; then
    r="FAIL (sim)"
  else
    r=PASS
    for l in 0 1 2 3; do
      cmp -s "$d/ref$l.jls" "$d/out$l.jls" || { r="FAIL (lane $l)"; break; }
    done
  fi
  printf '%s\t%s\t%s\t%s\n' "$stem" "$bits" "$near" "$r"
  [ "$r" = PASS ] && rm -rf "$d"
  return 0
}
export -f run_job
export OUT LOGD RUND LIBS NVC_HEAP

NJ=$(wc -l < "$JOBS")
echo "BATCH=$BATCH: $NJ job(s), $(wc -l < "$SKIPS") image(s) skipped, $NUMBER_OF_THREADS thread(s)"
RES="$OUT/results.tsv"
tr '\t' '\n' < "$JOBS" | xargs -d '\n' -n 4 -P "$NUMBER_OF_THREADS" bash -c 'run_job "$@"' _ \
  | tee "$RES.tmp" | grep --line-buffered -v $'\tPASS$' || true
{ printf 'Image\tBits\tNEAR\tResult\n'; sort "$RES.tmp"; } > "$RES"; rm -f "$RES.tmp"

pass=$(grep -c $'\tPASS$' "$RES" || true)
fail=$((NJ - pass))
echo "Golden NL suite: PASS=$pass FAIL=$fail SKIP=$(wc -l < "$SKIPS") images (cap $MAX_MP MP)"
[ "$fail" -eq 0 ] || { echo "Failures in $RES, logs under $LOGD"; exit 1; }
