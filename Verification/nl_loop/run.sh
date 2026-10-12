#!/usr/bin/env bash
# Random check of nl_loop over NEAR, STRIPES and BITNESS (NVC).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../Sources"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"
nvc --std=2008 -a "$SRC/nl_pkg.vhd" "$SRC/nl_loop.vhd" "$HERE/tb_nl_loop.vhd"
for B in ${BITNESS_LIST:-8 12 16}; do
  for N in ${NEAR_LIST:-0 1 2 3 7 31}; do
    for S in ${STRIPES_LIST:-1 4}; do
      nvc --std=2008 --ieee-warnings=off -e -gBITNESS=$B -gNEAR=$N -gSTRIPES=$S -gCYCLES=${CYCLES:-100000} tb_nl_loop -r --exit-severity=error 2>&1 \
        | grep -E "PASS|mismatch|missing|rror|Fatal" || true
    done
  done
done
