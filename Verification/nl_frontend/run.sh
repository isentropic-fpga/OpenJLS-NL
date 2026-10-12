#!/usr/bin/env bash
# nl_config + nl_frontend + nl_loop vs. per-stripe T.87 model (NVC).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../Sources"
OLO="$HERE/../../OpenJLS/ThirdParty/open-logic/src/base/vhdl"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"
nvc --std=2008 -a --relaxed "$OLO/olo_base_pkg_array.vhd" "$OLO/olo_base_pkg_math.vhd" \
  "$OLO/olo_base_pkg_string.vhd" "$OLO/olo_base_pkg_logic.vhd" "$OLO/olo_base_pkg_attribute.vhd" \
  "$OLO/olo_base_ram_sdp.vhd"
nvc --std=2008 -a --relaxed "$SRC/nl_pkg.vhd" "$SRC/nl_loop.vhd" "$SRC/nl_config.vhd" "$SRC/nl_frontend.vhd" \
  "$HERE/tb_nl_frontend.vhd"
for B in ${BITNESS_LIST:-8 12}; do
  for N in ${NEAR_LIST:-0 1 3 10}; do
    for W in ${WIDTH_LIST:-32 64}; do
      nvc --std=2008 --ieee-warnings=off -e -gBITNESS=$B -gNEAR=$N -gWIDTH=$W -gIMAGES=${IMAGES:-3} -gHEIGHT=${HEIGHT:-16} tb_nl_frontend \
        -r --exit-severity=error 2>&1 | grep -E "PASS|got|Errval|C/B/N|flag|missing|rror|Fatal" || true
    done
  done
done
