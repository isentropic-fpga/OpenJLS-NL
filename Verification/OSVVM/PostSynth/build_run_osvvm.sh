#!/usr/bin/env bash
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

# Post-synthesis OSVVM run: the nl_top stress TB (Top/tb_nl_top_osvvm.vhd,
# POST_SYNTH=true) against the 8-bit funcsim netlist (BITNESS 8, 4096x4096,
# OUT_WIDTH 64, from synth.sh). The netlist is analyzed as work.nl_top and the
# TB's POST_SYNTH component binds to it.
#
# Needs the OSVVM library from ../build_reports.sh (VHDL_LIBS/NVC-*).
#
# Usage:  ./build_run_osvvm.sh [--synth]
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
OSVVM_DIR="$HERE/.."
OUT="$HERE/Output"
LIB="$OUT/work-lib/osvvm"
NETLIST="$OUT/synth/nl_top_8bit_funcsim.vhd"
TB=tb_nl_top_osvvm

# 1. Netlist (synth.sh: rebuilt when the RTL hash changes).
BITS=8 "$HERE/synth.sh" "$@"

# 2. OSVVM library, built by the RTL flow.
OSVVM_LIBS="$(ls -d "$OSVVM_DIR"/VHDL_LIBS/NVC-* 2> /dev/null | tail -1)"
[ -n "$OSVVM_LIBS" ] && [ -d "$OSVVM_LIBS/OSVVM.08" ] \
  || { echo "OSVVM library missing: run Verification/OSVVM/build_reports.sh first" >&2; exit 1; }

# 3. TB packages, the netlist in place of the RTL, then the TB.
rm -rf "$LIB"; mkdir -p "$LIB"
NVC=(nvc --std=2008 --ieee-warnings=off -H 1g -M 256m -L "$OSVVM_LIBS" -L "$LIB")
OLO="$ROOT/OpenJLS/ThirdParty/open-logic/src/base/vhdl"
"${NVC[@]}" --work=tb_support:"$LIB/tb_support.08" -a --relaxed \
  "$ROOT/OpenJLS/Verification/OSVVM/Support/tb_support_pkg.vhd"
"${NVC[@]}" --work=work:"$LIB/work.08" -a --relaxed \
  "$OLO"/olo_base_pkg_{array,math,string,logic,attribute}.vhd \
  "$ROOT/OpenJLS/Sources/openjls_pkg.vhd" "$NETLIST" \
  "$OSVVM_DIR/Top/tb_nl_top_golden_pkg.vhd" "$OSVVM_DIR/Top/$TB.vhd"

# 4. Elaborate and run in Output/sim-osvvm (OSVVM drops its YAML there).
mkdir -p "$OUT/sim-osvvm/OsvvmTemp_NVC" "$OUT/sim-osvvm/reports"
cd "$OUT/sim-osvvm"
LOG="$OUT/postsynth_osvvm.log"
start=$(date +%s)
if "${NVC[@]}" --work=work:"$LIB/work.08" -e --jit --no-save \
    -g POST_SYNTH=true -g BITNESS=8 -g MAX_W=4096 -g MAX_H=4096 -g OUT_WIDTH=64 "$TB" \
    -r --exit-severity=error 2>&1 | tee "$LOG"; then rc=0; else rc=1; fi

aff=$(awk '/DONE/ && /Affirmations Checked:/ {
  for (i = 1; i <= NF; i++) if ($i == "Checked:") a = $(i+1) } END { print a + 0 }' "$LOG")
grep -q "DONE *PASSED" "$LOG" || rc=1
{
  echo "NAME=\"Post-synth OSVVM\""
  echo "NOTE=\"nl_top stress TB on the 8-bit funcsim netlist\""
  echo "STATUS=$([ "$rc" -eq 0 ] && echo PASS || echo FAIL)"
  echo "PCT=\"$([ "$rc" -eq 0 ] && echo 100% || echo 0%)\""
  echo "SUMMARY=\"nl_top stress test &middot; $(printf "%'d" "$aff") affirmations\""
  echo "DATE=\"$(date -Iseconds)\""
} > "$OUT/report_status_osvvm.env"
echo "Post-synth OSVVM: $([ "$rc" -eq 0 ] && echo PASS || echo FAIL), $aff affirmations, $(( $(date +%s) - start )) s, log $LOG"
exit "$rc"
