#!/usr/bin/env bash
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

# Funcsim netlists of nl_top (OOC, Vivado) into Output/synth, one per BITNESS,
# all at MAX 4096 x 4096 and OUT_WIDTH 64. Rebuilt when the RTL hash changes;
# --synth forces. Shared by build_run.sh and build_run_osvvm.sh.
#
# Usage:  [BITS="8"] ./synth.sh [--synth]
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
SYN="$HERE/Output/synth"
BITS="${BITS:-8}"
MAX_W=4096
MAX_H=4096
OUT_WIDTH=64
mkdir -p "$SYN"

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
RTL_HASH="$( (cat "${RTL_FILES[@]}" "$HERE/synth_funcsim.tcl"; echo "$BITS") | sha256sum | cut -c1-12)"
if [ "${1:-}" = --synth ] || [ "$(cat "$SYN/rtl.hash" 2>/dev/null)" != "$RTL_HASH" ]; then
  rm -f "$SYN/rtl.hash"
  ( cd "$SYN" && vivado -mode batch -nojournal -log synth.log -source "$HERE/synth_funcsim.tcl" \
      -tclargs "$BITS" "$MAX_W" "$MAX_H" "$OUT_WIDTH" > /dev/null )
  echo "$RTL_HASH" > "$SYN/rtl.hash"
fi
for b in $BITS; do
  [ -f "$SYN/nl_top_${b}bit_funcsim.vhd" ] || { echo "netlist missing: $b bit" >&2; exit 1; }
done
echo "Netlists: BITNESS $BITS, ${MAX_W}x${MAX_H}, OUT_WIDTH $OUT_WIDTH (RTL $RTL_HASH)"

if ! compgen -G "$HOME/.nvc/lib/unisim*" > /dev/null; then
  : "${XILINX_VIVADO:?set XILINX_VIVADO for the one-time nvc --install vivado}"
  ( cd /tmp && nvc --install vivado )
fi
