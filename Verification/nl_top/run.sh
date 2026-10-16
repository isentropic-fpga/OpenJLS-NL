#!/usr/bin/env bash
# nl_top vs. CharLS per stripe (byte-exact), NVC.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT="$HERE/../.."
SRC="$ROOT/Sources"
OJ="$ROOT/OpenJLS/Sources"
OLO="$ROOT/OpenJLS/ThirdParty/open-logic/src/base/vhdl"
# CharLS build and T.87 images come from a full OpenJLS checkout (fetched there).
OJ_FULL=${OJ_FULL:-$HOME/Repos/OpenJLS}
CHARLS="$OJ_FULL/ThirdParty/charls"
REFS="$OJ_FULL/Verification/T87 conformance/Reference Images"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

g++ -std=c++17 -O2 -DCHARLS_STATIC -I"$CHARLS/include" "$HERE/../ref/nl_charls.cpp" "$CHARLS/build/libcharls.a" -o nl_charls
./nl_charls "$REFS/TEST16.PGM" 3 1 gate && cmp -s gate0.jls "$REFS/T16E3.JLS" \
  || { echo "FATAL: CharLS does not reproduce T16E3.JLS"; exit 1; }

nvc --std=2008 -a --relaxed --psl "$OLO/olo_base_pkg_array.vhd" "$OLO/olo_base_pkg_math.vhd" \
  "$OLO/olo_base_pkg_string.vhd" "$OLO/olo_base_pkg_logic.vhd" "$OLO/olo_base_pkg_attribute.vhd" \
  "$OLO/olo_base_ram_sdp.vhd" "$OLO/olo_base_fifo_sync.vhd" \
  "$OJ/openjls_pkg.vhd" "$OJ/A11_2_bit_packer.vhd" \
  "$OJ"/{A10_compute_k,A11_error_mapping,A12_variables_update}.vhd \
  "$OJ"/{A20_compute_temp,A21_compute_map,A22_errval_mapping,A23_run_interruption_update}.vhd \
  "$SRC"/{A11_1_nl_golomb_encoder,A15_A16_nl_encode_run}.vhd \
  "$SRC/nl_pkg.vhd" "$SRC/nl_loop.vhd" "$SRC/nl_config.vhd" "$SRC/nl_frontend.vhd" "$SRC/nl_backend.vhd" \
  "$SRC/byte_stuffer_nl.vhd" "$SRC/nl_framer.vhd" "$SRC/nl_top.vhd" "$HERE/tb_nl_top.vhd"

fail=0
for B in ${BITNESS_LIST:-8 12}; do
  for N in ${NEAR_LIST:-0 1 3 10}; do
    for W in ${WIDTH_LIST:-32 128}; do
      H=${HEIGHT:-24}; I=${IMAGES:-2}; tag="B=$B NEAR=$N W=$W H=$H"
      python3 "$HERE/gen_image.py" $W $H $B $((B + N + W)) img.pgm img.hex
      ./nl_charls img.pgm $N 4 ref
      nvc --std=2008 --ieee-warnings=off -e -gBITNESS=$B -gNEAR=$N -gWIDTH=$W -gHEIGHT=$H -gIMAGES=$I \
        -gGAPS=${GAPS:-false} -gBP=${BP:-false} -gREADY_PCT=${READY_PCT:-60} tb_nl_top -r --exit-severity=error > sim.log 2>&1 \
        || { echo "FAIL $tag (sim)"; grep -m5 -E "rror|ailure|Fatal" sim.log; fail=1; continue; }
      ok=1
      for l in 0 1 2 3; do
        for ((k = 0; k < I; k++)); do cat ref$l.jls; done > exp$l.jls
        xxd -r -p out$l.hex > got$l.jls
        cmp -s exp$l.jls got$l.jls || { ok=0; echo "  lane $l: $(cmp exp$l.jls got$l.jls 2>&1 | head -1)"; }
      done
      rate=$(grep -o "[0-9.e+-]* pix/cycle" sim.log)
      if [ $ok = 1 ]; then echo "PASS $tag ($rate)"; else echo "FAIL $tag"; fail=1; fi
    done
  done
done
exit $fail
