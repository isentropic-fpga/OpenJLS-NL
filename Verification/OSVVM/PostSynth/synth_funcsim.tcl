# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

# Synthesize nl_top (OOC, xczu7eg-1) once per BITNESS and write a VHDL funcsim
# netlist for each. Driven by build_run.sh; outputs go to the cwd.
#   vivado -mode batch -source synth_funcsim.tcl -tclargs "<bitnesses>" MAX_W MAX_H OUT_WIDTH
lassign $argv BITS MAX_W MAX_H OUT_WIDTH
set ROOT [file normalize [file join [file dirname [info script]] .. .. ..]]
set OLO [file join $ROOT OpenJLS ThirdParty open-logic src base vhdl]
set OJ [file join $ROOT OpenJLS Sources]
set SRC [file join $ROOT Sources]
set FILES [concat \
  [lmap f {olo_base_pkg_array olo_base_pkg_math olo_base_pkg_string olo_base_pkg_logic olo_base_pkg_attribute \
           olo_base_ram_sdp olo_base_fifo_sync} {file join $OLO $f.vhd}] \
  [lmap f {openjls_pkg A11_2_bit_packer A10_compute_k A11_error_mapping A12_variables_update \
    A20_compute_temp A21_compute_map A22_errval_mapping A23_run_interruption_update} \
    {file join $OJ $f.vhd}] \
  [lmap f {A11_1_nl_golomb_encoder A15_A16_nl_encode_run nl_pkg nl_loop nl_config nl_frontend nl_backend \
    byte_stuffer_nl nl_framer nl_merge nl_top} {file join $SRC $f.vhd}]]

read_vhdl -vhdl2008 $FILES
foreach b $BITS {
  puts "Synthesizing nl_top: BITNESS=$b MAX=${MAX_W}x${MAX_H} OUT_WIDTH=$OUT_WIDTH"
  # out_of_context: no IO buffers, so the netlist ports match the RTL entity
  synth_design -top nl_top -part xczu7eg-fbvb900-1-e -mode out_of_context \
    -generic BITNESS=$b -generic MAX_IMAGE_WIDTH=$MAX_W \
    -generic MAX_IMAGE_HEIGHT=$MAX_H -generic OUT_WIDTH=$OUT_WIDTH
  write_vhdl -mode funcsim -force nl_top_${b}bit_funcsim.vhd
  report_utilization -file synth_util_${b}bit.rpt
}
