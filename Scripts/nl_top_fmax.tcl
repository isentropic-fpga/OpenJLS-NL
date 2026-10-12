# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# One fmax point of nl_top: non-project OOC synth (retiming) + place + route on
# xczu7eg-fbvb900-1-e. Usage: vivado -mode batch -source nl_top_fmax.tcl
#   -tclargs <BITNESS> <outdir>
# Frequency is estimated as 1000/(period-WNS).
#
lassign $argv BITNESS OUT
set PERIOD 2.000
set ROOT [file normalize [file join [file dirname [info script]] ..]]
set OLO [file join $ROOT OpenJLS ThirdParty open-logic src base vhdl]
set OJ [file join $ROOT OpenJLS Sources]
set SRC [file join $ROOT Sources]
file mkdir $OUT
read_vhdl -vhdl2008 [concat \
  [lmap f {olo_base_pkg_array olo_base_pkg_math olo_base_pkg_string olo_base_pkg_logic olo_base_pkg_attribute \
           olo_base_ram_sdp olo_base_fifo_sync} {file join $OLO $f.vhd}] \
  [lmap f {openjls_pkg A11_2_bit_packer byte_stuffer} {file join $OJ $f.vhd}] \
  [lmap f {nl_pkg nl_loop nl_config nl_frontend nl_backend nl_framer nl_top} {file join $SRC $f.vhd}]]
set fp [open $OUT/clk.xdc w]; puts $fp "create_clock -period $PERIOD -name iClk \[get_ports iClk\]"; close $fp
read_xdc -mode out_of_context $OUT/clk.xdc
synth_design -top nl_top -part xczu7eg-fbvb900-1-e -mode out_of_context -retiming \
  -generic BITNESS=$BITNESS
opt_design
place_design
phys_opt_design
route_design
set wns [get_property SLACK [lindex [get_timing_paths -delay_type max -max_paths 1] 0]]
set u [report_utilization -return_string]
regexp {CLB LUTs\s*\|\s*(\d+)} $u -> lut
regexp {CLB Registers\s*\|\s*(\d+)} $u -> ff
regexp {Block RAM Tile\s*\|\s*([\d.]+)} $u -> bram
regexp {DSPs\s*\|\s*(\d+)} $u -> dsp
report_timing -max_paths 10 -file $OUT/paths.log
report_utilization -hierarchical -file $OUT/util_hier.log
report_utilization -file $OUT/util.log
set fp [open $OUT/result.csv w]
puts $fp "$BITNESS,$wns,[format %.1f [expr {1000.0 / ($PERIOD - $wns)}]],$lut,$ff,$bram,$dsp"
close $fp
