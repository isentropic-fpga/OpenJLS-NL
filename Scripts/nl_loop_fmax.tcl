# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# One fmax point of nl_loop: non-project OOC synth (retiming) + place + route on
# xczu7eg-fbvb900-1-e. Usage: vivado -mode batch -source nl_loop_fmax.tcl
#   -tclargs <BITNESS> <outdir>
# Frequency is estimated as 1000/(period-WNS).
#
lassign $argv BITNESS OUT
set PERIOD 2.000
set ROOT [file normalize [file join [file dirname [info script]] ..]]
file mkdir $OUT
read_vhdl -vhdl2008 [list [file join $ROOT Sources nl_pkg.vhd] [file join $ROOT Sources nl_loop.vhd] [file join $ROOT Scripts nl_loop_wrap.vhd]]
set fp [open $OUT/clk.xdc w]; puts $fp "create_clock -period $PERIOD -name iClk \[get_ports iClk\]"; close $fp
read_xdc -mode out_of_context $OUT/clk.xdc
synth_design -top nl_loop_wrap -part xczu7eg-fbvb900-1-e -mode out_of_context -retiming \
  -generic BITNESS=$BITNESS
opt_design
place_design
phys_opt_design
route_design
set wns [get_property SLACK [lindex [get_timing_paths -delay_type max -max_paths 1] 0]]
set u [report_utilization -return_string]
regexp {CLB LUTs\s*\|\s*(\d+)} $u -> lut
regexp {CLB Registers\s*\|\s*(\d+)} $u -> ff
regexp {DSPs\s*\|\s*(\d+)} $u -> dsp
report_timing -max_paths 5 -file $OUT/paths.log
report_utilization -file $OUT/util.log
set fp [open $OUT/result.csv w]
puts $fp "$BITNESS,$wns,[format %.1f [expr {1000.0 / ($PERIOD - $wns)}]],$lut,$ff,$dsp"
close $fp
