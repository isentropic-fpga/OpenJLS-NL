# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

#  OpenJLS-NL OSVVM regression - script-flow entry point.
#
#  Run via ./build_reports.sh (or ./build_run.sh for code coverage). Same flow
#  as OpenJLS/Verification/OSVVM/OpenJls.pro; OSVVM, its scripts and the shared
#  tb_support package come from the OpenJLS submodule. Outputs land in this
#  directory: VHDL_LIBS/, the report tree OSVVM_OpenJlsNl/ and index.html.
#
#  This is the single source of the file list.

set OJ ../../OpenJLS

# --relaxed: shared variables of non-protected types (open-logic);
# --stderr=error keeps warnings on stdout (tcl exec fails on stderr);
# --psl + --exit-severity=error make "-- psl" contracts fatal.
# --ieee-warnings is a global option in NVC 1.23 (ignored after -r); the
# loop's idle lanes hold U state, which would flood the log with metavalue
# warnings (X on outputs is still caught by checked_integer).
set ::osvvm::ExtendedGlobalOptions {--stderr=error --ieee-warnings=off}
SetExtendedAnalyzeOptions {--relaxed --psl}
SetExtendedRunOptions     {--exit-severity=error}

# Statement+branch code coverage of NL's own Sources/ (./build_run.sh sets
# CODE_COVERAGE=1). Reused OpenJLS blocks are left to the OpenJLS suite.
set ::NlCodeCoverage [info exists ::env(CODE_COVERAGE)]
if {$::NlCodeCoverage} {
  SetCoverageSimulateEnable true
  file mkdir NVC_CodeCoverage
  set specfd [open NVC_CodeCoverage/dut_only.spec w]
  puts $specfd "-hierarchy *"
  foreach src [lsort [glob ../../Sources/*.vhd]] {
    set ent [file rootname [file tail $src]]
    if {$ent ne "nl_pkg"} { puts $specfd "+block $ent" }
    # A labelled generate is its own scope; +block it by label.
    set fd [open $src r]
    set body [read $fd]
    close $fd
    foreach {full lbl} [regexp -all -inline -line {^\s*(\w+)\s*:\s*(?:for|if)\y.*\ygenerate\y} $body] {
      puts $specfd "+block $lbl"
    }
  }
  close $specfd
}
# NL_ONLY=<substring> runs only the matching testbenches (quick iteration).
proc Test {TestFile args} {
  if {[info exists ::env(NL_ONLY)] && ![string match *$::env(NL_ONLY)* $TestFile]} {
    foreach v {GenericDict GenericNames GenericOptions} { set ::osvvm::$v "" }
    return
  }
  if {$::NlCodeCoverage} {
    set tn [file rootname [file tail $TestFile]]$::osvvm::GenericNames
    SetCoverageSimulateOptions [list --cover=statement,branch --cover-spec=NVC_CodeCoverage/dut_only.spec --cover-file=NVC_CodeCoverage/$tn.covdb]
  }
  RunTest $TestFile {*}$args
}

# Shared TB skeleton from OpenJLS (clk_tick, apply_reset, checked_integer,
# end_of_test).
library tb_support
analyze $OJ/Verification/OSVVM/Support/tb_support_pkg.vhd

# open-logic primitives, reused OpenJLS blocks, then NL RTL, one library.
library openjls_nl
set OLO $OJ/ThirdParty/open-logic/src/base/vhdl
foreach f {olo_base_pkg_array olo_base_pkg_math olo_base_pkg_string olo_base_pkg_logic
           olo_base_pkg_attribute olo_base_ram_sdp olo_base_fifo_sync} {
  analyze $OLO/$f.vhd
}
foreach f {openjls_pkg A11_2_bit_packer A10_compute_k A11_error_mapping A12_variables_update
           A20_compute_temp A21_compute_map A22_errval_mapping A23_run_interruption_update} {
  analyze $OJ/Sources/$f.vhd
}
foreach f {A11_1_nl_golomb_encoder A15_A16_nl_encode_run nl_pkg nl_loop nl_config
           nl_frontend nl_backend byte_stuffer_nl nl_framer nl_top} {
  analyze ../../Sources/$f.vhd
}
analyze Top/tb_nl_top_golden_pkg.vhd

# Near-lossless loop (Ra -> Rx recurrence + context forwarding). Default
# generics are 12-bit, NEAR 3, one stripe; the sweep covers both pixel-width
# ends, lossless, small/large NEAR, the T.87 NEAR ceiling min(255, MAXVAL/2),
# and a single stalled stream vs all 4 lanes.
TestSuite Loop
foreach bitness {8 12 16} {
  if {$bitness == 8} { set nearMax 127 } else { set nearMax 255 }
  foreach near [list 0 1 3 7 31 $nearMax] {
    foreach stripes {1 4} {
      Test Modules/tb_nl_loop_osvvm.vhd [generic BITNESS $bitness] [generic NEAR $near] [generic STRIPES $stripes]
    }
  }
}

# New NL modules (NL copies or NL-only RTL). Reused OpenJLS blocks are
# verified by the OpenJLS suite and not repeated here.
TestSuite Modules
foreach bitness {8 12 16} {
  Test Modules/tb_a11_1_nl_osvvm.vhd [generic BITNESS $bitness]
  Test Modules/tb_a15_a16_nl_osvvm.vhd [generic BITNESS $bitness]
  Test Modules/tb_nl_config_osvvm.vhd [generic BITNESS $bitness]
  Test Modules/tb_byte_stuffer_nl_osvvm.vhd [generic BITNESS $bitness]
  Test Modules/tb_nl_framer_osvvm.vhd [generic BITNESS $bitness]
  Test Modules/tb_nl_frontend_osvvm.vhd [generic BITNESS $bitness]
  Test Modules/tb_nl_backend_osvvm.vhd [generic BITNESS $bitness]
}
# OUT_WIDTH 200: 25-byte beats, the header ends on a full beat (nl_top allows 48..1024).
Test Modules/tb_nl_framer_osvvm.vhd [generic BITNESS 12] [generic OUT_WIDTH 200]

# nl_top stress (4 streams vs CharLS per stripe, backpressure, stall
# propagation, reset per phase). Defaults: MAX_W 64 / MAX_H 64 = the largest
# golden image, so W and H reach their max; then OUT_WIDTH at both ends and
# nl_top's default 4096 x 4096 limits.
TestSuite Top
foreach bitness {8 12 16} {
  Test Top/tb_nl_top_osvvm.vhd [generic BITNESS $bitness]
}
Test Top/tb_nl_top_osvvm.vhd [generic OUT_WIDTH 48]
Test Top/tb_nl_top_osvvm.vhd [generic OUT_WIDTH 1024]
Test Top/tb_nl_top_osvvm.vhd [generic MAX_W 4096] [generic MAX_H 4096]
