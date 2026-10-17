#!/usr/bin/env bash
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

# OSVVM script flow: full NL regression with YAML -> HTML reports.
#
# Builds the OSVVM core vendored in the OpenJLS submodule, then every TB via
# OpenJlsNl.pro. Outputs (gitignored): VHDL_LIBS/, OSVVM_OpenJlsNl/, index.html.
# With CODE_COVERAGE=1 each test is also instrumented (see build_run.sh).
#
# One-time: (cd ../../OpenJLS/ThirdParty && ./fetch_third_party.sh osvvm osvvm-scripts tcllib)
# Requires nvc and tclsh.
#
# Usage:  ./build_reports.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
TP="$HERE/../../OpenJLS/ThirdParty"

command -v nvc >/dev/null || { echo "nvc not found" >&2; exit 1; }
command -v tclsh >/dev/null || { echo "tclsh not found — install tcl" >&2; exit 1; }
[ -d "$TP/osvvm" ] && [ -d "$TP/osvvm-scripts" ] && [ -d "$TP/tcllib" ] || {
  echo "OSVVM not fetched — run: (cd $TP && ./fetch_third_party.sh osvvm osvvm-scripts tcllib)" >&2
  exit 1
}

export TCLLIBPATH="$TP/tcllib"

tclsh <<'TCL'
# Tcl reading stdin otherwise prints errors and can still exit successfully.
if {[catch {
source ../../OpenJLS/ThirdParty/osvvm-scripts/StartNVC.tcl
# 2008: the vendored snapshot lacks the 2019 support files; no NVC coverage
# vendor API either (functional coverage comes from OSVVM itself).
SetVHDLVersion 2008
set ::osvvm::FunctionalCoverageIntegratedInSimulator "default"
set ::osvvm::FailOnTestCaseErrors true
set ::osvvm::FailOnReportErrors true
build ../../OpenJLS/ThirdParty/osvvm/osvvm.pro
build OpenJlsNl.pro
} message options]} {
  puts stderr [dict get $options -errorinfo]
  exit 1
}
TCL
