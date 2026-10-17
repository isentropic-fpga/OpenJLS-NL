#!/usr/bin/env bash
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

# Main OSVVM entry point - full NL regression, HTML reports and NVC
# statement+branch coverage of Sources/ (merged into NVC_CodeCoverage/html/).
# The coverage helpers are shared with OpenJLS (submodule).
#
# Usage:  ./build_run.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
OJ_OSVVM="$HERE/../../OpenJLS/Verification/OSVVM"

COV="NVC_CodeCoverage"
rm -rf "$COV"
CODE_COVERAGE=1 ./build_reports.sh

echo "== merge + report =="
nvc --work=work:"$COV/work" --cover-merge -o "$COV/merged.covdb" "$COV"/tb_*.covdb
nvc --work=work:"$COV/work" --cover-report --per-file -o "$COV/html" "$COV/merged.covdb"
python3 "$OJ_OSVVM/cover_prune.py" "$COV/html"
echo "== Sources/ statement coverage (union over all tests) =="
python3 "$OJ_OSVVM/cover_summary.py" "$COV" ../../Sources

echo "== reports =="
echo "NVC code coverage     : $HERE/$COV/html/index.html"
echo "OSVVM report index    : $HERE/index.html"
echo "OSVVM build summary   : $HERE/OSVVM_OpenJlsNl/OSVVM_OpenJlsNl.html"
echo "OSVVM requirements    : $HERE/OSVVM_OpenJlsNl/reports/OSVVM_OpenJlsNl_req.html"
