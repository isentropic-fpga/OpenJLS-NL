#!/usr/bin/env bash
# Copyright (C) 2026 Vitor Mendes Camilo
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of OpenJLS. Available under GPLv3 or a
# commercial license. See LICENSE and README for details.
#

# Assemble the verification report tree under Docs/Reports/.
#
# Copies the OSVVM HTML and the NVC code coverage HTML, then stitches in the
# text-only suites (golden, post-synth) from the report_status.env each one
# drops in its Output/. Suites that haven't been run show as "not run".
# Page chrome (stylesheet, theme switch, Isentropic marks) is shared with
# OpenJLS (submodule).
#
# Typical flow - run the suites you want reflected, then publish:
#   ./build_run.sh                         # OSVVM regression + coverage
#   ( cd Golden    && ./build_run.sh )     # RTL golden vs CharLS
#   ( cd PostSynth && ./build_run.sh )     # gate-level golden (needs Vivado)
#   ( cd PostSynth && ./build_run_osvvm.sh )  # gate-level nl_top stress TB
#   ./publish_reports.sh                   # -> Docs/Reports/
#
# Usage:  ./publish_reports.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OJ="$ROOT/OpenJLS"
DEST="$ROOT/Docs/Reports"

OSVVM_YML="$HERE/OSVVM_OpenJlsNl/OSVVM_OpenJlsNl.yml"
COV_DIR="$HERE/NVC_CodeCoverage"
GOLDEN_ENV="$HERE/Golden/Output/report_status.env"
GOLDEN_TSV="$HERE/Golden/Output/golden_results.tsv"
PS_ENV="$HERE/PostSynth/Output/report_status.env"
PS_TSV="$HERE/PostSynth/Output/ps_results.tsv"
PS_OSVVM_ENV="$HERE/PostSynth/Output/report_status_osvvm.env"
PS_OSVVM_LOG="$HERE/PostSynth/Output/postsynth_osvvm.log"

rm -rf "$DEST"
mkdir -p "$DEST/assets"
cp "$OJ/Verification/OSVVM/report_assets/report.css" "$OJ/Verification/OSVVM/report_assets/theme.js" "$DEST/assets/"
for f in isentropic-wordmark isentropic-wordmark-dark; do
  cp "$OJ/Docs/Images/$f.svg" "$DEST/assets/"
done
cp "$OJ/Docs/Images/isentropic-icon-64.png" "$DEST/assets/"
cp "$ROOT/Docs/Images/openjls-nl-wordmark.svg" "$ROOT/Docs/Images/openjls-nl-wordmark-dark.svg" "$DEST/assets/"

# page_head <title> <asset-prefix> / page_foot: shared chrome.
page_head() {
  local title="$1" p="$2"
  cat <<HDR
<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>$title</title>
<link rel="icon" href="${p}assets/isentropic-icon-64.png">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow:wght@400;500;600;700&family=Barlow+Condensed:wght@400;600&display=swap">
<link rel="stylesheet" href="${p}assets/report.css">
<script src="${p}assets/theme.js"></script>
</head><body>
<nav class="site-nav">
  <a class="brand" href="https://isentropic.com.br" aria-label="Isentropic - home"><img class="wm-light" src="${p}assets/isentropic-wordmark.svg" alt="Isentropic"><img class="wm-dark" src="${p}assets/isentropic-wordmark-dark.svg" alt="Isentropic"></a>
  <a class="nlink" href="https://github.com/isentropic-fpga/OpenJLS-NL">GitHub</a>
  <button type="button" class="theme-btn" data-theme-toggle aria-label="Switch color theme" title="Switch color theme">
    <svg class="ico-moon" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M12 3a6 6 0 0 0 9 9 9 9 0 1 1-9-9Z"/></svg>
    <svg class="ico-sun" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><circle cx="12" cy="12" r="4"/><path d="M12 2v2"/><path d="M12 20v2"/><path d="m4.93 4.93 1.41 1.41"/><path d="m17.66 17.66 1.41 1.41"/><path d="M2 12h2"/><path d="M20 12h2"/><path d="m6.34 17.66-1.41 1.41"/><path d="m19.07 4.93-1.41 1.41"/></svg>
  </button>
</nav>
<main><div class="wrap">
HDR
}
page_foot() {
  cat <<FTR
</div></main>
<footer>
  <span style="font-weight:600;color:var(--color-text);">Isentropic</span>
  <a href="https://isentropic.com.br">isentropic.com.br</a>
  <a href="mailto:contact@isentropic.com.br">contact@isentropic.com.br</a>
  <a href="https://github.com/isentropic-fpga">GitHub</a>
  <span>&copy; $(date -u +%Y)</span>
</footer>
</body></html>
FTR
}

MATRIX="$(python3 "$HERE/matrix_html.py" "$ROOT/Docs/verification_requirements.md")"
ROWS=""
NOTES=""
# emit_row <name> <note> <status> <pct> <summary> <report-link-or-empty> <date-or-empty>
emit_row() {
  local name="$1" note="$2" status="$3" pct="$4" summary="$5" link="$6" date="$7"
  local cls report
  case "$status" in
    PASS) cls="ok" ;;
    FAIL) cls="bad" ;;
    INFO) cls="info" ;;
    *)    cls="na"; status="not run" ;;
  esac
  if [ -n "$link" ]; then report="<a href=\"$link\">view</a>"; else report="&mdash;"; fi
  ROWS+="    <tr><td class=\"suite\">$name</td><td><span class=\"badge $cls\">$status</span></td>"
  ROWS+="<td class=\"pct\">${pct:-&mdash;}</td><td>${summary:-&mdash;}</td>"
  ROWS+="<td>$report</td><td class=\"date\">${date:-&mdash;}</td></tr>"$'\n'
  [ -n "$note" ] && NOTES+=" <strong>$name.</strong> $note"
  return 0
}

# NVC code coverage (HTML tree)
if [ -d "$COV_DIR/html" ]; then
  cp -r "$COV_DIR/html" "$DEST/coverage"
  pct=""
  if [ -f "$COV_DIR/merged.covdb" ]; then
    pct=$(cd "$HERE" && python3 "$OJ/Verification/OSVVM/cover_summary.py" NVC_CodeCoverage ../../Sources 2>/dev/null \
            | awk '/^TOTAL/{print $3}')
  fi
  cov_date=$(date -u -r "$COV_DIR/html/index.html" '+%Y-%m-%d' 2>/dev/null || true)
  emit_row "NVC code coverage" "Statement coverage of the NL RTL (Sources/), unioned over every OSVVM test; the report breaks it down per source file." \
    "INFO" "${pct:-n/a}" "per-file breakdown in report" "coverage/index.html" "$cov_date"
else
  emit_row "NVC code coverage" "" "NA" "" "run ./build_run.sh to populate" "" ""
fi

# OSVVM regression (HTML tree)
if [ -f "$HERE/index.html" ] && [ -f "$OSVVM_YML" ]; then
  mkdir -p "$DEST/osvvm"
  cp -r "$HERE/index.html" "$HERE/OSVVM_OpenJlsNl" "$HERE/osvvm" "$DEST/osvvm/"
  pass=$(grep -c 'Status: PASSED' "$OSVVM_YML" || true)
  fail=$(grep -c 'Status: FAILED' "$OSVVM_YML" || true)
  total=$((pass + fail))
  affirm=$(grep -oE 'AffirmCount: [0-9]+' "$OSVVM_YML" | awk '{s+=$2} END{print s+0}')
  rdate=$(awk '/^Date:/{gsub(/"/, "", $2); print $2; exit}' "$OSVVM_YML")
  st=PASS; [ "$fail" -ne 0 ] && st=FAIL
  opct=$(awk -v p="$pass" -v t="$total" 'BEGIN{ if (t > 0) printf "%.0f%%", 100 * p / t }')
  emit_row "OSVVM suite" "Testbenches of every NL block against independent reference models, plus the nl_top stress test, with requirements tracking." \
    "$st" "$opct" "$total tests &middot; $(printf "%'d" "$affirm") affirmations" \
    "osvvm/OSVVM_OpenJlsNl/OSVVM_OpenJlsNl.html" "${rdate%T*}"
else
  emit_row "OSVVM suite" "" "NA" "" "run ./build_run.sh to populate" "" ""
fi

# text-only suites: read their report_status.env
# fields: NAME, NOTE, STATUS, PCT, SUMMARY, DATE  (shell key=value, quoted)
env_row() {
  local f="$1" fb_name="$2" fb_note="$3" link="$4"
  if [ ! -f "$f" ]; then
    emit_row "$fb_name" "$fb_note" "NA" "" "run the suite to populate" "" ""
    return
  fi
  local NAME="" NOTE="" STATUS="" PCT="" SUMMARY="" DATE=""
  # shellcheck disable=SC1090
  source "$f"
  emit_row "${NAME:-$fb_name}" "${fb_note:-$NOTE}" "${STATUS:-NA}" "${PCT:-}" \
    "${SUMMARY:-}" "$link" "${DATE%%T*}"
}

# render_tsv_page <tsv> <dest-subdir> <title> -> echoes the relative link (or
# nothing if the tsv is absent). First TSV line is the header; PASS/FAIL/SKIP
# cells get coloured.
render_tsv_page() {
  local tsv="$1" sub="$2" title="$3"
  [ -f "$tsv" ] || { echo ""; return; }
  mkdir -p "$DEST/$sub"
  {
    page_head "$title" "../"
    cat <<HDR
<a class="back" href="../index.html">&larr; All reports</a>
<div class="hero" style="margin-top:14px;"><h1 class="text">$title</h1></div>
<div class="tbl-scroll"><table><thead><tr>
HDR
    head -1 "$tsv" | awk -F'\t' '{for (i=1;i<=NF;i++) printf "<th>%s</th>", $i; print ""}'
    echo "</tr></thead><tbody>"
    tail -n +2 "$tsv" | awk -F'\t' '{
      printf "<tr>"
      for (i=1;i<=NF;i++) {
        cls=""
        if ($i=="PASS") cls=" class=\"r-ok\""
        else if ($i ~ /^FAIL/) cls=" class=\"r-bad\""
        else if ($i=="SKIP") cls=" class=\"r-na\""
        printf "<td%s>%s</td>", cls, $i
      }
      print "</tr>"
    }'
    echo "</tbody></table></div>"
    page_foot "../"
  } > "$DEST/$sub/index.html"
  echo "$sub/index.html"
}

# render_log_page <log> <dest-subdir> <title> -> echoes the relative link. For
# a suite with no table or OSVVM HTML (post-synth OSVVM is one plain NVC run),
# the captured console log is the report, HTML-escaped into a <pre>.
render_log_page() {
  local log="$1" sub="$2" title="$3"
  [ -f "$log" ] || { echo ""; return; }
  mkdir -p "$DEST/$sub"
  {
    page_head "$title" "../"
    cat <<HDR
<a class="back" href="../index.html">&larr; All reports</a>
<div class="hero" style="margin-top:14px;"><h1 class="text">$title</h1></div>
<pre class="log">
HDR
    sed -e 's/&/\&amp;/g' -e 's/</\&lt;/g' -e 's/>/\&gt;/g' "$log"
    echo "</pre>"
    page_foot "../"
  } > "$DEST/$sub/index.html"
  echo "$sub/index.html"
}

golden_link=""
[ -f "$GOLDEN_ENV" ] && golden_link="$(render_tsv_page "$GOLDEN_TSV" golden "Golden suite - processed images")"
ps_osvvm_link=""
[ -f "$PS_OSVVM_ENV" ] && ps_osvvm_link="$(render_log_page "$PS_OSVVM_LOG" post-synth-osvvm "Post-synth OSVVM - simulation log")"
ps_link=""
[ -f "$PS_ENV" ] && ps_link="$(render_tsv_page "$PS_TSV" post-synth "Post-synth golden - processed images")"

env_row "$GOLDEN_ENV" "Golden suite" \
  "RTL encode of the OpenJLS corpus and the NL corpus (images up to 0.5&nbsp;MP, depths 8&ndash;16, NEAR sweep), each of the 4 stripes compared byte-for-byte with CharLS." \
  "$golden_link"
env_row "$PS_OSVVM_ENV" "Post-synth OSVVM" \
  "The nl_top stress test (backpressure, stalls, back to back, reset rounds) re-run on the synthesized gate-level netlist (BITNESS 8, 4096x4096, OUT_WIDTH 64)." \
  "$ps_osvvm_link"
env_row "$PS_ENV" "Post-synth golden" \
  "Smoke test: 8-bit images up to 0.01&nbsp;MP encoded on the synthesized gate-level netlist (BITNESS 8, 4096x4096, OUT_WIDTH 64) and compared per stripe with CharLS; the cap keeps gate-level simulation time bounded." \
  "$ps_link"

# landing page
GEN_DATE="$(date -u '+%Y-%m-%d %H:%M UTC')"
{
  page_head "OpenJLS-NL - verification report" ""
  cat <<EOF
<div class="hero">
  <span class="kicker">Verification report</span>
  <h1><img class="wm-light" src="assets/openjls-nl-wordmark.svg" alt="OpenJLS-NL"><img class="wm-dark" src="assets/openjls-nl-wordmark-dark.svg" alt="OpenJLS-NL"></h1>
  <p class="lede">Results from the project's own verification suites for the OpenJLS-NL near-lossless JPEG-LS (ISO/IEC 14495-1) encoder IP core. Open a row's report for per-test and per-image detail.</p>
</div>
<div class="tbl-scroll"><table>
  <thead><tr><th>Suite</th><th>Status</th><th>Pass / cov</th><th>Summary</th><th>Report</th><th>Run date</th></tr></thead>
  <tbody>
$ROWS  </tbody>
</table></div>
<p class="notes">$NOTES The OSVVM and coverage reports are full HTML; the golden and post-synthesis pages list every image. Generated $GEN_DATE by <code>Verification/OSVVM/publish_reports.sh</code>.</p>
<h2 class="section">Verification matrix</h2>
<p class="notes">Every requirement the OSVVM suite tracks, with the RTL and testbench that cover it, the guaranteed minimum check count per run (Goal) and its status against verification rules G1&ndash;G6: &check; met, N/A not applicable, gap open. Source: <a href="https://github.com/isentropic-fpga/OpenJLS-NL/blob/main/Docs/verification_requirements.md"><code>Docs/verification_requirements.md</code></a>; rules in <a href="https://github.com/isentropic-fpga/OpenJLS-NL/blob/main/Docs/verification_guidelines.md"><code>Docs/verification_guidelines.md</code></a>.</p>
$MATRIX
EOF
  page_foot ""
} > "$DEST/index.html"

echo "Published report tree -> $DEST"
echo "  open $DEST/index.html"
