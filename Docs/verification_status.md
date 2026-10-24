# Verification status

Snapshot 2026-10-24, RTL with nl_merge (D24 B, OUT_WIDTH 64, header-inclusive
CHUNK_BYTES). Requirements: verification_requirements.md. Reports:
Docs/Reports (GitHub Pages).

## Results
| Flow | Scope | Result |
|---|---|---|
| OSVVM RTL | 67 test cases, all modules + nl_top (BITNESS 8/12/16, CHUNK_BEATS 2/128, MAX 4096) | 67/67 pass |
| Code coverage (NVC) | statements / branches | 1131/1131; branches 100% except byte_stuffer_nl 79/80 |
| Golden (CharLS, byte-exact per stripe) | Images corpus, 0.5 MP cap as OpenJLS, RTL hash 3e6e24fd0edf | 10919/10919 pass |
| Golden NL batch | wide images up to the 65532-wide max line | 147/147 pass |
| Post-synth | BITNESS 8 smoke: golden funcsim + nl_top OSVVM | 57/57 images; OSVVM 9/9 requirements |
| Synthesis | BITNESS 12, 12288 wide | decisions.md Status |

## Gaps (to cover later)
1. Branch coverage: byte_stuffer_nl 79/80, one branch never hit.
2. Golden limits: images with W < 32 skipped (250); NEAR 3/7 only on the
   full corpus, plus the sweep set. More NEARs (0, 1, 15, 31) triple runtime.
3. nl_merge verified at OUT_WIDTH 64 only; wider bus is a future increment
   (decisions.md D24).
4. Post-synth is a smoke test (one bitness, few images), not a regression.
5. Formal not started (deferred): first queue occupancy coverage and stall
   rate, then narrow formal on queue bounds and AXIS protocol, prototyped on
   OpenJLS jls_framer.
6. No back-pressure throughput figure under realistic DMA stalls; only the
   ~0.98 pix/cycle free-running number.
7. Output side: DMA guidance in output_format.md and a per-segment mode in
   Scripts/nl_unpack.py still to write.
8. No hardware/board test.
9. ranges.yaml / check_ranges.py checker is experimental (not a signoff gate).

## Rerun
- OSVVM: Verification/OSVVM/build_run.sh (NL_ONLY=1 for NL suites only; rc=1
  then comes from the empty OpenJLS suites).
- Golden: Verification/OSVVM/Golden/build_run.sh, cached by RTL hash in
  Output/golden_results.tsv (~1-2 h at 20 jobs for a full rerun).
- Post-synth: Verification/OSVVM/PostSynth/build_run.sh and build_run_osvvm.sh.
- Reports: Verification/OSVVM/publish_reports.sh.
