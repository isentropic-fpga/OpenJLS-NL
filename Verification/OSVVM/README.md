# OpenJLS-NL OSVVM verification

OSVVM regression for the NL blocks. It follows [`Docs/verification_guidelines.md`](../../Docs/verification_guidelines.md)
(G1-G6) and reuses the OpenJLS infrastructure: the OSVVM library, the
`tb_support` package, and the coverage helpers from the `OpenJLS` submodule.
Reused OpenJLS blocks are verified by the upstream suite and are not repeated
here.

## Layout

| Path | Contents |
|---|---|
| `OpenJlsNl.pro` | Build script: libraries, RTL, the `Loop`, `Modules` and `Top` suites |
| `build_reports.sh` | Runs the .pro under NVC and writes the OSVVM HTML reports |
| `build_run.sh` | Full regression with NVC statement+branch coverage of `Sources/` |
| `Modules/` | One OSVVM TB per NL block |
| `Top/` | `nl_top` stress TB, its CharLS golden package and `gen_golden.py` |
| `Golden/` | CharLS golden suite on the image corpus (`build_run.sh`, `ref/nl_charls.cpp`) |
| `PostSynth/` | Gate-level runs on the synthesized netlist: golden smoke test (`build_run.sh`), nl_top stress TB (`build_run_osvvm.sh`) |
| `publish_reports.sh` | Assembles all suite results into `Docs/Reports/` |
| `NVC_CodeCoverage/` | Coverage databases and HTML (generated) |
| `OSVVM_OpenJlsNl/` | OSVVM logs and reports (generated) |

## Running

```sh
./build_run.sh                  # 64 tests, reports + code coverage
NL_ONLY=nl_framer ./build_reports.sh   # only TBs whose path matches
```

The filtered run reports `BuildError` because the skipped tests count as
missing. Each test's own `DONE PASSED` line is the result to read.

## Golden suite

`Golden/build_run.sh` checks nl_top against CharLS byte-exact, per lane, on
real and synthetic images. Each image is cropped to W % 4 = 0, split into 4
stripes, and encoded per stripe by `ref/nl_charls.cpp` at the same NEAR.

```sh
cd Golden && ./build_run.sh             # incremental, 20 threads
RERUN=1 BATCH=nl ./build_run.sh         # force one batch
```

- **Gate**: CharLS must first reproduce T16E0.JLS and T16E3.JLS byte-exact.
- **Batches**: `wide` runs every OpenJLS corpus image <= `MAX_MP` (0.5) at
  NEAR 3 and 7. `sweep` runs `sweep_images.txt` at NEAR 0..16. `nl` runs the
  corpus from `gen_nl_images.py`:
  - stripe-seam textures, noise and 0/MAXVAL edges at every BITNESS 8..16,
    up to NEAR = NMAX;
  - W = 32 (minimal stripe), W = 36 (odd stripe), one-row, 32 x 8192 and
    65532 x 4 images (max lane line, RUNindex 30).
- **Incremental**: a job (image, NEAR) that passed against the same RTL hash
  is skipped. The hash covers the NL and reused OpenJLS sources, the TB and
  `nl_charls.cpp`. A job also re-runs when its PGM is newer than the table.
- **Output**: `Output/golden_results.tsv` has one row per job (Image, Dims,
  Bits, NEAR, matched/total bytes, Result, Run date, RTL hash). New rows
  replace old ones; images that no longer exist drop out.
  `Output/report_status.env` summarizes the table for a report page.
- **Status**: 10919/10919 jobs PASS, 250 images skipped (over the cap or
  W < 32). The full run takes ~1.5 h; an incremental run only re-runs what
  changed.

## Post-synth smoke test

`PostSynth/build_run.sh` synthesizes nl_top out of context (Vivado,
xczu7eg-1; BITNESS 8, 4096 x 4096, OUT_WIDTH 64) and runs the funcsim netlist
under NVC (`tb_nl_postsynth.vhd`). It uses the 8-bit golden images
<= `MAX_MP` (0.01), each at one NEAR rotating over 0, 3 and NMAX. Every lane
must match CharLS per stripe, byte-exact.

- The netlist is rebuilt only when the RTL hash changes (`--synth` forces).
- Images run in 16 shards, each capped at 1.5 GB.
- **Status**: 57/57 PASS, ~3 min.

`PostSynth/build_run_osvvm.sh` runs the nl_top stress TB
(`Top/tb_nl_top_osvvm.vhd`, `POST_SYNTH=true`) on the same 8-bit netlist. The
netlist is analyzed as `work.nl_top` and the TB's generic-free component binds
to it. `sClr` carries `keep` in `nl_top.vhd` so the TB's phase probe survives
synthesis. Needs the OSVVM library from `build_reports.sh`.

- **Status**: PASS, 129,061 affirmations, 7/7 requirements, ~3.5 min of simulation.

## Reports

`./publish_reports.sh` builds `Docs/Reports/index.html` from the OSVVM HTML,
coverage HTML and each suite's `Output/report_status.env`. Suites that have
not been run show as "not run".

## Suites

- **Loop**: `tb_nl_loop_osvvm` x BITNESS {8, 12, 16} x NEAR {0, 1, 3, 7, 31,
  max} x STRIPES {1, 4}, giving 36 variants.
- **Modules**: every other TB x BITNESS {8, 12, 16}, plus one `nl_framer`
  variant at OUT_WIDTH 200. That variant uses 25-byte beats, so the header
  ends exactly on a full beat. `tb_nl_merge_osvvm` (no BITNESS) runs with
  4-beat, 2-beat (1-beat payload) and the 1 KB default chunks.
- **Top**: `tb_nl_top_osvvm` x BITNESS {8, 12, 16}, plus 2-beat and 1 KB
  chunks and MAX_W / MAX_H 4096 at 12 bits. Six images per bitness
  (structured, smooth, NEAR 3 / NMAX, one row, 64x64 noise) are compared
  byte-exact per lane against CharLS. `Top/tb_nl_top_golden_pkg.vhd` is
  generated: rerun `Top/gen_golden.py` after changing the image set.

## Requirements

The requirement registry, owners, goals and the G1-G6 verification matrix are
in [`Docs/verification_requirements.md`](../../Docs/verification_requirements.md).
The rules are in [`Docs/verification_guidelines.md`](../../Docs/verification_guidelines.md).
Reused OpenJLS blocks (A10, A11, A11.2, A12, A20-A23) are verified upstream;
`tb_nl_backend` re-checks their IDs per lane, which covers the 4-lane
time-sharing of one instance.

## Code coverage (G6)

`build_run.sh` reports 100 % statement coverage (1131/1131) on every NL
source, and 100 % branch coverage except one line in `byte_stuffer_nl` (79/80).
The script prints statements only; branches are in the HTML report.
Justified gaps:

- **`byte_stuffer_nl` line 485, `sFifoFull = '0'` false**: the burst FIFO
  never fills. AlmFull leaves a 5-entry cushion for the ~4-cycle stall
  latency, but a lane writes only on its own slot (1 in 4 cycles), so at most
  about one word lands after AlmFull. `tb_byte_stuffer_nl` at 1 % iReady duty
  and `tb_nl_top` under backpressure do not reach it. The assert above the
  branch checks that nothing is pending whenever the FIFO is full.

- **`nl_framer` `get_header_byte` `when others`**: unreachable, because every
  caller passes an index below `HEADER_LEN`. It is excluded with
  `-- coverage off/on`.
- **`nl_framer` exact-end header beat**: reachable only when `BYTES_OUT`
  divides 25. The OUT_WIDTH 200 variant covers it.
- **`nl_pkg`**: types and constants only, nothing to instrument.
- **`nl_top`** is covered by `tb_nl_top` (42/42 statements, 26/26 branches).
  That TB also runs the `nl_top` AXIS hold and reset PSL under backpressure.

## Contracts the TBs rely on

- One lane line is at most 16383 pixels (image width <= 65532), so
  RUNcnt <= 16383 and RUNindex <= 30. `nl_backend` asserts this, and
  `tb_nl_backend` reaches RUNindex 30. See `Docs/decisions.md` D20.
