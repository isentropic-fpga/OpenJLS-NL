# OpenJLS-NL OSVVM verification

OSVVM regression for the NL blocks. It follows `Docs/verification_guidelines.md`
(G1–G6) and reuses the OpenJLS infrastructure: the OSVVM library, the
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
| `PostSynth/` | Gate-level smoke test of the synthesized netlist (`build_run.sh`) |
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
- **Batches**: `wide` runs every OpenJLS corpus image ≤ `MAX_MP` (0.5) at
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
≤ `MAX_MP` (0.01), each at one NEAR rotating over 0, 3 and NMAX. Every lane
must match CharLS per stripe, byte-exact.

- The netlist is rebuilt only when the RTL hash changes (`--synth` forces).
- Images run in 16 shards, each capped at 1.5 GB.
- **Status**: 57/57 PASS, ~3 min.

## Reports

`./publish_reports.sh` builds `Docs/Reports/index.html` from the OSVVM HTML,
coverage HTML and each suite's `Output/report_status.env`. Suites that have
not been run show as "not run".

## Suites

- **Loop**: `tb_nl_loop_osvvm` × BITNESS {8, 12, 16} × NEAR {0, 1, 3, 7, 31,
  max} × STRIPES {1, 4}, giving 36 variants.
- **Modules**: every other TB × BITNESS {8, 12, 16}, plus one `nl_framer`
  variant at OUT_WIDTH 200. That variant uses 25-byte beats, so the header
  ends exactly on a full beat.
- **Top**: `tb_nl_top_osvvm` × BITNESS {8, 12, 16}, plus OUT_WIDTH 48,
  OUT_WIDTH 1024 and MAX_W / MAX_H 4096 at 12 bits. Six images per bitness
  (structured, smooth, NEAR 3 / NMAX, one row, 64x64 noise) are compared
  byte-exact per lane against CharLS. `Top/tb_nl_top_golden_pkg.vhd` is
  generated: rerun `Top/gen_golden.py` after changing the image set.

## Requirements registry and owners

There are two ID families:

- `T87.*` is conformance. Clause numbers follow `Docs/Requirements.md`.
- `NL.*` covers NL-specific behaviour: lanes, the context store, framing and
  reset.

An ID can be checked by more than one TB. The **owner** is the TB that
verifies the requirement at the block that implements it. Other TBs re-check
it through their own golden as an integration check.

The goal is the minimum check count each TB registers with `GetReqID`, per
variant. `M` = `MODE_AT * STRIPES` and `F` = `FWD_AT * STRIPES` in
`tb_nl_loop`.

| ID | Requirement | Owner | Also checked by | Goal (owner) |
|---|---|---|---|---:|
| `T87.A2` | Mode selection, NEAR > 0 | `tb_nl_loop` | | 3·M |
| `T87.A2.1` | RANGE, qbpp, bpp, LIMIT from NEAR | `tb_nl_config` | | 768 |
| `T87.A3` | Mode selection, NEAR = 0 | `tb_nl_loop` | | 3·M |
| `T87.A4` | Gradient quantization (Q3 in the loop, Q1/Q2 in the front end) | `tb_nl_frontend` | `tb_nl_loop` | 1000 |
| `T87.A4.1` | Sign flip of the quantized vector | `tb_nl_frontend` | | 1000 |
| `T87.A4.2` | One-to-one vector → context map | `tb_nl_frontend` | | 1000 |
| `T87.A5` | Edge-detecting predictor | `tb_nl_loop` | | M |
| `T87.A6` | Bias correction and clamp | `tb_nl_loop` | | M |
| `T87.A7` | Prediction error and sign | `tb_nl_loop` | | M |
| `T87.A8` | Error quantization, Rx (NEAR > 0) | `tb_nl_loop` | | M |
| `T87.A8.1` | Rx = Ix (NEAR = 0) | `tb_nl_loop` | | 3·M |
| `T87.A9` | Modulo reduction | `tb_nl_loop` | | 2·WRAP_AT |
| `T87.A10` | Golomb k | `tb_nl_backend` | | 1000 |
| `T87.A11` | Error mapping incl. special case | `tb_nl_backend` | | 1000 |
| `T87.A11.1` | LG(k, LIMIT) with escape | `tb_a11_1_nl` | `tb_nl_backend` | qbpp × 32 × k × 6 |
| `T87.A12` | Context update: B and N in the loop, A in the back end | `tb_nl_loop`, `tb_nl_backend` | | 4·M / 1000 |
| `T87.A13` | Bias update B, C | `tb_nl_loop` | | 2·M |
| `T87.A14` | Run hit, Rx = Ra | `tb_nl_loop` | | M |
| `T87.A15` | Run segments of length rg | `tb_a15_a16_nl` | `tb_nl_backend` | 400 |
| `T87.A16` | Run end (EOL / interruption + J bits) | `tb_a15_a16_nl` | `tb_nl_backend` | 400 |
| `T87.A17` | RItype | `tb_nl_loop` | | M |
| `T87.A18` | RI prediction error | `tb_nl_loop` | | M |
| `T87.A19` | RI error quantization, modulo, Rx | `tb_nl_loop` | | 2·M |
| `T87.A20` | TEMP | `tb_nl_backend` | | 1000 |
| `T87.A21` | map bit | `tb_nl_backend` | | 1000 |
| `T87.A22` | RI EMErrval | `tb_nl_backend` | | 1000 |
| `T87.A22.1` | RI LG(k, glimit) | `tb_a11_1_nl` | | qbpp × 32 × 6 |
| `T87.A23` | RI update A / N / Nn | `tb_nl_backend` | | 1000 |
| `T87.C2.4.1.1` | Default T1/T2/T3 | `tb_nl_config` | | 768 |
| `NL.CtxForward` | A lane's pending context update is forwarded to its next pixel | `tb_nl_loop` | | 2·F |
| `NL.NearClamp` | NEAR > min(255, MAXVAL/2) is clamped | `tb_nl_config` | | 255 − NMAX |
| `NL.Config` | RECIP, RANGE·(2·NEAR+1), derived constants | `tb_nl_config` | | 1024 |
| `NL.LaneIssue` | Column j of lanes 0..3 on 4 consecutive slots | `tb_nl_frontend` | | 1000 |
| `NL.Neighbours` | Rb, Rc, Rd from the lane's Rx lines | `tb_nl_frontend` | | 1000 |
| `NL.CtxNumbering` | ctx = 9·(9·Q1 + Q2) + Q3 + 4 after the flip | `tb_nl_frontend` | | 1000 |
| `NL.Q3Bounds` | Q3 from the Ra bounds equals A.4(Rc − Ra) | `tb_nl_frontend` | | 1000 |
| `NL.CtxStore` | Candidate C, B, N = latest regular exit of the lane | `tb_nl_frontend` | | 1000 |
| `NL.ClearSweep` | 512-cycle context clear before each image | `tb_nl_frontend` | | 100 |
| `NL.ExitTags` | oExLane / FirstPix / LastPix / Eol with oExValid | `tb_nl_frontend` | | 1000 |
| `NL.RunLaneState` | Run state per lane; idle and reset slots hold it | `tb_a15_a16_nl` | | 400 |
| `NL.LaneState` | A RAM and run state per lane, cleared per image | `tb_nl_backend` | | 1000 |
| `NL.WordTags` | One tagged word per coded token, in order | `tb_nl_backend` | | 1000 |
| `NL.LaneStuff` | Per-lane FF stuffing and flush on the shared ring | `tb_byte_stuffer_nl` | | 2·4·(N+1) |
| `NL.LaneFrame` | Per-lane header (with NEAR), payload, FF D9, oLast | `tb_nl_framer` | | 2·4·N |
| `NL.StripeGolden` | Each lane stream equals CharLS on that stripe, byte-exact | `tb_nl_top` | Golden suite | 4·(36 + N_RST) |
| `NL.BackToBack` | Images back to back without reset, each one framed | `tb_nl_top` | | 20 |
| `NL.AxisOut` | oKeep contiguous from the MSB, oLast per image, beat held while iReady is low | `tb_nl_top` | | 1000 |
| `NL.StallPropagation` | A held lane drops oReady (skid FIFO almost full); nothing lost | `tb_nl_top` | | 5 |
| `NL.NoStall` | Without backpressure oReady drops only in the config divider, the clear sweep and the row-0 wait (D3) | `tb_nl_top` | | 7 |
| `NL.ConfigInReset` | NEAR, W, H sampled in reset, also on a 1-cycle reset (D23) | `tb_nl_top` | | 24 |
| `NL.Reset` | Reset in every phase clears outputs; the next session is scored | every stateful TB | | per TB |

### Reused blocks (verified upstream, OpenJLS suite)

| Block | Used by | Upstream ID |
|---|---|---|
| `A10_compute_k` | `nl_backend` | `T87.A10`, `T87.A20.2` |
| `A11_error_mapping` | `nl_backend` | `T87.A11` |
| `A11_2_bit_packer` | `nl_backend` | `T87.A11.2` |
| `A12_variables_update` | `nl_backend` | `T87.A12` |
| `A20_compute_temp` | `nl_backend` | `T87.A20` |
| `A21_compute_map` | `nl_backend` | `T87.A21` |
| `A22_errval_mapping` | `nl_backend` | `T87.A22` |
| `A23_run_interruption_update` | `nl_backend` | `T87.A23` |
| `olo_base_*` (Open Logic) | FIFOs and RAMs | third party |

`tb_nl_backend` still checks these IDs per lane. The NL addition is the
4-lane time-sharing of a single instance of each block, which is what the
check covers.

## Code coverage (G6)

`build_run.sh` reports 100 % statement coverage (1024/1024) on every NL
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

- One lane line is at most 16383 pixels (image width ≤ 65532), so
  RUNcnt ≤ 16383 and RUNindex ≤ 30. `nl_backend` asserts this, and
  `tb_nl_backend` reaches RUNindex 30. See `Docs/decisions.md` D20.
