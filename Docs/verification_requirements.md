# Verification requirements

Requirement registry and traceability for the OSVVM suite in [`Verification/OSVVM`](../Verification/OSVVM).
Rules G1-G6 are defined in [`verification_guidelines.md`](verification_guidelines.md); coverage IDs carry the rule tags, and N/A reasons are in each TB's `Guidelines:` header.
Base: the OpenJLS matrix ([`OpenJLS/Docs/verification_requirements.md`](../OpenJLS/Docs/verification_requirements.md)), extended with the near-lossless paths and the NL lane/stripe behaviour.

## ID families
- `T87.*`: conformance. The RTL matches a reference derived from [T.87](Requirements.md) with the runtime NEAR. Clause numbers follow `Requirements.md` (`A4.1`, `A11.1` are sub-blocks of A.4, A.11).
- `NL.*`: NL behaviour outside the standard: lanes, stripes, context store and forwarding, framing, AXI, reset (G1.Nl).

TB is the owner (G1.Owner): the TB that checks the ID at the block that implements it. TBs in "(also ...)" re-check it through their own golden as an integration check.

Goal = guaranteed minimum check count per run, per variant (a TB that silently stops checking fails its requirement).
`M` = `MODE_AT * STRIPES`, `F` = `FWD_AT * STRIPES` in `tb_nl_loop`; `K` = `K_HI`, `N_IMAGES` and `N_RST` are TB constants.
Each build writes the measured counts to `reports/OSVVM_OpenJlsNl_req.csv`.

## Matrix
Status: `ok` met, `N/A` not applicable (reason in the TB header), `gap` open (see below). A column is `ok` when every sub-rule is met or N/A.

| ID | Requirement | Source | RTL | TB | Goal | G1 | G2 | G3 | G4 | G5 | G6 |
|---|---|---|---|---|---:|:-:|:-:|:-:|:-:|:-:|:-:|
| `T87.A2` | Mode selection, NEAR > 0 | T.87 A.2 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | 3*M | ok | ok | ok | ok | N/A | ok |
| `T87.A2.1` | RANGE, qbpp, bpp, LIMIT from NEAR | T.87 A.2.1 | [`nl_config`](../Sources/nl_config.vhd) | [`tb_nl_config`](../Verification/OSVVM/Modules/tb_nl_config_osvvm.vhd) | 768 | ok | ok | ok | ok | ok | ok |
| `T87.A3` | Mode selection, NEAR = 0 | T.87 A.3 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | 3*M | ok | ok | ok | ok | N/A | ok |
| `T87.A4` | Gradient quantization (Q3 in the loop, Q1/Q2 in the front end) | T.87 A.4 | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) (also `tb_nl_loop`) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A4.1` | Sign flip of the quantized vector | T.87 A.4.1 | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A4.2` | One-to-one vector -> context map | T.87 A.4.2 | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A5` | Edge-detecting predictor | T.87 A.5 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | M | ok | ok | ok | ok | N/A | ok |
| `T87.A6` | Bias correction and clamp | T.87 A.6 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | M | ok | ok | ok | ok | N/A | ok |
| `T87.A7` | Prediction error and sign | T.87 A.7 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | M | ok | ok | ok | ok | N/A | ok |
| `T87.A8` | Error quantization, Rx (NEAR > 0) | T.87 A.8 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | M | ok | ok | ok | ok | N/A | ok |
| `T87.A8.1` | Rx = Ix (NEAR = 0) | T.87 A.8.1 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | 3*M | ok | ok | ok | ok | N/A | ok |
| `T87.A9` | Modulo reduction | T.87 A.9 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | 2*WRAP_AT | ok | ok | ok | ok | N/A | ok |
| `T87.A10` | Golomb k | T.87 A.10 | [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A11` | Error mapping incl. special case | T.87 A.11 | [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A11.1` | LG(k, LIMIT) with escape | T.87 A.11.1 | [`A11_1_nl_golomb_encoder`](../Sources/A11_1_nl_golomb_encoder.vhd) | [`tb_a11_1_nl`](../Verification/OSVVM/Modules/tb_a11_1_nl_osvvm.vhd) (also `tb_nl_backend`) | qbpp*32*(K+1)*6 | ok | ok | ok | ok | N/A | ok |
| `T87.A12` | Context update: B and N in the loop, A in the back end | T.87 A.12 | [`nl_loop`](../Sources/nl_loop.vhd), [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd), [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 4*M / 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A13` | Bias update B, C | T.87 A.13 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | 2*M | ok | ok | ok | ok | N/A | ok |
| `T87.A14` | Run hit, Rx = Ra | T.87 A.14 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | M | ok | ok | ok | ok | N/A | ok |
| `T87.A15` | Run segments of length rg | T.87 A.15 | [`A15_A16_nl_encode_run`](../Sources/A15_A16_nl_encode_run.vhd) | [`tb_a15_a16_nl`](../Verification/OSVVM/Modules/tb_a15_a16_nl_osvvm.vhd) (also `tb_nl_backend`) | 400 | ok | ok | ok | ok | N/A | ok |
| `T87.A16` | Run end (EOL / interruption + J bits) | T.87 A.16 | [`A15_A16_nl_encode_run`](../Sources/A15_A16_nl_encode_run.vhd) | [`tb_a15_a16_nl`](../Verification/OSVVM/Modules/tb_a15_a16_nl_osvvm.vhd) (also `tb_nl_backend`) | 400 | ok | ok | ok | ok | N/A | ok |
| `T87.A17` | RItype | T.87 A.17 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | M | ok | ok | ok | ok | N/A | ok |
| `T87.A18` | RI prediction error | T.87 A.18 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | M | ok | ok | ok | ok | N/A | ok |
| `T87.A19` | RI error quantization, modulo, Rx | T.87 A.19 | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | 2*M | ok | ok | ok | ok | N/A | ok |
| `T87.A20` | TEMP | T.87 A.20 | [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A21` | map bit | T.87 A.21 | [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A22` | RI EMErrval | T.87 A.22 | [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.A22.1` | RI LG(k, glimit) | T.87 A.22.1 | [`A11_1_nl_golomb_encoder`](../Sources/A11_1_nl_golomb_encoder.vhd) | [`tb_a11_1_nl`](../Verification/OSVVM/Modules/tb_a11_1_nl_osvvm.vhd) | qbpp*32*6 | ok | ok | ok | ok | N/A | ok |
| `T87.A23` | RI update A / N / Nn | T.87 A.23 | [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `T87.C2.4.1.1` | Default T1/T2/T3 | T.87 C.2.4.1.1 | [`nl_config`](../Sources/nl_config.vhd) | [`tb_nl_config`](../Verification/OSVVM/Modules/tb_nl_config_osvvm.vhd) | 768 | ok | ok | ok | ok | ok | ok |
| `NL.CtxForward` | A lane's pending context update is forwarded to its next pixel | NL | [`nl_loop`](../Sources/nl_loop.vhd) | [`tb_nl_loop`](../Verification/OSVVM/Modules/tb_nl_loop_osvvm.vhd) | 2*F | ok | ok | ok | ok | N/A | ok |
| `NL.NearClamp` | NEAR > min(255, MAXVAL/2) is clamped | NL | [`nl_config`](../Sources/nl_config.vhd) | [`tb_nl_config`](../Verification/OSVVM/Modules/tb_nl_config_osvvm.vhd) | 255 - NMAX | ok | ok | ok | ok | ok | ok |
| `NL.Config` | RECIP, RANGE*(2*NEAR+1), derived constants | NL | [`nl_config`](../Sources/nl_config.vhd) | [`tb_nl_config`](../Verification/OSVVM/Modules/tb_nl_config_osvvm.vhd) | 1024 | ok | ok | ok | ok | ok | ok |
| `NL.LaneIssue` | Column j of lanes 0..3 on 4 consecutive slots | NL | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.Neighbours` | Rb, Rc, Rd from the lane's Rx lines | NL | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.CtxNumbering` | ctx = 9*(9*Q1 + Q2) + Q3 + 4 after the flip | NL | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.Q3Bounds` | Q3 from the Ra bounds equals A.4(Rc - Ra) | NL | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.CtxStore` | Candidate C, B, N = latest regular exit of the lane | NL | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.ClearSweep` | 512-cycle context clear before each image | NL | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 100 | ok | ok | ok | ok | ok | ok |
| `NL.ExitTags` | oExLane / FirstPix / LastPix / Eol with oExValid | NL | [`nl_frontend`](../Sources/nl_frontend.vhd) | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.RunLaneState` | Run state per lane; idle and reset slots hold it | NL | [`A15_A16_nl_encode_run`](../Sources/A15_A16_nl_encode_run.vhd) | [`tb_a15_a16_nl`](../Verification/OSVVM/Modules/tb_a15_a16_nl_osvvm.vhd) | 400 | ok | ok | ok | ok | N/A | ok |
| `NL.LaneState` | A RAM and run state per lane, cleared per image | NL | [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.WordTags` | One tagged word per coded token, in order | NL | [`nl_backend`](../Sources/nl_backend.vhd) | [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.LaneStuff` | Per-lane FF stuffing and flush on the shared ring | NL | [`byte_stuffer_nl`](../Sources/byte_stuffer_nl.vhd) | [`tb_byte_stuffer_nl`](../Verification/OSVVM/Modules/tb_byte_stuffer_nl_osvvm.vhd) | 2*4*(N_IMAGES+1) | ok | ok | ok | ok | ok | ok |
| `NL.LaneFrame` | Per-lane header (with NEAR), payload, FF D9, oLast | NL | [`nl_framer`](../Sources/nl_framer.vhd) | [`tb_nl_framer`](../Verification/OSVVM/Modules/tb_nl_framer_osvvm.vhd) | 2*4*N_IMAGES | ok | ok | ok | ok | ok | ok |
| `NL.StripeGolden` | Each slice's chunk payloads, concatenated, equal CharLS on that stripe, byte-exact | NL | [`nl_top`](../Sources/nl_top.vhd) | [`tb_nl_top`](../Verification/OSVVM/Top/tb_nl_top_osvvm.vhd) (also Golden suite) | 4*(36 + N_RST) | ok | ok | ok | ok | ok | ok |
| `NL.BackToBack` | Images back to back without reset, each one framed | NL | [`nl_top`](../Sources/nl_top.vhd) | [`tb_nl_top`](../Verification/OSVVM/Top/tb_nl_top_osvvm.vhd) | 20 | ok | ok | ok | ok | ok | ok |
| `NL.ImageFlood` | Minimal back-to-back images, outputs ready or held, stall the input at the in-flight cap instead of overflowing the per-image queues; every image byte-exact | NL | [`nl_top`](../Sources/nl_top.vhd) | [`tb_nl_top`](../Verification/OSVVM/Top/tb_nl_top_osvvm.vhd) | 18 | ok | ok | ok | ok | ok | ok |
| `NL.AxisOut` | oKeep all ones except on oLast, contiguous from the MSB; beat held while iReady is low; last chunk only for a queued image | NL | [`nl_merge`](../Sources/nl_merge.vhd) | [`tb_nl_merge`](../Verification/OSVVM/Modules/tb_nl_merge_osvvm.vhd) (also `tb_nl_top`) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.StallPropagation` | A held output (from the start or mid image) drops oReady once the chunk buffers fill; nothing lost | NL | [`nl_top`](../Sources/nl_top.vhd) | [`tb_nl_top`](../Verification/OSVVM/Top/tb_nl_top_osvvm.vhd) | 2 | ok | ok | ok | ok | ok | ok |
| `NL.Chunks` | Chunk stream per [output_format](output_format.md): FF B0, version 1, reserved 0, band = BAND_ID, image_count per slice, header + length = CHUNK_BYTES except the last; oLast exactly at each chunk end | NL D24 | [`nl_merge`](../Sources/nl_merge.vhd) | [`tb_nl_merge`](../Verification/OSVVM/Modules/tb_nl_merge_osvvm.vhd) (also `tb_nl_top`) | 1000 | ok | ok | ok | ok | ok | ok |
| `NL.MergeOrder` | Each slice's chunks out in order, every input byte once, under any lane order and backpressure | NL D24 | [`nl_merge`](../Sources/nl_merge.vhd) | [`tb_nl_merge`](../Verification/OSVVM/Modules/tb_nl_merge_osvvm.vhd) | 1000 | ok | ok | N/A | ok | ok | ok |
| `NL.MergeFair` | Round robin: a closed chunk waits at most 8 other-lane packets | NL D24 | [`nl_merge`](../Sources/nl_merge.vhd) | [`tb_nl_merge`](../Verification/OSVVM/Modules/tb_nl_merge_osvvm.vhd) | 1000 | ok | ok | N/A | ok | ok | ok |
| `NL.NoStall` | Without backpressure oReady drops only in the config divider, the clear sweep and the row-0 wait | NL D3 | [`nl_top`](../Sources/nl_top.vhd) | [`tb_nl_top`](../Verification/OSVVM/Top/tb_nl_top_osvvm.vhd) | 7 | ok | ok | ok | ok | ok | ok |
| `NL.ConfigInReset` | NEAR, W, H sampled in reset, also on a 1-cycle reset | NL D23 | [`nl_top`](../Sources/nl_top.vhd) | [`tb_nl_top`](../Verification/OSVVM/Top/tb_nl_top_osvvm.vhd) | 24 | ok | ok | ok | ok | ok | ok |
| `NL.Reset` | Reset in every phase clears outputs; the next session is scored | NL | every stateful block | [`tb_nl_frontend`](../Verification/OSVVM/Modules/tb_nl_frontend_osvvm.vhd), [`tb_nl_backend`](../Verification/OSVVM/Modules/tb_nl_backend_osvvm.vhd), [`tb_byte_stuffer_nl`](../Verification/OSVVM/Modules/tb_byte_stuffer_nl_osvvm.vhd), [`tb_nl_framer`](../Verification/OSVVM/Modules/tb_nl_framer_osvvm.vhd), [`tb_nl_merge`](../Verification/OSVVM/Modules/tb_nl_merge_osvvm.vhd), [`tb_nl_config`](../Verification/OSVVM/Modules/tb_nl_config_osvvm.vhd), [`tb_nl_top`](../Verification/OSVVM/Top/tb_nl_top_osvvm.vhd) | per TB | ok | N/A | N/A | ok | ok | ok |

## Open gaps
None.

## Not tracked
- Reused OpenJLS blocks (`A10_compute_k`, `A11_error_mapping`, `A11_2_bit_packer`, `A12_variables_update`, `A20`-`A23`, `olo_base_*`): verified by the upstream suite, see [`OpenJLS/Docs/verification_requirements.md`](../OpenJLS/Docs/verification_requirements.md). `tb_nl_backend` re-checks their IDs per lane, which covers the 4-lane time-sharing of one instance.
- Frame sizes above `MAX_W` / `MAX_H` (generic-bounded).
