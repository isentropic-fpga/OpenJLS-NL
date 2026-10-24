# Design decisions log

Crossroads hit while building the 4-lane core, the options seen, and what was
picked. **Picked** marks the choice; everything here is open for review.

## D1. Lane = stripe = independent JPEG-LS image
- A: each of the 4 vertical stripes is its own Ws x H image with its own
  header/scan (4 output streams). **Picked** - compliant with zero decoder
  tricks; matches "4 parallel outputs like OpenJLS".
- B: one image, 4 scans/restart intervals. Needs DRI-like structure that
  JPEG-LS does not have for vertical stripes. Rejected.

## D2. Stripe width
- A: require W divisible by 4, Ws = W/4, Ws >= 8. **Picked** (simplest).
  Ws >= 8 keeps the Rx line-buffer write ahead of the next row's read.
- B: uneven stripes (last stripe takes the remainder). Later if needed.

## D3. Input reorder (raster -> lane interleave)
- A: double-buffered row RAM (2 x MAX_W), lanes in lockstep one column group
  (4 slots) at a time. 1 px/cycle, 1 row latency. Replaced by C.
- B: per-lane FIFOs. Same storage, more control.
- C: one-row RAM (MAX_W) at the raster column, lanes skewed. Lane k issues on
  slot phase k as soon as its pixel is written; the writer only overwrites a
  pixel its lane has read. A lane reads at 1/4 px/cycle, so by the end of its
  stripe it trails the writer by ~3*Ws < W pixels and the writer never waits.
  Lanes wait for each other at the image end (shared clear sweep).
  **Picked**: half the input RAM, 1 px/cycle, lane k starts k*Ws cycles in.

## D4. MAXVAL and thresholds
- A: MAXVAL = 2^BITNESS-1 and default T1..T3 (computed from runtime NEAR).
  No LSE marker needed, the header only changes in the NEAR byte. **Picked**.
- B: runtime MAXVAL/T + LSE marker. Later if needed.

## D5. Context numbering
- ctx = 9*row + bank, row = 9|Q1|+Q2 after sign merge (0..40), bank = Q3+4.
  Row 0 uses banks 5..8 only, so ids 0..4 are free: RI contexts use 0 (RItype 0)
  and 1 (RItype 1). Keeps CO_CTX_WIDTH = 9, no loop change.

## D6. Context RAM forwarding
- RAM read is 2 cycles before the loop input; writes are registered after the
  loop exit. Only one lane write can be missing from the RAM data (the one in
  the write register the cycle the slot enters the loop): forwarded there by
  row compare + bank match. The loop forwards the newest one itself.

## D7. Context reset between images
- A: clear sweep of the context and A RAMs at each image start (~512 cycles,
  input keeps filling the reorder buffer meanwhile). **Picked**.
- B: ping-pong context memories (clear one while using the other). 2x RAM.
- C: per-entry valid bits / epoch tags. More logic, aliasing headaches.

## D8. A (and RI N/Nn) storage
- A RAM per lane (512 deep), RMW within the 4-cycle lane spacing (no
  forwarding needed). RI contexts store {A, N, Nn} in the same word.
  RI update uses A += |Errval| - RItype (equal to (EMErrval+1-RItype)>>1),
  so the RMW does not wait for k/map.

## D9. Backpressure
- A: global clock enable on the whole pipeline, loop included (OpenJLS style).
  Touches the loop's timing (big CE fanout).
- B: stop issuing new slots when any lane is near full; in-flight
  pixels drain into a per-lane skid FIFO between bit packer and byte stuffer.
  **Picked**: loop stays CE-free. Costs 4 small FIFOs.

## D10. Output streams
- 4 independent OpenJLS-style streams (oData/oValid/oKeep/oLast/iReady per
  lane), each with its own bit packer, byte stuffer and framer. AXI/merging is
  deferred (open question for review).
- Framer copied as nl_framer with an iNear port (the OpenJLS one hardcodes 0).

## D11. Runtime NEAR setup
- iNear latched while iRst = 1 (like width/height). After reset a serial
  divider computes RANGE and the 1/(2N+1) reciprocal; oReady stays low meanwhile.

## D12. Front-end pipeline depth
- Loop input at issue+8 (I8), exit at I15. Ctx RAM read at I6, data at I8.
  The only write the RAM can miss is the same lane's previous pixel, still in
  the loop: compared against the loop's exit ctx at I7 and forwarded (D6).
- Neighbours: rotating 4-entry ring per lane (prev Rd, prev Rb, col-0 Rb),
  so one line-buffer read per pixel.

## D13. A RAM layout
- A: one 512-deep RAM per lane (4 small RAMs). **Picked**: no port sharing,
  clear runs in parallel with the ctx clear.
- B: one shared 2048-deep RAM, lane in the address. Fewer BRAMs, but 4 lanes
  RMW in the same 4-cycle window needs a second port. Later if area matters.

## D14. Back end sharing (revises D10)
- A: one Golomb/run/bit-packer pipeline per lane (D10 as written). 4x logic,
  each idle 3 cycles out of 4.
- B: one shared back end and bit packer, words tagged with the lane and
  demuxed into the per-lane skid FIFOs. **Picked**: run state and A RAMs are
  per lane, but the arithmetic is shared because lanes never collide (one
  loop exit per cycle). Byte stuffers and framers stay per lane.

## D15. Skid FIFO sizing
- Per lane: 32 words of {last, len, LIMIT bits}, AlmFull at 20 drives the
  front-end hold (OR of 4, registered). Pops when the byte stuffer's
  AlmostFull (registered) is low. Tested with iReady at 1% duty; removing the
  hold trips the overflow assertion.

## D16. Framer start / header
- Framer iStart = loop exit FirstPix of that lane (not the input), so the
  start is in order with that lane's words. Header width = W/4, NEAR in the
  SOF/SOS as in T.87; default thresholds, so no LSE marker (matches CharLS).

## D17. Reference
- Verification/OSVVM/Golden/ref/nl_charls.cpp: CharLS per stripe at the
  same NEAR. Gated on T16E0.JLS and T16E3.JLS byte-exact. nl_top output is compared byte-exact per lane.

## D18. Back end reuses OpenJLS blocks
- A10, A11, A12, A15/A16, A20-A23 and A11_2 are used unchanged. NEAR-only
  changes go in `Sources/A<n>_nl_*.vhd` overrides, changed lines marked `-- NL`:
  A11_1_nl (QBPP is a runtime port).
- A11's special map (k=0, 2B <= -N) is off when NEAR /= 0 by driving iBq = 0.
- A23 is combinational, and the A RAM word settles deltas after r(2). Its inputs
  are built in one process (one delta), and RItype is dropped when A = Errval = 0.
  That is a sim-only guard: real RItype=1 tokens have Errval /= 0.
- Golden suite (Verification/OSVVM/Golden): all corpus images <= 0.5 MP at
  NEAR 3, 7, a sweep set at NEAR 0..16 and the NL corpus (stripe seams,
  BITNESS 8..16, W 32 / 36 / 65532), byte-exact vs CharLS per stripe:
  10919/10919 jobs.

## D19. One byte stuffer and framer for all 4 lanes
- The 4 lanes take turns on one byte stuffer and one framer (lane = slot
  counter, one lane per clock). Each register in them is a 4-slot ring that
  rotates every clock, so each lane keeps its own state.
  Generated by `Scripts/cslow.py` into `byte_stuffer_nl.vhd`; same
  edit in `nl_framer.vhd`.
- Per-lane skid FIFOs in front (popped at the lane's slot), per-lane output
  FIFOs (depth 4) behind. Output ports stay 4 plain per-lane streams.
- Limit: 4 payload bytes per clock for the whole core (1 per lane).
- Saves ~10k LUT and ~750 FF at 12 bits; fmax unchanged.

## D20. Lane line <= 16383 pixels
- MAX_IMAGE_WIDTH <= 65532, so one lane line is at most 16383 pixels.
- So RUNcnt <= 16383 and RUNindex <= 30 (a J=14 segment needs 16384).
- nl_backend keeps the full A.14 run count per lane (not the remainder of
  the segment) and asserts RUNcnt <= 16383. tb_nl_backend reaches
  RUNindex 30.

## D21. Stale loop exits dropped (OSVVM finding)
- After a reset, nl_loop can still deliver exits for pixels issued before
  it. nl_frontend now gates every use of iExValid with p(E).valid (a slot
  issued since reset): forwarding, line-buffer write, context write back,
  and oExValid to the back end.
- Found by tb_nl_frontend (reset in every phase, G5).

## D22. Golomb encoder asserts qualified by valid
- The range assertions in A11_1_nl_golomb_encoder hold only for valid
  tokens.
- nl_backend drives its iValid from a registered sGolV (valid and not a
  run-segment token), aligned with the registered k / MErrval. A
  combinational valid glitched in delta time and fired the k assert.

## D23. Front end samples the size ports in reset (OSVVM finding)
- nl_frontend latches width / height while iRst = 1. nl_top fed it the
  registered sWidth / sHeight, which lag the ports by one cycle, so a
  one-cycle reset kept the previous image's size.
- nl_top now drives the front end from iImageWidth / iImageHeight directly.
  The framer still uses the registered copies (it reads them after reset).
- Found by tb_nl_top (1-cycle reset with a new config, G5).

## D24. Output structure: one chunk stream per band
- Chosen: B (below). Spec: [output_format.md](output_format.md); reference
  unpacker: `Scripts/nl_unpack.py`. RTL: `nl_merge` after the framer.
- Constraints: vendor-agnostic core, Xilinx specifics only in the AXI
  wrappers; must not require MCDMA; no mission policy in the IP (packet
  standards, APIDs or other IDs that could clash with the client's).
- Facts: slice rates differ by content, so a fixed-ratio merge pads or
  deadlocks. Beat interleave needs a tag per beat and is illegal into a DMA
  (TID/TDEST fixed per packet, PG021). TID alone steers nothing: plain AXI DMA
  drops it, MCDMA only writes it to the BD status; only MCDMA TDEST picks a
  DDR region (max 16 channels).
- Options:
  - A. Keep separate streams (4 per band, 20 for 5 bands). Rejected: pushes
    the merge onto every client.
  - B. One stream per band: per-slice ping-pong chunk buffers (generic
    `CHUNK_BYTES` including the header, default 1 KB, all full except a
    slice's last), each chunk sent after an 8-byte header. Chosen.
    Header (64 bits): FF B0 | ver(4) slice(2) last(1) rsvd(1) | band(8) |
    image_count(16) | length(16).
    - FF B0: reserved T.81 marker (RES range). Never in coded data (stuffed
      bit after FF); our marker segments only put bytes < 0x40 or fixed
      marker bytes after FF. Recheck if APP/COM/LSE segments are added. Can
      appear inside a header (band, image_count), so resync must also check
      the version, the length and the next header at +length.
    - band: generic BAND_ID, set by the client per instance. Bands merged
      outside the IP stay identifiable.
    - image_count: internal, cleared on reset, +1 per image. Consecutive
      images overlap at the output (lane skew), and a lost last chunk would
      otherwise merge two images.
    - CHUNK_BYTES counts the header, so a full chunk fills a power-of-two
      buffer. OUT_WIDTH is fixed at 64: the header is one beat and the
      payload whole beats, no realign. The core emits at most 4 bytes per
      clock, so 64 bits has 2x headroom; a standard AXIS width converter
      widens it. A native wider bus would need a write-side realign whose
      shift moves 8 bytes per chunk (payload CHUNK_BYTES - 8 is not a beat
      multiple); deferred until needed.
    - oLast ends every chunk (one DMA packet each); oKeep is all ones except
      on oLast.
    - A lost middle chunk is not visible in the headers; left to the link
      layer (sequence count, CRC).
  - C. As B, with the slice index as sideband (TID/TUSER) and no header.
    Rejected: only works for consumers that keep the sideband.
- No in-IP AXI4 DDR writer. Scope rule: D27.

## D25. Lane count fixed at 4
- 4 lanes already match OpenJLS throughput; a lane-count generic (8 lanes to
  drop the Q3 speculation) is over-engineering.

## D26. Full-beat stuffer + word framer (tried, rejected)
- Stuffer emitting only full 4-byte beats (33 refill alignments) and a framer
  rebuilt around a word accumulator; both passed OSVVM and the golden slice.
- Stuffer +642 LUT (8-bit) / +1202 (16-bit), framer only -357: the framer cost
  is the per-cycle ring logic, not the byte-offset mux. nl_top P&R: 8-bit +246
  LUT, 292 -> 273 MHz; 16-bit +787 LUT, 260 -> 274 MHz. Not ported to OpenJLS.

## D27. IP scope: FPGA vs software
- Rule: a feature goes in the FPGA only if it must run at line rate or needs
  information only the FPGA has. Everything else is software or the client's.
- FPGA: compress (1 pixel/cycle, 4 valid .jls slices per band); merge the
  slices into one stream per band without loss or deadlock (D24); write the
  header (slice, last, length, version, band, image_count); report status
  (config errors, overflow, done) as flags.
- Software / client: packet standards (CCSDS/PUS), mission IDs, timestamps,
  link CRC and framing, DMA setup and storage, merging bands, demux, decoding,
  stitching slices and bands into an image.
- Shipped with the RTL: output format spec and a reference unpacker. AXI
  wrappers only map signals.

## Status (nl_top, xczu7eg-1, OOC, Vivado 2025.2)
Before nl_merge (D24), MAX_IMAGE_WIDTH 4096:
| BITNESS | est. fmax | LUT | FF | BRAM | DSP |
|---|---|---|---|---|---|
| 8  | 288 MHz | 8646  | 4737 | 9.5  | 3 |
| 12 | 281 MHz | 10027 | 5460 | 11   | 4 |
| 16 | 281 MHz | 11363 | 6173 | 14.5 | 4 |
- Shared stuffer + framer (D19): ~2.0-3.3k + ~2.8k LUT.
- Worst path is still the loop (r(3).dS to r(4).ra).
- Sim throughput ~0.98 pix/cycle on 512-wide images (no back-pressure).

With nl_merge (D24 B), BITNESS 12, MAX_IMAGE_WIDTH 12288, 2 ns target, retiming:
| Design | est. fmax | LUT | FF | BRAM | DSP |
|---|---|---|---|---|---|
| OpenJLS | 269 MHz | 6578  | 2040 | 5.5  | 0 |
| NL      | 287 MHz | 11375 | 5765 | 17.5 | 4 |
- LUT: framer 3.9k, stuffer 2.5k, frontend 1.6k, backend 1.3k, loop 1.2k,
  merge 0.3k. Worst path moved to the framer (header byte index to buffer).
- BRAM: frontend rows 9, context banks 4.5, backend 2, merge 2. Context banks
  to LUTRAM would save ~4.5 for ~0.8k LUT (not done).

## Open (for review)
- Constraint W % 4 = 0, W/4 >= 8 (asserted in nl_top).
