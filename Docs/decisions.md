# Design decisions log

Crossroads hit while building the 4-lane core, the options seen, and what was
picked. **Picked** marks the choice; everything here is open for review.

## D1. Lane = stripe = independent JPEG-LS image
- A: each of the 4 vertical stripes is its own Ws x H image with its own
  header/scan (4 output streams). **Picked** — compliant with zero decoder
  tricks; matches "4 parallel outputs like OpenJLS".
- B: one image, 4 scans/restart intervals. Needs DRI-like structure that
  JPEG-LS does not have for vertical stripes. Rejected.

## D2. Stripe width
- A: require W divisible by 4, Ws = W/4, Ws >= 8. **Picked** (simplest).
  Ws >= 8 keeps the Rx line-buffer write ahead of the next row's read.
- B: uneven stripes (last stripe takes the remainder). Later if needed.

## D3. Input reorder (raster -> lane interleave)
- A: double-buffered row RAM (2 x MAX_W). Row r+1 is written while row r is
  read as (col j, lane 0..3). **Picked**: deterministic, 1 px/cycle, 1 row latency.
- B: per-lane FIFOs. Same storage, more control.
- Lanes advance in lockstep, one column group (4 slots) at a time, starting
  on slot phase 0, so lane = slot mod 4 always holds.

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
- B: stop issuing new column groups when any lane is near full; in-flight
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
- Verification/ref/nl_charls.cpp: CharLS per stripe at the same NEAR. Gated
  on T16E3.JLS byte-exact. nl_top output is compared byte-exact per lane.

## D18. Back end reuses OpenJLS blocks
- A10, A11, A12, A15/A16, A20-A23 and A11_2 are used unchanged. NEAR-only
  changes go in `Sources/A<n>_nl_*.vhd` overrides, changed lines marked `-- NL`:
  A11_1_nl (QBPP is a runtime port).
- A11's special map (k=0, 2B <= -N) is off when NEAR /= 0 by driving iBq = 0.
- A23 is combinational, and the A RAM word settles deltas after r(2). Its inputs
  are built in one process (one delta), and RItype is dropped when A = Errval = 0.
  That is a sim-only guard: real RItype=1 tokens have Errval /= 0.
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

## Status (nl_top, xczu7eg-1, OOC, Vivado 2025.2)
| BITNESS | est. fmax | LUT | FF | BRAM | DSP |
|---|---|---|---|---|---|
| 8  | 288 MHz | 8646  | 4737 | 9.5  | 3 |
| 12 | 281 MHz | 10027 | 5460 | 11   | 4 |
| 16 | 281 MHz | 11363 | 6173 | 14.5 | 4 |
- Shared stuffer + framer (D19): ~2.0-3.3k + ~2.8k LUT.
- Worst path is still the loop (r(3).dS to r(4).ra).
- Sim throughput ~0.98 pix/cycle on 512-wide images (no back-pressure).

## Open (for review)
- Output: 4 separate JPEG-LS streams today. Options: keep 4 AXI-S ports,
  merge into one stream (needs a container or restart-marker scheme), or
  DMA each stripe to its own buffer.
- Constraint W % 4 = 0, W/4 >= 8 (asserted in nl_top).
