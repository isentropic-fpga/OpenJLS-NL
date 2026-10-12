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
