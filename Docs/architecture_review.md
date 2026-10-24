# Architecture review (2026-10-19)

Full read of nl_top, nl_config, nl_frontend, nl_loop, nl_backend, the A-block
overrides, byte_stuffer_nl and nl_framer against T.87. No compliance issue found.

## Keep
- D1 vertical stripes: the only compliant parallel split (raster-order context
  updates rule out row wavefronts and bands).
- 4-way C-slowed loop + 9-way Q3 speculation: minimal, only Q3 depends on Ra.
- Shared back end, stuffer and framer with per-lane state (D14, D19).
- Reciprocal division is exact for every legal NEAR: with s = B+10,
  c = ceil(2^s/D): c*D - 2^s <= D-1 <= 510 < 2^(s-(B+1)) = 512, and
  m = |e| + NEAR < 1.5*2^B.

## Findings, in priority order
1. ~~Golden sweeps NEAR 0..16 only.~~ Wrong: the `nl` batch already runs NEAR = min(255, MAXVAL/2)
   at every BITNESS 8..16 (all PASS).
2. Buffering per lane is 4 deep: skid FIFO (32) -> stuffer FIFO (16) + last-word
   queue -> framer byte buffer -> output FIFO (4). The skid exists only because
   stuffer stage 1 cannot backpressure. Driving iHold from the stuffer FIFO
   AlmFull (cushion = issue-to-stuffer latency / 4) drops one layer. Count the
   latency before sizing.
3. C-slow rings in the stuffer/framer rotate every wide register every clock
   (toggle power) and reset all slots (no SRL). For space: FF bank per lane with
   one-hot write enable + 4:1 read mux, reset only control bits. LUTRAM state
   file is smaller but not scrubbable. Measure power before deciding.
4. ~~Lane count as a generic tied to loop depth.~~ Decided: fixed at 4 (D25).
   Lane count as a generic tied to loop depth. 8 lanes: Ra known before the ctx
   read, so no 9-bank speculation, no Q3 mux, no forwarding, and the two
   multiplies get their own stages (fmax DSP/BRAM-bound). Cost: 2x per-lane
   state, W >= 64, compression loss from more stripes (measure with CharLS per
   stripe first). Main lever for slower space fabrics.
5. Framer byte buffer (variable byte shifts, nl_framer.vhd:471/518/546) is most
   of its ~2.8k LUT. Alternative: stuffer emits only full beats, header padded
   to 28 bytes with 0xFF fill bytes before SOS (T.81 B.1.1.2; check decoders),
   framer becomes a mux. Costs a larger stuffer hold buffer (9 -> 32 refill
   alignments). Prototyped and rejected (D26): stuffer +642 LUT, framer
   only -357; the framer cost is the per-lane ring logic, not the byte shifts.
6. Per-image overhead (DRAIN + CLEAR = 544 cycles + lane skew) matters only for
   small tiles. Fix if needed: 1-bit generation tag per ctx entry, stale
   generation swept in the background on idle write ports.
7. decisions.md: D10/D14 still describe per-lane stuffer/framer (superseded by
   D19). D13's reason is wrong: one shared A RAM needs only 1R + 1W per cycle
   (simple dual port); keep the design, fix the text.

Rejected: single 4-component JPEG-LS file (Nf=4, ILV=0, one scan per stripe).
Compliant, but decodes to 4 planes (app still stitches), needs ~3/4 of the
compressed frame buffered to emit scans in order, and lane 0's header becomes a
single point of failure.

## Output: separate streams vs one merged stream
Facts:
- Lanes run concurrently for the whole frame; lane 3 finishes ~3W/4 cycles
  after lane 0 (constant k*Ws skew, D3). Finish timing is not the problem.
- Compressed rate per stripe depends on content. A fixed-ratio merge (one word
  per lane per round) either pads to 4x the largest stripe or deadlocks: the
  fast lane fills, iHold stops every lane, the awaited slow lane never produces.
- AXI DMA/MCDMA: TDEST must not change within a packet (PG021). Beat-level lane
  interleave into a DMA is not possible; each chunk must be its own packet.
- 5 bands x 4 lanes = 20 streams > 16 MCDMA S2MM channels (verify in PG288).

Options:
- A. Separate streams, one DMA channel each: cleanest, but 20 channels.
- B. Chunked TDEST packets into MCDMA: still 20 channels. Out.
- C. Tagged chunks on one stream: per-stream chunk buffer (2 chunks, ping-pong),
  arbiter sends any full chunk or the final partial one at EOI, each chunk gets
  a {band, lane, length} header (or is a CCSDS Space Packet with APID per
  stream). One plain DMA channel or straight to mass memory; demux on ground.
  No padding, no deadlock (a full chunk is always sendable).
- D. TDEST-addressed AXI4 writer in the IP: one write pointer per stream, writes
  each stream into its own DDR buffer. No channel limit, no descriptors. Needs
  per-stream burst coalescing (same chunk buffer as C, burst-sized).

Status (2026-10-23): open, see D24. Leaning towards one stream per band with
chunked slices and a small {slice, last, length} header; no mission IDs or
packet standards in the IP.

What the sideband does at a DMA to DDR (checked in PG021/PG288):
- MCDMA S2MM: TDEST (4 bits) picks the channel = descriptor ring = memory
  region, max 16. TID (8 bits) and TUSER (16 bits, TLAST beat) only get
  written into each BD's Sideband Status word for software to read.
- AXI DMA: plain mode ignores TDEST/TID/TUSER. Its multichannel mode is being
  discontinued (PG021 points to PG288).
- So with one ring, chunks land in BD order and software regroups them per
  stream using the TID in each BD. Contiguous per-stream buffers need TDEST
  rings (<= 16) or a client-side writer.
- Plain AXI DMA users need an in-band header (or the S2MM status stream into
  the APP fields): keep it as a generic.
Chunk size ~1 KB: 20 x 2 x 1 KB = 40 KB (~10 BRAM36, one shared RAM),
header overhead < 1%. Check aggregate rate: 5 cores x up to 4 B/cycle = 20
B/cycle worst case vs 16 B/cycle on a 128-bit port; typical NL rates are far
below.
