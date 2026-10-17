# Verification guidelines

Rules for every OSVVM TB in [`Verification/OSVVM`](../Verification/OSVVM).
A rule that doesn't apply is marked N/A in the TB header.

## G1. Requirements and golden model
- Every requirement in [`Requirements.md`](Requirements.md) implemented in `Sources/` has a `T87.<id>` requirement with a goal in an OSVVM TB.
- NL-only behaviour (lanes, stripes, forwarding, runtime NEAR) uses `NL.<Name>` IDs.
- Golden = sequential VHDL transcription of the T.87 C code: integers, same statements, order and names, no RTL packages.
- Requirements without C code (e.g. A.4.2): check properties (range, one-to-one, totality).
- Non-spec RTL behaviour (clamps, saturation) gets its own `NL.*` ID, never counted as T.87.

## G2. Overflow / underflow
- Every arithmetic signal/variable: legal range commented at its declaration, for worst-case generics.
- RTL asserts that range (`assert` for variables, PSL `never` for signals).
- TB drives each node to its min and max, as named coverage bins.
- Accumulators (A, B, C, N, RUNindex) driven to their clamp/reset point and one step past.
- Read outputs with `checked_integer` / `checked_bit` (X/U never passes).

## G3. Type conversions
- unsigned → signed: `signed('0' & x)`, never `signed(x)`.
- Widen with `resize`; narrow only with a proven range and an assert.
- Stimulus includes source MSB set and most negative value, as coverage bins.

## G4. Corner cases
Each one is a named coverage bin; TBs close on `IsCovered`.
- Edges, per stripe (D1): first line (Rb = Rc = Rd = 0), first column (Ra = Rb), last column (Rd = Rb), 1-line image, Ws = 8.
- Run mode: run ending at end of line, RUNindex 0 and 31, both RItype values.
- Coding: k = 0 and max, escape code, A.11 special case, N = RESET, C at MIN_C/MAX_C.
- Config: BITNESS min/max, NEAR = 0 and max, NEAR change between images.
- Lanes: every lane hits every bin, forwarding hit/miss, back-to-back images, backpressure.

## G5. Reset recovery (stateful modules)
- Reset from any state returns the module to its initial condition: outputs, counters, FSMs, context memories.
- Reset is asserted in every FSM state / pipeline phase, as coverage bins.
- After reset, the next transaction matches a golden run from power-up.
- Output of the aborted transaction is ignored, never pushed to the scoreboard.

## G6. Done
- Zero errors, all requirement goals met, all coverage `IsCovered`.
- Code coverage reviewed; uncovered lines covered or justified in the TB header.
