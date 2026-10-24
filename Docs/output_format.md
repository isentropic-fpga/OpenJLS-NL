# nl_top output format (version 1)

One nl_top codes one band as 4 vertical slices, each a complete JPEG-LS file
(`.jls`, SOI to EOI). The 4 slices leave on one AXI-Stream as chunks, each
after an 8-byte header. Reference unpacker: [`Scripts/nl_unpack.py`](../Scripts/nl_unpack.py).
Rationale: D24 and D27 in [decisions.md](decisions.md).

## Byte stream

```
header | payload (length bytes) | header | payload | ...
```

- Bytes in stream order are the `oKeep` bytes of each beat, MSB first.
- Payload bytes of one slice, concatenated in stream order, are that slice's
  `.jls` for one image.
- Chunks of different slices and of consecutive images interleave. Chunks of
  one slice are in order.
- Every chunk, header included, is `CHUNK_BYTES` long except the last of a
  slice's image (header + 1 to `CHUNK_BYTES - 8` bytes).

## Header

| Byte | Field | Value |
|---|---|---|
| 0-1 | marker | `FF B0` |
| 2 | bits 7-4: version | 1 |
|   | bits 3-2: slice | 0-3, left to right |
|   | bit 1: last | 1 on the last chunk of the slice's image |
|   | bit 0: reserved | 0 |
| 3 | band | generic `BAND_ID` |
| 4-5 | image_count | big endian; 0 after reset, +1 per image, wraps at 65536 |
| 6-7 | length | big endian; payload bytes after this header, 1 to `CHUNK_BYTES - 8` |

- `FF B0` is a reserved JPEG marker (T.81 RES range). It never occurs in the
  payload: coded data stuffs a 0 bit after every `FF`, and the marker segments
  nl_top writes only put bytes below `0x40` or fixed marker bytes after `FF`.
- It can occur inside a header (band, image_count). To resync, take an
  `FF B0` only if the version and reserved bit match, length is not 0, and
  another valid header (or the end of the data) follows at +8+length.

## AXI-Stream

- `oLast` ends every chunk, so each chunk is one packet.
- `oKeep` is all ones except on the `oLast` beat, where it is contiguous from
  the MSB (AXI DMA S2MM accepts this).
- No `TID`, `TDEST` or `TUSER`: everything is in the header.

## Generics

| Generic | Default | |
|---|---|---|
| `OUT_WIDTH` | 64 | 64 only; the header is one beat (D24) |
| `CHUNK_BYTES` | 1024 | header included; multiple of 8, 16 to 65528 |
| `BAND_ID` | 0 | written to every header; set per instance |

Buffer: 8 chunk payloads (2 per slice) of `CHUNK_BYTES - 8`, one RAM.

## Not in the format

Packet standards (CCSDS, PUS), mission IDs, timestamps and link CRC belong to
the client (D27). A lost middle chunk is not visible in the headers; the link
layer's sequence count or CRC catches it, and the slice will not decode.
