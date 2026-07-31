# Interop / cross-validation

Tools that validate `canopen-rs`'s wire format against an **independent**
implementation, so correctness does not rest on our own reading of CiA 301.

## `python_canopen_oracle.py`

Drives the real encoders of [`python-canopen`] — a mature library used against
real hardware — entirely offline, and diffs the frames it produces against the
same golden byte sequences asserted in the Rust unit tests.

```bash
python3 -m pip install canopen      # pulls python-can
python3 tools/interop/python_canopen_oracle.py
```

It cross-checks, in both directions:

- **Our frames vs. python:** SDO expedited download (U32/U8/I16), upload
  request, abort, and a full segmented download (initiate + data segments with
  toggle / last / unused-byte bits); NMT node-control commands; EMCY; SYNC; the
  PDO mapping-value format; LSS (switch-global, configure-node-id, selective
  switch, store, inquire-node-id); and SDO block download (initiate / sub-block
  response / end command bytes, and the CRC-16/XMODEM against `binascii.crc_hqx`).
- **python vs. our responses:** python's client decodes our expedited upload
  response and accepts our download-response / segment-ack frames to drive a
  segmented transfer to completion.

All 30 checks currently pass. This validates the **wire format**; it does not
replace on-bus testing of the SocketCAN transport runtime (see the `vcan0`
loopback harness) or real-hardware validation.

[`python-canopen`]: https://github.com/christiansandberg/canopen
