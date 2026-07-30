# canopen-rs

A **`no_std`-first [CANopen] (CiA 301) protocol stack in Rust**, designed to
run unchanged on a bare-metal microcontroller node *and* on a host
(Linux/SocketCAN).

> **Status: early development.** APIs will change. Not yet published.

## Why

The existing Rust CANopen landscape has several independent, partial attempts
but no clear winner — and none delivers a full-spec stack that works on both
embedded and host targets. That dual target is exactly what people keep asking
for. `canopen-rs` aims to close that gap.

## Design

- **`core` (`canopen-rs`)** — `no_std`, allocation-free, transport-agnostic.
  Object dictionary model, SDO/PDO encode-decode, NMT state machine. CAN
  frames flow through the [`embedded-can`] traits, so any controller or socket
  implementing them can carry traffic.
- **`host` (`canopen-host`)** — `std` tooling built on the core: a Linux
  SocketCAN transport and EDS/DCF parsing. SocketCAN is gated to Linux; the
  rest (e.g. EDS parsing) builds everywhere.

```
canopen-rs/
├── core/   # no_std core protocol stack  (crate: canopen-rs)
└── host/   # std host transport + EDS     (crate: canopen-host)
```

## Roadmap

1. **Object dictionary + SDO expedited transfer** — typed entries by
   `(index, subindex)`; read/write a single value, tested against known-good
   CANopen frame byte sequences.
2. **NMT** — node state machine + heartbeat.
3. **PDO** — mapping and RPDO/TPDO transmission.
4. **EDS parsing**, then **SYNC / EMCY / LSS**.

## References studied (not copied)

- The Python [`canopen`] library — the most complete reference for the object
  model, SDO/PDO semantics, and EDS handling (host-only, BSD-licensed).
- [`zencan`] — the most advanced Rust prior art.
- The **CiA 301** specification as the authoritative source of truth.

## License

Licensed under either of [MIT](LICENSE-MIT) or Apache-2.0 (`LICENSE-APACHE`,
TODO) at your option.

[CANopen]: https://www.can-cia.org/canopen/
[`embedded-can`]: https://docs.rs/embedded-can
[`canopen`]: https://github.com/christiansandberg/canopen
[`zencan`]: https://github.com/mcbridejc/zencan
