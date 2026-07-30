# canopen-rs

[![crates.io](https://img.shields.io/crates/v/canopen-rs.svg)](https://crates.io/crates/canopen-rs)
[![docs.rs](https://docs.rs/canopen-rs/badge.svg)](https://docs.rs/canopen-rs)
[![license](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](#license)

A **`no_std`-first [CANopen] (CiA 301) protocol stack in Rust**, built to run
unchanged on a bare-metal microcontroller node *and* on a host
(Linux/SocketCAN).

> **Status: early development (0.1).** The API will change before 1.0, but the
> object dictionary, SDO client/server, PDO, NMT, SYNC, EMCY, and EDS parsing
> are implemented and tested.

## Why

The Rust CANopen landscape has several partial attempts but no clear winner —
and none delivers a stack that works on both embedded and host targets. That
dual target is exactly what people keep asking for. `canopen-rs` closes that
gap with a transport-agnostic, allocation-free core and a thin host layer on
top.

## What works today

| Area | Status |
|------|--------|
| Object dictionary (typed entries, access control) | ✅ |
| Data types + little-endian value codec | ✅ |
| SDO — expedited **and** segmented transfer | ✅ |
| SDO **server** (services the OD) and **client** (drives transactions) | ✅ |
| NMT state machine, node-control, heartbeat / boot-up | ✅ |
| PDO mapping, TPDO pack / RPDO unpack, connection-set COB-IDs | ✅ |
| SYNC (producer counter) and EMCY (emergency + error register) | ✅ |
| `embedded-can` bridge + Linux SocketCAN transport | ✅ |
| EDS / DCF file parsing → object dictionary | ✅ |
| SDO block transfer, LSS, heartbeat-consumer timeout | planned |

Everything in the core is `#![no_std]`, `#![deny(unsafe_code)]`, and builds for
`thumbv7em-none-eabihf`; SDO frame codecs are validated against known-good byte
sequences, and the client/server are exercised end-to-end.

## Quickstart

### Serve an object dictionary (a device node)

```rust
use canopen_rs::{Address, Entry, NodeId, ObjectDictionary, Value};
use canopen_rs::sdo::SdoServer;

// Build the node's object dictionary.
let mut od = ObjectDictionary::<16>::new();
od.insert(Address::new(0x1000, 0), Entry::constant(Value::Unsigned32(0x0004_0192)))?;
od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000)))?;

let mut server = SdoServer::new(NodeId::new(0x10)?);

// In your CAN receive loop, for each SDO request frame addressed to this node:
if let Some(response) = server.handle(&mut od, &request) {
    // send `response` on `server.response_cob_id()`
}
```

The server picks expedited or segmented transfer automatically and answers with
the correct standard SDO abort codes on error.

### Talk to a device (a master), over SocketCAN

```rust
use std::time::Duration;
use canopen_host::transport::SocketCan;
use canopen_rs::{Address, DataType, NodeId, Value};

let bus = SocketCan::open("can0")?;                 // or "vcan0" for a virtual bus
bus.set_read_timeout(Duration::from_millis(500))?;
let node = NodeId::new(0x10)?;

// Read object 0x1000 (device type). Expedited or segmented is handled for you.
let device_type = bus.sdo_read(node, Address::new(0x1000, 0), DataType::Unsigned32)?;

// Write object 0x1017 (producer heartbeat time).
bus.sdo_write(node, Address::new(0x1017, 0), Value::Unsigned16(1000))?;
```

On a microcontroller, use the `canopen_rs::transport::{frame_from, cob_id}`
helpers with any HAL that implements the [`embedded-can`] traits instead of
SocketCAN — the client and server code is identical.

### Load a device's EDS file into an object dictionary

```rust
use canopen_host::eds::Eds;

let eds = Eds::from_file("device.eds")?;
println!("{:?}: {} objects", eds.vendor_name, eds.objects.len());
let od = eds.object_dictionary::<256>()?;   // the same OD type a node runs
```

## Design

- **`core` (`canopen-rs`)** — `no_std`, allocation-free, transport-agnostic.
  The object dictionary, the message codecs, the NMT state machine, and the SDO
  server/client. The server and client are **sans-I/O**: they consume and
  produce frames but never touch a bus, so the same logic runs on host and MCU.
  CAN frames flow through the [`embedded-can`] traits.
- **`host` (`canopen-host`)** — `std` layer on the core: a Linux SocketCAN
  transport with one-call `sdo_read` / `sdo_write`, and EDS/DCF parsing.
  SocketCAN is gated to Linux; EDS parsing builds everywhere.

```
canopen-rs/
├── core/   # no_std core protocol stack  (crate: canopen-rs)
└── host/   # std host transport + EDS     (crate: canopen-host)
```

## References studied (not copied)

- The Python [`canopen`] library — the most complete reference for the object
  model, SDO/PDO semantics, and EDS handling (host-only, BSD-licensed).
- [`zencan`] — the most advanced Rust prior art.
- The **CiA 301** specification as the authoritative source of truth.

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE) at
your option. Unless you explicitly state otherwise, any contribution you submit
for inclusion shall be dual-licensed as above, without additional terms.

[CANopen]: https://www.can-cia.org/canopen/
[`embedded-can`]: https://docs.rs/embedded-can
[`canopen`]: https://github.com/christiansandberg/canopen
[`zencan`]: https://github.com/mcbridejc/zencan
