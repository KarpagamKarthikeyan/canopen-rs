# canopen-rs

[![CI](https://github.com/KarpagamKarthikeyan/canopen-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/KarpagamKarthikeyan/canopen-rs/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/canopen-rs.svg)](https://crates.io/crates/canopen-rs)
[![docs.rs](https://docs.rs/canopen-rs/badge.svg)](https://docs.rs/canopen-rs)
[![license](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](#license)

A **`no_std`-first [CANopen] (CiA 301) protocol stack in Rust**, built to run
unchanged on a bare-metal microcontroller node *and* on a host
(Linux/SocketCAN).

> **Status: early development (0.3).** The API will change before 1.0, but the
> object dictionary, SDO client/server (expedited, segmented, block), PDO, NMT,
> SYNC, EMCY, LSS, and EDS parsing are implemented and tested — the whole stack
> is verified running on a bus.

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
| NMT state machine, node-control, heartbeat + node guarding | ✅ |
| PDO mapping, TPDO pack / RPDO unpack, connection-set COB-IDs | ✅ |
| SYNC, EMCY (emergency + error register), and TIME | ✅ |
| `Node` runtime (OD + SDO + NMT + PDO exchange) | ✅ |
| LSS (node-id assignment, incl. in `Node`) | ✅ |
| SDO block transfer (download + upload, CRC-16) | ✅ |
| `embedded-can` bridge + Linux SocketCAN transport | ✅ |
| EDS / DCF file parsing → object dictionary | ✅ |
| Host NMT master: `send_nmt` + heartbeat monitor | ✅ |
| String / `DOMAIN` value types; block transfer in the SDO client/server | planned |

Everything in the core is `#![no_std]`, `#![deny(unsafe_code)]`, and builds for
`thumbv7em-none-eabihf`; SDO frame codecs are validated against known-good byte
sequences, and the client/server are exercised end-to-end.

## Install

```toml
[dependencies]
canopen-rs = "0.1"      # no_std core: OD, SDO, PDO, NMT, SYNC, EMCY
canopen-host = "0.1"    # std host layer: SocketCAN transport + EDS parsing
```

The core is `no_std` by default; enable `std` for `std::error::Error` impls
(`canopen-rs = { version = "0.1", features = ["std"] }`). On a microcontroller,
depend only on `canopen-rs` and drive it with your HAL's [`embedded-can`]
implementation — no host crate needed. In `canopen-host`, the SocketCAN
transport is compiled only on Linux; EDS parsing builds everywhere. **MSRV:
Rust 1.75.**

## Command-line tool

```bash
cargo install canopen-cli    # installs the `canopen` binary

canopen eds device.eds                       # inspect an EDS/DCF file (any OS)
canopen codegen device.eds > src/od.rs       # EDS -> compile-time ObjectDictionary
canopen read  can0 0x10 1017 u16             # SDO read  (Linux SocketCAN)
canopen write can0 0x10 1017 u16 1000        # SDO write
canopen nmt   can0 start 0x10                # send an NMT command
canopen monitor can0                         # decode and print bus traffic
```

## Quickstart

### Serve an object dictionary (a device node)

```rust
use canopen_rs::{Address, Entry, NodeId, ObjectDictionary, Value};
use canopen_rs::node::Node;

// Build the node's object dictionary and wrap it in a Node.
let mut od = ObjectDictionary::<16>::new();
od.insert(Address::new(0x1000, 0), Entry::constant(Value::Unsigned32(0x0004_0192)))?;
od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000)))?;
let mut node = Node::new(NodeId::new(0x10)?, od);

let boot = node.boot();                 // enter pre-operational, announce boot-up
// send `boot` on the bus, then in your receive loop:
if let Some(tx) = node.on_frame(cob_id, &data) {
    // send `tx.data()` on `tx.cob_id`
}
```

`Node` bundles the object dictionary, SDO server, and NMT state: `on_frame`
serves SDO requests (expedited or segmented, with standard abort codes),
advances NMT from node-control commands, and applies received PDOs. Add PDOs
with `node.add_rpdo`/`add_tpdo`, and get SYNC-triggered transmit frames from
`node.sync_tpdos()`. It's sans-I/O — the same code runs on a host or an MCU.

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

## Testing & validation

```bash
cargo test --workspace                                     # unit + integration + doc tests
cargo build -p canopen-rs --target thumbv7em-none-eabihf   # confirm the core stays no_std
cargo clippy --workspace --all-targets
```

- **Independent wire-format cross-check.** The frame codecs are validated
  against [`python-canopen`], a mature implementation used against real
  hardware — every SDO / NMT / EMCY / SYNC / PDO / LSS / block-transfer frame
  matches byte-for-byte (33/33). It runs offline:

  ```bash
  python3 -m pip install canopen
  python3 tools/interop/python_canopen_oracle.py
  ```

- **On-bus loopback (Linux, no hardware).** Walk the whole stack over a virtual
  CAN interface — LSS node-id assignment, SDO (expedited + segmented), NMT, a
  SYNC-triggered PDO exchange, and a block transfer — exercising the SocketCAN
  transport on a real bus:

  ```bash
  sudo tools/vcan_setup.sh                               # bring up vcan0
  cargo run -p canopen-host --example vcan_loopback
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

## Contributing

Contributions are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md) for the
workflow (DCO sign-off, local checks, and the `no_std` core vs `host` split).
Good entry points are issues labelled
[`good first issue`](https://github.com/KarpagamKarthikeyan/canopen-rs/labels/good%20first%20issue).
Questions and ideas are welcome in
[Discussions](https://github.com/KarpagamKarthikeyan/canopen-rs/discussions).
Participation is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE) at
your option. Unless you explicitly state otherwise, any contribution you submit
for inclusion shall be dual-licensed as above, without additional terms.

[CANopen]: https://www.can-cia.org/canopen/
[`embedded-can`]: https://docs.rs/embedded-can
[`canopen`]: https://github.com/christiansandberg/canopen
[`zencan`]: https://github.com/mcbridejc/zencan
