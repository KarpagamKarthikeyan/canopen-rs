# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-07-30

Adds the node runtime and master-side tooling on top of the 0.1.0 codecs, plus
runnable documentation. Fully backwards compatible with 0.1.0.

### Added

- **`Node`** (`canopen-rs`): a sans-I/O device node bundling the object
  dictionary, SDO server, and NMT state. `on_frame` serves SDO requests and
  advances NMT (gated to the correct states); `boot`/`heartbeat` produce the
  error-control frames. Plus the `TxFrame` output type.
- **PDO exchange in `Node`**: `add_rpdo` / `add_tpdo`, RPDO frames unpacked into
  the object dictionary via `on_frame`, and transmit PDOs via `sync_tpdos`
  (SYNC-triggered) and `tpdo` (event-driven) — all gated to the operational
  state.
- **`HeartbeatMonitor`** (`canopen-host`): track per-node heartbeats and flag
  nodes that time out; cross-platform.
- **`SocketCan::send_nmt`** (`canopen-host`): send NMT node-control commands.
- Runnable doctests across the core modules (crate root, `sdo`,
  `object_dictionary`, `datatypes`, `nmt`, `pdo`, `emcy`).
- The vcan loopback example now exercises NMT and a full RPDO → SYNC → TPDO
  round-trip in addition to SDO.

## [0.1.0] - 2026-07-30

Initial release: a `no_std`-first CANopen (CiA 301) protocol stack targeting
both host (Linux/SocketCAN) and bare-metal MCU.

### Added

- **Object dictionary**: typed entries addressed by `(index, subindex)` with
  access-rights and data-type enforcement, backed by `heapless` (no allocator).
- **Data types**: the CiA 301 numeric basic types with little-endian encode/
  decode.
- **SDO**: expedited *and* segmented transfer; a sans-I/O `SdoServer` (services
  an object dictionary) and `SdoClient` (drives transactions), with standard
  abort codes.
- **NMT**: state machine, node-control command codec, heartbeat / boot-up.
- **PDO**: mapping model, TPDO `pack` / RPDO `unpack`, predefined-connection-set
  COB-IDs, transmission types.
- **SYNC** and **EMCY** codecs.
- **Transport**: an `embedded-can` frame bridge (core) and a Linux SocketCAN
  transport with one-call `sdo_read` / `sdo_write` (host).
- **EDS/DCF parser** (host): build an object dictionary from a device file.
- Wire format cross-checked byte-for-byte against `python-canopen`; CI with a
  virtual-CAN on-bus loopback.

[0.2.0]: https://github.com/KarpagamKarthikeyan/canopen-rs/releases/tag/v0.2.0
[0.1.0]: https://github.com/KarpagamKarthikeyan/canopen-rs/releases/tag/v0.1.0
