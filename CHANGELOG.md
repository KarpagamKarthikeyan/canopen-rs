# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Async SocketCAN transport** (`canopen-host`, `async_transport`, behind the
  `tokio` feature): `AsyncSocketCan` mirrors the blocking `SocketCan` with
  `async` `send`/`recv`/`sdo_read`/`sdo_write`/`send_nmt`, built on socketcan's
  tokio integration, for use in async host applications.

## [0.4.0] - 2026-07-31

### Added

- **`canopen-cli`** — an installable command-line tool (binary `canopen`):
  inspect EDS/DCF files and generate a compile-time object dictionary anywhere,
  and read/write objects over SDO, send NMT commands, and monitor bus traffic on
  a Linux SocketCAN interface.
- **EDS → object-dictionary codegen** (`canopen-host`, `codegen`):
  `generate_object_dictionary` emits Rust source for a typed `ObjectDictionary`
  from a parsed EDS/DCF — run from a `build.rs` (or `canopen codegen`) so a
  device file becomes a compile-time, zero-runtime-parse OD.
- **`Node::configure_pdos_from_od`** (`canopen-rs`): (re)build the node's PDO
  configuration from the PDO parameter objects in the object dictionary
  (`0x1400`/`0x1600`/`0x1800`/`0x1A00`) — the standard way a master configures
  PDOs over SDO. Honours the COB-ID validity bit.
- **Object-dictionary write notification**: `Node::take_written_object` /
  `SdoServer::take_write` report the object a master most recently wrote over
  SDO, so a node can react to configuration writes (e.g. re-read PDOs, restart a
  timer) — the sans-I/O equivalent of a write callback.
- **TIME object** (`canopen-rs`, `time`): `TimeOfDay` producer/consumer codec
  for the network time-of-day on COB-ID `0x100`.
- **Node guarding** (CiA 301 §7.3.1): `nmt::encode_node_guard` /
  `decode_node_guard` and `Node::node_guard_response` — the legacy RTR-based
  error-control alternative to heartbeat, with the alternating toggle bit.

With these, the classic CiA 301 communication objects are complete: object
dictionary, SDO, PDO, NMT (heartbeat + node guarding), SYNC, EMCY, TIME, and LSS.

## [0.3.1] - 2026-07-31

Documentation only — no code changes. Published so crates.io reflects the
current README (a published version's README cannot be edited in place).

### Changed

- README: corrected the status line (0.1 → 0.3) and interop count (21/21 →
  33/33), marked block transfer and LSS as done, and added a Contributing
  section. `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, and issue/PR templates are
  now part of the repository.

## [0.3.0] - 2026-07-30

Completes the CANopen lifecycle: node-id assignment over the bus, the last SDO
transfer mode, and full on-bus verification. Backwards compatible with 0.2.x.

### Added

- **LSS (Layer Setting Services, CiA 305)** (`canopen-rs`): an `LssSlave` state
  machine (switch global / selective, configure node-id, inquire identity,
  store) plus master-side codecs — assign a node-id to an unconfigured node
  over the bus.
- **LSS in `Node`**: `enable_lss`, `apply_lss_node_id`, `set_node_id` — a node
  can come up unconfigured, answer only LSS, then become a full node once a
  master assigns its id (the SDO COB-IDs relocate with it).
- **SDO block transfer** (`canopen-rs`, `sdo::block`): both directions
  (download + upload), CRC-16/XMODEM, and `BlockWriter` / `BlockReceiver`
  helpers.
- A full-lifecycle integration test and an expanded `vcan0` example that walks
  LSS → SDO → NMT → PDO → block transfer over a real bus.

### Validated

- The complete stack **runs and passes on-bus over `vcan0`**.
- Wire format cross-checked against python-canopen (33/33 frames, now including
  LSS and block transfer).

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

[0.4.0]: https://github.com/KarpagamKarthikeyan/canopen-rs/releases/tag/v0.4.0
[0.3.1]: https://github.com/KarpagamKarthikeyan/canopen-rs/releases/tag/v0.3.1
[0.3.0]: https://github.com/KarpagamKarthikeyan/canopen-rs/releases/tag/v0.3.0
[0.2.0]: https://github.com/KarpagamKarthikeyan/canopen-rs/releases/tag/v0.2.0
[0.1.0]: https://github.com/KarpagamKarthikeyan/canopen-rs/releases/tag/v0.1.0
