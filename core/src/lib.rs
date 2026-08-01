//! # canopen-rs
//!
//! A `no_std`-first [CANopen] (CiA 301) protocol stack in Rust.
//!
//! This core crate is transport-agnostic and allocation-free. It is designed to
//! run unchanged on a bare-metal MCU node and on a host (Linux/SocketCAN) via
//! the companion `canopen-host` crate.
//!
//! CAN frames are represented through the [`embedded_can`] traits, so any
//! controller or socket that implements them can carry CANopen traffic.
//!
//! # What it implements
//!
//! - **[Object dictionary](object_dictionary)** — typed entries addressed by
//!   `(index, subindex)`, with access control, backed by `heapless` (no
//!   allocator). The numeric basic types plus the variable-length ones
//!   (`VISIBLE_STRING` / `OCTET_STRING` / `DOMAIN`, see [`ByteString`]).
//!   [`StandardObjects`] builds the mandatory CiA 301 communication objects in
//!   one call.
//! - **[SDO](sdo)** — expedited, segmented, and block-transfer codecs, with a
//!   sans-I/O [`SdoServer`] (serves a dictionary) and [`SdoClient`] (drives
//!   transactions).
//! - **[PDO](pdo)** — mapping model, TPDO pack / RPDO unpack, the predefined
//!   connection set, and transmission types.
//! - **[NMT](nmt)** — the state machine, node-control, heartbeat / boot-up, and
//!   node guarding.
//! - **[SYNC](sync)**, **[EMCY](emcy)** (emergency + error register), and
//!   **[TIME](time)**.
//! - **[LSS](lss)** (CiA 305) — node-id assignment, selective switch, and
//!   [`Fastscan`](lss::FastscanMaster) discovery of an unknown node.
//! - **[`Node`](node)** — bundles all of the above into one frame-driven device
//!   runtime: SDO service, NMT, PDO exchange, EMCY production, SYNC production,
//!   and LSS (including coming up unconfigured for Fastscan).
//!
//! # Example: an SDO read, transport-free
//!
//! A device exposes an [`ObjectDictionary`]; a [client](sdo::SdoClient) reads it
//! from a [server](sdo::SdoServer) one frame at a time — no bus required, so the
//! same logic runs on a host or an MCU.
//!
//! ```
//! use canopen_rs::sdo::{SdoClient, SdoEvent, SdoServer};
//! use canopen_rs::{Address, DataType, Entry, NodeId, ObjectDictionary, Value};
//!
//! let node = NodeId::new(0x10).unwrap();
//!
//! // The device's object dictionary, served by an SDO server.
//! let mut od = ObjectDictionary::<8>::new();
//! od.insert(Address::new(0x1000, 0), Entry::constant(Value::Unsigned32(0x1234))).unwrap();
//! let mut server = SdoServer::new(node);
//!
//! // The client drives the transaction; expedited vs segmented is automatic.
//! let mut client = SdoClient::new(node);
//! let mut frame = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
//! let value = loop {
//!     let response = server.handle(&mut od, &frame).expect("server replies");
//!     match client.on_response(&response) {
//!         SdoEvent::Send(next) => frame = next,
//!         SdoEvent::Complete(v) => break v.unwrap(),
//!         SdoEvent::Aborted(code) => panic!("aborted {code:#010x}"),
//!     }
//! };
//! assert_eq!(value, Value::Unsigned32(0x1234));
//! ```
//!
//! ## Status
//!
//! Early development (0.x): the classic CiA 301 communication objects and LSS
//! are implemented and tested — cross-checked byte-for-byte against
//! python-canopen and exercised on a virtual CAN bus — but the API will change
//! before 1.0. Not yet wired up: SDO block transfer through the [`Node`] (the
//! codecs exist), sub-byte PDO packing, and the device profiles (CiA 401/402).
//! See the roadmap in the workspace `README.md`.
//!
//! [CANopen]: https://www.can-cia.org/canopen/
#![no_std]
#![deny(unsafe_code)]
#![warn(missing_debug_implementations)]
// Enable once the public API stabilises:
// #![warn(missing_docs)]

pub mod datatypes;
pub mod emcy;
pub mod lss;
pub mod nmt;
pub mod node;
pub mod object_dictionary;
pub mod pdo;
pub mod sdo;
pub mod standard;
pub mod sync;
pub mod time;
pub mod transport;
pub mod types;

pub use datatypes::{ByteString, DataType, Value, MAX_STRING_LEN};
pub use emcy::{EmergencyMessage, ErrorRegister};
pub use lss::{LssAddress, LssSlave, LssState};
pub use nmt::{NmtCommand, NmtState, NmtStateMachine};
pub use node::{Node, TxFrame};
pub use object_dictionary::{AccessType, Address, Entry, ObjectDictionary};
pub use pdo::{MappingEntry, PdoKind, PdoMapping, TransmissionType};
pub use sdo::{SdoClient, SdoEvent, SdoServer, Segment, SegmentReader, SegmentWriter};
pub use standard::StandardObjects;
pub use sync::SyncCounter;
pub use time::TimeOfDay;
pub use types::{Error, NodeId, Result};
