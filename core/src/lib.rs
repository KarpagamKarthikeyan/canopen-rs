//! # canopen-rs
//!
//! A `no_std`-first [CANopen] (CiA 301) protocol stack in Rust.
//!
//! This core crate is transport-agnostic and allocation-free: it models the
//! object dictionary, encodes/decodes SDO and PDO messages, and drives the
//! NMT state machine. It is designed to run unchanged on a bare-metal MCU
//! node and on a host (Linux/SocketCAN) via the companion `canopen-host`
//! crate.
//!
//! CAN frames are represented through the [`embedded_can`] traits, so any
//! controller or socket that implements them can carry CANopen traffic.
//!
//! ## Status
//!
//! Early development. The API will change. See the roadmap in the workspace
//! `README.md`.
//!
//! [CANopen]: https://www.can-cia.org/canopen/
#![no_std]
#![deny(unsafe_code)]
#![warn(missing_debug_implementations)]
// Enable once the public API stabilises:
// #![warn(missing_docs)]

pub mod datatypes;
pub mod emcy;
pub mod nmt;
pub mod object_dictionary;
pub mod pdo;
pub mod sdo;
pub mod sync;
pub mod transport;
pub mod types;

pub use datatypes::{DataType, Value};
pub use emcy::{EmergencyMessage, ErrorRegister};
pub use nmt::{NmtCommand, NmtState, NmtStateMachine};
pub use object_dictionary::{AccessType, Address, Entry, ObjectDictionary};
pub use pdo::{MappingEntry, PdoKind, PdoMapping, TransmissionType};
pub use sdo::{SdoClient, SdoEvent, SdoServer, Segment, SegmentReader, SegmentWriter};
pub use sync::SyncCounter;
pub use types::{Error, NodeId, Result};
