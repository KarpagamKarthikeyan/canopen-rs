//! # canopen-host
//!
//! Host-side (`std`) transport and tooling for [`canopen_rs`]: a Linux
//! SocketCAN transport and EDS/DCF file parsing built on top of the
//! `no_std` core.
//!
//! The core protocol logic lives in [`canopen_rs`] and is re-exported here
//! for convenience.

pub use canopen_rs;

/// Electronic Data Sheet (EDS) / Device Configuration File (DCF) parsing.
///
/// Cross-platform: EDS files are plain text and carry no OS dependency.
pub mod eds;

/// Generate a compile-time object dictionary from an EDS/DCF file (for `build.rs`).
///
/// Cross-platform.
pub mod codegen;

/// NMT master tooling: heartbeat monitoring and node health.
///
/// Cross-platform; sending NMT commands rides on the Linux `transport` module.
pub mod nmt;

/// Linux SocketCAN transport (compiled only on `target_os = "linux"`).
#[cfg(target_os = "linux")]
pub mod transport;

/// Async (tokio) SocketCAN transport (Linux, `tokio` feature).
#[cfg(all(target_os = "linux", feature = "tokio"))]
pub mod async_transport;
