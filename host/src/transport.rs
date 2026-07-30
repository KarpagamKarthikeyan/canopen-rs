//! Linux SocketCAN transport for `canopen-rs`.
//!
//! Bridges the CANopen core codecs to a Linux SocketCAN interface. The
//! [`socketcan`] crate implements the [`embedded_can`] traits, so the same
//! frame types the core speaks flow straight onto the bus.
//!
//! [`socketcan`]: https://docs.rs/socketcan

// TODO(milestone: host transport): open a named CAN interface, send/receive
// `embedded_can::Frame`s, and offer blocking + non-blocking helpers for the
// SDO client and NMT master.
