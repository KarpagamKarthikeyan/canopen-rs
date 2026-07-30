//! Service Data Object (SDO) protocol (CiA 301 §7.2.4).
//!
//! SDOs provide confirmed, addressed read/write access to any object
//! dictionary entry. Each transfer step is a client request frame answered
//! by a server response frame on a pair of CAN ids (default `0x600 + node`
//! for requests, `0x580 + node` for responses).
//!
//! The first milestone implements **expedited** transfer: a value of up to
//! four bytes carried inline in a single request/response exchange.
//! Segmented and block transfer follow.
//!
//! This module encodes and decodes the raw 8-byte CAN data field only;
//! choosing the CAN id and moving the frame is the transport's job.

// TODO(next milestone):
//   - `SdoCommand` / command-specifier bitfields,
//   - expedited download (write) and upload (read) encode + decode,
//   - `SdoAbortCode`,
//   validated against known-good CANopen frame byte sequences.
