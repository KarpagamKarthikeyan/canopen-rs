//! Service Data Object (SDO) protocol (CiA 301 §7.2.4).
//!
//! SDOs provide confirmed, addressed read/write access to any object
//! dictionary entry. Each transfer step is a client request frame answered by
//! a server response frame, exchanged on a pair of CAN ids (by default
//! `0x600 + node` for requests and `0x580 + node` for responses).
//!
//! This module implements **expedited** and **segmented** transfer: a value of
//! one to four bytes carried inline in a single exchange, or a larger value
//! split across a run of segment frames. Block transfer follows.
//!
//! The free functions here encode and decode the raw 8-byte CAN *data field*.
//! On top of them, [`SdoServer`] services requests against an
//! [`ObjectDictionary`](crate::object_dictionary::ObjectDictionary) and
//! [`SdoClient`] drives read/write transactions — both as sans-I/O state
//! machines that consume and produce frames without touching a bus, so the
//! transport (see [`request_cob_id`]/[`response_cob_id`]) stays a separate
//! concern.
//!
//! # Example: encode an expedited download
//!
//! Writing `UNSIGNED32 0x1234_5678` to object `0x2000` is a single 8-byte
//! request frame (see [`SdoClient`] / [`SdoServer`] to drive a whole exchange):
//!
//! ```
//! use canopen_rs::sdo::encode_download_expedited;
//! use canopen_rs::{Address, Value};
//!
//! let frame = encode_download_expedited(
//!     Address::new(0x2000, 0),
//!     &Value::Unsigned32(0x1234_5678),
//! )
//! .unwrap();
//! // cmd | index (LE) | subindex | value (LE)
//! assert_eq!(frame, [0x23, 0x00, 0x20, 0x00, 0x78, 0x56, 0x34, 0x12]);
//! ```

use heapless::Vec;

use crate::datatypes::{DataType, Value};
use crate::object_dictionary::Address;
use crate::types::NodeId;
use crate::{Error, Result};

pub mod block;
pub mod client;
pub mod server;

pub use client::{SdoClient, SdoEvent};
pub use server::SdoServer;

/// COB-ID base for SDO client→server (request) frames: `0x600 + node id`.
pub const SDO_REQUEST_COB_BASE: u16 = 0x600;
/// COB-ID base for SDO server→client (response) frames: `0x580 + node id`.
pub const SDO_RESPONSE_COB_BASE: u16 = 0x580;

/// The 8-byte SDO payload carried in a CAN frame's data field.
pub type SdoPayload = [u8; 8];

/// The COB-ID of the SDO request channel (client → server) for `node`.
pub const fn request_cob_id(node: NodeId) -> u16 {
    SDO_REQUEST_COB_BASE + node.raw() as u16
}

/// The COB-ID of the SDO response channel (server → client) for `node`.
pub const fn response_cob_id(node: NodeId) -> u16 {
    SDO_RESPONSE_COB_BASE + node.raw() as u16
}

// --- Command specifiers (top three bits of byte 0) -------------------------
const CCS_DOWNLOAD_SEGMENT: u8 = 0x00; // client: 000xxxxx
const CCS_DOWNLOAD_INITIATE: u8 = 0x20; // client: 001xxxxx
const CCS_UPLOAD_INITIATE: u8 = 0x40; // client: 010xxxxx
const CCS_UPLOAD_SEGMENT: u8 = 0x60; // client: 011xxxxx
                                     // The server's upload-segment specifier (scs 000) equals CCS_DOWNLOAD_SEGMENT,
                                     // which is why one data-segment codec serves both directions.
const SCS_DOWNLOAD_SEGMENT: u8 = 0x20; // server: 001xxxxx
const SCS_UPLOAD_INITIATE: u8 = 0x40; // server: 010xxxxx
const SCS_DOWNLOAD_INITIATE: u8 = 0x60; // server: 011xxxxx
const CS_ABORT: u8 = 0x80; // either:  100xxxxx

// Top-three-bit command-specifier mask.
const CS_MASK: u8 = 0xE0;

// Low-byte flag bits.
const EXPEDITED: u8 = 0x02; // 'e' in an initiate frame
const SIZE_INDICATED: u8 = 0x01; // 's' in an initiate frame
const EXPEDITED_SIZED: u8 = EXPEDITED | SIZE_INDICATED; // expedited + size (0x03)
const TOGGLE: u8 = 0x10; // 't' in a segment frame
const NO_MORE_SEGMENTS: u8 = 0x01; // 'c' in a data-segment frame (last segment)

/// Maximum data bytes carried by a single SDO segment frame.
pub const SEGMENT_DATA_MAX: usize = 7;

/// SDO abort codes (CiA 301 §7.2.4.3.17). The value is the 32-bit code sent
/// little-endian in bytes 4..8 of an abort frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SdoAbortCode {
    /// Toggle bit not alternated.
    ToggleBitNotAlternated = 0x0503_0000,
    /// SDO protocol timed out.
    ProtocolTimedOut = 0x0504_0000,
    /// Client/server command specifier not valid or unknown.
    CommandInvalid = 0x0504_0001,
    /// Unsupported access to an object.
    UnsupportedAccess = 0x0601_0000,
    /// Attempt to read a write-only object.
    ReadOfWriteOnly = 0x0601_0001,
    /// Attempt to write a read-only object.
    WriteOfReadOnly = 0x0601_0002,
    /// Object does not exist in the object dictionary.
    ObjectDoesNotExist = 0x0602_0000,
    /// Data type does not match; length of service parameter too high.
    DataTypeMismatchLengthHigh = 0x0607_0012,
    /// Data type does not match; length of service parameter too low.
    DataTypeMismatchLengthLow = 0x0607_0013,
    /// Sub-index does not exist.
    SubIndexDoesNotExist = 0x0609_0011,
    /// General error.
    General = 0x0800_0000,
}

/// Encode an expedited SDO **download** (write) request writing `value` to
/// `addr`.
///
/// Returns [`Error::UnsupportedTransfer`] for values larger than four bytes,
/// which require segmented transfer.
pub fn encode_download_expedited(addr: Address, value: &Value) -> Result<SdoPayload> {
    let len = value.size();
    if len == 0 || len > 4 {
        return Err(Error::UnsupportedTransfer);
    }
    let mut p = [0u8; 8];
    // n = number of *unused* data bytes = 4 - len.
    p[0] = CCS_DOWNLOAD_INITIATE | (((4 - len) as u8) << 2) | EXPEDITED_SIZED;
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
    value.encode_le(&mut p[4..4 + len])?;
    Ok(p)
}

/// Encode the server's **download response** (write confirmation) for `addr`.
pub fn encode_download_response(addr: Address) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = SCS_DOWNLOAD_INITIATE;
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
    p
}

/// Decode a server download response, returning the confirmed address.
pub fn decode_download_response(p: &SdoPayload) -> Result<Address> {
    if p[0] != SCS_DOWNLOAD_INITIATE {
        return Err(Error::UnexpectedCommand);
    }
    Ok(Address::new(u16::from_le_bytes([p[1], p[2]]), p[3]))
}

/// Encode an SDO **upload** (read) request for `addr`.
pub fn encode_upload_request(addr: Address) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CCS_UPLOAD_INITIATE;
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
    p
}

/// Encode the server's expedited **upload response** carrying `value` for
/// `addr`.
///
/// Returns [`Error::UnsupportedTransfer`] for values larger than four bytes.
pub fn encode_upload_expedited_response(addr: Address, value: &Value) -> Result<SdoPayload> {
    let len = value.size();
    if len == 0 || len > 4 {
        return Err(Error::UnsupportedTransfer);
    }
    let mut p = [0u8; 8];
    p[0] = SCS_UPLOAD_INITIATE | (((4 - len) as u8) << 2) | EXPEDITED_SIZED;
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
    value.encode_le(&mut p[4..4 + len])?;
    Ok(p)
}

/// Decode an expedited upload response into `(address, value)`, interpreting
/// the inline data as `data_type` (the client knows the expected type from
/// its OD/EDS).
///
/// Returns [`Error::UnexpectedCommand`] if the frame is not an expedited
/// upload response, or [`Error::TypeMismatch`] if the server's data length
/// disagrees with `data_type`.
pub fn decode_upload_expedited_response(
    p: &SdoPayload,
    data_type: DataType,
) -> Result<(Address, Value)> {
    let cmd = p[0];
    // scs must be "upload initiate" and the frame must be expedited + sized.
    if cmd & 0xE0 != SCS_UPLOAD_INITIATE || cmd & EXPEDITED_SIZED != EXPEDITED_SIZED {
        return Err(Error::UnexpectedCommand);
    }
    let n = (cmd >> 2) & 0x03;
    let len = 4 - n as usize;
    if len != data_type.size() {
        return Err(Error::TypeMismatch);
    }
    let addr = Address::new(u16::from_le_bytes([p[1], p[2]]), p[3]);
    let value = Value::decode_le(data_type, &p[4..4 + len])?;
    Ok((addr, value))
}

/// Encode an SDO **abort** for `addr` with `code`.
pub fn encode_abort(addr: Address, code: SdoAbortCode) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_ABORT;
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
    p[4..8].copy_from_slice(&(code as u32).to_le_bytes());
    p
}

/// Decode an SDO abort frame into `(address, raw_abort_code)`.
pub fn decode_abort(p: &SdoPayload) -> Result<(Address, u32)> {
    if p[0] != CS_ABORT {
        return Err(Error::UnexpectedCommand);
    }
    let addr = Address::new(u16::from_le_bytes([p[1], p[2]]), p[3]);
    let code = u32::from_le_bytes([p[4], p[5], p[6], p[7]]);
    Ok((addr, code))
}

// === Segmented transfer ====================================================
//
// For values larger than four bytes, transfer proceeds in two phases: an
// *initiate* exchange declaring the total byte count, then a run of *segment*
// exchanges each carrying up to seven data bytes. A per-transfer *toggle* bit
// alternates on every segment (starting at 0) to detect lost or duplicated
// frames, and the final data segment sets the "no more segments" bit.
//
// The initiate *download response* (server) and initiate *upload request*
// (client) are byte-identical to the expedited case, so reuse
// [`encode_download_response`] / [`decode_download_response`] and
// [`encode_upload_request`] for them.

/// A decoded SDO data segment: its toggle bit, whether it is the last segment,
/// and the (borrowed) data bytes it carries.
///
/// The download-segment request (client → server) and the upload-segment
/// response (server → client) share this exact frame layout, so one type and
/// one codec serve both directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment<'a> {
    /// The toggle bit for this segment (alternates each segment from `false`).
    pub toggle: bool,
    /// Whether this is the final segment of the transfer.
    pub last: bool,
    /// The segment's payload (1..=7 bytes).
    pub data: &'a [u8],
}

/// Encode a client **segmented download initiate** request declaring a
/// `size`-byte transfer to `addr` (command `0x21`).
pub fn encode_download_initiate_segmented(addr: Address, size: u32) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CCS_DOWNLOAD_INITIATE | SIZE_INDICATED;
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
    p[4..8].copy_from_slice(&size.to_le_bytes());
    p
}

/// Decode a download initiate request into `(address, size)`, requiring a
/// segmented (non-expedited), size-indicated request.
pub fn decode_download_initiate_segmented(p: &SdoPayload) -> Result<(Address, u32)> {
    if p[0] & CS_MASK != CCS_DOWNLOAD_INITIATE
        || p[0] & EXPEDITED != 0
        || p[0] & SIZE_INDICATED == 0
    {
        return Err(Error::UnexpectedCommand);
    }
    let addr = Address::new(u16::from_le_bytes([p[1], p[2]]), p[3]);
    Ok((addr, u32::from_le_bytes([p[4], p[5], p[6], p[7]])))
}

/// Encode the server's **segmented upload initiate response** declaring a
/// `size`-byte transfer for `addr` (command `0x41`).
pub fn encode_upload_initiate_segmented_response(addr: Address, size: u32) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = SCS_UPLOAD_INITIATE | SIZE_INDICATED;
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
    p[4..8].copy_from_slice(&size.to_le_bytes());
    p
}

/// Decode a segmented upload initiate response into `(address, size)`.
///
/// Returns [`Error::UnexpectedCommand`] if the frame is not an upload initiate
/// response, or if it is expedited (use [`decode_upload_expedited_response`]
/// for that case).
pub fn decode_upload_initiate_segmented_response(p: &SdoPayload) -> Result<(Address, u32)> {
    if p[0] & CS_MASK != SCS_UPLOAD_INITIATE || p[0] & EXPEDITED != 0 || p[0] & SIZE_INDICATED == 0
    {
        return Err(Error::UnexpectedCommand);
    }
    let addr = Address::new(u16::from_le_bytes([p[1], p[2]]), p[3]);
    Ok((addr, u32::from_le_bytes([p[4], p[5], p[6], p[7]])))
}

/// Encode a **data segment** carrying 1..=7 bytes of `data`.
///
/// Used for both the download-segment request and the upload-segment response
/// (identical layout). `toggle` alternates each segment (the first is
/// `false`); `last` marks the final segment. Returns [`Error::BadLength`]
/// unless `data` is 1..=7 bytes.
pub fn encode_data_segment(data: &[u8], toggle: bool, last: bool) -> Result<SdoPayload> {
    if data.is_empty() || data.len() > SEGMENT_DATA_MAX {
        return Err(Error::BadLength);
    }
    let mut p = [0u8; 8];
    // ccs/scs for a data segment are both 000, so byte 0's top bits stay 0.
    let n = (SEGMENT_DATA_MAX - data.len()) as u8;
    p[0] = (n << 1) & 0x0E;
    if toggle {
        p[0] |= TOGGLE;
    }
    if last {
        p[0] |= NO_MORE_SEGMENTS;
    }
    p[1..1 + data.len()].copy_from_slice(data);
    Ok(p)
}

/// Decode a **data segment** frame (download request or upload response).
pub fn decode_data_segment(p: &SdoPayload) -> Result<Segment<'_>> {
    if p[0] & CS_MASK != CCS_DOWNLOAD_SEGMENT {
        return Err(Error::UnexpectedCommand);
    }
    let n = ((p[0] >> 1) & 0x07) as usize;
    if n > SEGMENT_DATA_MAX {
        return Err(Error::BadLength);
    }
    Ok(Segment {
        toggle: p[0] & TOGGLE != 0,
        last: p[0] & NO_MORE_SEGMENTS != 0,
        data: &p[1..1 + (SEGMENT_DATA_MAX - n)],
    })
}

/// Encode the server's **download segment response** (acknowledgement) with the
/// segment's `toggle` bit (command `0x20 | toggle`).
pub fn encode_download_segment_response(toggle: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = SCS_DOWNLOAD_SEGMENT | if toggle { TOGGLE } else { 0 };
    p
}

/// Decode a download segment response, returning its toggle bit.
pub fn decode_download_segment_response(p: &SdoPayload) -> Result<bool> {
    if p[0] & CS_MASK != SCS_DOWNLOAD_SEGMENT {
        return Err(Error::UnexpectedCommand);
    }
    Ok(p[0] & TOGGLE != 0)
}

/// Encode the client's **upload segment request** (poll for the next segment)
/// with the expected `toggle` bit (command `0x60 | toggle`).
pub fn encode_upload_segment_request(toggle: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CCS_UPLOAD_SEGMENT | if toggle { TOGGLE } else { 0 };
    p
}

/// Decode an upload segment request, returning its toggle bit.
pub fn decode_upload_segment_request(p: &SdoPayload) -> Result<bool> {
    if p[0] & CS_MASK != CCS_UPLOAD_SEGMENT {
        return Err(Error::UnexpectedCommand);
    }
    Ok(p[0] & TOGGLE != 0)
}

/// Splits a byte buffer into SDO download data segments, tracking the toggle
/// bit. Drive it after a successful download-initiate handshake: emit each
/// frame, await its acknowledgement, then take the next.
#[derive(Debug)]
pub struct SegmentWriter<'a> {
    data: &'a [u8],
    pos: usize,
    toggle: bool,
}

impl<'a> SegmentWriter<'a> {
    /// Start splitting `data` (which should be the >4-byte value already
    /// declared in the initiate request).
    pub const fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0,
            toggle: false,
        }
    }

    /// Whether every byte has been emitted.
    pub const fn is_done(&self) -> bool {
        self.pos >= self.data.len()
    }

    /// Produce the next download data-segment frame, or `None` when finished.
    pub fn next_segment(&mut self) -> Option<SdoPayload> {
        if self.is_done() {
            return None;
        }
        let remaining = self.data.len() - self.pos;
        let len = remaining.min(SEGMENT_DATA_MAX);
        let last = remaining <= SEGMENT_DATA_MAX;
        let frame = encode_data_segment(&self.data[self.pos..self.pos + len], self.toggle, last)
            .expect("len is 1..=7");
        self.pos += len;
        self.toggle = !self.toggle;
        Some(frame)
    }
}

/// Reassembles SDO upload data segments into a bounded buffer of capacity `N`,
/// tracking and validating the toggle bit. Push each decoded [`Segment`] until
/// [`SegmentReader::is_done`], then read [`SegmentReader::data`].
#[derive(Debug)]
pub struct SegmentReader<const N: usize> {
    buf: Vec<u8, N>,
    toggle: bool,
    done: bool,
}

impl<const N: usize> Default for SegmentReader<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> SegmentReader<N> {
    /// A new, empty reassembler.
    pub const fn new() -> Self {
        Self {
            buf: Vec::new(),
            toggle: false,
            done: false,
        }
    }

    /// Whether the final segment has been received.
    pub const fn is_done(&self) -> bool {
        self.done
    }

    /// The reassembled bytes so far.
    pub fn data(&self) -> &[u8] {
        &self.buf
    }

    /// Append a decoded segment.
    ///
    /// Returns [`Error::ToggleMismatch`] if the segment's toggle bit is out of
    /// sequence, [`Error::UnexpectedCommand`] if the transfer is already
    /// complete, or [`Error::Overflow`] if the data exceeds capacity `N`.
    pub fn push(&mut self, segment: &Segment) -> Result<()> {
        if self.done {
            return Err(Error::UnexpectedCommand);
        }
        if segment.toggle != self.toggle {
            return Err(Error::ToggleMismatch);
        }
        self.buf
            .extend_from_slice(segment.data)
            .map_err(|_| Error::Overflow)?;
        self.toggle = !self.toggle;
        if segment.last {
            self.done = true;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- COB-IDs -----------------------------------------------------------
    #[test]
    fn cob_ids_follow_convention() {
        let node = NodeId::new(0x05).unwrap();
        assert_eq!(request_cob_id(node), 0x605);
        assert_eq!(response_cob_id(node), 0x585);
    }

    // --- Download (write) --------------------------------------------------
    // Known-good frame: expedited download of UNSIGNED32 0x12345678 to
    // object 0x2000 sub 0. Command 0x23 = download initiate, expedited,
    // size indicated, 4 data bytes. Index and value are little-endian.
    #[test]
    fn download_u32_matches_known_frame() {
        let f = encode_download_expedited(Address::new(0x2000, 0), &Value::Unsigned32(0x1234_5678))
            .unwrap();
        assert_eq!(f, [0x23, 0x00, 0x20, 0x00, 0x78, 0x56, 0x34, 0x12]);
    }

    // Known-good frame: expedited download of UNSIGNED8 0x7F to 0x2001 sub 0.
    // Command 0x2F = download initiate, expedited, size indicated, 1 data byte.
    #[test]
    fn download_u8_matches_known_frame() {
        let f =
            encode_download_expedited(Address::new(0x2001, 0), &Value::Unsigned8(0x7F)).unwrap();
        assert_eq!(f, [0x2F, 0x01, 0x20, 0x00, 0x7F, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn download_i16_matches_known_frame() {
        // -2 as INTEGER16 = 0xFFFE little-endian; command 0x2B = 2 data bytes.
        let f = encode_download_expedited(Address::new(0x6000, 1), &Value::Integer16(-2)).unwrap();
        assert_eq!(f, [0x2B, 0x00, 0x60, 0x01, 0xFE, 0xFF, 0x00, 0x00]);
    }

    #[test]
    fn download_response_roundtrips() {
        let f = encode_download_response(Address::new(0x2000, 0));
        assert_eq!(f, [0x60, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(
            decode_download_response(&f).unwrap(),
            Address::new(0x2000, 0)
        );
    }

    #[test]
    fn value_too_large_for_expedited_rejected() {
        assert_eq!(
            encode_download_expedited(Address::new(0x2000, 0), &Value::Unsigned64(1)),
            Err(Error::UnsupportedTransfer)
        );
    }

    // --- Upload (read) -----------------------------------------------------
    // Known-good frame: upload (read) request for object 0x1000 sub 0.
    #[test]
    fn upload_request_matches_known_frame() {
        let f = encode_upload_request(Address::new(0x1000, 0));
        assert_eq!(f, [0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00]);
    }

    // Known-good frame: expedited upload response for device type object
    // 0x1000 = UNSIGNED32 0x00000192. Command 0x43 = upload initiate,
    // expedited, size indicated, 4 data bytes.
    #[test]
    fn upload_response_device_type_decodes() {
        let f = [0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x00, 0x00];
        let (addr, value) = decode_upload_expedited_response(&f, DataType::Unsigned32).unwrap();
        assert_eq!(addr, Address::new(0x1000, 0));
        assert_eq!(value, Value::Unsigned32(0x0000_0192));
    }

    #[test]
    fn upload_response_encode_matches_known_frame() {
        let f = encode_upload_expedited_response(
            Address::new(0x1000, 0),
            &Value::Unsigned32(0x0000_0192),
        )
        .unwrap();
        assert_eq!(f, [0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x00, 0x00]);
    }

    #[test]
    fn upload_response_wrong_type_size_errors() {
        // Frame declares 4 data bytes; decoding as U16 (2 bytes) must fail.
        let f = [0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x00, 0x00];
        assert_eq!(
            decode_upload_expedited_response(&f, DataType::Unsigned16),
            Err(Error::TypeMismatch)
        );
    }

    #[test]
    fn decode_upload_rejects_non_upload_frame() {
        let f = encode_download_response(Address::new(0x1000, 0));
        assert_eq!(
            decode_upload_expedited_response(&f, DataType::Unsigned32),
            Err(Error::UnexpectedCommand)
        );
    }

    // --- Abort -------------------------------------------------------------
    // Known-good frame: abort of 0x1000 sub 0 with code 0x06020000
    // (object does not exist), sent little-endian in bytes 4..8.
    #[test]
    fn abort_object_missing_matches_known_frame() {
        let f = encode_abort(Address::new(0x1000, 0), SdoAbortCode::ObjectDoesNotExist);
        assert_eq!(f, [0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x02, 0x06]);
        let (addr, code) = decode_abort(&f).unwrap();
        assert_eq!(addr, Address::new(0x1000, 0));
        assert_eq!(code, 0x0602_0000);
    }

    // --- Segmented transfer ------------------------------------------------
    // Known-good frame: segmented download initiate of a 20-byte value to
    // 0x2000 sub 0. Command 0x21 = download initiate, size indicated, not
    // expedited; size 20 little-endian in bytes 4..8.
    #[test]
    fn download_initiate_segmented_matches_known_frame() {
        let f = encode_download_initiate_segmented(Address::new(0x2000, 0), 20);
        assert_eq!(f, [0x21, 0x00, 0x20, 0x00, 20, 0x00, 0x00, 0x00]);
        assert_eq!(
            decode_download_initiate_segmented(&f).unwrap(),
            (Address::new(0x2000, 0), 20)
        );
    }

    // Known-good frame: segmented upload initiate response, 20-byte value from
    // 0x2000 sub 0. Command 0x41 = upload initiate, size indicated, segmented.
    #[test]
    fn upload_initiate_segmented_response_matches_known_frame() {
        let f = encode_upload_initiate_segmented_response(Address::new(0x2000, 0), 20);
        assert_eq!(f, [0x41, 0x00, 0x20, 0x00, 20, 0x00, 0x00, 0x00]);
        assert_eq!(
            decode_upload_initiate_segmented_response(&f).unwrap(),
            (Address::new(0x2000, 0), 20)
        );
    }

    // A segmented initiate must not decode an expedited response and vice versa.
    #[test]
    fn segmented_initiate_rejects_expedited_frame() {
        let expedited = [0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x00, 0x00];
        assert_eq!(
            decode_upload_initiate_segmented_response(&expedited),
            Err(Error::UnexpectedCommand)
        );
    }

    // Known-good frame: a full 7-byte first data segment, toggle 0, not last.
    // Command 0x00: n = 0 unused bytes, toggle clear, continue.
    #[test]
    fn data_segment_full_matches_known_frame() {
        let f = encode_data_segment(&[1, 2, 3, 4, 5, 6, 7], false, false).unwrap();
        assert_eq!(f, [0x00, 1, 2, 3, 4, 5, 6, 7]);
    }

    // Known-good frame: a final 3-byte segment, toggle 1, last. Command 0x19 =
    // toggle (0x10) | n=4 unused bytes (4 << 1 = 0x08) | last (0x01).
    #[test]
    fn data_segment_last_matches_known_frame() {
        let f = encode_data_segment(&[0xAA, 0xBB, 0xCC], true, true).unwrap();
        assert_eq!(f, [0x19, 0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x00, 0x00]);
        let seg = decode_data_segment(&f).unwrap();
        assert!(seg.toggle);
        assert!(seg.last);
        assert_eq!(seg.data, &[0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn segment_ack_and_poll_toggle_roundtrip() {
        assert_eq!(
            encode_download_segment_response(false),
            [0x20, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            encode_download_segment_response(true),
            [0x30, 0, 0, 0, 0, 0, 0, 0]
        );
        assert!(decode_download_segment_response(&encode_download_segment_response(true)).unwrap());

        assert_eq!(
            encode_upload_segment_request(false),
            [0x60, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            encode_upload_segment_request(true),
            [0x70, 0, 0, 0, 0, 0, 0, 0]
        );
        assert!(decode_upload_segment_request(&encode_upload_segment_request(true)).unwrap());
    }

    // End-to-end: split a 12-byte value into segments and reassemble it. The
    // writer emits two frames (7 + 5), toggles alternating false/true, the
    // second marked last; the reader validates the toggle and rebuilds it.
    #[test]
    fn segment_writer_reader_roundtrip() {
        let payload: [u8; 12] = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120];
        let mut writer = SegmentWriter::new(&payload);
        let mut reader = SegmentReader::<64>::new();

        let f1 = writer.next_segment().unwrap();
        assert_eq!(f1[0] & TOGGLE, 0); // first toggle is 0
        reader.push(&decode_data_segment(&f1).unwrap()).unwrap();
        assert!(!reader.is_done());

        let f2 = writer.next_segment().unwrap();
        assert_ne!(f2[0] & TOGGLE, 0); // second toggle is 1
        reader.push(&decode_data_segment(&f2).unwrap()).unwrap();

        assert!(reader.is_done());
        assert!(writer.next_segment().is_none());
        assert_eq!(reader.data(), &payload);
    }

    #[test]
    fn reader_rejects_toggle_out_of_sequence() {
        let mut reader = SegmentReader::<16>::new();
        // Second push should expect toggle=true; supplying false must fail.
        reader
            .push(&Segment {
                toggle: false,
                last: false,
                data: &[1, 2, 3],
            })
            .unwrap();
        assert_eq!(
            reader.push(&Segment {
                toggle: false,
                last: true,
                data: &[4, 5]
            }),
            Err(Error::ToggleMismatch)
        );
    }

    #[test]
    fn reader_overflow_is_reported() {
        let mut reader = SegmentReader::<4>::new();
        assert_eq!(
            reader.push(&Segment {
                toggle: false,
                last: false,
                data: &[1, 2, 3, 4, 5]
            }),
            Err(Error::Overflow)
        );
    }
}
