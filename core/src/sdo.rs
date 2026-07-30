//! Service Data Object (SDO) protocol (CiA 301 §7.2.4).
//!
//! SDOs provide confirmed, addressed read/write access to any object
//! dictionary entry. Each transfer step is a client request frame answered by
//! a server response frame, exchanged on a pair of CAN ids (by default
//! `0x600 + node` for requests and `0x580 + node` for responses).
//!
//! This module implements **expedited** transfer: a value of one to four
//! bytes carried inline in a single request/response exchange. Segmented and
//! block transfer follow.
//!
//! The functions here encode and decode the raw 8-byte CAN *data field* only.
//! Selecting the CAN id (see [`request_cob_id`]/[`response_cob_id`]) and
//! moving the frame is the transport's responsibility, keeping this codec
//! transport-agnostic.

use crate::datatypes::{DataType, Value};
use crate::object_dictionary::Address;
use crate::types::NodeId;
use crate::{Error, Result};

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
const CCS_DOWNLOAD_INITIATE: u8 = 0x20; // client: 001xxxxx
const CCS_UPLOAD_INITIATE: u8 = 0x40; // client: 010xxxxx
const SCS_UPLOAD_INITIATE: u8 = 0x40; // server: 010xxxxx
const SCS_DOWNLOAD_INITIATE: u8 = 0x60; // server: 011xxxxx
const CS_ABORT: u8 = 0x80; // either:  100xxxxx

// Expedited + size-indicated flags (low two bits of byte 0).
const EXPEDITED_SIZED: u8 = 0x03;

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
        let f =
            encode_download_expedited(Address::new(0x2000, 0), &Value::Unsigned32(0x1234_5678))
                .unwrap();
        assert_eq!(f, [0x23, 0x00, 0x20, 0x00, 0x78, 0x56, 0x34, 0x12]);
    }

    // Known-good frame: expedited download of UNSIGNED8 0x7F to 0x2001 sub 0.
    // Command 0x2F = download initiate, expedited, size indicated, 1 data byte.
    #[test]
    fn download_u8_matches_known_frame() {
        let f = encode_download_expedited(Address::new(0x2001, 0), &Value::Unsigned8(0x7F)).unwrap();
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
        assert_eq!(decode_download_response(&f).unwrap(), Address::new(0x2000, 0));
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
        let (addr, value) =
            decode_upload_expedited_response(&f, DataType::Unsigned32).unwrap();
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
}
