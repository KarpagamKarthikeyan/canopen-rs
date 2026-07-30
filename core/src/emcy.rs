//! Emergency object (EMCY) — device error signalling (CiA 301 §7.2.7).
//!
//! A node transmits an EMCY frame when an internal error appears or clears. The
//! default COB-ID is `0x080 + node` ([`emcy_cob_id`]) — the same base as SYNC,
//! but SYNC is broadcast at exactly `0x080` while EMCY node ids are `1..=127`,
//! so they never collide.
//!
//! An EMCY frame is always eight bytes:
//!
//! | bytes | field |
//! |-------|-------|
//! | 0..2  | emergency error code (little-endian) |
//! | 2     | error register (a copy of object `0x1001`) |
//! | 3..8  | manufacturer-specific error field |
//!
//! A frame with error code `0x0000` signals *error reset / no error*.

use crate::types::NodeId;
use crate::{Error, Result};

/// COB-ID base for the EMCY channel: `0x080 + node`.
pub const EMCY_COB_BASE: u16 = 0x080;

/// The COB-ID of a node's EMCY producer channel.
pub const fn emcy_cob_id(node: NodeId) -> u16 {
    EMCY_COB_BASE + node.raw() as u16
}

/// The 8-byte EMCY payload carried in a CAN frame's data field.
pub type EmcyPayload = [u8; 8];

/// Well-known emergency error-code *classes* — the high byte of the 16-bit
/// code (CiA 301 §7.2.7.1, Table 22). The low byte refines the class.
pub mod error_code {
    /// No error / error reset.
    pub const ERROR_RESET: u16 = 0x0000;
    /// Generic error.
    pub const GENERIC: u16 = 0x1000;
    /// Current.
    pub const CURRENT: u16 = 0x2000;
    /// Voltage.
    pub const VOLTAGE: u16 = 0x3000;
    /// Temperature.
    pub const TEMPERATURE: u16 = 0x4000;
    /// Device hardware.
    pub const DEVICE_HARDWARE: u16 = 0x5000;
    /// Device software.
    pub const DEVICE_SOFTWARE: u16 = 0x6000;
    /// Additional modules.
    pub const ADDITIONAL_MODULES: u16 = 0x7000;
    /// Monitoring.
    pub const MONITORING: u16 = 0x8000;
    /// Communication (subset of monitoring).
    pub const COMMUNICATION: u16 = 0x8100;
    /// External error.
    pub const EXTERNAL: u16 = 0x9000;
    /// Additional functions.
    pub const ADDITIONAL_FUNCTIONS: u16 = 0xF000;
    /// Device-specific.
    pub const DEVICE_SPECIFIC: u16 = 0xFF00;
}

/// The error register (object `0x1001`): a bitfield summarising which error
/// classes are currently active on the node (CiA 301 §7.5.2.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ErrorRegister(pub u8);

impl ErrorRegister {
    /// Bit 0 — a generic error is present. Set whenever any other bit is.
    pub const GENERIC: u8 = 0x01;
    /// Bit 1 — current.
    pub const CURRENT: u8 = 0x02;
    /// Bit 2 — voltage.
    pub const VOLTAGE: u8 = 0x04;
    /// Bit 3 — temperature.
    pub const TEMPERATURE: u8 = 0x08;
    /// Bit 4 — communication (overrun, error state).
    pub const COMMUNICATION: u8 = 0x10;
    /// Bit 5 — device-profile-specific.
    pub const DEVICE_PROFILE: u8 = 0x20;
    /// Bit 7 — manufacturer-specific. (Bit 6 is reserved, always 0.)
    pub const MANUFACTURER: u8 = 0x80;

    /// An error register with no bits set (no active error).
    pub const NONE: ErrorRegister = ErrorRegister(0);

    /// Whether all of `bits` are set.
    pub const fn contains(self, bits: u8) -> bool {
        self.0 & bits == bits
    }

    /// Whether any error is signalled (any bit set).
    pub const fn has_error(self) -> bool {
        self.0 != 0
    }
}

/// An emergency message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmergencyMessage {
    /// The 16-bit emergency error code (see [`error_code`]).
    pub error_code: u16,
    /// The error register (object `0x1001`) at the time of the event.
    pub error_register: ErrorRegister,
    /// Five bytes of manufacturer-specific error information.
    pub vendor_specific: [u8; 5],
}

impl EmergencyMessage {
    /// A new emergency message.
    pub const fn new(error_code: u16, error_register: ErrorRegister, vendor_specific: [u8; 5]) -> Self {
        Self { error_code, error_register, vendor_specific }
    }

    /// The *error reset / no error* message (code `0x0000`, register clear).
    pub const fn error_reset() -> Self {
        Self {
            error_code: error_code::ERROR_RESET,
            error_register: ErrorRegister::NONE,
            vendor_specific: [0; 5],
        }
    }

    /// Whether this message signals *error reset / no error*.
    pub const fn is_error_reset(&self) -> bool {
        self.error_code == error_code::ERROR_RESET
    }

    /// Encode this message into its 8-byte payload.
    pub fn encode(&self) -> EmcyPayload {
        let mut p = [0u8; 8];
        p[0..2].copy_from_slice(&self.error_code.to_le_bytes());
        p[2] = self.error_register.0;
        p[3..8].copy_from_slice(&self.vendor_specific);
        p
    }

    /// Decode an EMCY frame's data field.
    ///
    /// Returns [`Error::BadLength`] unless `data` is exactly eight bytes.
    pub fn decode(data: &[u8]) -> Result<Self> {
        if data.len() != 8 {
            return Err(Error::BadLength);
        }
        let mut vendor_specific = [0u8; 5];
        vendor_specific.copy_from_slice(&data[3..8]);
        Ok(Self {
            error_code: u16::from_le_bytes([data[0], data[1]]),
            error_register: ErrorRegister(data[2]),
            vendor_specific,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cob_id_follows_convention() {
        assert_eq!(emcy_cob_id(NodeId::new(0x05).unwrap()), 0x085);
    }

    // Known-good frame: overvoltage (code 0x3210) with the generic+voltage
    // register bits set (0x05) and vendor bytes 0xAA..0xEE. Code is
    // little-endian in bytes 0..2.
    #[test]
    fn encode_matches_known_frame() {
        let msg = EmergencyMessage::new(
            0x3210,
            ErrorRegister(ErrorRegister::GENERIC | ErrorRegister::VOLTAGE),
            [0xAA, 0xBB, 0xCC, 0xDD, 0xEE],
        );
        assert_eq!(msg.encode(), [0x10, 0x32, 0x05, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE]);
    }

    #[test]
    fn decode_matches_known_frame() {
        let msg = EmergencyMessage::decode(&[0x10, 0x32, 0x05, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE]).unwrap();
        assert_eq!(msg.error_code, 0x3210);
        assert!(msg.error_register.contains(ErrorRegister::VOLTAGE));
        assert!(msg.error_register.has_error());
        assert_eq!(msg.vendor_specific, [0xAA, 0xBB, 0xCC, 0xDD, 0xEE]);
    }

    #[test]
    fn encode_decode_roundtrip() {
        let msg = EmergencyMessage::new(
            error_code::COMMUNICATION,
            ErrorRegister(ErrorRegister::GENERIC | ErrorRegister::COMMUNICATION),
            [1, 2, 3, 4, 5],
        );
        assert_eq!(EmergencyMessage::decode(&msg.encode()).unwrap(), msg);
    }

    #[test]
    fn error_reset_is_recognised() {
        let reset = EmergencyMessage::error_reset();
        assert!(reset.is_error_reset());
        assert!(!reset.error_register.has_error());
        assert_eq!(reset.encode(), [0; 8]);
    }

    #[test]
    fn decode_rejects_wrong_length() {
        assert_eq!(EmergencyMessage::decode(&[0; 7]), Err(Error::BadLength));
    }
}
