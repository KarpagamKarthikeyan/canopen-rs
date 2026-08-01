//! Time stamp object (TIME) — the network time-of-day (CiA 301 §7.2.6).
//!
//! A single node (the TIME producer) broadcasts the absolute time on the
//! default COB-ID `0x100` ([`TIME_COB_ID`]). The payload is a six-byte
//! `TIME_OF_DAY`: milliseconds since midnight and days since 1 January 1984.
//!
//! ```
//! use canopen_rs::time::TimeOfDay;
//!
//! let t = TimeOfDay::new(0x0123_4567, 0x1234);
//! let frame = t.encode();
//! assert_eq!(frame, [0x67, 0x45, 0x23, 0x01, 0x34, 0x12]);
//! assert_eq!(TimeOfDay::decode(&frame).unwrap(), t);
//! ```

use crate::{Error, Result};

/// The default TIME COB-ID (`0x100`), configurable via object `0x1012`.
pub const TIME_COB_ID: u16 = 0x100;

/// The number of milliseconds in a day — the exclusive upper bound of a valid
/// [`TimeOfDay::milliseconds`].
pub const MS_PER_DAY: u32 = 86_400_000;

/// A CANopen `TIME_OF_DAY`: milliseconds after midnight and days since the
/// CANopen epoch (1 January 1984).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeOfDay {
    /// Milliseconds after midnight. The wire field is 28 bits, so only the low
    /// 28 bits are transmitted (values up to [`MS_PER_DAY`]).
    pub milliseconds: u32,
    /// Days since 1 January 1984.
    pub days: u16,
}

impl TimeOfDay {
    /// Construct a time of day.
    pub const fn new(milliseconds: u32, days: u16) -> Self {
        Self { milliseconds, days }
    }

    /// Encode into the six-byte TIME data field: the 28-bit millisecond count
    /// little-endian in bytes 0..4 (top four bits reserved, zero) and the day
    /// count little-endian in bytes 4..6.
    pub fn encode(&self) -> [u8; 6] {
        let ms = self.milliseconds & 0x0FFF_FFFF;
        let mut frame = [0u8; 6];
        frame[0..4].copy_from_slice(&ms.to_le_bytes());
        frame[4..6].copy_from_slice(&self.days.to_le_bytes());
        frame
    }

    /// Decode a TIME data field. Requires at least six bytes; the reserved top
    /// four bits of the millisecond field are masked off.
    pub fn decode(data: &[u8]) -> Result<Self> {
        if data.len() < 6 {
            return Err(Error::BadLength);
        }
        Ok(Self {
            milliseconds: u32::from_le_bytes([data[0], data[1], data[2], data[3]]) & 0x0FFF_FFFF,
            days: u16::from_le_bytes([data[4], data[5]]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cob_id_is_default() {
        assert_eq!(TIME_COB_ID, 0x100);
    }

    #[test]
    fn encode_decode_roundtrips() {
        let t = TimeOfDay::new(45_000_000, 15_600); // ~12:30, a plausible day count
        assert_eq!(TimeOfDay::decode(&t.encode()).unwrap(), t);
    }

    #[test]
    fn masks_reserved_millisecond_bits() {
        // Bits above 28 are reserved and must not survive an encode/decode.
        let t = TimeOfDay::new(0xF000_0001, 1);
        assert_eq!(t.encode()[..4], [0x01, 0x00, 0x00, 0x00]);
        assert_eq!(TimeOfDay::decode(&t.encode()).unwrap().milliseconds, 1);
    }

    #[test]
    fn decode_rejects_short_frame() {
        assert_eq!(TimeOfDay::decode(&[0; 5]), Err(Error::BadLength));
    }
}
