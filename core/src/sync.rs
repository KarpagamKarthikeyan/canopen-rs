//! Synchronisation object (SYNC) — the network heartbeat for synchronous
//! communication (CiA 301 §7.2.5).
//!
//! SYNC is a high-priority, broadcast frame produced periodically by a single
//! node (the SYNC producer). It carries no addressed data: consumers act on
//! its *arrival* — sampling inputs, applying synchronous RPDOs, transmitting
//! synchronous TPDOs. The frame is empty by default, or carries a single
//! **counter** byte when the producer's synchronous-counter-overflow value
//! (object `0x1019`) is set to `2..=240`.
//!
//! The default COB-ID is `0x080` ([`SYNC_COB_ID`]), configurable via object
//! `0x1005`.

use crate::{Error, Result};

/// The default SYNC COB-ID (`0x080`), configurable via object `0x1005`.
pub const SYNC_COB_ID: u16 = 0x080;

/// Whether a synchronous-counter-overflow value (object `0x1019`) is valid:
/// `0` disables the counter (empty SYNC), and `2..=240` enables a counting
/// SYNC. The value `1` is reserved.
pub const fn is_valid_counter_overflow(overflow: u8) -> bool {
    overflow == 0 || (2 <= overflow && overflow <= 240)
}

/// Decode a received SYNC frame's data field.
///
/// Returns `Ok(None)` for an empty (counter-less) SYNC, or `Ok(Some(count))`
/// for a counting SYNC. Returns [`Error::BadLength`] if the frame carries more
/// than one byte.
pub fn decode(data: &[u8]) -> Result<Option<u8>> {
    match data.len() {
        0 => Ok(None),
        1 => Ok(Some(data[0])),
        _ => Err(Error::BadLength),
    }
}

/// The data field of a counting SYNC carrying `count`.
///
/// The empty (counter-less) SYNC has no data field, so there is nothing to
/// encode for it — transmit a zero-length frame.
pub fn encode_counter(count: u8) -> [u8; 1] {
    [count]
}

/// The counter for a SYNC producer (CiA 301 §7.2.5.2.1).
///
/// When enabled, the counter cycles `1..=overflow` and wraps back to `1`.
/// Constructed from the object `0x1019` overflow value: `0` (or the reserved
/// `1`) means the producer emits empty SYNC frames, so [`SyncCounter::advance`]
/// yields `None`.
#[derive(Debug, Clone, Copy)]
pub struct SyncCounter {
    overflow: u8,
    value: u8,
}

impl SyncCounter {
    /// Create a SYNC counter from an overflow value.
    ///
    /// Returns [`Error::InvalidSyncCounter`] unless `overflow` is `0` or in
    /// `2..=240`.
    pub fn new(overflow: u8) -> Result<Self> {
        if !is_valid_counter_overflow(overflow) {
            return Err(Error::InvalidSyncCounter);
        }
        // `value` holds the last emitted count; the first `advance` yields 1.
        Ok(Self { overflow, value: 0 })
    }

    /// Whether this producer emits a counter (overflow in `2..=240`).
    pub const fn is_counting(&self) -> bool {
        self.overflow >= 2
    }

    /// Advance to the next SYNC counter value.
    ///
    /// Returns `None` for a counter-less producer (emit an empty SYNC), or
    /// `Some(count)` cycling `1..=overflow`.
    pub fn advance(&mut self) -> Option<u8> {
        if !self.is_counting() {
            return None;
        }
        self.value = if self.value >= self.overflow { 1 } else { self.value + 1 };
        Some(self.value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cob_id_is_default() {
        assert_eq!(SYNC_COB_ID, 0x080);
    }

    #[test]
    fn counter_overflow_validation() {
        assert!(is_valid_counter_overflow(0));
        assert!(!is_valid_counter_overflow(1)); // reserved
        assert!(is_valid_counter_overflow(2));
        assert!(is_valid_counter_overflow(240));
        assert!(!is_valid_counter_overflow(241));
    }

    #[test]
    fn decode_empty_and_counting_frames() {
        assert_eq!(decode(&[]).unwrap(), None);
        assert_eq!(decode(&[7]).unwrap(), Some(7));
        assert_eq!(decode(&[1, 2]), Err(Error::BadLength));
    }

    #[test]
    fn encode_counter_frame() {
        assert_eq!(encode_counter(5), [5]);
    }

    #[test]
    fn counter_disabled_yields_none() {
        let mut c = SyncCounter::new(0).unwrap();
        assert!(!c.is_counting());
        assert_eq!(c.advance(), None);
    }

    #[test]
    fn counter_cycles_and_wraps() {
        let mut c = SyncCounter::new(3).unwrap();
        assert_eq!(c.advance(), Some(1));
        assert_eq!(c.advance(), Some(2));
        assert_eq!(c.advance(), Some(3));
        assert_eq!(c.advance(), Some(1)); // wraps back to 1
    }

    #[test]
    fn new_rejects_reserved_overflow() {
        assert_eq!(SyncCounter::new(1).err(), Some(Error::InvalidSyncCounter));
        assert_eq!(SyncCounter::new(241).err(), Some(Error::InvalidSyncCounter));
    }
}
