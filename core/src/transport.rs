//! Bridging CANopen frames to the [`embedded_can`] traits.
//!
//! The protocol modules speak in COB-IDs (an 11-bit CAN identifier) and 8-byte
//! payloads; a CAN controller speaks in [`embedded_can::Frame`]s. These two
//! helpers convert between the two, so any driver implementing the
//! `embedded-can` traits — a bare-metal HAL or a host SocketCAN socket alike —
//! can carry CANopen traffic.
//!
//! ```
//! # use canopen_rs::sdo::{self, SdoClient};
//! # use canopen_rs::transport::{frame_from, cob_id};
//! # use canopen_rs::{NodeId, Address, DataType};
//! # fn example<F: embedded_can::Frame>(bus_send: impl Fn(&F), bus_recv: impl Fn() -> F) {
//! let mut client = SdoClient::new(NodeId::new(0x10).unwrap());
//! let payload = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
//! // Turn the CANopen request into a frame and put it on the bus:
//! let frame: F = frame_from(client.request_cob_id(), &payload).unwrap();
//! bus_send(&frame);
//! // Later, pull a frame off the bus and recover its COB-ID:
//! let reply = bus_recv();
//! if cob_id(&reply) == Some(client.response_cob_id()) {
//!     // dispatch reply.data() to client.on_response(..)
//! }
//! # }
//! ```
//!
//! ```rust
//! use canopen_rs::transport::{cob_id, frame_from};
//! use embedded_can::{Frame, Id, StandardId};
//!
//! #[derive(Debug)]
//! struct MockFrame {
//!     id: Id,
//!     data: [u8; 8],
//!     len: usize,
//! }
//!
//! impl Frame for MockFrame {
//!     fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
//!         if data.len() > 8 {
//!             return None;
//!         }
//!         let mut raw = [0u8; 8];
//!         raw[..data.len()].copy_from_slice(data);
//!         Some(Self {
//!             id: id.into(),
//!             data: raw,
//!             len: data.len(),
//!         })
//!     }
//!
//!     fn new_remote(_id: impl Into<Id>, _dlc: usize) -> Option<Self> {
//!         None
//!     }
//!
//!     fn is_extended(&self) -> bool {
//!         matches!(self.id, Id::Extended(_))
//!     }
//!
//!     fn is_remote_frame(&self) -> bool {
//!         false
//!     }
//!
//!     fn id(&self) -> Id {
//!         self.id
//!     }
//!
//!     fn dlc(&self) -> usize {
//!         self.len
//!     }
//!
//!     fn data(&self) -> &[u8] {
//!         &self.data[..self.len]
//!     }
//! }
//!
//! let frame: MockFrame = frame_from(0x581, &[0x43, 0x00, 0x10, 0x00]).unwrap();
//! assert_eq!(frame.id(), Id::Standard(StandardId::new(0x581).unwrap()));
//! assert_eq!(cob_id(&frame), Some(0x581));
//! assert_eq!(frame.data(), &[0x43, 0x00, 0x10, 0x00]);
//! ```

use embedded_can::{Frame, Id, StandardId};

/// Build a standard (11-bit) CAN frame carrying `data` on COB-ID `cob_id`.
///
/// Returns `None` if `cob_id` is not a valid 11-bit identifier, `data` is
/// longer than the frame can hold, or the driver's frame constructor rejects
/// it.
pub fn frame_from<F: Frame>(cob_id: u16, data: &[u8]) -> Option<F> {
    F::new(StandardId::new(cob_id)?, data)
}

/// The CANopen COB-ID of a received frame.
///
/// Returns `None` for a frame with a 29-bit extended identifier, which the
/// CANopen predefined connection set does not use.
pub fn cob_id<F: Frame>(frame: &F) -> Option<u16> {
    match frame.id() {
        Id::Standard(id) => Some(id.as_raw()),
        Id::Extended(_) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use heapless::Vec;

    /// A minimal in-memory [`Frame`] for exercising the helpers.
    #[derive(Debug)]
    struct MockFrame {
        id: Id,
        data: Vec<u8, 8>,
    }

    impl Frame for MockFrame {
        fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
            let mut buf = Vec::new();
            buf.extend_from_slice(data).ok()?;
            Some(Self {
                id: id.into(),
                data: buf,
            })
        }

        fn new_remote(_id: impl Into<Id>, _dlc: usize) -> Option<Self> {
            None
        }

        fn is_extended(&self) -> bool {
            matches!(self.id, Id::Extended(_))
        }

        fn is_remote_frame(&self) -> bool {
            false
        }

        fn id(&self) -> Id {
            self.id
        }

        fn dlc(&self) -> usize {
            self.data.len()
        }

        fn data(&self) -> &[u8] {
            &self.data
        }
    }

    #[test]
    fn round_trips_cob_id_and_data() {
        let frame: MockFrame = frame_from(0x581, &[0x43, 0x00, 0x10, 0x00]).unwrap();
        assert_eq!(cob_id(&frame), Some(0x581));
        assert_eq!(frame.data(), &[0x43, 0x00, 0x10, 0x00]);
    }

    #[test]
    fn rejects_out_of_range_cob_id() {
        // 0x800 exceeds the 11-bit standard id range.
        assert!(frame_from::<MockFrame>(0x800, &[]).is_none());
    }

    #[test]
    fn extended_id_has_no_cob_id() {
        use embedded_can::ExtendedId;
        let frame = MockFrame {
            id: Id::Extended(ExtendedId::new(0x1234).unwrap()),
            data: Vec::new(),
        };
        assert_eq!(cob_id(&frame), None);
    }
}
