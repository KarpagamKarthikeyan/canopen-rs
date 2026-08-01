//! The mandatory CiA 301 communication-profile objects.
//!
//! Every conformant CANopen device exposes a common baseline of objects in the
//! communication profile area (`0x1000`–`0x1FFF`): the device type, error
//! register, heartbeat time, and identity. Hand-inserting them into an
//! [`ObjectDictionary`] for every node is tedious and easy to get wrong, so
//! [`StandardObjects`] builds that baseline in one call.
//!
//! It ties the recent pieces together: the identity (`0x1018`) is the same
//! [`LssAddress`] an LSS master matches (see [`crate::lss`]), and the error
//! register (`0x1001`) / pre-defined error field (`0x1003`) are exactly the
//! objects [`Node::raise_emergency`](crate::node::Node::raise_emergency) mirrors
//! into, so a node built this way is ready for both LSS and EMCY out of the box.
//!
//! ```
//! use canopen_rs::{LssAddress, Node, NodeId, ObjectDictionary};
//! use canopen_rs::standard::StandardObjects;
//!
//! let identity = LssAddress { vendor_id: 0x0000_012E, product_code: 0x4001, revision_number: 1, serial_number: 0xC0FFEE };
//! let mut od = ObjectDictionary::<16>::new();
//! StandardObjects::new(0x0004_0192, identity)
//!     .with_heartbeat(1000)
//!     .with_error_history(4)
//!     .insert_into(&mut od)
//!     .unwrap();
//!
//! let mut node = Node::new(NodeId::new(0x10).unwrap(), od);
//! // 0x1018 is populated, so the same identity enables LSS/Fastscan…
//! node.enable_lss(identity);
//! ```

use crate::datatypes::{ByteString, Value, MAX_STRING_LEN};
use crate::lss::LssAddress;
use crate::object_dictionary::{Address, Entry, ObjectDictionary};
use crate::Result;

/// Builder for the mandatory CiA 301 communication-profile objects.
///
/// The objects it inserts are all node-id-independent, so it can populate a
/// dictionary before a node-id is known (e.g. an LSS-unconfigured node). COB-ID
/// objects that depend on the node-id (`0x1014` EMCY, `0x1005` SYNC) are left to
/// the application, which knows the assigned id.
#[derive(Debug, Clone, Copy)]
pub struct StandardObjects {
    /// Object `0x1000` — device type (profile and type information).
    pub device_type: u32,
    /// Object `0x1018` — the identity record (vendor / product / revision /
    /// serial), the same value an LSS master matches.
    pub identity: LssAddress,
    /// Object `0x1017` — producer heartbeat time, in milliseconds (`0` disables).
    pub producer_heartbeat_ms: u16,
    /// Number of `0x1003` pre-defined-error-field sub-entries to pre-allocate
    /// (`0` omits the object entirely).
    pub error_history_len: u8,
    /// Object `0x1008` — the manufacturer device name (a `VISIBLE_STRING`), or
    /// `None` to omit it.
    pub device_name: Option<ByteString>,
}

impl StandardObjects {
    /// A baseline with the given device type and identity, no heartbeat, and no
    /// pre-defined error field. Refine with [`with_heartbeat`](Self::with_heartbeat)
    /// and [`with_error_history`](Self::with_error_history).
    pub const fn new(device_type: u32, identity: LssAddress) -> Self {
        Self {
            device_type,
            identity,
            producer_heartbeat_ms: 0,
            error_history_len: 0,
            device_name: None,
        }
    }

    /// Set the producer heartbeat time (`0x1017`), in milliseconds.
    pub fn with_heartbeat(mut self, ms: u16) -> Self {
        self.producer_heartbeat_ms = ms;
        self
    }

    /// Pre-allocate `entries` sub-entries of the pre-defined error field
    /// (`0x1003`) so [`Node::raise_emergency`](crate::node::Node::raise_emergency)
    /// can mirror the error history into the dictionary.
    pub fn with_error_history(mut self, entries: u8) -> Self {
        self.error_history_len = entries;
        self
    }

    /// Set the device name (`0x1008`, a `VISIBLE_STRING`), truncated to
    /// [`MAX_STRING_LEN`] bytes.
    pub fn with_device_name(mut self, name: &str) -> Self {
        let end = name.len().min(MAX_STRING_LEN);
        // `end` is a byte length ≤ MAX_STRING_LEN, so from_bytes cannot fail.
        self.device_name = ByteString::from_bytes(&name.as_bytes()[..end]).ok();
        self
    }

    /// Insert the objects into `od`.
    ///
    /// Returns [`Error::DictionaryFull`](crate::Error::DictionaryFull) if the
    /// dictionary lacks capacity — it needs room for the 9 base objects, the
    /// optional device name, and the error-history entries plus their count
    /// sub-index. Existing objects at these addresses are overwritten.
    pub fn insert_into<const N: usize>(&self, od: &mut ObjectDictionary<N>) -> Result<()> {
        // 0x1000 device type, 0x1001 error register, 0x1017 heartbeat time.
        od.insert(
            Address::new(0x1000, 0),
            Entry::constant(Value::Unsigned32(self.device_type)),
        )?;
        od.insert(Address::new(0x1001, 0), Entry::ro(Value::Unsigned8(0)))?;
        od.insert(
            Address::new(0x1017, 0),
            Entry::rw(Value::Unsigned16(self.producer_heartbeat_ms)),
        )?;

        // 0x1018 identity record: sub 0 = highest sub-index (4), then the four
        // 32-bit identity values.
        od.insert(
            Address::new(0x1018, 0),
            Entry::constant(Value::Unsigned8(4)),
        )?;
        od.insert(
            Address::new(0x1018, 1),
            Entry::constant(Value::Unsigned32(self.identity.vendor_id)),
        )?;
        od.insert(
            Address::new(0x1018, 2),
            Entry::constant(Value::Unsigned32(self.identity.product_code)),
        )?;
        od.insert(
            Address::new(0x1018, 3),
            Entry::constant(Value::Unsigned32(self.identity.revision_number)),
        )?;
        od.insert(
            Address::new(0x1018, 4),
            Entry::constant(Value::Unsigned32(self.identity.serial_number)),
        )?;

        // 0x1008 manufacturer device name (optional).
        if let Some(name) = self.device_name {
            od.insert(
                Address::new(0x1008, 0),
                Entry::constant(Value::VisibleString(name)),
            )?;
        }

        // 0x1003 pre-defined error field: sub 0 is the writable count (a master
        // writes 0 to clear), subs 1.. hold the recorded error codes.
        if self.error_history_len > 0 {
            od.insert(Address::new(0x1003, 0), Entry::rw(Value::Unsigned8(0)))?;
            for sub in 1..=self.error_history_len {
                od.insert(Address::new(0x1003, sub), Entry::ro(Value::Unsigned32(0)))?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Error;

    fn identity() -> LssAddress {
        LssAddress {
            vendor_id: 0x0000_012E,
            product_code: 0x4001_0002,
            revision_number: 0x0000_0003,
            serial_number: 0x00C0_FFEE,
        }
    }

    #[test]
    fn inserts_the_mandatory_baseline() {
        let mut od = ObjectDictionary::<16>::new();
        StandardObjects::new(0x0004_0192, identity())
            .with_heartbeat(1000)
            .insert_into(&mut od)
            .unwrap();

        assert_eq!(
            od.read(Address::new(0x1000, 0)).unwrap(),
            Value::Unsigned32(0x0004_0192)
        );
        assert_eq!(
            od.read(Address::new(0x1001, 0)).unwrap(),
            Value::Unsigned8(0)
        );
        assert_eq!(
            od.read(Address::new(0x1017, 0)).unwrap(),
            Value::Unsigned16(1000)
        );
    }

    #[test]
    fn identity_record_matches_the_lss_address() {
        let mut od = ObjectDictionary::<16>::new();
        let id = identity();
        StandardObjects::new(0, id).insert_into(&mut od).unwrap();

        assert_eq!(
            od.read(Address::new(0x1018, 0)).unwrap(),
            Value::Unsigned8(4) // highest sub-index
        );
        assert_eq!(
            od.read(Address::new(0x1018, 1)).unwrap(),
            Value::Unsigned32(id.vendor_id)
        );
        assert_eq!(
            od.read(Address::new(0x1018, 2)).unwrap(),
            Value::Unsigned32(id.product_code)
        );
        assert_eq!(
            od.read(Address::new(0x1018, 3)).unwrap(),
            Value::Unsigned32(id.revision_number)
        );
        assert_eq!(
            od.read(Address::new(0x1018, 4)).unwrap(),
            Value::Unsigned32(id.serial_number)
        );
    }

    #[test]
    fn error_history_is_optional_and_sized() {
        // None by default.
        let mut od = ObjectDictionary::<16>::new();
        StandardObjects::new(0, identity())
            .insert_into(&mut od)
            .unwrap();
        assert!(od.entry(Address::new(0x1003, 0)).is_none());

        // Requested: count sub-index plus one entry each.
        let mut od = ObjectDictionary::<16>::new();
        StandardObjects::new(0, identity())
            .with_error_history(3)
            .insert_into(&mut od)
            .unwrap();
        assert_eq!(
            od.read(Address::new(0x1003, 0)).unwrap(),
            Value::Unsigned8(0)
        );
        assert!(od.entry(Address::new(0x1003, 3)).is_some());
        assert!(od.entry(Address::new(0x1003, 4)).is_none());
    }

    #[test]
    fn node_built_from_standard_objects_mirrors_emergency() {
        use crate::emcy::{error_code, ErrorRegister};
        use crate::node::Node;
        use crate::types::NodeId;

        let mut od = ObjectDictionary::<16>::new();
        StandardObjects::new(0x0004_0192, identity())
            .with_heartbeat(1000)
            .with_error_history(4)
            .insert_into(&mut od)
            .unwrap();
        let mut node = Node::new(NodeId::new(0x10).unwrap(), od);

        node.raise_emergency(
            error_code::VOLTAGE,
            ErrorRegister(ErrorRegister::VOLTAGE),
            [0; 5],
        );
        // The builder's 0x1001 / 0x1003 objects received the mirror, so a master
        // can read the register and the recorded code over SDO.
        assert_eq!(
            node.od().read(Address::new(0x1001, 0)).unwrap(),
            Value::Unsigned8(ErrorRegister::GENERIC | ErrorRegister::VOLTAGE)
        );
        assert_eq!(
            node.od().read(Address::new(0x1003, 0)).unwrap(),
            Value::Unsigned8(1)
        );
        assert_eq!(
            node.od().read(Address::new(0x1003, 1)).unwrap(),
            Value::Unsigned32(u32::from(error_code::VOLTAGE))
        );
    }

    #[test]
    fn device_name_is_optional_visible_string() {
        // Omitted by default.
        let mut od = ObjectDictionary::<16>::new();
        StandardObjects::new(0, identity())
            .insert_into(&mut od)
            .unwrap();
        assert!(od.entry(Address::new(0x1008, 0)).is_none());

        // Present when set, as a VISIBLE_STRING.
        let mut od = ObjectDictionary::<16>::new();
        StandardObjects::new(0, identity())
            .with_device_name("canopen-rs node")
            .insert_into(&mut od)
            .unwrap();
        match od.read(Address::new(0x1008, 0)).unwrap() {
            Value::VisibleString(bs) => assert_eq!(bs.as_str(), Some("canopen-rs node")),
            other => panic!("expected a VISIBLE_STRING, got {other:?}"),
        }
    }

    #[test]
    fn reports_dictionary_full() {
        // The baseline needs 9 objects; a smaller dictionary overflows.
        let mut od = ObjectDictionary::<4>::new();
        assert_eq!(
            StandardObjects::new(0, identity()).insert_into(&mut od),
            Err(Error::DictionaryFull)
        );
    }
}
