//! Foundational CANopen types shared across the stack.

use core::fmt;

/// Result alias for fallible `canopen-rs` operations.
pub type Result<T> = core::result::Result<T, Error>;

/// Errors surfaced by the `canopen-rs` core.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// A frame or buffer was too short or too long for the operation.
    BadLength,
    /// A value did not fit the target CANopen data type.
    Overflow,
    /// No object exists at the requested (index, subindex).
    ObjectNotFound,
    /// A node id outside the valid `1..=127` range was supplied.
    InvalidNodeId,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msg = match self {
            Error::BadLength => "frame or buffer length invalid for operation",
            Error::Overflow => "value does not fit the target CANopen data type",
            Error::ObjectNotFound => "no object at the requested index/subindex",
            Error::InvalidNodeId => "node id out of range (must be 1..=127)",
        };
        f.write_str(msg)
    }
}

#[cfg(feature = "std")]
extern crate std;
#[cfg(feature = "std")]
impl std::error::Error for Error {}

/// A CANopen node identifier.
///
/// Valid *device* node ids are `1..=127`. The value `0` is reserved for
/// broadcast in NMT and LSS; use [`NodeId::BROADCAST`] for it and
/// [`NodeId::new`] for a checked device id.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeId(u8);

impl NodeId {
    /// The broadcast node id (`0`), used by NMT and LSS.
    pub const BROADCAST: NodeId = NodeId(0);

    /// Create a checked device node id in `1..=127`.
    ///
    /// Returns [`Error::InvalidNodeId`] for `0` or any value above `127`.
    pub fn new(id: u8) -> Result<Self> {
        if (1..=127).contains(&id) {
            Ok(NodeId(id))
        } else {
            Err(Error::InvalidNodeId)
        }
    }

    /// The raw node id byte.
    pub const fn raw(self) -> u8 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_id_accepts_valid_range() {
        assert_eq!(NodeId::new(1).unwrap().raw(), 1);
        assert_eq!(NodeId::new(127).unwrap().raw(), 127);
    }

    #[test]
    fn node_id_rejects_out_of_range() {
        assert_eq!(NodeId::new(0), Err(Error::InvalidNodeId));
        assert_eq!(NodeId::new(128), Err(Error::InvalidNodeId));
    }

    #[test]
    fn broadcast_is_zero() {
        assert_eq!(NodeId::BROADCAST.raw(), 0);
    }
}
