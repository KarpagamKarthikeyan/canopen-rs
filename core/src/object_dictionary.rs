//! The CANopen object dictionary (OD) model.
//!
//! Every CANopen device exposes its data as a dictionary of objects addressed
//! by a 16-bit `index` and an 8-bit `subindex` (CiA 301 §7.4). Communication
//! objects live in `0x1000..=0x1FFF`, manufacturer-specific objects in
//! `0x2000..=0x5FFF`, and standardised device-profile objects in
//! `0x6000..=0x9FFF`.
//!
//! [`ObjectDictionary`] is backed by a [`heapless::LinearMap`] with a
//! compile-time capacity, so the same type serves both `no_std` embedded
//! nodes and host tooling with no allocator.
//!
//! ```
//! use canopen_rs::{Address, Entry, ObjectDictionary, Value};
//!
//! let mut od = ObjectDictionary::<8>::new();
//! // 0x1017 producer heartbeat time — a read/write UNSIGNED16.
//! od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000))).unwrap();
//!
//! assert_eq!(od.read(Address::new(0x1017, 0)).unwrap(), Value::Unsigned16(1000));
//! od.write(Address::new(0x1017, 0), Value::Unsigned16(500)).unwrap();
//! assert_eq!(od.read(Address::new(0x1017, 0)).unwrap(), Value::Unsigned16(500));
//!
//! // Access rights and data types are enforced.
//! od.insert(Address::new(0x1000, 0), Entry::constant(Value::Unsigned32(0x192))).unwrap();
//! assert!(od.write(Address::new(0x1000, 0), Value::Unsigned32(1)).is_err()); // read-only
//! ```

use heapless::LinearMap;

use crate::datatypes::Value;
use crate::{Error, Result};

/// The address of an object dictionary entry: a 16-bit index and 8-bit
/// subindex.
///
/// Subindex `0` conventionally holds the "number of entries" for `RECORD` and
/// `ARRAY` objects; simple `VAR` objects use subindex `0` for the value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Address {
    /// 16-bit object index.
    pub index: u16,
    /// 8-bit subindex.
    pub subindex: u8,
}

impl Address {
    /// Construct an address from an index and subindex.
    pub const fn new(index: u16, subindex: u8) -> Self {
        Self { index, subindex }
    }
}

/// Access rights for an object dictionary entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessType {
    /// Read-only.
    Ro,
    /// Write-only.
    Wo,
    /// Read/write.
    Rw,
    /// Read-only constant (value fixed at design time).
    Const,
}

impl AccessType {
    /// Whether an SDO/PDO read is permitted.
    pub const fn is_readable(self) -> bool {
        matches!(self, AccessType::Ro | AccessType::Rw | AccessType::Const)
    }

    /// Whether an SDO/PDO write is permitted.
    pub const fn is_writable(self) -> bool {
        matches!(self, AccessType::Wo | AccessType::Rw)
    }
}

/// A single object dictionary entry: a typed value and its access rights.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Entry {
    /// The current value (its variant fixes the object's data type).
    pub value: Value,
    /// The entry's access rights.
    pub access: AccessType,
}

impl Entry {
    /// A read/write entry.
    pub const fn rw(value: Value) -> Self {
        Self {
            value,
            access: AccessType::Rw,
        }
    }

    /// A read-only entry.
    pub const fn ro(value: Value) -> Self {
        Self {
            value,
            access: AccessType::Ro,
        }
    }

    /// A read-only constant entry.
    pub const fn constant(value: Value) -> Self {
        Self {
            value,
            access: AccessType::Const,
        }
    }
}

/// An object dictionary holding up to `N` entries, addressed by
/// `(index, subindex)`.
#[derive(Debug)]
pub struct ObjectDictionary<const N: usize> {
    entries: LinearMap<Address, Entry, N>,
}

impl<const N: usize> Default for ObjectDictionary<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ObjectDictionary<N> {
    /// Create an empty object dictionary.
    pub const fn new() -> Self {
        Self {
            entries: LinearMap::new(),
        }
    }

    /// Insert or replace the entry at `addr`.
    ///
    /// Returns [`Error::DictionaryFull`] when capacity `N` is exhausted.
    pub fn insert(&mut self, addr: Address, entry: Entry) -> Result<()> {
        self.entries
            .insert(addr, entry)
            .map(|_| ())
            .map_err(|_| Error::DictionaryFull)
    }

    /// Borrow the raw entry at `addr`, ignoring access rights.
    pub fn entry(&self, addr: Address) -> Option<&Entry> {
        self.entries.get(&addr)
    }

    /// Read the value at `addr`, honouring access rights.
    ///
    /// Returns [`Error::ObjectNotFound`] if absent, or [`Error::WriteOnly`] if
    /// the object is not readable.
    pub fn read(&self, addr: Address) -> Result<Value> {
        let entry = self.entries.get(&addr).ok_or(Error::ObjectNotFound)?;
        if !entry.access.is_readable() {
            return Err(Error::WriteOnly);
        }
        Ok(entry.value)
    }

    /// Write `value` at `addr`, honouring access rights and the object's data
    /// type.
    ///
    /// Returns [`Error::ObjectNotFound`] if absent, [`Error::ReadOnly`] if not
    /// writable, or [`Error::TypeMismatch`] if `value`'s type differs from the
    /// stored object's type.
    pub fn write(&mut self, addr: Address, value: Value) -> Result<()> {
        let entry = self.entries.get_mut(&addr).ok_or(Error::ObjectNotFound)?;
        if !entry.access.is_writable() {
            return Err(Error::ReadOnly);
        }
        if entry.value.data_type() != value.data_type() {
            return Err(Error::TypeMismatch);
        }
        entry.value = value;
        Ok(())
    }

    /// Set the value at `addr` from the **device side**, ignoring master access
    /// rights — those govern SDO access, not the application, so this can update
    /// a master-read-only object (e.g. publish process data into a TPDO source,
    /// or the node's error register `0x1001`). The object must exist and the
    /// value's type must match.
    ///
    /// Returns [`Error::ObjectNotFound`] if absent or [`Error::TypeMismatch`] on
    /// a type mismatch.
    pub fn set(&mut self, addr: Address, value: Value) -> Result<()> {
        let entry = self.entries.get_mut(&addr).ok_or(Error::ObjectNotFound)?;
        if entry.value.data_type() != value.data_type() {
            return Err(Error::TypeMismatch);
        }
        entry.value = value;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datatypes::Value;

    fn od() -> ObjectDictionary<8> {
        let mut od = ObjectDictionary::new();
        // 0x1000 Device type — read-only constant.
        od.insert(
            Address::new(0x1000, 0),
            Entry::constant(Value::Unsigned32(0x0000_0192)),
        )
        .unwrap();
        // 0x1017 Producer heartbeat time — read/write.
        od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000)))
            .unwrap();
        od
    }

    #[test]
    fn read_returns_stored_value() {
        assert_eq!(
            od().read(Address::new(0x1000, 0)).unwrap(),
            Value::Unsigned32(0x192)
        );
    }

    #[test]
    fn read_missing_object_errors() {
        assert_eq!(
            od().read(Address::new(0x9999, 0)),
            Err(Error::ObjectNotFound)
        );
    }

    #[test]
    fn write_updates_value() {
        let mut od = od();
        od.write(Address::new(0x1017, 0), Value::Unsigned16(500))
            .unwrap();
        assert_eq!(
            od.read(Address::new(0x1017, 0)).unwrap(),
            Value::Unsigned16(500)
        );
    }

    #[test]
    fn write_read_only_object_errors() {
        let mut od = od();
        assert_eq!(
            od.write(Address::new(0x1000, 0), Value::Unsigned32(1)),
            Err(Error::ReadOnly)
        );
    }

    #[test]
    fn write_wrong_type_errors() {
        let mut od = od();
        assert_eq!(
            od.write(Address::new(0x1017, 0), Value::Unsigned32(1)),
            Err(Error::TypeMismatch)
        );
    }

    #[test]
    fn dictionary_full_errors() {
        let mut od: ObjectDictionary<1> = ObjectDictionary::new();
        od.insert(Address::new(0x2000, 0), Entry::rw(Value::Unsigned8(1)))
            .unwrap();
        assert_eq!(
            od.insert(Address::new(0x2001, 0), Entry::rw(Value::Unsigned8(2))),
            Err(Error::DictionaryFull)
        );
    }
}
