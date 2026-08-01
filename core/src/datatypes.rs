//! CANopen basic data types and their little-endian value encoding.
//!
//! CANopen is little-endian on the wire (CiA 301 §7.1). This module models the
//! numeric basic data types plus the variable-length ones — `VISIBLE_STRING`,
//! `OCTET_STRING`, and `DOMAIN`, carried by a bounded [`ByteString`] — and
//! converts between typed [`Value`]s and their byte encoding. Variable-length
//! values (over four bytes) travel by segmented SDO transfer.
//!
//! ```
//! use canopen_rs::{DataType, Value};
//!
//! let mut buf = [0u8; 8];
//! let n = Value::Unsigned32(0xDEAD_BEEF).encode_le(&mut buf).unwrap();
//! assert_eq!(&buf[..n], &[0xEF, 0xBE, 0xAD, 0xDE]); // little-endian
//!
//! let value = Value::decode_le(DataType::Unsigned32, &buf[..n]).unwrap();
//! assert_eq!(value, Value::Unsigned32(0xDEAD_BEEF));
//! ```

use crate::{Error, Result};

/// The maximum length, in bytes, of a variable-length value (`VISIBLE_STRING`,
/// `OCTET_STRING`, `DOMAIN`).
///
/// Chosen to keep [`Value`] `Copy` and reasonably small while covering the
/// typical string objects (device name `0x1008`, versions `0x1009` / `0x100A`).
/// It also bounds the SDO segmented-transfer buffers, so it is the largest
/// object value the sans-I/O SDO server and client will transfer.
pub const MAX_STRING_LEN: usize = 32;

/// A bounded, `Copy` byte string backing the variable-length CANopen types
/// (`VISIBLE_STRING`, `OCTET_STRING`, `DOMAIN`): up to [`MAX_STRING_LEN`] bytes.
#[derive(Clone, Copy)]
pub struct ByteString {
    data: [u8; MAX_STRING_LEN],
    len: u8,
}

impl ByteString {
    /// An empty byte string.
    pub const fn new() -> Self {
        Self {
            data: [0; MAX_STRING_LEN],
            len: 0,
        }
    }

    /// Build from a byte slice, returning [`Error::BadLength`] if it is longer
    /// than [`MAX_STRING_LEN`].
    pub fn from_bytes(bytes: &[u8]) -> Result<Self> {
        if bytes.len() > MAX_STRING_LEN {
            return Err(Error::BadLength);
        }
        let mut data = [0u8; MAX_STRING_LEN];
        data[..bytes.len()].copy_from_slice(bytes);
        Ok(Self {
            data,
            len: bytes.len() as u8,
        })
    }

    /// Build from a string slice (a `VISIBLE_STRING`), returning
    /// [`Error::BadLength`] if it is longer than [`MAX_STRING_LEN`].
    // Inherent (fallible, bounded) constructor, not the `FromStr` trait.
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Result<Self> {
        Self::from_bytes(s.as_bytes())
    }

    /// The bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// Interpret the bytes as UTF-8 text, if valid.
    pub fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(self.as_bytes()).ok()
    }

    /// The length in bytes.
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Whether the string is empty.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Default for ByteString {
    fn default() -> Self {
        Self::new()
    }
}

// Equality and Debug compare/show only the meaningful bytes, ignoring the
// unused tail of the fixed buffer.
impl PartialEq for ByteString {
    fn eq(&self, other: &Self) -> bool {
        self.as_bytes() == other.as_bytes()
    }
}

impl Eq for ByteString {}

impl core::fmt::Debug for ByteString {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.as_str() {
            Some(s) => write!(f, "ByteString({s:?})"),
            None => write!(f, "ByteString({:?})", self.as_bytes()),
        }
    }
}

/// A CANopen basic data type.
///
/// The discriminant is the CiA 301 data type index (the object index under
/// which the type is described, e.g. `UNSIGNED32` is `0x0007`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum DataType {
    /// `BOOLEAN` (0x0001), 1 byte.
    Boolean = 0x01,
    /// `INTEGER8` (0x0002), 1 byte.
    Integer8 = 0x02,
    /// `INTEGER16` (0x0003), 2 bytes.
    Integer16 = 0x03,
    /// `INTEGER32` (0x0004), 4 bytes.
    Integer32 = 0x04,
    /// `UNSIGNED8` (0x0005), 1 byte.
    Unsigned8 = 0x05,
    /// `UNSIGNED16` (0x0006), 2 bytes.
    Unsigned16 = 0x06,
    /// `UNSIGNED32` (0x0007), 4 bytes.
    Unsigned32 = 0x07,
    /// `REAL32` (0x0008), 4 bytes.
    Real32 = 0x08,
    /// `REAL64` (0x0011), 8 bytes.
    Real64 = 0x11,
    /// `INTEGER64` (0x0015), 8 bytes.
    Integer64 = 0x15,
    /// `UNSIGNED64` (0x001B), 8 bytes.
    Unsigned64 = 0x1B,
    /// `VISIBLE_STRING` (0x0009), variable length (ASCII/UTF-8 text).
    VisibleString = 0x09,
    /// `OCTET_STRING` (0x000A), variable length (arbitrary bytes).
    OctetString = 0x0A,
    /// `DOMAIN` (0x000F), variable length (arbitrary bytes).
    Domain = 0x0F,
}

impl DataType {
    /// The CiA 301 data type index (e.g. `0x0007` for `UNSIGNED32`).
    pub const fn index(self) -> u16 {
        self as u16
    }

    /// The basic data type for a CiA 301 data type index, or `None` for an
    /// index this crate does not model.
    pub const fn from_index(index: u16) -> Option<Self> {
        Some(match index {
            0x01 => DataType::Boolean,
            0x02 => DataType::Integer8,
            0x03 => DataType::Integer16,
            0x04 => DataType::Integer32,
            0x05 => DataType::Unsigned8,
            0x06 => DataType::Unsigned16,
            0x07 => DataType::Unsigned32,
            0x08 => DataType::Real32,
            0x09 => DataType::VisibleString,
            0x0A => DataType::OctetString,
            0x0F => DataType::Domain,
            0x11 => DataType::Real64,
            0x15 => DataType::Integer64,
            0x1B => DataType::Unsigned64,
            _ => return None,
        })
    }

    /// The fixed encoded size of a value of this type in bytes, or `None` for a
    /// variable-length type (`VISIBLE_STRING`, `OCTET_STRING`, `DOMAIN`).
    pub const fn fixed_size(self) -> Option<usize> {
        Some(match self {
            DataType::Boolean | DataType::Integer8 | DataType::Unsigned8 => 1,
            DataType::Integer16 | DataType::Unsigned16 => 2,
            DataType::Integer32 | DataType::Unsigned32 | DataType::Real32 => 4,
            DataType::Integer64 | DataType::Unsigned64 | DataType::Real64 => 8,
            DataType::VisibleString | DataType::OctetString | DataType::Domain => return None,
        })
    }

    /// Whether this is a variable-length type.
    pub const fn is_variable(self) -> bool {
        self.fixed_size().is_none()
    }

    /// The fixed encoded size of a value of this type in bytes, or `0` for a
    /// variable-length type (whose size depends on the value — see
    /// [`fixed_size`](DataType::fixed_size)).
    pub const fn size(self) -> usize {
        match self.fixed_size() {
            Some(n) => n,
            None => 0,
        }
    }
}

/// A typed CANopen value.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum Value {
    /// A `BOOLEAN`.
    Boolean(bool),
    /// An `INTEGER8`.
    Integer8(i8),
    /// An `INTEGER16`.
    Integer16(i16),
    /// An `INTEGER32`.
    Integer32(i32),
    /// An `UNSIGNED8`.
    Unsigned8(u8),
    /// An `UNSIGNED16`.
    Unsigned16(u16),
    /// An `UNSIGNED32`.
    Unsigned32(u32),
    /// A `REAL32`.
    Real32(f32),
    /// A `REAL64`.
    Real64(f64),
    /// An `INTEGER64`.
    Integer64(i64),
    /// An `UNSIGNED64`.
    Unsigned64(u64),
    /// A `VISIBLE_STRING` (ASCII/UTF-8 text).
    VisibleString(ByteString),
    /// An `OCTET_STRING` (arbitrary bytes).
    OctetString(ByteString),
    /// A `DOMAIN` (arbitrary bytes).
    Domain(ByteString),
}

impl Value {
    /// The data type of this value.
    pub const fn data_type(&self) -> DataType {
        match self {
            Value::Boolean(_) => DataType::Boolean,
            Value::Integer8(_) => DataType::Integer8,
            Value::Integer16(_) => DataType::Integer16,
            Value::Integer32(_) => DataType::Integer32,
            Value::Unsigned8(_) => DataType::Unsigned8,
            Value::Unsigned16(_) => DataType::Unsigned16,
            Value::Unsigned32(_) => DataType::Unsigned32,
            Value::Real32(_) => DataType::Real32,
            Value::Real64(_) => DataType::Real64,
            Value::Integer64(_) => DataType::Integer64,
            Value::Unsigned64(_) => DataType::Unsigned64,
            Value::VisibleString(_) => DataType::VisibleString,
            Value::OctetString(_) => DataType::OctetString,
            Value::Domain(_) => DataType::Domain,
        }
    }

    /// The encoded size of this value, in bytes. For a variable-length value
    /// this is its current content length.
    pub const fn size(&self) -> usize {
        match self {
            Value::VisibleString(s) | Value::OctetString(s) | Value::Domain(s) => s.len(),
            _ => self.data_type().size(),
        }
    }

    /// Encode this value little-endian into `buf`, returning the number of
    /// bytes written.
    ///
    /// Returns [`Error::BadLength`] if `buf` is smaller than [`Value::size`].
    pub fn encode_le(&self, buf: &mut [u8]) -> Result<usize> {
        let n = self.size();
        if buf.len() < n {
            return Err(Error::BadLength);
        }
        match self {
            Value::Boolean(v) => buf[0] = *v as u8,
            Value::Integer8(v) => buf[0] = *v as u8,
            Value::Unsigned8(v) => buf[0] = *v,
            Value::Integer16(v) => buf[..2].copy_from_slice(&v.to_le_bytes()),
            Value::Unsigned16(v) => buf[..2].copy_from_slice(&v.to_le_bytes()),
            Value::Integer32(v) => buf[..4].copy_from_slice(&v.to_le_bytes()),
            Value::Unsigned32(v) => buf[..4].copy_from_slice(&v.to_le_bytes()),
            Value::Real32(v) => buf[..4].copy_from_slice(&v.to_le_bytes()),
            Value::Integer64(v) => buf[..8].copy_from_slice(&v.to_le_bytes()),
            Value::Unsigned64(v) => buf[..8].copy_from_slice(&v.to_le_bytes()),
            Value::Real64(v) => buf[..8].copy_from_slice(&v.to_le_bytes()),
            Value::VisibleString(s) | Value::OctetString(s) | Value::Domain(s) => {
                buf[..n].copy_from_slice(s.as_bytes())
            }
        }
        Ok(n)
    }

    /// Decode a little-endian value of `data_type` from `bytes`.
    ///
    /// For a fixed-size type `bytes` must be exactly [`DataType::fixed_size`]
    /// long; for a variable-length type it may be any length up to
    /// [`MAX_STRING_LEN`]. Otherwise returns [`Error::BadLength`].
    pub fn decode_le(data_type: DataType, bytes: &[u8]) -> Result<Value> {
        match data_type.fixed_size() {
            Some(fixed) if bytes.len() != fixed => return Err(Error::BadLength),
            None if bytes.len() > MAX_STRING_LEN => return Err(Error::BadLength),
            _ => {}
        }
        let v = match data_type {
            DataType::Boolean => Value::Boolean(bytes[0] != 0),
            DataType::Integer8 => Value::Integer8(bytes[0] as i8),
            DataType::Unsigned8 => Value::Unsigned8(bytes[0]),
            DataType::Integer16 => Value::Integer16(i16::from_le_bytes([bytes[0], bytes[1]])),
            DataType::Unsigned16 => Value::Unsigned16(u16::from_le_bytes([bytes[0], bytes[1]])),
            DataType::Integer32 => {
                Value::Integer32(i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            }
            DataType::Unsigned32 => {
                Value::Unsigned32(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            }
            DataType::Real32 => {
                Value::Real32(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            }
            DataType::Integer64 => Value::Integer64(i64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            DataType::Unsigned64 => Value::Unsigned64(u64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            DataType::Real64 => Value::Real64(f64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            DataType::VisibleString => Value::VisibleString(ByteString::from_bytes(bytes)?),
            DataType::OctetString => Value::OctetString(ByteString::from_bytes(bytes)?),
            DataType::Domain => Value::Domain(ByteString::from_bytes(bytes)?),
        };
        Ok(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes_match_spec() {
        assert_eq!(DataType::Boolean.size(), 1);
        assert_eq!(DataType::Unsigned16.size(), 2);
        assert_eq!(DataType::Unsigned32.size(), 4);
        assert_eq!(DataType::Real64.size(), 8);
    }

    #[test]
    fn data_type_index_roundtrips() {
        for dt in [
            DataType::Boolean,
            DataType::Unsigned32,
            DataType::Real64,
            DataType::Integer64,
            DataType::Unsigned64,
            DataType::VisibleString,
            DataType::OctetString,
            DataType::Domain,
        ] {
            assert_eq!(DataType::from_index(dt.index()), Some(dt));
        }
        assert_eq!(DataType::Unsigned32.index(), 0x0007);
        assert_eq!(DataType::VisibleString.index(), 0x0009);
        // A genuinely unmodelled type (e.g. TIME_OF_DAY 0x0C) is rejected.
        assert_eq!(DataType::from_index(0x0C), None);
    }

    #[test]
    fn variable_length_types_have_no_fixed_size() {
        for dt in [
            DataType::VisibleString,
            DataType::OctetString,
            DataType::Domain,
        ] {
            assert!(dt.is_variable());
            assert_eq!(dt.fixed_size(), None);
            assert_eq!(dt.size(), 0);
        }
        assert!(!DataType::Unsigned32.is_variable());
        assert_eq!(DataType::Unsigned32.fixed_size(), Some(4));
    }

    #[test]
    fn string_values_round_trip() {
        let mut buf = [0u8; MAX_STRING_LEN];
        for v in [
            Value::VisibleString(ByteString::from_str("canopen-rs").unwrap()),
            Value::OctetString(ByteString::from_bytes(&[0, 1, 2, 0xFF]).unwrap()),
            Value::Domain(ByteString::from_bytes(&[]).unwrap()), // empty
        ] {
            let n = v.encode_le(&mut buf).unwrap();
            assert_eq!(n, v.size());
            let back = Value::decode_le(v.data_type(), &buf[..n]).unwrap();
            assert_eq!(v, back);
        }
    }

    #[test]
    fn bytestring_rejects_oversized_input() {
        let too_long = [0u8; MAX_STRING_LEN + 1];
        assert_eq!(ByteString::from_bytes(&too_long), Err(Error::BadLength));
        assert_eq!(
            Value::decode_le(DataType::VisibleString, &too_long),
            Err(Error::BadLength)
        );
    }

    #[test]
    fn bytestring_equality_ignores_unused_tail() {
        let a = ByteString::from_str("hi").unwrap();
        let b = ByteString::from_bytes(b"hi").unwrap();
        assert_eq!(a, b);
        assert_eq!(a.as_str(), Some("hi"));
        assert_eq!(a.len(), 2);
    }

    #[test]
    fn u32_is_little_endian() {
        let mut buf = [0u8; 8];
        let n = Value::Unsigned32(0x1234_5678).encode_le(&mut buf).unwrap();
        assert_eq!(n, 4);
        assert_eq!(&buf[..4], &[0x78, 0x56, 0x34, 0x12]);
    }

    #[test]
    fn decode_roundtrips() {
        let cases = [
            Value::Boolean(true),
            Value::Integer8(-5),
            Value::Unsigned16(0xBEEF),
            Value::Integer32(-123456),
            Value::Unsigned32(0xDEAD_BEEF),
            Value::Unsigned64(0x0102_0304_0506_0708),
        ];
        let mut buf = [0u8; 8];
        for v in cases {
            let n = v.encode_le(&mut buf).unwrap();
            let back = Value::decode_le(v.data_type(), &buf[..n]).unwrap();
            assert_eq!(v, back);
        }
    }

    #[test]
    fn encode_rejects_short_buffer() {
        let mut buf = [0u8; 2];
        assert_eq!(
            Value::Unsigned32(0).encode_le(&mut buf),
            Err(Error::BadLength)
        );
    }

    #[test]
    fn decode_rejects_wrong_length() {
        assert_eq!(
            Value::decode_le(DataType::Unsigned32, &[0, 0]),
            Err(Error::BadLength)
        );
    }
}
