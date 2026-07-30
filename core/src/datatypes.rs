//! CANopen basic data types and their little-endian value encoding.
//!
//! CANopen is little-endian on the wire (CiA 301 §7.1). This module models
//! the numeric basic data types and converts between typed [`Value`]s and
//! their byte encoding. Variable-length types (strings, `DOMAIN`) arrive with
//! segmented/block transfer and are added later.

use crate::{Error, Result};

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
}

impl DataType {
    /// The encoded size of a value of this type, in bytes.
    pub const fn size(self) -> usize {
        match self {
            DataType::Boolean | DataType::Integer8 | DataType::Unsigned8 => 1,
            DataType::Integer16 | DataType::Unsigned16 => 2,
            DataType::Integer32 | DataType::Unsigned32 | DataType::Real32 => 4,
            DataType::Integer64 | DataType::Unsigned64 | DataType::Real64 => 8,
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
        }
    }

    /// The encoded size of this value, in bytes.
    pub const fn size(&self) -> usize {
        self.data_type().size()
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
        }
        Ok(n)
    }

    /// Decode a little-endian value of `data_type` from `bytes`.
    ///
    /// `bytes` must be exactly [`DataType::size`] long, else
    /// [`Error::BadLength`].
    pub fn decode_le(data_type: DataType, bytes: &[u8]) -> Result<Value> {
        if bytes.len() != data_type.size() {
            return Err(Error::BadLength);
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
        assert_eq!(Value::Unsigned32(0).encode_le(&mut buf), Err(Error::BadLength));
    }

    #[test]
    fn decode_rejects_wrong_length() {
        assert_eq!(
            Value::decode_le(DataType::Unsigned32, &[0, 0]),
            Err(Error::BadLength)
        );
    }
}
