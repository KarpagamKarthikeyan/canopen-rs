//! Process Data Object (PDO) — real-time process data (CiA 301 §7.2.2).
//!
//! PDOs carry mapped process data with no protocol overhead: up to eight bytes
//! of application data per frame, laid out by the PDO *mapping parameters*.
//! Transmit PDOs (TPDO) publish data *from* a node's object dictionary;
//! receive PDOs (RPDO) consume data *into* it.
//!
//! A PDO is described by two object-dictionary records:
//!
//! * a **communication parameter** (`0x1400`+ RPDO, `0x1800`+ TPDO) — the
//!   COB-ID (with a validity bit) and the transmission type, and
//! * a **mapping parameter** (`0x1600`+ RPDO, `0x1A00`+ TPDO) — the ordered
//!   list of `(index, subindex, bit length)` triples packed into the frame.
//!
//! This module models the mapping ([`MappingEntry`], [`PdoMapping`]), packs a
//! mapping's objects out of an [`ObjectDictionary`] into a frame ([`pack`]),
//! and unpacks a received frame back into it ([`unpack`]). It also parses the
//! transmission type and the default (predefined connection set) COB-IDs.
//!
//! **Scope:** mappings are byte-aligned — every mapped object's bit length is a
//! whole number of bytes, which covers the overwhelming majority of real
//! devices. Sub-byte bit packing (e.g. several `BOOLEAN`s in one byte) is a
//! later addition.
//!
//! ```
//! use canopen_rs::pdo::{pack, MappingEntry, PdoMapping};
//! use canopen_rs::{Address, Entry, ObjectDictionary, Value};
//!
//! let mut od = ObjectDictionary::<4>::new();
//! od.insert(Address::new(0x6000, 1), Entry::rw(Value::Unsigned16(0xBEEF))).unwrap();
//! od.insert(Address::new(0x6000, 2), Entry::rw(Value::Unsigned8(0x42))).unwrap();
//!
//! // Map both objects into one PDO, then pack them into a frame.
//! let mut mapping = PdoMapping::<8>::new();
//! mapping.push(MappingEntry::new(0x6000, 1, 16)).unwrap();
//! mapping.push(MappingEntry::new(0x6000, 2, 8)).unwrap();
//!
//! let mut buf = [0u8; 8];
//! let len = pack(&mapping, &od, &mut buf).unwrap();
//! assert_eq!(&buf[..len], &[0xEF, 0xBE, 0x42]); // U16 little-endian, then U8
//! ```

use heapless::Vec;

use crate::datatypes::Value;
use crate::object_dictionary::{Address, ObjectDictionary};
use crate::types::NodeId;
use crate::{Error, Result};

/// Whether a PDO transmits from, or receives into, the node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PdoKind {
    /// Transmit PDO: the node publishes data from its object dictionary.
    Transmit,
    /// Receive PDO: the node consumes data into its object dictionary.
    Receive,
}

/// Bit 31 of a communication-parameter COB-ID: set means the PDO is *invalid*
/// (unused). CiA 301 §7.5.2.35.
pub const PDO_COB_ID_INVALID: u32 = 0x8000_0000;

/// Whether a communication-parameter COB-ID marks the PDO as valid (in use).
pub const fn pdo_is_valid(comm_cob_id: u32) -> bool {
    comm_cob_id & PDO_COB_ID_INVALID == 0
}

/// The 11-bit CAN identifier carried in a communication-parameter COB-ID.
pub const fn pdo_can_id(comm_cob_id: u32) -> u16 {
    (comm_cob_id & 0x7FF) as u16
}

/// The default COB-ID of PDO `number` (`1..=4`) for `node`, from the
/// predefined connection set (CiA 301 §7.3.5): TPDOs at `0x180/0x280/0x380/
/// 0x480 + node`, RPDOs at `0x200/0x300/0x400/0x500 + node`.
///
/// Returns `None` for a PDO number outside `1..=4`.
pub const fn default_cob_id(kind: PdoKind, number: u8, node: NodeId) -> Option<u16> {
    if number < 1 || number > 4 {
        return None;
    }
    let base = match kind {
        // TPDO1 = 0x180, TPDO2 = 0x280, … = 0x80 + number * 0x100.
        PdoKind::Transmit => 0x80 + (number as u16) * 0x100,
        // RPDO1 = 0x200, RPDO2 = 0x300, … = 0x100 + number * 0x100.
        PdoKind::Receive => 0x100 + (number as u16) * 0x100,
    };
    Some(base + node.raw() as u16)
}

/// The transmission type of a PDO (CiA 301 §7.5.2.36), byte 2 of the
/// communication parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum TransmissionType {
    /// `0`: synchronous, acyclic — transmitted after the next SYNC when the
    /// mapped data has changed.
    SynchronousAcyclic,
    /// `1..=240`: synchronous, cyclic — transmitted every *n*-th SYNC.
    SynchronousCyclic(u8),
    /// `252`: synchronous, RTR-only.
    SynchronousRtrOnly,
    /// `253`: event-driven, RTR-only.
    EventDrivenRtrOnly,
    /// `254`: event-driven, manufacturer-specific event.
    EventDrivenManufacturer,
    /// `255`: event-driven, device-profile / application event.
    EventDrivenProfile,
}

impl TransmissionType {
    /// Decode a transmission-type byte.
    ///
    /// Returns [`Error::UnsupportedTransfer`] for the reserved range
    /// `241..=251`.
    pub const fn from_byte(byte: u8) -> Result<Self> {
        Ok(match byte {
            0 => TransmissionType::SynchronousAcyclic,
            1..=240 => TransmissionType::SynchronousCyclic(byte),
            252 => TransmissionType::SynchronousRtrOnly,
            253 => TransmissionType::EventDrivenRtrOnly,
            254 => TransmissionType::EventDrivenManufacturer,
            255 => TransmissionType::EventDrivenProfile,
            _ => return Err(Error::UnsupportedTransfer),
        })
    }

    /// The transmission-type byte for this variant.
    pub const fn to_byte(self) -> u8 {
        match self {
            TransmissionType::SynchronousAcyclic => 0,
            TransmissionType::SynchronousCyclic(n) => n,
            TransmissionType::SynchronousRtrOnly => 252,
            TransmissionType::EventDrivenRtrOnly => 253,
            TransmissionType::EventDrivenManufacturer => 254,
            TransmissionType::EventDrivenProfile => 255,
        }
    }
}

/// One entry of a PDO mapping: the object it references and its width in bits.
///
/// On the wire (and in the mapping-parameter record) this is a `u32`
/// `0xIIII_SSLL` — index `IIII`, subindex `SS`, bit length `LL`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MappingEntry {
    /// The mapped object's address.
    pub address: Address,
    /// The number of bits contributed to the PDO frame.
    pub bit_length: u8,
}

impl MappingEntry {
    /// Construct a mapping entry.
    pub const fn new(index: u16, subindex: u8, bit_length: u8) -> Self {
        Self {
            address: Address::new(index, subindex),
            bit_length,
        }
    }

    /// The `0xIIII_SSLL` mapping value stored in the OD mapping record.
    pub const fn to_u32(self) -> u32 {
        ((self.address.index as u32) << 16)
            | ((self.address.subindex as u32) << 8)
            | self.bit_length as u32
    }

    /// Parse a `0xIIII_SSLL` mapping value.
    pub const fn from_u32(raw: u32) -> Self {
        Self {
            address: Address::new((raw >> 16) as u16, (raw >> 8) as u8),
            bit_length: raw as u8,
        }
    }
}

/// An ordered PDO mapping of up to `N` entries.
///
/// The mapped objects are packed into the frame in insertion order, and the
/// total width may not exceed eight bytes (64 bits).
#[derive(Debug)]
pub struct PdoMapping<const N: usize> {
    entries: Vec<MappingEntry, N>,
}

impl<const N: usize> Default for PdoMapping<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> PdoMapping<N> {
    /// An empty mapping.
    pub const fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// Append a mapping entry.
    ///
    /// Returns [`Error::PdoTooLong`] if it would push the total past 64 bits,
    /// or [`Error::MappingFull`] if the fixed capacity `N` is exhausted.
    pub fn push(&mut self, entry: MappingEntry) -> Result<()> {
        if self.total_bits() + entry.bit_length as u32 > 64 {
            return Err(Error::PdoTooLong);
        }
        self.entries.push(entry).map_err(|_| Error::MappingFull)
    }

    /// The mapping entries, in packing order.
    pub fn entries(&self) -> &[MappingEntry] {
        self.entries.as_slice()
    }

    /// The number of mapped entries (the value of mapping subindex `0`).
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the mapping is empty (PDO disabled).
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The total width of the mapped data, in bits.
    pub fn total_bits(&self) -> u32 {
        self.entries.iter().map(|e| e.bit_length as u32).sum()
    }

    /// The total width of the mapped data, in whole bytes (the CAN frame DLC).
    pub fn total_bytes(&self) -> usize {
        (self.total_bits() as usize).div_ceil(8)
    }
}

/// Pack a TPDO: read each mapped object out of `od` and lay it into `buf`
/// little-endian, in mapping order. Returns the number of bytes written (the
/// frame's data length).
///
/// Errors: [`Error::BadLength`] if `buf` is shorter than
/// [`PdoMapping::total_bytes`]; [`Error::UnsupportedTransfer`] for a sub-byte
/// mapping entry; [`Error::TypeMismatch`] if a mapped object's stored width
/// disagrees with its mapping bit length; plus any error from reading the
/// object (e.g. [`Error::ObjectNotFound`], [`Error::WriteOnly`]).
pub fn pack<const N: usize, const K: usize>(
    mapping: &PdoMapping<N>,
    od: &ObjectDictionary<K>,
    buf: &mut [u8],
) -> Result<usize> {
    let total = mapping.total_bytes();
    if buf.len() < total {
        return Err(Error::BadLength);
    }
    let mut offset = 0;
    for entry in mapping.entries() {
        let width = byte_width(entry.bit_length)?;
        let value = od.read(entry.address)?;
        if value.size() != width {
            return Err(Error::TypeMismatch);
        }
        value.encode_le(&mut buf[offset..offset + width])?;
        offset += width;
    }
    Ok(offset)
}

/// Unpack an RPDO: decode each mapped field from `data` and write it into `od`,
/// in mapping order. The data type of each field is taken from the object
/// currently stored in `od`.
///
/// Errors: [`Error::BadLength`] if `data` is shorter than
/// [`PdoMapping::total_bytes`]; [`Error::UnsupportedTransfer`] for a sub-byte
/// mapping entry; [`Error::ObjectNotFound`] for an unmapped target;
/// [`Error::TypeMismatch`] if a target object's width disagrees with its
/// mapping bit length; plus any error from writing the object (e.g.
/// [`Error::ReadOnly`]).
pub fn unpack<const N: usize, const K: usize>(
    mapping: &PdoMapping<N>,
    od: &mut ObjectDictionary<K>,
    data: &[u8],
) -> Result<()> {
    let total = mapping.total_bytes();
    if data.len() < total {
        return Err(Error::BadLength);
    }
    let mut offset = 0;
    for entry in mapping.entries() {
        let width = byte_width(entry.bit_length)?;
        let data_type = od
            .entry(entry.address)
            .ok_or(Error::ObjectNotFound)?
            .value
            .data_type();
        if data_type.size() != width {
            return Err(Error::TypeMismatch);
        }
        let value = Value::decode_le(data_type, &data[offset..offset + width])?;
        od.write(entry.address, value)?;
        offset += width;
    }
    Ok(())
}

/// The whole-byte width of a mapping bit length, rejecting sub-byte entries.
const fn byte_width(bit_length: u8) -> Result<usize> {
    if bit_length == 0 || bit_length % 8 != 0 {
        return Err(Error::UnsupportedTransfer);
    }
    Ok((bit_length / 8) as usize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datatypes::Value;
    use crate::object_dictionary::Entry;

    // --- Default COB-IDs ---------------------------------------------------
    #[test]
    fn default_cob_ids_follow_connection_set() {
        let node = NodeId::new(4).unwrap();
        assert_eq!(default_cob_id(PdoKind::Transmit, 1, node), Some(0x184));
        assert_eq!(default_cob_id(PdoKind::Receive, 1, node), Some(0x204));
        assert_eq!(default_cob_id(PdoKind::Transmit, 4, node), Some(0x484));
        assert_eq!(default_cob_id(PdoKind::Receive, 4, node), Some(0x504));
        assert_eq!(default_cob_id(PdoKind::Transmit, 5, node), None);
    }

    #[test]
    fn comm_param_cob_id_validity() {
        // TPDO1 of node 4, valid: 0x00000184. Invalid sets bit 31.
        assert!(pdo_is_valid(0x0000_0184));
        assert!(!pdo_is_valid(0x8000_0184));
        assert_eq!(pdo_can_id(0x8000_0184), 0x184);
    }

    // --- Transmission type -------------------------------------------------
    #[test]
    fn transmission_types_roundtrip() {
        for byte in [0u8, 1, 240, 252, 253, 254, 255] {
            let t = TransmissionType::from_byte(byte).unwrap();
            assert_eq!(t.to_byte(), byte);
        }
        assert_eq!(
            TransmissionType::from_byte(1).unwrap(),
            TransmissionType::SynchronousCyclic(1)
        );
        assert_eq!(
            TransmissionType::from_byte(245),
            Err(Error::UnsupportedTransfer)
        );
    }

    // --- Mapping entry codec ----------------------------------------------
    // Known-good mapping value: object 0x6000 sub 0x01, 8 bits → 0x60000108.
    #[test]
    fn mapping_entry_matches_known_value() {
        let e = MappingEntry::new(0x6000, 0x01, 8);
        assert_eq!(e.to_u32(), 0x6000_0108);
        assert_eq!(MappingEntry::from_u32(0x6000_0108), e);
    }

    #[test]
    fn mapping_rejects_over_eight_bytes() {
        let mut m: PdoMapping<8> = PdoMapping::new();
        // 4 × U16 = 64 bits, exactly full.
        for i in 0..4 {
            m.push(MappingEntry::new(0x2000 + i, 0, 16)).unwrap();
        }
        assert_eq!(m.total_bytes(), 8);
        // One more bit over the limit is rejected.
        assert_eq!(
            m.push(MappingEntry::new(0x2100, 0, 8)),
            Err(Error::PdoTooLong)
        );
    }

    // --- Pack / unpack -----------------------------------------------------
    // A TPDO mapping two objects: U16 at 0x6000/1 then U8 at 0x6000/2.
    fn sample_od() -> ObjectDictionary<8> {
        let mut od = ObjectDictionary::new();
        od.insert(
            Address::new(0x6000, 1),
            Entry::rw(Value::Unsigned16(0xBEEF)),
        )
        .unwrap();
        od.insert(Address::new(0x6000, 2), Entry::rw(Value::Unsigned8(0x42)))
            .unwrap();
        od
    }

    fn sample_mapping() -> PdoMapping<8> {
        let mut m = PdoMapping::new();
        m.push(MappingEntry::new(0x6000, 1, 16)).unwrap();
        m.push(MappingEntry::new(0x6000, 2, 8)).unwrap();
        m
    }

    #[test]
    fn pack_lays_out_little_endian_in_order() {
        let od = sample_od();
        let mapping = sample_mapping();
        let mut buf = [0u8; 8];
        let n = pack(&mapping, &od, &mut buf).unwrap();
        // U16 0xBEEF little-endian (EF BE) then U8 0x42.
        assert_eq!(n, 3);
        assert_eq!(&buf[..n], &[0xEF, 0xBE, 0x42]);
    }

    #[test]
    fn unpack_writes_fields_back_into_od() {
        let mut od = sample_od();
        let mapping = sample_mapping();
        // New values: U16 0x1234, U8 0x99.
        unpack(&mapping, &mut od, &[0x34, 0x12, 0x99]).unwrap();
        assert_eq!(
            od.read(Address::new(0x6000, 1)).unwrap(),
            Value::Unsigned16(0x1234)
        );
        assert_eq!(
            od.read(Address::new(0x6000, 2)).unwrap(),
            Value::Unsigned8(0x99)
        );
    }

    #[test]
    fn pack_unpack_roundtrip() {
        let od = sample_od();
        let mapping = sample_mapping();
        let mut buf = [0u8; 8];
        let n = pack(&mapping, &od, &mut buf).unwrap();

        let mut dest = ObjectDictionary::<8>::new();
        dest.insert(Address::new(0x6000, 1), Entry::rw(Value::Unsigned16(0)))
            .unwrap();
        dest.insert(Address::new(0x6000, 2), Entry::rw(Value::Unsigned8(0)))
            .unwrap();
        unpack(&mapping, &mut dest, &buf[..n]).unwrap();

        assert_eq!(
            dest.read(Address::new(0x6000, 1)).unwrap(),
            Value::Unsigned16(0xBEEF)
        );
        assert_eq!(
            dest.read(Address::new(0x6000, 2)).unwrap(),
            Value::Unsigned8(0x42)
        );
    }

    #[test]
    fn pack_rejects_short_buffer() {
        let od = sample_od();
        let mapping = sample_mapping();
        let mut buf = [0u8; 2];
        assert_eq!(pack(&mapping, &od, &mut buf), Err(Error::BadLength));
    }

    #[test]
    fn unpack_into_read_only_object_errors() {
        let mut od = ObjectDictionary::<4>::new();
        od.insert(Address::new(0x6000, 1), Entry::ro(Value::Unsigned16(0)))
            .unwrap();
        let mut m: PdoMapping<4> = PdoMapping::new();
        m.push(MappingEntry::new(0x6000, 1, 16)).unwrap();
        assert_eq!(unpack(&m, &mut od, &[0, 0]), Err(Error::ReadOnly));
    }

    #[test]
    fn mapping_width_mismatch_errors() {
        // Map 0x6000/2 (a U8) as 16 bits — inconsistent with the stored type.
        let od = sample_od();
        let mut m: PdoMapping<4> = PdoMapping::new();
        m.push(MappingEntry::new(0x6000, 2, 16)).unwrap();
        let mut buf = [0u8; 8];
        assert_eq!(pack(&m, &od, &mut buf), Err(Error::TypeMismatch));
    }
}
