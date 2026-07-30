//! The CANopen object dictionary (OD) model.
//!
//! Every CANopen device exposes its data as a dictionary of objects
//! addressed by a 16-bit `index` and an 8-bit `subindex` (CiA 301 §7.4).
//! Communication objects live in `0x1000..=0x1FFF`, manufacturer-specific
//! objects in `0x2000..=0x5FFF`, and standardised device-profile objects in
//! `0x6000..=0x9FFF`.
//!
//! The storage strategy is deliberately abstracted behind a trait so the
//! same OD model serves both a `Vec`-backed host tool and a `const`- or
//! [`heapless`]-backed embedded node. Typed values and that trait land in
//! the next milestone.
//!
//! [`heapless`]: https://docs.rs/heapless

/// The address of an object dictionary entry: a 16-bit index plus an 8-bit
/// subindex.
///
/// Subindex `0` conventionally holds the "number of entries" for `RECORD`
/// and `ARRAY` objects; simple `VAR` objects use subindex `0` for the value
/// itself.
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

// TODO(next milestone): a typed `Value` enum over the CiA 301 basic data
// types, an `ObjectEntry` (value + access type + data type), and an
// `ObjectDictionary` trait with host (`Vec`) and embedded (`heapless` /
// `const` slice) backings — validated against known CiA 301 objects such as
// 0x1000 (device type) and 0x1017 (producer heartbeat time).
