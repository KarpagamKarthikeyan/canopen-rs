//! EDS / DCF parsing (CiA 306) — INI-style device description files.
//!
//! An Electronic Data Sheet describes a device's object dictionary in an
//! INI-like text format: a `[<index>]` section per object and
//! `[<index>sub<subindex>]` sections per subindex, each carrying fields such
//! as `ParameterName`, `DataType`, `AccessType`, and `DefaultValue`.
//!
//! The parser produces an OD description that can build or validate a node's
//! object dictionary.

// TODO(milestone: EDS): parse the [FileInfo]/[DeviceInfo]/[<index>] sections
// into typed object-dictionary entries; a Device Configuration File (DCF) is
// the same grammar with concrete values.
