//! EDS / DCF parsing (CiA 306) — INI-style device description files.
//!
//! An Electronic Data Sheet describes a device's object dictionary in an
//! INI-like text format: a `[<index>]` section per object and
//! `[<index>sub<subindex>]` sections per subindex, each carrying fields such
//! as `ParameterName`, `DataType`, `AccessType`, and `DefaultValue`. Indices
//! and subindices are hexadecimal. A Device Configuration File (DCF) shares
//! the grammar, adding concrete `ParameterValue` fields.
//!
//! [`Eds::parse`](crate::eds::Eds::parse) turns that text into a list of typed
//! [`ObjectDescription`](crate::eds::ObjectDescription)s, and
//! [`Eds::object_dictionary`](crate::eds::Eds::object_dictionary) builds a
//! ready-to-use core [`ObjectDictionary`](canopen_rs::ObjectDictionary) from
//! them — the same dictionary type an embedded node runs, populated here from a
//! file.
//!
//! ## Supported subset
//!
//! * The numeric basic data types (see [`DataType`](canopen_rs::DataType));
//!   `VISIBLE_STRING`,
//!   `OCTET_STRING`, and `DOMAIN` objects are skipped, as the core does not
//!   yet model variable-length values.
//! * `DefaultValue` / `ParameterValue` as decimal or `0x…` hex, including the
//!   `$NODEID` substitution used for COB-IDs (e.g. `$NODEID+0x200`), resolved
//!   against the file's `[DeviceComissioning] NodeID` when present.

use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::io;
use std::path::Path;

use canopen_rs::datatypes::{DataType, Value};
use canopen_rs::object_dictionary::{AccessType, Address, Entry, ObjectDictionary};

/// An error encountered while parsing an EDS/DCF file.
#[derive(Debug)]
pub enum EdsError {
    /// The file could not be read.
    Io(io::Error),
    /// An object section was missing a required field.
    MissingField {
        /// The `[section]` the field was expected in.
        section: String,
        /// The absent field name.
        field: &'static str,
    },
    /// An `AccessType` value was not one of `ro`/`wo`/`rw`/`rwr`/`rww`/`const`.
    UnknownAccessType(String),
    /// A `DefaultValue`/`ParameterValue` could not be parsed for its type.
    InvalidValue {
        /// The `[section]` the value came from.
        section: String,
        /// The offending text.
        value: String,
    },
    /// More objects than the target [`ObjectDictionary`] capacity.
    TooManyObjects,
}

impl fmt::Display for EdsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EdsError::Io(e) => write!(f, "reading EDS file: {e}"),
            EdsError::MissingField { section, field } => {
                write!(f, "section [{section}] is missing required field `{field}`")
            }
            EdsError::UnknownAccessType(s) => write!(f, "unknown AccessType `{s}`"),
            EdsError::InvalidValue { section, value } => {
                write!(f, "section [{section}] has an invalid value `{value}`")
            }
            EdsError::TooManyObjects => f.write_str("more objects than dictionary capacity"),
        }
    }
}

impl std::error::Error for EdsError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            EdsError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for EdsError {
    fn from(e: io::Error) -> Self {
        EdsError::Io(e)
    }
}

/// A single object dictionary entry described by the EDS.
#[derive(Debug, Clone)]
pub struct ObjectDescription {
    /// The `(index, subindex)` address.
    pub address: Address,
    /// The human-readable `ParameterName`.
    pub parameter_name: String,
    /// The object's data type.
    pub data_type: DataType,
    /// The declared access rights.
    pub access: AccessType,
    /// The default (EDS) or configured (DCF) value.
    pub default_value: Value,
    /// Whether the object may be mapped into a PDO.
    pub pdo_mappable: bool,
}

impl ObjectDescription {
    /// The core [`Entry`] this description builds: its value and access rights.
    pub fn entry(&self) -> Entry {
        Entry {
            value: self.default_value,
            access: self.access,
        }
    }
}

/// A parsed Electronic Data Sheet (or Device Configuration File).
#[derive(Debug, Clone)]
pub struct Eds {
    /// `[DeviceInfo] VendorName`, if present.
    pub vendor_name: Option<String>,
    /// `[DeviceInfo] ProductName`, if present.
    pub product_name: Option<String>,
    /// `[DeviceComissioning] NodeID`, if present — also used to resolve
    /// `$NODEID` expressions in default values.
    pub node_id: Option<u8>,
    /// The described objects, ordered by address.
    pub objects: Vec<ObjectDescription>,
}

impl Eds {
    /// Parse EDS/DCF text.
    pub fn parse(text: &str) -> Result<Eds, EdsError> {
        let sections = parse_ini(text);

        let device = sections.get("deviceinfo");
        let vendor_name = device.and_then(|s| s.get("vendorname")).cloned();
        let product_name = device.and_then(|s| s.get("productname")).cloned();

        // "DeviceComissioning" is the CiA 306 spelling (sic); accept both.
        let node_id = sections
            .get("devicecomissioning")
            .or_else(|| sections.get("devicecommissioning"))
            .and_then(|s| s.get("nodeid"))
            .and_then(|v| eval_int_expr(v, 0))
            .and_then(|n| u8::try_from(n).ok());
        let node_for_expr = node_id.unwrap_or(0);

        let mut objects = Vec::new();
        for (name, fields) in &sections {
            let Some(address) = parse_object_section(name) else {
                continue;
            };
            // Only sections that carry a DataType describe a stored value; a
            // RECORD/ARRAY parent section (its subindices hold the values) has
            // none and is skipped.
            let Some(dt_raw) = fields.get("datatype") else {
                continue;
            };
            let Some(dt_index) = eval_int_expr(dt_raw, node_for_expr) else {
                return Err(EdsError::InvalidValue {
                    section: name.clone(),
                    value: dt_raw.clone(),
                });
            };
            // Skip unmodelled types (strings, DOMAIN) rather than failing the
            // whole file.
            let Some(data_type) = u16::try_from(dt_index).ok().and_then(DataType::from_index)
            else {
                continue;
            };

            let access = match fields.get("accesstype") {
                Some(a) => access_from_str(a)?,
                None => AccessType::Ro,
            };

            let raw_value = fields
                .get("parametervalue")
                .or_else(|| fields.get("defaultvalue"))
                .map(String::as_str)
                .unwrap_or("");
            let default_value =
                value_from_str(data_type, raw_value, node_for_expr).ok_or_else(|| {
                    EdsError::InvalidValue {
                        section: name.clone(),
                        value: raw_value.to_string(),
                    }
                })?;

            objects.push(ObjectDescription {
                address,
                parameter_name: fields.get("parametername").cloned().unwrap_or_default(),
                data_type,
                access,
                default_value,
                pdo_mappable: fields
                    .get("pdomapping")
                    .map(|v| v.trim() != "0")
                    .unwrap_or(false),
            });
        }

        objects.sort_by_key(|o| o.address);
        Ok(Eds {
            vendor_name,
            product_name,
            node_id,
            objects,
        })
    }

    /// Read and parse an EDS/DCF file from `path`.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Eds, EdsError> {
        Self::parse(&fs::read_to_string(path)?)
    }

    /// The described object at `address`, if any.
    pub fn get(&self, address: Address) -> Option<&ObjectDescription> {
        self.objects.iter().find(|o| o.address == address)
    }

    /// Build a core [`ObjectDictionary`] of capacity `N` from the described
    /// objects.
    ///
    /// Returns [`EdsError::TooManyObjects`] if there are more objects than `N`.
    pub fn object_dictionary<const N: usize>(&self) -> Result<ObjectDictionary<N>, EdsError> {
        let mut od = ObjectDictionary::new();
        for obj in &self.objects {
            od.insert(obj.address, obj.entry())
                .map_err(|_| EdsError::TooManyObjects)?;
        }
        Ok(od)
    }
}

// --- INI + value helpers ---------------------------------------------------

/// Parse INI text into `section -> (key -> value)`, with section names and
/// keys lower-cased for case-insensitive lookup and values kept verbatim.
fn parse_ini(text: &str) -> BTreeMap<String, BTreeMap<String, String>> {
    let mut sections = BTreeMap::new();
    let mut current: Option<String> = None;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with(';') {
            continue;
        }
        if let Some(inner) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            let name = inner.trim().to_ascii_lowercase();
            sections.entry(name.clone()).or_insert_with(BTreeMap::new);
            current = Some(name);
        } else if let (Some(section), Some((key, value))) = (&current, line.split_once('=')) {
            sections
                .get_mut(section)
                .expect("current section was inserted")
                .insert(key.trim().to_ascii_lowercase(), value.trim().to_string());
        }
    }
    sections
}

/// Parse an object/subindex section name (`"1018"` or `"1018sub2"`) into an
/// [`Address`]; `None` for a non-object section such as `"deviceinfo"`.
fn parse_object_section(name: &str) -> Option<Address> {
    if let Some((index, subindex)) = name.split_once("sub") {
        Some(Address::new(
            u16::from_str_radix(index, 16).ok()?,
            u8::from_str_radix(subindex, 16).ok()?,
        ))
    } else {
        Some(Address::new(u16::from_str_radix(name, 16).ok()?, 0))
    }
}

fn access_from_str(s: &str) -> Result<AccessType, EdsError> {
    match s.trim().to_ascii_lowercase().as_str() {
        "ro" => Ok(AccessType::Ro),
        "wo" => Ok(AccessType::Wo),
        "rw" | "rwr" | "rww" => Ok(AccessType::Rw),
        "const" => Ok(AccessType::Const),
        other => Err(EdsError::UnknownAccessType(other.to_string())),
    }
}

/// Evaluate a sum of `$NODEID` and integer literal terms, e.g. `"$NODEID+0x200"`
/// or `"1280+$NODEID"`. Literals may be decimal or `0x…` hex.
fn eval_int_expr(s: &str, node_id: u8) -> Option<i128> {
    let mut sum: i128 = 0;
    for term in s.split('+') {
        let term = term.trim();
        if term.is_empty() {
            continue;
        } else if term.eq_ignore_ascii_case("$nodeid") {
            sum += node_id as i128;
        } else if let Some(hex) = term.strip_prefix("0x").or_else(|| term.strip_prefix("0X")) {
            sum += i128::from_str_radix(hex, 16).ok()?;
        } else {
            sum += term.parse::<i128>().ok()?;
        }
    }
    Some(sum)
}

/// Parse a default/configured value string into a typed [`Value`].
fn value_from_str(data_type: DataType, s: &str, node_id: u8) -> Option<Value> {
    let s = s.trim();
    // Floating-point types parse the literal directly.
    match data_type {
        DataType::Real32 => {
            return Some(Value::Real32(if s.is_empty() {
                0.0
            } else {
                s.parse().ok()?
            }))
        }
        DataType::Real64 => {
            return Some(Value::Real64(if s.is_empty() {
                0.0
            } else {
                s.parse().ok()?
            }))
        }
        _ => {}
    }
    // Integer types evaluate a (possibly `$NODEID`-relative) expression.
    let n = if s.is_empty() {
        0
    } else {
        eval_int_expr(s, node_id)?
    };
    Some(match data_type {
        DataType::Boolean => Value::Boolean(n != 0),
        DataType::Integer8 => Value::Integer8(n as i8),
        DataType::Integer16 => Value::Integer16(n as i16),
        DataType::Integer32 => Value::Integer32(n as i32),
        DataType::Unsigned8 => Value::Unsigned8(n as u8),
        DataType::Unsigned16 => Value::Unsigned16(n as u16),
        DataType::Unsigned32 => Value::Unsigned32(n as u32),
        DataType::Integer64 => Value::Integer64(n as i64),
        DataType::Unsigned64 => Value::Unsigned64(n as u64),
        // Reals handled above; `#[non_exhaustive]` requires this catch-all.
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // A compact but representative EDS: device info, node id, a VAR, a RECORD
    // with subindices, a $NODEID COB-ID, a hex value, a float, and a skipped
    // string object.
    const SAMPLE: &str = "\
[DeviceInfo]
VendorName=Acme Robotics
ProductName=Widget 3000

[DeviceComissioning]
NodeID=0x10

[MandatoryObjects]
SupportedObjects=2
1=0x1000
2=0x1018

[1000]
ParameterName=Device type
ObjectType=0x7
DataType=0x0007
AccessType=ro
DefaultValue=0x00040192
PDOMapping=0

[1008]
ParameterName=Device name
ObjectType=0x7
DataType=0x0009
AccessType=const
DefaultValue=Widget

[1017]
ParameterName=Producer heartbeat time
ObjectType=0x7
DataType=0x0006
AccessType=rw
DefaultValue=1000
PDOMapping=1

[1018]
ParameterName=Identity object
ObjectType=0x9
SubNumber=2

[1018sub0]
ParameterName=Highest sub-index supported
DataType=0x0005
AccessType=const
DefaultValue=1

[1018sub1]
ParameterName=Vendor-ID
DataType=0x0007
AccessType=ro
DefaultValue=0x000000AB

[1400sub1]
ParameterName=COB-ID used by RPDO
DataType=0x0007
AccessType=rw
DefaultValue=$NODEID+0x200

[6000]
ParameterName=Scale factor
ObjectType=0x7
DataType=0x0008
AccessType=rw
DefaultValue=1.5
";

    fn sample() -> Eds {
        Eds::parse(SAMPLE).unwrap()
    }

    #[test]
    fn reads_device_metadata_and_node_id() {
        let eds = sample();
        assert_eq!(eds.vendor_name.as_deref(), Some("Acme Robotics"));
        assert_eq!(eds.product_name.as_deref(), Some("Widget 3000"));
        assert_eq!(eds.node_id, Some(0x10));
    }

    #[test]
    fn parses_var_with_hex_default() {
        let obj = sample().get(Address::new(0x1000, 0)).unwrap().clone();
        assert_eq!(obj.parameter_name, "Device type");
        assert_eq!(obj.data_type, DataType::Unsigned32);
        assert_eq!(obj.access, AccessType::Ro);
        assert_eq!(obj.default_value, Value::Unsigned32(0x0004_0192));
    }

    #[test]
    fn parses_record_subindices() {
        let eds = sample();
        assert_eq!(
            eds.get(Address::new(0x1018, 0)).unwrap().default_value,
            Value::Unsigned8(1)
        );
        assert_eq!(
            eds.get(Address::new(0x1018, 1)).unwrap().default_value,
            Value::Unsigned32(0x0000_00AB)
        );
    }

    #[test]
    fn resolves_nodeid_expression() {
        // $NODEID (0x10) + 0x200 = 0x210.
        let obj = sample().get(Address::new(0x1400, 1)).unwrap().clone();
        assert_eq!(obj.default_value, Value::Unsigned32(0x210));
    }

    #[test]
    fn parses_float_and_pdo_flag() {
        let eds = sample();
        assert_eq!(
            eds.get(Address::new(0x6000, 0)).unwrap().default_value,
            Value::Real32(1.5)
        );
        assert!(eds.get(Address::new(0x1017, 0)).unwrap().pdo_mappable);
        assert!(!eds.get(Address::new(0x1000, 0)).unwrap().pdo_mappable);
    }

    #[test]
    fn skips_unmodelled_string_object() {
        // 0x1008 Device name is VISIBLE_STRING (0x09) — not represented yet.
        assert!(sample().get(Address::new(0x1008, 0)).is_none());
    }

    #[test]
    fn builds_object_dictionary() {
        let eds = sample();
        let od = eds.object_dictionary::<16>().unwrap();
        assert_eq!(
            od.read(Address::new(0x1000, 0)).unwrap(),
            Value::Unsigned32(0x0004_0192)
        );
        assert_eq!(
            od.read(Address::new(0x1017, 0)).unwrap(),
            Value::Unsigned16(1000)
        );
    }

    #[test]
    fn object_dictionary_capacity_is_enforced() {
        // The sample has more than two modelled objects.
        assert!(matches!(
            sample().object_dictionary::<2>(),
            Err(EdsError::TooManyObjects)
        ));
    }

    #[test]
    fn unknown_access_type_errors() {
        let text = "[2000]\nDataType=0x0005\nAccessType=bogus\n";
        assert!(matches!(
            Eds::parse(text),
            Err(EdsError::UnknownAccessType(_))
        ));
    }
}
