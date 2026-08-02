//! Generate a compile-time object dictionary from an EDS/DCF file.
//!
//! [`generate_object_dictionary`](crate::codegen::generate_object_dictionary)
//! emits Rust source for a function that builds a
//! [`canopen_rs::ObjectDictionary`] populated from a parsed
//! [`Eds`](crate::eds::Eds). Run it from a `build.rs` so a device's `.eds`
//! becomes a typed, zero-runtime-parse OD:
//!
//! ```ignore
//! // build.rs
//! use canopen_host::{codegen::generate_object_dictionary, eds::Eds};
//! use std::{env, fs, path::Path};
//!
//! fn main() {
//!     let eds = Eds::from_file("device.eds").unwrap();
//!     let src = generate_object_dictionary(&eds, "device_od");
//!     let out = Path::new(&env::var("OUT_DIR").unwrap()).join("device_od.rs");
//!     fs::write(out, src).unwrap();
//!     println!("cargo:rerun-if-changed=device.eds");
//! }
//! ```
//!
//! ```ignore
//! // in your crate:
//! include!(concat!(env!("OUT_DIR"), "/device_od.rs"));
//! let od = device_od();   // ObjectDictionary<N>, ready to serve
//! ```

use core::fmt::Write as _;

use canopen_rs::{AccessType, Value};

use crate::eds::Eds;

/// Emit Rust source for a `pub fn <fn_name>() -> canopen_rs::ObjectDictionary<N>`
/// that inserts every object in `eds`, with `N` sized to fit exactly.
///
/// The generated code refers to everything by absolute `canopen_rs::` path, so
/// it compiles regardless of the including module's imports (the crate need only
/// depend on `canopen-rs`).
pub fn generate_object_dictionary(eds: &Eds, fn_name: &str) -> String {
    let n = eds.objects.len();
    let mut out = String::new();
    let _ = writeln!(
        out,
        "/// Object dictionary generated from an EDS by canopen-host."
    );
    let _ = writeln!(
        out,
        "pub fn {fn_name}() -> canopen_rs::ObjectDictionary<{n}> {{"
    );
    let _ = writeln!(out, "    let mut od = canopen_rs::ObjectDictionary::new();");
    for obj in &eds.objects {
        let _ = writeln!(
            out,
            "    od.insert(canopen_rs::Address::new({:#06x}, {:#04x}), canopen_rs::Entry {{ value: {}, access: canopen_rs::AccessType::{} }}).unwrap();",
            obj.address.index,
            obj.address.subindex,
            value_literal(&obj.default_value),
            access_variant(obj.access),
        );
    }
    let _ = writeln!(out, "    od");
    let _ = writeln!(out, "}}");
    out
}

fn access_variant(access: AccessType) -> &'static str {
    match access {
        AccessType::Ro => "Ro",
        AccessType::Wo => "Wo",
        AccessType::Rw => "Rw",
        AccessType::Const => "Const",
    }
}

/// A Rust literal expression for a [`Value`], fully qualified.
fn value_literal(value: &Value) -> String {
    match value {
        Value::Boolean(b) => format!("canopen_rs::Value::Boolean({b})"),
        Value::Unsigned8(x) => format!("canopen_rs::Value::Unsigned8({x:#04x})"),
        Value::Unsigned16(x) => format!("canopen_rs::Value::Unsigned16({x:#06x})"),
        Value::Unsigned32(x) => format!("canopen_rs::Value::Unsigned32({x:#010x})"),
        Value::Unsigned64(x) => format!("canopen_rs::Value::Unsigned64({x:#018x})"),
        Value::Integer8(x) => format!("canopen_rs::Value::Integer8({x})"),
        Value::Integer16(x) => format!("canopen_rs::Value::Integer16({x})"),
        Value::Integer32(x) => format!("canopen_rs::Value::Integer32({x})"),
        Value::Integer64(x) => format!("canopen_rs::Value::Integer64({x})"),
        Value::Real32(x) => format!("canopen_rs::Value::Real32({x:?}f32)"),
        Value::Real64(x) => format!("canopen_rs::Value::Real64({x:?}f64)"),
        Value::VisibleString(s) => string_literal("VisibleString", s.as_bytes()),
        Value::OctetString(s) => string_literal("OctetString", s.as_bytes()),
        Value::Domain(s) => string_literal("Domain", s.as_bytes()),
        // `Value` is #[non_exhaustive]; every current variant is handled above.
        _ => panic!("codegen: unsupported value variant {value:?}"),
    }
}

/// A Rust literal for a variable-length value: reconstruct the bounded
/// [`ByteString`](canopen_rs::ByteString) from its bytes. The EDS parser already
/// caps content at `MAX_STRING_LEN`, so `from_bytes` cannot fail at runtime.
fn string_literal(variant: &str, bytes: &[u8]) -> String {
    let mut list = String::new();
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 {
            list.push_str(", ");
        }
        let _ = write!(list, "{b:#04x}");
    }
    format!("canopen_rs::Value::{variant}(canopen_rs::ByteString::from_bytes(&[{list}]).unwrap())")
}

#[cfg(test)]
mod tests {
    use super::*;
    use canopen_rs::{Address, ObjectDictionary};

    const SAMPLE: &str = "\
[1000]
ParameterName=Device type
DataType=0x0007
AccessType=ro
DefaultValue=0x00040192

[1008]
ParameterName=Device name
DataType=0x0009
AccessType=const
DefaultValue=Widget

[1017]
ParameterName=Producer heartbeat time
DataType=0x0006
AccessType=rw
DefaultValue=1000
";

    // The exact shape `generate_object_dictionary` emits for SAMPLE — kept here
    // so the compiler proves the generated pattern is valid Rust that builds a
    // correct object dictionary. If the generator's format changes, update both.
    #[allow(clippy::needless_return)]
    fn generated_sample() -> ObjectDictionary<3> {
        let mut od = ObjectDictionary::new();
        od.insert(
            Address::new(0x1000, 0x00),
            canopen_rs::Entry {
                value: canopen_rs::Value::Unsigned32(0x00040192),
                access: AccessType::Ro,
            },
        )
        .unwrap();
        od.insert(
            Address::new(0x1008, 0x00),
            canopen_rs::Entry {
                value: canopen_rs::Value::VisibleString(
                    canopen_rs::ByteString::from_bytes(&[0x57, 0x69, 0x64, 0x67, 0x65, 0x74])
                        .unwrap(),
                ),
                access: AccessType::Const,
            },
        )
        .unwrap();
        od.insert(
            Address::new(0x1017, 0x00),
            canopen_rs::Entry {
                value: canopen_rs::Value::Unsigned16(0x03e8),
                access: AccessType::Rw,
            },
        )
        .unwrap();
        od
    }

    #[test]
    fn generated_shape_builds_a_correct_od() {
        let od = generated_sample();
        assert_eq!(
            od.read(Address::new(0x1000, 0)).unwrap(),
            Value::Unsigned32(0x0004_0192)
        );
        assert_eq!(
            od.read(Address::new(0x1017, 0)).unwrap(),
            Value::Unsigned16(1000)
        );
        match od.read(Address::new(0x1008, 0)).unwrap() {
            Value::VisibleString(bs) => assert_eq!(bs.as_str(), Some("Widget")),
            other => panic!("expected VISIBLE_STRING, got {other:?}"),
        }
    }

    #[test]
    fn generator_emits_the_expected_source() {
        let eds = Eds::parse(SAMPLE).unwrap();
        let src = generate_object_dictionary(&eds, "device_od");

        assert!(src.contains("pub fn device_od() -> canopen_rs::ObjectDictionary<3> {"));
        assert!(src.contains(
            "od.insert(canopen_rs::Address::new(0x1000, 0x00), canopen_rs::Entry { value: canopen_rs::Value::Unsigned32(0x00040192), access: canopen_rs::AccessType::Ro }).unwrap();"
        ));
        assert!(src.contains(
            "od.insert(canopen_rs::Address::new(0x1017, 0x00), canopen_rs::Entry { value: canopen_rs::Value::Unsigned16(0x03e8), access: canopen_rs::AccessType::Rw }).unwrap();"
        ));
        // The VISIBLE_STRING object (the variant that used to panic codegen).
        assert!(src.contains(
            "canopen_rs::Value::VisibleString(canopen_rs::ByteString::from_bytes(&[0x57, 0x69, 0x64, 0x67, 0x65, 0x74]).unwrap())"
        ));
    }
}
