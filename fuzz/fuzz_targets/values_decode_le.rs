#![no_main]

use canopen_rs::{DataType, Value};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = Value::decode_le(DataType::Boolean, data);
    let _ = Value::decode_le(DataType::Domain, data);
    let _ = Value::decode_le(DataType::Integer8, data);
    let _ = Value::decode_le(DataType::Integer16, data);
    let _ = Value::decode_le(DataType::Integer32, data);
    let _ = Value::decode_le(DataType::Integer64, data);
    let _ = Value::decode_le(DataType::Real32, data);
    let _ = Value::decode_le(DataType::Real64, data);
    let _ = Value::decode_le(DataType::Unsigned8, data);
    let _ = Value::decode_le(DataType::Unsigned16, data);
    let _ = Value::decode_le(DataType::Unsigned32, data);
    let _ = Value::decode_le(DataType::Unsigned64, data);
    let _ = Value::decode_le(DataType::OctetString, data);
    let _ = Value::decode_le(DataType::VisibleString, data);
});
