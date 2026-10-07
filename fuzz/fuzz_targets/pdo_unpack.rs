#![no_main]

use canopen_rs::{pdo, ObjectDictionary, PdoMapping};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mapping: PdoMapping<10> = PdoMapping::new();
    let mut od: ObjectDictionary<10> = ObjectDictionary::new();
    let _ = pdo::unpack(&mapping, &mut od, data);
});
