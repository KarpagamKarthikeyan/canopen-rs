#![no_main]

use canopen_rs::sdo;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [u8; 8]| {
    let _ = sdo::decode_data_segment(&data);
});
