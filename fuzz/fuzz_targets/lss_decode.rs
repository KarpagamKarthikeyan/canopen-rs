#![no_main]

use canopen_rs::lss;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [u8; 8]| {
    let _ = lss::decode_configure_node_id_response(&data);
    let _ = lss::decode_inquire_node_id_response(&data);
});
