#![no_main]

use canopen_rs::{LssAddress, LssSlave};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [u8; 8]| {
    let mut lss_slave = LssSlave::new(
        LssAddress {
            vendor_id: 0,
            product_code: 0,
            revision_number: 0,
            serial_number: 0,
        },
        1,
    );
    let _ = lss_slave.handle(&data);
});
