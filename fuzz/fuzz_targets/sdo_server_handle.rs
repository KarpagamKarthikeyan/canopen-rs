#![no_main]

use canopen_rs::{NodeId, ObjectDictionary, SdoServer};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut sdo_server = SdoServer::new(NodeId::new(1).unwrap());
    let mut object_dictionary: ObjectDictionary<10> = ObjectDictionary::new();
    for chunk in data.as_chunks().0 {
        sdo_server.handle(&mut object_dictionary, chunk);
    }
});
