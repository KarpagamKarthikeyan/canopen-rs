//! End-to-end SDO transactions: drive [`SdoClient`] against [`SdoServer`]
//! through a shared object dictionary, with no bus in between. This is the
//! integration check that the codecs, server, and client compose into a
//! working node for both expedited and segmented transfers.

use canopen_rs::object_dictionary::Entry;
use canopen_rs::sdo::{SdoClient, SdoEvent, SdoServer};
use canopen_rs::{Address, DataType, NodeId, ObjectDictionary, Value};

const NODE: u8 = 0x10;

fn make_od() -> ObjectDictionary<8> {
    let mut od = ObjectDictionary::new();
    // 0x1000: device type, read-only, 4 bytes → expedited.
    od.insert(Address::new(0x1000, 0), Entry::constant(Value::Unsigned32(0x0004_0192))).unwrap();
    // 0x1017: heartbeat time, read/write, 2 bytes → expedited.
    od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000))).unwrap();
    // 0x2000: an 8-byte value, read/write → forces segmented transfer.
    od.insert(Address::new(0x2000, 0), Entry::rw(Value::Unsigned64(0))).unwrap();
    od
}

/// Run a read to completion, ferrying frames between client and server.
fn read(
    client: &mut SdoClient,
    server: &mut SdoServer,
    od: &mut ObjectDictionary<8>,
    addr: Address,
    dt: DataType,
) -> Result<Value, u32> {
    let mut frame = client.read(addr, dt);
    loop {
        let resp = server.handle(od, &frame).expect("server always replies to a request");
        match client.on_response(&resp) {
            SdoEvent::Send(next) => frame = next,
            SdoEvent::Complete(value) => return Ok(value.expect("read yields a value")),
            SdoEvent::Aborted(code) => return Err(code),
        }
    }
}

/// Run a write to completion, ferrying frames between client and server.
fn write(
    client: &mut SdoClient,
    server: &mut SdoServer,
    od: &mut ObjectDictionary<8>,
    addr: Address,
    value: Value,
) -> Result<(), u32> {
    let mut frame = client.write(addr, value);
    loop {
        let resp = server.handle(od, &frame).expect("server always replies to a request");
        match client.on_response(&resp) {
            SdoEvent::Send(next) => frame = next,
            SdoEvent::Complete(_) => return Ok(()),
            SdoEvent::Aborted(code) => return Err(code),
        }
    }
}

fn peers() -> (SdoClient, SdoServer) {
    let node = NodeId::new(NODE).unwrap();
    (SdoClient::new(node), SdoServer::new(node))
}

#[test]
fn expedited_read() {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    let v = read(&mut c, &mut s, &mut od, Address::new(0x1000, 0), DataType::Unsigned32);
    assert_eq!(v, Ok(Value::Unsigned32(0x0004_0192)));
}

#[test]
fn expedited_write_then_read_back() {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    assert_eq!(
        write(&mut c, &mut s, &mut od, Address::new(0x1017, 0), Value::Unsigned16(4321)),
        Ok(())
    );
    let v = read(&mut c, &mut s, &mut od, Address::new(0x1017, 0), DataType::Unsigned16);
    assert_eq!(v, Ok(Value::Unsigned16(4321)));
}

#[test]
fn segmented_read_of_eight_byte_value() {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    od.write(Address::new(0x2000, 0), Value::Unsigned64(0x0102_0304_0506_0708)).unwrap();
    let v = read(&mut c, &mut s, &mut od, Address::new(0x2000, 0), DataType::Unsigned64);
    assert_eq!(v, Ok(Value::Unsigned64(0x0102_0304_0506_0708)));
}

#[test]
fn segmented_write_then_read_back() {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    let big = Value::Unsigned64(0xDEAD_BEEF_CAFE_F00D);
    assert_eq!(write(&mut c, &mut s, &mut od, Address::new(0x2000, 0), big), Ok(()));
    assert_eq!(od.read(Address::new(0x2000, 0)).unwrap(), big);
    // And read it back over the wire too.
    let v = read(&mut c, &mut s, &mut od, Address::new(0x2000, 0), DataType::Unsigned64);
    assert_eq!(v, Ok(big));
}

#[test]
fn read_missing_object_aborts_with_standard_code() {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    let frame = c.read(Address::new(0x9999, 0), DataType::Unsigned32);
    let resp = s.handle(&mut od, &frame).unwrap();
    // 0x06020000 — object does not exist in the dictionary.
    assert_eq!(c.on_response(&resp), SdoEvent::Aborted(0x0602_0000));
}

#[test]
fn write_read_only_object_aborts() {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    let err = write(&mut c, &mut s, &mut od, Address::new(0x1000, 0), Value::Unsigned32(1));
    assert_eq!(err, Err(0x0601_0002)); // attempt to write a read-only object
}
