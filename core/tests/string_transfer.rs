//! Adversarial end-to-end coverage of the variable-length value types
//! (`VISIBLE_STRING`, `OCTET_STRING`, `DOMAIN`) over SDO expedited and
//! segmented transfer. Drives [`SdoClient`] against [`SdoServer`] through a
//! shared object dictionary, plus a few raw-frame probes of the server against
//! malformed peer input (which must abort, never panic).
//!
//! These began as a bug hunt: the empty-value corruption and the expedited
//! out-of-bounds panic it first surfaced are now fixed, and the corresponding
//! tests assert the corrected behaviour.

use canopen_rs::object_dictionary::Entry;
use canopen_rs::sdo::{
    encode_download_expedited, encode_download_initiate_segmented, SdoClient, SdoEvent, SdoServer,
};
use canopen_rs::{Address, ByteString, DataType, NodeId, ObjectDictionary, Value};

const NODE: u8 = 0x10;

// Object map used across the tests.
const STR_SHORT: Address = Address::new(0x2001, 0); // VISIBLE_STRING, rw
const STR_NAME: Address = Address::new(0x1008, 0); // VISIBLE_STRING, rw (device name)
const OCTET: Address = Address::new(0x3000, 0); // OCTET_STRING, rw
const DOMAIN: Address = Address::new(0x3001, 0); // DOMAIN, rw
const NUM16: Address = Address::new(0x1017, 0); // UNSIGNED16, rw
const NUM64: Address = Address::new(0x2000, 0); // UNSIGNED64, rw

fn make_od() -> ObjectDictionary<16> {
    let mut od = ObjectDictionary::new();
    od.insert(
        STR_SHORT,
        Entry::rw(Value::VisibleString(ByteString::new())),
    )
    .unwrap();
    od.insert(STR_NAME, Entry::rw(Value::VisibleString(ByteString::new())))
        .unwrap();
    od.insert(OCTET, Entry::rw(Value::OctetString(ByteString::new())))
        .unwrap();
    od.insert(DOMAIN, Entry::rw(Value::Domain(ByteString::new())))
        .unwrap();
    od.insert(NUM16, Entry::rw(Value::Unsigned16(1000)))
        .unwrap();
    od.insert(NUM64, Entry::rw(Value::Unsigned64(0))).unwrap();
    od
}

fn peers() -> (SdoClient, SdoServer) {
    let node = NodeId::new(NODE).unwrap();
    (SdoClient::new(node), SdoServer::new(node))
}

fn read(
    client: &mut SdoClient,
    server: &mut SdoServer,
    od: &mut ObjectDictionary<16>,
    addr: Address,
    dt: DataType,
) -> Result<Value, u32> {
    let mut frame = client.read(addr, dt);
    loop {
        let resp = server
            .handle(od, &frame)
            .expect("server replies to a request");
        match client.on_response(&resp) {
            SdoEvent::Send(next) => frame = next,
            SdoEvent::Complete(value) => return Ok(value.expect("read yields a value")),
            SdoEvent::Aborted(code) => return Err(code),
        }
    }
}

fn write(
    client: &mut SdoClient,
    server: &mut SdoServer,
    od: &mut ObjectDictionary<16>,
    addr: Address,
    value: Value,
) -> Result<(), u32> {
    let mut frame = client.write(addr, value);
    loop {
        let resp = server
            .handle(od, &frame)
            .expect("server replies to a request");
        match client.on_response(&resp) {
            SdoEvent::Send(next) => frame = next,
            SdoEvent::Complete(_) => return Ok(()),
            SdoEvent::Aborted(code) => return Err(code),
        }
    }
}

/// Write `bytes` as a VISIBLE_STRING to `addr`, read it back, return the bytes
/// the server actually stored (as observed over the wire).
fn round_trip_visible(addr: Address, bytes: &[u8]) -> Result<Vec<u8>, u32> {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    let value = Value::VisibleString(ByteString::from_bytes(bytes).unwrap());
    write(&mut c, &mut s, &mut od, addr, value)?;
    match read(&mut c, &mut s, &mut od, addr, DataType::VisibleString)? {
        Value::VisibleString(bs) => Ok(bs.as_bytes().to_vec()),
        other => panic!("expected VisibleString, got {other:?}"),
    }
}

// === Boundary lengths (segmented path, addr chosen to force segmented) ======

#[test]
fn boundary_lengths_round_trip() {
    // Lengths that exercise expedited (<=4), the expedited/segmented boundary,
    // single/double/exact-multiple segments, and the max buffer.
    for len in [1usize, 4, 5, 7, 8, 14, 31, 32] {
        let bytes: Vec<u8> = (0..len).map(|i| b'A' + (i % 26) as u8).collect();
        let got = round_trip_visible(STR_NAME, &bytes)
            .unwrap_or_else(|c| panic!("len {len} aborted with 0x{c:08X}"));
        assert_eq!(got, bytes, "round-trip mismatch at len {len}");
    }
}

#[test]
fn empty_string_round_trip() {
    // A zero-length VISIBLE_STRING round-trips as empty. Expedited cannot
    // express a 0-byte payload (the 2-bit unused-byte field maxes at "1 data
    // byte"), so an empty value is routed through segmented transfer with a
    // declared size of 0 and a single empty last segment.
    let got = round_trip_visible(STR_SHORT, &[]).expect("no abort");
    assert_eq!(got, Vec::<u8>::new());
}

// === Round-trip fidelity: OCTET_STRING and DOMAIN, incl. 0x00/0xFF ==========

#[test]
fn octet_string_arbitrary_bytes_round_trip() {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    // Includes 0x00 and 0xFF, length 10 => segmented.
    let bytes = [0x00u8, 0xFF, 0x01, 0x00, 0x7F, 0x80, 0xFF, 0x00, 0xAB, 0xCD];
    let v = Value::OctetString(ByteString::from_bytes(&bytes).unwrap());
    write(&mut c, &mut s, &mut od, OCTET, v).unwrap();
    let back = read(&mut c, &mut s, &mut od, OCTET, DataType::OctetString).unwrap();
    assert_eq!(back, v);
    if let Value::OctetString(bs) = back {
        assert_eq!(bs.as_bytes(), &bytes);
    }
}

#[test]
fn octet_string_short_expedited_round_trip() {
    let (mut c, mut s) = peers();
    let mut od = make_od();
    // 3 bytes incl. 0x00 and 0xFF => expedited.
    let bytes = [0x00u8, 0xFF, 0x7F];
    let v = Value::OctetString(ByteString::from_bytes(&bytes).unwrap());
    write(&mut c, &mut s, &mut od, OCTET, v).unwrap();
    let back = read(&mut c, &mut s, &mut od, OCTET, DataType::OctetString).unwrap();
    assert_eq!(back, v);
}

#[test]
fn domain_round_trip_expedited_and_segmented() {
    for bytes in [
        vec![0xDEu8, 0xAD],             // 2 => expedited
        (0..20u8).collect::<Vec<u8>>(), // 20 => segmented
    ] {
        let (mut c, mut s) = peers();
        let mut od = make_od();
        let v = Value::Domain(ByteString::from_bytes(&bytes).unwrap());
        write(&mut c, &mut s, &mut od, DOMAIN, v).unwrap();
        let back = read(&mut c, &mut s, &mut od, DOMAIN, DataType::Domain).unwrap();
        assert_eq!(back, v, "domain round-trip failed at len {}", bytes.len());
    }
}

// === Segment boundaries: 7 and 14 bytes, toggle sequence & last flag ========

#[test]
fn segment_toggle_sequence_upload_14_bytes() {
    // Drive a segmented *upload* of a 14-byte value (exactly two full segments)
    // frame-by-frame and check the toggle bits and the last-segment flag.
    let (mut c, mut s) = peers();
    let mut od = make_od();
    let bytes: Vec<u8> = (0..14u8).collect();
    write(
        &mut c,
        &mut s,
        &mut od,
        STR_NAME,
        Value::VisibleString(ByteString::from_bytes(&bytes).unwrap()),
    )
    .unwrap();

    // Manually walk the read so we can inspect each segment frame.
    let mut frame = c.read(STR_NAME, DataType::VisibleString);
    // 1: upload-initiate response (segmented, size 14).
    let resp = s.handle(&mut od, &frame).unwrap();
    assert_eq!(resp[0], 0x41, "segmented upload-initiate response");
    assert_eq!(u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]), 14);
    frame = match c.on_response(&resp) {
        SdoEvent::Send(f) => f,
        e => panic!("expected Send, got {e:?}"),
    };
    assert_eq!(frame[0] & 0x10, 0x00, "first poll toggle = 0");

    // 2: first data segment, toggle 0, not last (7 bytes).
    let seg0 = s.handle(&mut od, &frame).unwrap();
    assert_eq!(seg0[0] & 0x10, 0x00, "seg0 toggle = 0");
    assert_eq!(seg0[0] & 0x01, 0x00, "seg0 not last");
    assert_eq!(&seg0[1..8], &bytes[0..7]);
    frame = match c.on_response(&seg0) {
        SdoEvent::Send(f) => f,
        e => panic!("expected Send, got {e:?}"),
    };
    assert_eq!(frame[0] & 0x10, 0x10, "second poll toggle = 1");

    // 3: second data segment, toggle 1, last (7 bytes).
    let seg1 = s.handle(&mut od, &frame).unwrap();
    assert_eq!(seg1[0] & 0x10, 0x10, "seg1 toggle = 1");
    assert_eq!(seg1[0] & 0x01, 0x01, "seg1 is last");
    assert_eq!(&seg1[1..8], &bytes[7..14]);
    match c.on_response(&seg1) {
        SdoEvent::Complete(Some(Value::VisibleString(bs))) => assert_eq!(bs.as_bytes(), &bytes),
        e => panic!("expected Complete, got {e:?}"),
    }
    assert!(!c.is_busy());
    assert!(!s.is_busy());
}

#[test]
fn seven_byte_string_is_single_last_segment() {
    // 7 bytes => one full segment marked last; nothing dropped or duplicated.
    let bytes: Vec<u8> = (0..7u8).map(|i| b'a' + i).collect();
    let got = round_trip_visible(STR_NAME, &bytes).unwrap();
    assert_eq!(got, bytes);
}

// === Type mismatch ==========================================================

#[test]
fn string_longer_than_numeric_object_aborts() {
    // Writing a 3-byte string to a U16 (2-byte) object: the length disagrees
    // with the fixed size, so the server must abort, not corrupt.
    let (mut c, mut s) = peers();
    let mut od = make_od();
    let v = Value::VisibleString(ByteString::from_str("ABC").unwrap());
    let err = write(&mut c, &mut s, &mut od, NUM16, v);
    assert_eq!(err, Err(0x0607_0012)); // DataTypeMismatch, length too high
                                       // The object is untouched.
    assert_eq!(od.read(NUM16).unwrap(), Value::Unsigned16(1000));
}

#[test]
fn same_length_cross_type_write_is_not_detected() {
    // OBSERVATION (protocol limitation, not a code defect): SDO carries no type
    // on the wire, only bytes + length. Writing a 2-byte string to a 2-byte
    // numeric object succeeds and silently reinterprets the bytes. Documented,
    // not asserted as an abort.
    let (mut c, mut s) = peers();
    let mut od = make_od();
    let v = Value::VisibleString(ByteString::from_str("AB").unwrap());
    assert_eq!(write(&mut c, &mut s, &mut od, NUM16, v), Ok(()));
    // 'A'=0x41, 'B'=0x42, little-endian => 0x4241.
    assert_eq!(od.read(NUM16).unwrap(), Value::Unsigned16(0x4241));
}

#[test]
fn numeric_written_to_string_object_is_stored_as_string() {
    // The reverse: a U16 written to a VISIBLE_STRING object is stored as the
    // 2-byte string of its little-endian encoding. Also undetectable.
    let (mut c, mut s) = peers();
    let mut od = make_od();
    assert_eq!(
        write(
            &mut c,
            &mut s,
            &mut od,
            STR_SHORT,
            Value::Unsigned16(0x4241)
        ),
        Ok(())
    );
    let back = read(&mut c, &mut s, &mut od, STR_SHORT, DataType::VisibleString).unwrap();
    assert_eq!(
        back,
        Value::VisibleString(ByteString::from_str("AB").unwrap())
    );
}

// === Malformed-peer probes: the server must abort, never panic ==============

#[test]
fn segmented_download_declaring_oversize_aborts() {
    // A peer declares a segmented download larger than MAX_STRING_LEN (32).
    let mut s = SdoServer::new(NodeId::new(NODE).unwrap());
    let mut od = make_od();
    let frame = encode_download_initiate_segmented(STR_NAME, 33);
    let resp = s.handle(&mut od, &frame).expect("server replies");
    // Must be an abort (0x80 command), not a download-response, and not busy.
    assert_eq!(resp[0], 0x80, "expected abort for oversize declaration");
    assert!(!s.is_busy());

    // And a wildly oversize declaration is likewise rejected cleanly.
    let frame = encode_download_initiate_segmented(STR_NAME, 100_000);
    let resp = s.handle(&mut od, &frame).expect("server replies");
    assert_eq!(resp[0], 0x80);
    assert!(!s.is_busy());
}

#[test]
fn expedited_download_without_size_to_u64_object_aborts() {
    // A download-initiate frame with the expedited bit set but SIZE_INDICATED
    // clear, targeting a fixed >4-byte object (U64): expedited can carry at most
    // four data bytes, so the server must abort cleanly — never fall back to the
    // 8-byte fixed size and slice `req[4..12]` out of bounds.
    let frame: [u8; 8] = [
        0x22, // ccs=download-initiate(0x20) | expedited(0x02), size-indicated clear
        (NUM64.index & 0xFF) as u8,
        (NUM64.index >> 8) as u8,
        NUM64.subindex,
        0,
        0,
        0,
        0,
    ];
    let mut s = SdoServer::new(NodeId::new(NODE).unwrap());
    let mut od = make_od();
    let resp = s
        .handle(&mut od, &frame)
        .expect("server replies with an abort");
    assert_eq!(resp[0], 0x80, "must abort, not panic or write");
    assert!(!s.is_busy());
    assert_eq!(od.read(NUM64).unwrap(), Value::Unsigned64(0)); // untouched
}

#[test]
fn expedited_download_without_size_to_u32_object_is_fine() {
    // The same shape but a 4-byte object stays in bounds: no panic, and the
    // write succeeds (len == fixed_size == 4). Sanity boundary for the bug above.
    let mut s = SdoServer::new(NodeId::new(NODE).unwrap());
    let mut od = make_od();
    // 0x1017 is U16 (2 bytes) — also in bounds. Use a crafted U16 frame.
    let frame: [u8; 8] = [
        0x22,
        (NUM16.index & 0xFF) as u8,
        (NUM16.index >> 8) as u8,
        0,
        0,
        0,
        0,
        0,
    ];
    let resp = s.handle(&mut od, &frame).expect("server replies");
    // Download response (0x60) => the write went through with 2 bytes of zeros.
    assert_eq!(resp[0], 0x60);
    assert_eq!(od.read(NUM16).unwrap(), Value::Unsigned16(0));
}

// === Expedited encode boundary: value larger than 4 bytes is refused ========

#[test]
fn expedited_encode_refuses_over_four_bytes() {
    // A 5-byte string cannot be expedited; the encoder must reject it (the
    // client would instead pick segmented).
    let v = Value::VisibleString(ByteString::from_bytes(&[1, 2, 3, 4, 5]).unwrap());
    assert!(encode_download_expedited(STR_NAME, &v).is_err());
}

// === ByteString invariants ==================================================

#[test]
fn bytestring_equality_ignores_unused_tail() {
    let a = ByteString::from_str("hi").unwrap();
    let b = ByteString::from_bytes(b"hi").unwrap();
    assert_eq!(a, b);
    // Same content but constructed with a longer intermediate then trimmed is
    // not directly expressible; instead confirm distinct-length inequality.
    let c = ByteString::from_bytes(b"hi\0").unwrap();
    assert_ne!(a, c, "trailing NUL is meaningful content, not tail padding");
    assert_eq!(a.len(), 2);
    assert!(!a.is_empty());
}

#[test]
fn bytestring_as_str_none_for_invalid_utf8() {
    let bs = ByteString::from_bytes(&[0xFF, 0xFE, 0x00, 0x80]).unwrap();
    assert_eq!(bs.as_str(), None, "invalid UTF-8 must yield None");
    assert_eq!(bs.as_bytes(), &[0xFF, 0xFE, 0x00, 0x80]);
    assert_eq!(bs.len(), 4);
}

#[test]
fn bytestring_empty_and_default() {
    let e = ByteString::new();
    assert!(e.is_empty());
    assert_eq!(e.len(), 0);
    assert_eq!(e.as_bytes(), &[] as &[u8]);
    assert_eq!(e.as_str(), Some(""));
    assert_eq!(ByteString::default(), e);
}

#[test]
fn bytestring_max_len_ok_over_max_rejected() {
    let max = [0x5Au8; 32];
    assert!(ByteString::from_bytes(&max).is_ok());
    let over = [0x5Au8; 33];
    assert!(ByteString::from_bytes(&over).is_err());
}
