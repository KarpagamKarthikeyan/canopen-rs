//! Fresh end-to-end consumer flow, driven entirely through the **public** API
//! (`use canopen_rs::...` only — no private items).
//!
//! This is written from the perspective of an external user of the published
//! crate: it stands up a device [`Node`] that comes up *unconfigured*, has an
//! LSS master discover it by Fastscan and assign a node-id, then drives an
//! [`SdoClient`] against it (expedited + segmented, incl. strings), exchanges a
//! PDO, raises/clears an emergency, and produces a SYNC frame — all with no real
//! bus, by hand-passing frames between the "master" side and `Node::on_frame`.

use canopen_rs::emcy::{error_code, EmergencyMessage};
use canopen_rs::lss::{
    self, encode_configure_node_id, FastscanMaster, LssAddress, LssState, UNCONFIGURED_NODE_ID,
};
use canopen_rs::nmt::NMT_COMMAND_COB_ID;
use canopen_rs::node::Node;
use canopen_rs::standard::StandardObjects;
use canopen_rs::sync::SYNC_COB_ID;
use canopen_rs::{
    Address, ByteString, DataType, Entry, ErrorRegister, MappingEntry, NmtCommand, NodeId,
    ObjectDictionary, PdoMapping, SdoClient, SdoEvent, TransmissionType, Value,
};

// --- Object addresses this device exposes ----------------------------------
const DEVICE_TYPE: Address = Address::new(0x1000, 0); // U32, const (expedited read)
const ERROR_REGISTER: Address = Address::new(0x1001, 0); // U8, ro (EMCY mirror)
const ERROR_FIELD_COUNT: Address = Address::new(0x1003, 0); // U8
const ERROR_FIELD_1: Address = Address::new(0x1003, 1); // U32
const DEVICE_NAME: Address = Address::new(0x1008, 0); // VISIBLE_STRING
const SCRATCH_U32: Address = Address::new(0x2000, 0); // U32, rw (write/read-back)
const SCRATCH_STR: Address = Address::new(0x2100, 0); // VISIBLE_STRING, rw
const PROCESS_DATA: Address = Address::new(0x6000, 1); // U16, rw (PDO source/sink)

const DEVICE_TYPE_VALUE: u32 = 0x0004_0192;

/// The device's fixed 128-bit identity (object 0x1018 / the LSS address).
fn identity() -> LssAddress {
    LssAddress {
        vendor_id: 0x0000_012E,
        product_code: 0x4001,
        revision_number: 0x0000_0001,
        serial_number: 0x00C0_FFEE,
    }
}

/// Build the device's object dictionary the way a real integrator would: the
/// mandatory CiA 301 baseline via [`StandardObjects`], plus a couple of
/// application objects (a scratch numeric, a scratch string, and a PDO source).
fn build_node() -> Node<24> {
    let mut od = ObjectDictionary::<24>::new();
    StandardObjects::new(DEVICE_TYPE_VALUE, identity())
        .with_heartbeat(1000)
        .with_error_history(4)
        .with_device_name("canopen-rs node")
        .insert_into(&mut od)
        .expect("standard objects fit");

    od.insert(SCRATCH_U32, Entry::rw(Value::Unsigned32(0)))
        .expect("scratch u32 fits");
    od.insert(
        SCRATCH_STR,
        Entry::rw(Value::VisibleString(ByteString::new())),
    )
    .expect("scratch string fits");
    od.insert(PROCESS_DATA, Entry::rw(Value::Unsigned16(0)))
        .expect("process-data object fits");

    // Come up unconfigured: no node-id yet, serving only LSS.
    let mut node = Node::new(NodeId::new(1).expect("placeholder id"), od);
    node.enable_lss_unconfigured(identity());
    node
}

// --- A tiny sans-I/O SDO master: drive an SdoClient against a Node ----------

/// Run a complete SDO read (upload) transaction against `node`, panicking on an
/// abort. This is exactly the loop a consumer must hand-write today.
fn sdo_read<const N: usize>(
    node: &mut Node<N>,
    client: &mut SdoClient,
    addr: Address,
    dt: DataType,
) -> Value {
    let mut frame = client.read(addr, dt);
    loop {
        let tx = node
            .on_frame(client.request_cob_id(), &frame)
            .expect("node answers an SDO request");
        assert_eq!(tx.cob_id, client.response_cob_id());
        let resp: [u8; 8] = tx.data().try_into().expect("an SDO response is 8 bytes");
        match client.on_response(&resp) {
            SdoEvent::Send(next) => frame = next,
            SdoEvent::Complete(value) => return value.expect("a read yields a value"),
            SdoEvent::Aborted(code) => panic!("SDO read of {addr:?} aborted: {code:#010x}"),
        }
    }
}

/// Run a complete SDO write (download) transaction against `node`.
fn sdo_write<const N: usize>(
    node: &mut Node<N>,
    client: &mut SdoClient,
    addr: Address,
    value: Value,
) {
    let mut frame = client.write(addr, value);
    loop {
        let tx = node
            .on_frame(client.request_cob_id(), &frame)
            .expect("node answers an SDO request");
        let resp: [u8; 8] = tx.data().try_into().expect("an SDO response is 8 bytes");
        match client.on_response(&resp) {
            SdoEvent::Send(next) => frame = next,
            SdoEvent::Complete(_) => return,
            SdoEvent::Aborted(code) => panic!("SDO write of {addr:?} aborted: {code:#010x}"),
        }
    }
}

/// One mapping entry as a single-object PDO map.
fn single_map(index: u16, sub: u8, bits: u8) -> PdoMapping<{ canopen_rs::node::MAX_PDO_MAPPING }> {
    let mut m = PdoMapping::new();
    m.push(MappingEntry::new(index, sub, bits))
        .expect("one entry fits");
    m
}

#[test]
fn full_device_lifecycle_through_public_api() {
    // ================================================================
    // 1. The node comes up UNCONFIGURED: it must serve only LSS.
    // ================================================================
    let mut node = build_node();
    assert_eq!(
        node.lss().expect("LSS enabled").node_id(),
        UNCONFIGURED_NODE_ID
    );

    // An SDO request to the placeholder id is ignored while unconfigured.
    let mut early_client = SdoClient::new(NodeId::new(1).unwrap());
    let probe = early_client.read(DEVICE_TYPE, DataType::Unsigned32);
    assert!(
        node.on_frame(early_client.request_cob_id(), &probe)
            .is_none(),
        "an unconfigured node must not answer SDO"
    );

    // ================================================================
    // 2. An LSS master DISCOVERS the node by Fastscan, then assigns an id.
    // ================================================================
    let mut master = FastscanMaster::new();
    let mut steps = 0;
    while let Some(req) = master.next_request() {
        let answered = node.on_frame(lss::LSS_MASTER_COB_ID, &req).is_some();
        master.on_response(answered);
        steps += 1;
        assert!(steps < 500, "Fastscan failed to converge");
    }
    assert!(master.found(), "Fastscan found the unconfigured node");
    assert_eq!(
        master.address(),
        identity(),
        "Fastscan recovered the exact 128-bit identity"
    );
    assert_eq!(
        node.lss().unwrap().state(),
        LssState::Configuration,
        "the matched node is in the LSS configuration state"
    );

    // Assign node-id 0x22, then adopt it on the node's reset and boot.
    const ASSIGNED: u8 = 0x22;
    let cfg = node
        .on_frame(lss::LSS_MASTER_COB_ID, &encode_configure_node_id(ASSIGNED))
        .expect("configure-node-id draws a response");
    assert_eq!(cfg.cob_id, lss::LSS_SLAVE_COB_ID);
    assert_eq!(&cfg.data()[..2], &[0x11, 0x00], "configure success");

    assert_eq!(
        node.apply_lss_node_id(),
        Some(NodeId::new(ASSIGNED).unwrap()),
        "the node adopts the assigned id"
    );
    assert_eq!(node.node_id(), NodeId::new(ASSIGNED).unwrap());
    let boot = node.boot();
    assert_eq!(boot.cob_id, 0x700 + u16::from(ASSIGNED));
    assert_eq!(boot.data(), &[0x00], "boot-up frame");

    // ================================================================
    // 3. SDO client against the node: numeric + string round-trips.
    // ================================================================
    let mut client = SdoClient::new(NodeId::new(ASSIGNED).unwrap());
    assert_eq!(client.request_cob_id(), 0x600 + u16::from(ASSIGNED));

    // (a) Expedited read of a numeric object.
    assert_eq!(
        sdo_read(&mut node, &mut client, DEVICE_TYPE, DataType::Unsigned32),
        Value::Unsigned32(DEVICE_TYPE_VALUE),
    );

    // (b) Write + read-back of a numeric object (expedited both ways).
    sdo_write(
        &mut node,
        &mut client,
        SCRATCH_U32,
        Value::Unsigned32(0xDEAD_BEEF),
    );
    assert_eq!(
        sdo_read(&mut node, &mut client, SCRATCH_U32, DataType::Unsigned32),
        Value::Unsigned32(0xDEAD_BEEF),
    );
    // ...and the write is visible in the node's OD.
    assert_eq!(
        node.od().read(SCRATCH_U32).unwrap(),
        Value::Unsigned32(0xDEAD_BEEF)
    );

    // (c) Read the pre-populated device name (segmented upload, > 8 bytes).
    let name = "canopen-rs node";
    assert!(name.len() > 8);
    match sdo_read(&mut node, &mut client, DEVICE_NAME, DataType::VisibleString) {
        Value::VisibleString(bs) => assert_eq!(bs.as_str(), Some(name)),
        other => panic!("expected a VISIBLE_STRING, got {other:?}"),
    }

    // (d) Write + read-back a VISIBLE_STRING longer than 8 bytes (segmented).
    let long = "canopen-rs round-trip!"; // 22 bytes
    assert!(long.len() > 8);
    sdo_write(
        &mut node,
        &mut client,
        SCRATCH_STR,
        Value::VisibleString(ByteString::from_str(long).unwrap()),
    );
    match sdo_read(&mut node, &mut client, SCRATCH_STR, DataType::VisibleString) {
        Value::VisibleString(bs) => assert_eq!(bs.as_str(), Some(long), "long string round-trip"),
        other => panic!("expected a VISIBLE_STRING, got {other:?}"),
    }

    // (e) Write + read-back an EMPTY string (zero-length segmented transfer).
    sdo_write(
        &mut node,
        &mut client,
        SCRATCH_STR,
        Value::VisibleString(ByteString::new()),
    );
    match sdo_read(&mut node, &mut client, SCRATCH_STR, DataType::VisibleString) {
        Value::VisibleString(bs) => {
            assert!(bs.is_empty(), "empty string round-trip");
            assert_eq!(bs.as_bytes(), b"");
        }
        other => panic!("expected a VISIBLE_STRING, got {other:?}"),
    }

    // ================================================================
    // 4. Configure and exchange a PDO.
    //    RPDO 0x200+id -> 0x6000/1; TPDO 0x180+id <- 0x6000/1 (SYNC-triggered).
    // ================================================================
    let id = u16::from(ASSIGNED);
    node.add_rpdo(0x200 + id, single_map(0x6000, 1, 16))
        .expect("rpdo configured");
    node.add_tpdo(
        0x180 + id,
        single_map(0x6000, 1, 16),
        TransmissionType::SynchronousAcyclic,
    )
    .expect("tpdo configured");

    // PDOs flow only in the operational state — the master sends NMT "start".
    assert!(node.sync_tpdos().is_empty(), "no PDOs before operational");
    node.on_frame(
        NMT_COMMAND_COB_ID,
        &[NmtCommand::StartRemoteNode as u8, ASSIGNED],
    );

    // Drive an RPDO in: bytes land in 0x6000/1 (little-endian U16).
    node.on_frame(0x200 + id, &[0xCD, 0xAB]);
    assert_eq!(
        node.od().read(PROCESS_DATA).unwrap(),
        Value::Unsigned16(0xABCD),
        "RPDO unpacked into the OD"
    );

    // A SYNC triggers the TPDO, which packs 0x6000/1 straight back out.
    let tpdos = node.sync_tpdos();
    assert_eq!(tpdos.len(), 1);
    assert_eq!(tpdos[0].cob_id, 0x180 + id);
    assert_eq!(tpdos[0].data(), &[0xCD, 0xAB], "TPDO packed from the OD");

    // ================================================================
    // 5. Raise and clear an EMCY; assert the OD mirror + frame.
    // ================================================================
    let info = [0x11, 0x22, 0x33, 0x44, 0x55];
    let emcy = node.raise_emergency(
        error_code::VOLTAGE,
        ErrorRegister(ErrorRegister::VOLTAGE),
        info,
    );
    // The frame is correct.
    assert_eq!(emcy.cob_id, 0x080 + id);
    let decoded = EmergencyMessage::decode(emcy.data()).unwrap();
    assert_eq!(decoded.error_code, error_code::VOLTAGE);
    assert_eq!(
        decoded.error_register.0,
        ErrorRegister::GENERIC | ErrorRegister::VOLTAGE
    );
    assert_eq!(decoded.vendor_specific, info);

    // The error register (0x1001) and error field (0x1003) mirror into the OD.
    assert_eq!(
        node.error_register(),
        ErrorRegister(ErrorRegister::GENERIC | ErrorRegister::VOLTAGE)
    );
    assert_eq!(
        node.od().read(ERROR_REGISTER).unwrap(),
        Value::Unsigned8(ErrorRegister::GENERIC | ErrorRegister::VOLTAGE)
    );
    assert_eq!(
        node.od().read(ERROR_FIELD_COUNT).unwrap(),
        Value::Unsigned8(1)
    );
    assert_eq!(
        node.od().read(ERROR_FIELD_1).unwrap(),
        Value::Unsigned32(u32::from(error_code::VOLTAGE))
    );
    // A master can read the mirrored register over SDO, too.
    assert_eq!(
        sdo_read(&mut node, &mut client, ERROR_REGISTER, DataType::Unsigned8),
        Value::Unsigned8(ErrorRegister::GENERIC | ErrorRegister::VOLTAGE),
    );

    // Now clear it: register + field reset, and an error-reset EMCY is emitted.
    let reset = node.clear_errors();
    let reset_msg = EmergencyMessage::decode(reset.data()).unwrap();
    assert!(reset_msg.is_error_reset());
    assert_eq!(reset.cob_id, 0x080 + id);
    assert_eq!(node.error_register(), ErrorRegister::NONE);
    assert!(node.error_history().is_empty());
    assert_eq!(node.od().read(ERROR_REGISTER).unwrap(), Value::Unsigned8(0));
    assert_eq!(
        node.od().read(ERROR_FIELD_COUNT).unwrap(),
        Value::Unsigned8(0)
    );

    // ================================================================
    // 6. The node acts as the network SYNC producer.
    // ================================================================
    node.enable_sync_producer(0)
        .expect("valid counter overflow");
    let sync = node.produce_sync().expect("a SYNC frame");
    assert_eq!(sync.cob_id, SYNC_COB_ID);
    assert_eq!(sync.data(), &[] as &[u8], "counter-less SYNC is empty");
}
