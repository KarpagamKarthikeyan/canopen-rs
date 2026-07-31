//! Full node lifecycle through the public API — the "overall flow" test.
//!
//! Drives a [`Node`] the way a master would over the bus (but sans-I/O): an
//! unconfigured node is assigned an id over LSS, boots, is read/written over
//! SDO, is started via NMT, and then exchanges process data over PDO. Every
//! step goes through `Node::on_frame` and the public codecs, so this exercises
//! the modules *composed*, not just in isolation.

use canopen_rs::lss::{
    self, encode_configure_node_id, encode_store, encode_switch_global, LssAddress, LssState,
};
use canopen_rs::nmt::{self, encode_command};
use canopen_rs::node::{Node, MAX_PDO_MAPPING};
use canopen_rs::sdo::{SdoClient, SdoEvent};
use canopen_rs::{
    Address, DataType, Entry, MappingEntry, NmtCommand, NmtState, NodeId, ObjectDictionary,
    PdoMapping, TransmissionType, Value,
};

fn build_node() -> Node<8> {
    let mut od = ObjectDictionary::new();
    od.insert(
        Address::new(0x1000, 0),
        Entry::constant(Value::Unsigned32(0x0004_0192)),
    )
    .unwrap();
    od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(0)))
        .unwrap();
    od.insert(Address::new(0x6000, 1), Entry::rw(Value::Unsigned16(0)))
        .unwrap();
    // Provisional id 1 while unconfigured; LSS assigns the real one.
    Node::new(NodeId::new(1).unwrap(), od)
}

fn pad(data: &[u8]) -> [u8; 8] {
    let mut p = [0u8; 8];
    p[..data.len()].copy_from_slice(data);
    p
}

fn single_map(index: u16, sub: u8, bits: u8) -> PdoMapping<MAX_PDO_MAPPING> {
    let mut m = PdoMapping::new();
    m.push(MappingEntry::new(index, sub, bits)).unwrap();
    m
}

/// Run an SDO read against the node, ferrying frames through `on_frame`.
fn sdo_read(node: &mut Node<8>, client: &mut SdoClient, addr: Address, dt: DataType) -> Value {
    let mut req = client.read(addr, dt);
    loop {
        let resp = node
            .on_frame(client.request_cob_id(), &req)
            .expect("node answers SDO");
        match client.on_response(&pad(resp.data())) {
            SdoEvent::Send(next) => req = next,
            SdoEvent::Complete(v) => return v.expect("read yields a value"),
            SdoEvent::Aborted(code) => panic!("SDO aborted {code:#010x}"),
        }
    }
}

/// Run an SDO write against the node.
fn sdo_write(node: &mut Node<8>, client: &mut SdoClient, addr: Address, value: Value) {
    let mut req = client.write(addr, value);
    loop {
        let resp = node
            .on_frame(client.request_cob_id(), &req)
            .expect("node answers SDO");
        match client.on_response(&pad(resp.data())) {
            SdoEvent::Send(next) => req = next,
            SdoEvent::Complete(_) => return,
            SdoEvent::Aborted(code) => panic!("SDO aborted {code:#010x}"),
        }
    }
}

#[test]
fn unconfigured_node_full_lifecycle() {
    let mut node = build_node();
    let address = LssAddress {
        vendor_id: 0x1F,
        product_code: 0x2A,
        revision_number: 1,
        serial_number: 0x99,
    };
    node.enable_lss(address);

    // --- 1. LSS: a master assigns node-id 0x20 over the bus. ---
    node.on_frame(lss::LSS_MASTER_COB_ID, &encode_switch_global(true));
    assert_eq!(node.lss().unwrap().state(), LssState::Configuration);
    node.on_frame(lss::LSS_MASTER_COB_ID, &encode_configure_node_id(0x20));
    node.on_frame(lss::LSS_MASTER_COB_ID, &encode_store());
    node.on_frame(lss::LSS_MASTER_COB_ID, &encode_switch_global(false));

    // --- 2. The node resets and adopts the assigned id. ---
    let node_id = node.apply_lss_node_id().expect("assigned id adopted");
    assert_eq!(node_id, NodeId::new(0x20).unwrap());

    // --- 3. Boot into pre-operational. ---
    let boot = node.boot();
    assert_eq!(boot.cob_id, 0x720); // 0x700 + 0x20
    assert_eq!(node.state(), NmtState::PreOperational);

    // --- 4. SDO read/write on the new COB-IDs (0x600 + 0x20 = 0x620). ---
    let mut client = SdoClient::new(node_id);
    assert_eq!(client.request_cob_id(), 0x620);
    let device_type = sdo_read(
        &mut node,
        &mut client,
        Address::new(0x1000, 0),
        DataType::Unsigned32,
    );
    assert_eq!(device_type, Value::Unsigned32(0x0004_0192));

    sdo_write(
        &mut node,
        &mut client,
        Address::new(0x1017, 0),
        Value::Unsigned16(500),
    );
    assert_eq!(
        node.od().read(Address::new(0x1017, 0)).unwrap(),
        Value::Unsigned16(500)
    );

    // --- 5. Configure PDOs, then start (NMT) into operational. ---
    node.add_rpdo(0x200 + 0x20, single_map(0x6000, 1, 16))
        .unwrap();
    node.add_tpdo(
        0x180 + 0x20,
        single_map(0x6000, 1, 16),
        TransmissionType::SynchronousAcyclic,
    )
    .unwrap();
    node.on_frame(
        nmt::NMT_COMMAND_COB_ID,
        &encode_command(NmtCommand::StartRemoteNode, node_id),
    );
    assert_eq!(node.state(), NmtState::Operational);

    // --- 6. Real-time data: RPDO in, then SYNC triggers the TPDO out. ---
    node.on_frame(0x220, &[0x34, 0x12]); // RPDO1 -> 0x6000/1 = 0x1234
    assert_eq!(
        node.od().read(Address::new(0x6000, 1)).unwrap(),
        Value::Unsigned16(0x1234)
    );

    let tpdos = node.sync_tpdos();
    assert_eq!(tpdos.len(), 1);
    assert_eq!(tpdos[0].cob_id, 0x1A0); // 0x180 + 0x20
    assert_eq!(tpdos[0].data(), &[0x34, 0x12]);

    // --- 7. Stop (NMT) — PDOs and SDO go inactive. ---
    node.on_frame(
        nmt::NMT_COMMAND_COB_ID,
        &encode_command(NmtCommand::StopRemoteNode, node_id),
    );
    assert_eq!(node.state(), NmtState::Stopped);
    assert!(node.sync_tpdos().is_empty());
    let probe = SdoClient::new(node_id).read(Address::new(0x1000, 0), DataType::Unsigned32);
    assert!(node.on_frame(0x620, &probe).is_none()); // SDO not served when stopped
}
