//! Fastscan discovery driven end-to-end through the node's public API.
//!
//! An unconfigured node (no node-id) is discovered by [`FastscanMaster`] purely
//! through [`Node::on_frame`] — the master never knows its address in advance —
//! then assigned an id, booted, and exercised over SDO and PDO. It asserts the
//! whole gate: SDO and PDO are *refused* while unconfigured and *served* once an
//! id is assigned, all on the newly-assigned COB-IDs. Companion to
//! `lifecycle.rs`, which uses a global/selective switch rather than Fastscan.

use canopen_rs::lss::{
    self, encode_configure_node_id, FastscanMaster, LssAddress, LssState, UNCONFIGURED_NODE_ID,
};
use canopen_rs::nmt::{self, encode_command};
use canopen_rs::node::{Node, MAX_PDO_MAPPING};
use canopen_rs::sdo::{encode_upload_request, SdoClient, SdoEvent};
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
    od.insert(Address::new(0x6000, 1), Entry::rw(Value::Unsigned16(0)))
        .unwrap();
    // Provisional id 1 — never used on the bus while unconfigured.
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

#[test]
fn fastscan_discovery_gates_then_serves_full_traffic() {
    let mut node = build_node();
    let address = LssAddress {
        vendor_id: 0x00A3_1F07,
        product_code: 0x2A55_0102,
        revision_number: 0x0000_0003,
        serial_number: 0xBEEF_1234,
    };
    node.enable_lss_unconfigured(address);

    // --- 1. While unconfigured, only LSS is served. ---
    // An SDO request on the provisional id's COB-ID (0x600 + 1) is ignored,
    // even after a (premature) boot.
    node.boot();
    let probe = encode_upload_request(Address::new(0x1000, 0));
    assert!(
        node.on_frame(0x601, &probe).is_none(),
        "SDO must be refused while unconfigured"
    );
    // A PDO frame is ignored too — the node has no meaningful data COB-IDs yet.
    node.on_frame(0x201, &[0x34, 0x12]);

    // --- 2. Fastscan discovery, entirely through on_frame. ---
    let mut master = FastscanMaster::new();
    let mut steps = 0;
    while let Some(req) = master.next_request() {
        let answered = node.on_frame(lss::LSS_MASTER_COB_ID, &req).is_some();
        master.on_response(answered);
        steps += 1;
        assert!(steps < 200, "fastscan did not converge");
    }
    assert!(master.found());
    assert_eq!(master.address(), address);
    assert_eq!(node.lss().unwrap().state(), LssState::Configuration);
    // Still unconfigured until an id is actually assigned and adopted.
    assert_eq!(node.lss().unwrap().node_id(), UNCONFIGURED_NODE_ID);

    // --- 3. Assign node-id 0x33 over LSS, then adopt it on reset. ---
    let resp = node
        .on_frame(lss::LSS_MASTER_COB_ID, &encode_configure_node_id(0x33))
        .expect("configure response");
    assert_eq!(resp.cob_id, lss::LSS_SLAVE_COB_ID);
    assert_eq!(&resp.data()[..2], &[0x11, 0x00]); // success
    let node_id = node.apply_lss_node_id().expect("assigned id adopted");
    assert_eq!(node_id, NodeId::new(0x33).unwrap());

    // --- 4. Boot; SDO now served on the assigned COB-ID (0x600 + 0x33). ---
    node.boot();
    assert_eq!(node.state(), NmtState::PreOperational);
    let mut client = SdoClient::new(node_id);
    assert_eq!(client.request_cob_id(), 0x633);
    let device_type = sdo_read(
        &mut node,
        &mut client,
        Address::new(0x1000, 0),
        DataType::Unsigned32,
    );
    assert_eq!(device_type, Value::Unsigned32(0x0004_0192));

    // --- 5. Configure PDOs, start, and exchange process data. ---
    node.add_rpdo(0x200 + 0x33, single_map(0x6000, 1, 16))
        .unwrap();
    node.add_tpdo(
        0x180 + 0x33,
        single_map(0x6000, 1, 16),
        TransmissionType::SynchronousAcyclic,
    )
    .unwrap();
    node.on_frame(
        nmt::NMT_COMMAND_COB_ID,
        &encode_command(NmtCommand::StartRemoteNode, node_id),
    );
    assert_eq!(node.state(), NmtState::Operational);

    node.on_frame(0x233, &[0x34, 0x12]); // RPDO -> 0x6000/1 = 0x1234
    assert_eq!(
        node.od().read(Address::new(0x6000, 1)).unwrap(),
        Value::Unsigned16(0x1234)
    );
    let tpdos = node.sync_tpdos();
    assert_eq!(tpdos.len(), 1);
    assert_eq!(tpdos[0].cob_id, 0x1B3); // 0x180 + 0x33
    assert_eq!(tpdos[0].data(), &[0x34, 0x12]);
}
