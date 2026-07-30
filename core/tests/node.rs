//! Drive a real [`SdoClient`] against a [`Node`] (not a bare `SdoServer`),
//! confirming the bundled node serves full multi-frame transactions.

use canopen_rs::node::Node;
use canopen_rs::sdo::{SdoClient, SdoEvent};
use canopen_rs::{Address, DataType, Entry, NodeId, ObjectDictionary, Value};

const NODE: u8 = 0x10;

fn make_node() -> Node<8> {
    let mut od = ObjectDictionary::new();
    od.insert(
        Address::new(0x1000, 0),
        Entry::constant(Value::Unsigned32(0x0004_0192)),
    )
    .unwrap();
    od.insert(Address::new(0x2000, 0), Entry::rw(Value::Unsigned64(0)))
        .unwrap();
    Node::new(NodeId::new(NODE).unwrap(), od)
}

/// Ferry frames between a client and a node until the transaction resolves.
fn read(
    client: &mut SdoClient,
    node: &mut Node<8>,
    addr: Address,
    dt: DataType,
) -> Result<Value, u32> {
    let mut request = client.read(addr, dt);
    loop {
        let tx = node
            .on_frame(client.request_cob_id(), &request)
            .expect("a booted node answers SDO requests");
        let mut payload = [0u8; 8];
        payload[..tx.data().len()].copy_from_slice(tx.data());
        match client.on_response(&payload) {
            SdoEvent::Send(next) => request = next,
            SdoEvent::Complete(value) => return Ok(value.expect("read yields a value")),
            SdoEvent::Aborted(code) => return Err(code),
        }
    }
}

#[test]
fn segmented_read_through_node() {
    let mut node = make_node();
    node.boot(); // pre-operational — SDO now served
    node.od_mut()
        .write(
            Address::new(0x2000, 0),
            Value::Unsigned64(0x0102_0304_0506_0708),
        )
        .unwrap();

    let mut client = SdoClient::new(NodeId::new(NODE).unwrap());
    let value = read(
        &mut client,
        &mut node,
        Address::new(0x2000, 0),
        DataType::Unsigned64,
    );
    assert_eq!(value, Ok(Value::Unsigned64(0x0102_0304_0506_0708)));
}

#[test]
fn read_before_boot_is_not_served() {
    let mut node = make_node(); // still initialising
    let request = SdoClient::new(NodeId::new(NODE).unwrap())
        .read(Address::new(0x1000, 0), DataType::Unsigned32);
    assert!(node.on_frame(0x610, &request).is_none());
}
