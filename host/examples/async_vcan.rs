//! Async SDO round-trips over a Linux `vcan0` virtual CAN bus (tokio).
//!
//! The async counterpart of `vcan_loopback`: a server task runs a device
//! [`Node`], while the main task reads and writes it with `await`ed SDO
//! transactions — expedited and segmented.
//!
//! Set up the interface first (see `tools/vcan_setup.sh`), then:
//!
//! ```text
//! cargo run -p canopen-host --example async_vcan --features tokio
//! ```

#[cfg(all(target_os = "linux", feature = "tokio"))]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use canopen_host::async_transport::AsyncSocketCan;
    use canopen_rs::node::Node;
    use canopen_rs::{Address, DataType, Entry, NodeId, ObjectDictionary, Value};

    const IFACE: &str = "vcan0";
    let node_id = NodeId::new(0x10)?;

    // Open the master's socket first: a downed or missing interface fails fast
    // here with a clear message, rather than as a panic buried in the server
    // task (which would surface only as a baffling channel `RecvError`).
    let bus = AsyncSocketCan::open(IFACE)?;

    // --- Server task: serve an object dictionary on vcan0. ---
    // The ready channel carries the server's open result so a failure there is
    // reported cleanly instead of panicking the task.
    let (ready_tx, ready_rx) = tokio::sync::oneshot::channel::<Result<(), String>>();
    let server = tokio::spawn(async move {
        let bus = match AsyncSocketCan::open(IFACE) {
            Ok(bus) => bus,
            Err(e) => {
                let _ = ready_tx.send(Err(e.to_string()));
                return;
            }
        };
        let mut od = ObjectDictionary::<8>::new();
        od.insert(
            Address::new(0x1000, 0),
            Entry::constant(Value::Unsigned32(0x0004_0192)),
        )
        .unwrap();
        od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000)))
            .unwrap();
        od.insert(Address::new(0x2000, 0), Entry::rw(Value::Unsigned64(0)))
            .unwrap();
        let mut node = Node::new(node_id, od);
        node.boot();
        let _ = ready_tx.send(Ok(())); // socket is open and listening
        while let Ok(frame) = bus.recv().await {
            if let Some(tx) = node.on_frame(frame.cob_id, frame.data()) {
                if bus.send(tx.cob_id, tx.data()).await.is_err() {
                    break;
                }
            }
        }
    });
    match ready_rx.await {
        Ok(Ok(())) => {}
        Ok(Err(e)) => return Err(format!("server failed to start: {e}").into()),
        Err(_) => return Err("server task exited before signalling ready".into()),
    }

    // --- Client: async SDO read/write (reusing the socket opened above). ---

    let device_type = bus
        .sdo_read(node_id, Address::new(0x1000, 0), DataType::Unsigned32)
        .await?;
    println!("read  0x1000 device type      = {device_type:?}");
    assert_eq!(device_type, Value::Unsigned32(0x0004_0192));

    bus.sdo_write(node_id, Address::new(0x1017, 0), Value::Unsigned16(2500))
        .await?;
    let heartbeat = bus
        .sdo_read(node_id, Address::new(0x1017, 0), DataType::Unsigned16)
        .await?;
    println!("write 0x1017 heartbeat -> read = {heartbeat:?}");
    assert_eq!(heartbeat, Value::Unsigned16(2500));

    // 8-byte value -> segmented transfer, awaited.
    let big = Value::Unsigned64(0x0102_0304_0506_0708);
    bus.sdo_write(node_id, Address::new(0x2000, 0), big).await?;
    let back = bus
        .sdo_read(node_id, Address::new(0x2000, 0), DataType::Unsigned64)
        .await?;
    println!("write 0x2000 (segmented) -> read = {back:?}");
    assert_eq!(back, big);

    println!("\nasync vcan0 OK — expedited and segmented SDO round-trips over tokio.");
    server.abort();
    Ok(())
}

#[cfg(not(all(target_os = "linux", feature = "tokio")))]
fn main() {
    eprintln!(
        "This example needs Linux + the `tokio` feature:\n  \
         cargo run -p canopen-host --example async_vcan --features tokio"
    );
}
